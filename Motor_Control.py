import RPi.GPIO as GPIO
import time
class MotorController:
    def __init__(self):
        super().__init__("motorandtemp")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.ptoxsub = self.create_subscription(Bool,"proxcheck",self.proxchecker,10)
        self.counter = 0

        # Servo Motor Pins (Software PWM)
        self.servo1_pin = 12  # GPIO12 (change as needed)
        self.servo2_pin = 18  # GPIO18

        # L298N DC Motor Pins
        self.ENA = 13  # PWM Speed Control (GPIO13)
        self.IN1 = 24  # Direction 1 (GPIO24)
        self.IN2 = 23  # Direction 2 (GPIO23)

        # Servo PWM Frequency (50Hz for standard servos)
        self.servo_pwm_freq = 50

        # Initialize Servos
        GPIO.setup(self.servo1_pin, GPIO.OUT)
        GPIO.setup(self.servo2_pin, GPIO.OUT)
        self.servo1_pwm = GPIO.PWM(self.servo1_pin, self.servo_pwm_freq)
        self.servo2_pwm = GPIO.PWM(self.servo2_pin, self.servo_pwm_freq)

        # Initialize L298N DC Motor
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        self.dc_motor_pwm = GPIO.PWM(self.ENA, 1000)  # 1kHz PWM for L298N
        self.dc_motor_pwm.start(0)
        
        # Set initial positions
        self.initialize_motors()

        self.initialize_MLX90614_ADDR = 0x5A
        self.MLX90614_TA = 0x06
        self.MLX90614_TOBJ1 = 0x07
        self.bus = smbus2.SMBus(1)  # I2C bus 1

        self.temppub = self.create_publisher(Float32, '/temp_pub',10)
        self.object_temp = Float32()
        self.object_temp.data = 0.0
        self.prox = Bool()
    
    def read_temperature(self,register):
        data = self.bus.read_i2c_block_data(self.MLX90614_ADDR, register,2)
        raw_data = (data[1] << 8) + data[0]
        temperature = (raw_data * 0.02) - 273.15
        return temperature
    def proxchecker(self,msg):
        self.prox = msg
        print(self.prox)
        if self.prox.data:
            self.mainlauncher()
    def fn(self):
        try:
            while True:
                ambient_temp = self.read_temperature(MLX90614_TA)
                object_temp = self.read_temperature(MLX90614_TOBJ1)

                self.object_temp.data = object_temp
                print(f"Ambient Temp: {ambient_temp:.2f} degrees Celsius")
                print(f"Object Temp: {object_temp:.2f} degrees Celsius")
                print(f"\n")
                self.temppub.publish(self,temp)
                time.sleep(0.3)
        except KeyboardInterrupt:
            print("Exiting..")
        finally:
            GPIO.cleanup()

    def initialize_motors(self):
        self.servo1_pwm.start(7.5)
        self.servo2_pwm.start(12.5)
        self._dc_motor_stop()

    def _dc_motor_stop(self):
        """Stop DC motor (L298N)."""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        self.dc_motor_pwm.ChangeDutyCycle(0)

    def _dc_motor_rotate(self, direction, speed=50, duration=None):
        """
        Rotate DC motor via L298N.
        - direction: 1 (forward), -1 (backward)
        - speed: PWM duty cycle (0-100)
        - duration: time in seconds (None = continuous)
        """
        if direction == 1:  # Forward
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else: 
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        self.dc_motor_pwm.ChangeDutyCycle(100)

        if duration:
            time.sleep(duration) #The motor will run for exactly that many seconds
            self._dc_motor_stop()
            
    def stopper(self, servo_no):
        if servo_no == 1:
            servo_pwm = self.servo1_pwm
            servo_pwm.ChangeDutyCycle(2.5)
            time.sleep(0.3)
            servo_pwm.ChangeDutyCycle(7.5)
            time.sleep(0.3)
        else:
            servo_pwm = self.servo2_pwm
            servo_pwm.ChangeDutyCycle(5.5)
            time.sleep(0.3)
            servo_pwm.ChangeDutyCycle(12.5)
            time.sleep(0.3)
        
    def launch(self,stopper_no):
        self.stopper(stopper_no)  
        time.sleep(0)
        self._dc_motor_rotate(direction=1, speed=100, duration=1.11)
        self._dc_motor_stop()
        time.sleep(0.2)
        self._dc_motor_rotate(direction=1, speed=100, duration=0.58)

    def launch_1(self):
        self.counter = 1
        time.sleep(0.1)
        self.launch(1)
        time.sleep(1)
        self.launch(1)
        time.sleep(0.1)
        self.launch(1)

    def launch_2(self):
        self.counter = 2
        self.launch(1)
        time.sleep(3)
        self.launch(1)
        time.sleep(1.7)
        self.launch(2)
        
    def launch_3(self):
        self.counter = 3
        time.sleep(2)
        self.launch(2)
        time.sleep(4)
        self.launch(2)
        time.sleep(2)
        self.launch(2)
      
    def cleanup(self):
        self.servo1_pwm.stop()
        self.servo2_pwm.stop()
        self.dc_motor_pwm.stop()
        GPIO.cleanup()
    
    def mainlauncher(self):
        if self.counter == 0:
            print("missles away")
            self.launch_1()
            time.sleep(10)
            return
        elif self.counter == 1:
            print("missles away second time")
            self.launch_2()
            time.sleep(10)
            return
        elif self.counter == 2:
            print("wah bonus also")
            self.launch_3()
        elif self.counter == 3:
            return

# Example Usage
if __name__ == "__main__":       #With this line, the motor control code won't run on import.
    controller = MotorController()
    try:
        controller.fn()        
    except KeyboardInterrupt:
        print("Program Interrupted")
        controller.destroy_node()
        rclpy.shutdown()
    finally:
        controller.cleanup()
