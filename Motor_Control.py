import RPi.GPIO as GPIO
import time

class MotorController:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

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

    def initialize_motors(self):
        self.servo1_pwm.start(7.5)
        self.servo2_pwm.start(7.5)
        self._dc_motor_stop()
        time.sleep(1)  # Allow time for servos to move

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

        self.dc_motor_pwm.ChangeDutyCycle(speed)
        if duration:
            time.sleep(duration) #The motor will run for exactly that many seconds
            self._dc_motor_stop()
            
    def stopper(self, servo_no):
        if servo_no == 1:
            servo_pwm = self.servo1_pwm
        else:
            servo_pwm = self.servo2_pwm
        servo_pwm.ChangeDutyCycle(2.5)
        time.sleep(1)
        servo_pwm.ChangeDutyCycle(7.5)
        time.sleep(1)
        
    def launch(self):
        global launch_count   # a global variable in the main control code
        if launch_count < 3:
            self.stopper(1)  
        else:
            self.stopper(2)
        # Rotate DC motor 1 full revolution (adjust duration based on motor speed)
        self._dc_motor_rotate(direction=1, speed=50, duration=0.87)
        time.sleep(1)
        self._dc_motor_rotate(direction=1, speed=50, duration=0.82)
        time.sleep(1)

    def cleanup(self):
        self.servo1_pwm.stop()
        self.servo2_pwm.stop()
        self.dc_motor_pwm.stop()
        GPIO.cleanup()


# Example Usage
if __name__ == "__main__":       #With this line, the motor control code won't run on import.
    controller = MotorController()
    try:
        launch_count = 0
        controller.launch()
        launch_count = 3
        controller.launch()
    except KeyboardInterrupt:
        print("Program Interrupted")
    finally:
        controller.cleanup()
