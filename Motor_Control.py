import RPi.GPIO as GPIO
import time

class MotorController:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Servo Motor Pins (Software PWM)
        self.servo1_pin = 12  # GPIO17 (change as needed)
        self.servo2_pin = 18  # GPIO18

        # L298N DC Motor Pins
        self.ENA = 13  # PWM Speed Control (GPIO23)
        self.IN1 = 24  # Direction 1 (GPIO24)
        self.IN2 = 23  # Direction 2 (GPIO25)

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
        self._set_servo_angle(self.servo1_pwm, 90)
        self._set_servo_angle(self.servo2_pwm, 90)
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

    def launch1(self):
        self.servo1_pwm.ChangeDutyCycle(2.5)
        time.sleep(1)
        self.servo1_pwm.ChangeDutyCycle(7.5)
        time.sleep(1)
        # Rotate DC motor 1 full revolution (adjust duration based on motor speed)
        self._dc_motor_rotate(direction=1, speed=50, duration=1)  # Forward
        self._dc_motor_rotate(direction=-1, speed=50, duration=1)  # Back to start
    
    def launch2(self):
        self.servo1_pwm.ChangeDutyCycle(2.5)
        time.sleep(1)
        self.servo1_pwm.ChangeDutyCycle(7.5)
        time.sleep(1)
        # Rotate DC motor 1 full revolution (adjust duration based on motor speed)
        self._dc_motor_rotate(direction=1, speed=50, duration=1)  # Forward
        self._dc_motor_rotate(direction=-1, speed=50, duration=1)  # Back to start

    def cleanup(self):
        """Clean up GPIO."""
        self.servo1_pwm.stop()
        self.servo2_pwm.stop()
        self.dc_motor_pwm.stop()
        GPIO.cleanup()


# Example Usage
if __name__ == "__main__":       #With this line, the motor control code won't run on import.
    controller = MotorController()
    try:
        launch_count = 0
        while launch_count <=6:
            if launch_count <=3:
                controller.launch1()
            else:
                controller.launch2()
            launch_count += 1
    except KeyboardInterrupt:
        print("Program Interrupted")
    finally:
        controller.cleanup()
