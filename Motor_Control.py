import RPi.GPIO as GPIO
import time

class MotorController:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Servo Motor Pins (Software PWM)
        self.servo1_pin = 17  # GPIO17 (change as needed)
        self.servo2_pin = 18  # GPIO18

        # L298N DC Motor Pins
        self.ENA = 23  # PWM Speed Control (GPIO23)
        self.IN1 = 24  # Direction 1 (GPIO24)
        self.IN2 = 25  # Direction 2 (GPIO25)

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
        """Set servos to 90° and stop DC motor."""
        self._set_servo_angle(self.servo1_pwm, 90)
        self._set_servo_angle(self.servo2_pwm, 90)
        self._dc_motor_stop()
        time.sleep(1)  # Allow time for servos to move

    def _set_servo_angle(self, servo_pwm, angle):
        """Convert angle (0-180°) to duty cycle (2.5%-12.5%)."""
        duty_cycle = 2.5 + (angle / 18)  # 2.5% (0°), 7.5% (90°), 12.5% (180°)
        servo_pwm.ChangeDutyCycle(duty_cycle)

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
        else:  # Backward
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)

        self.dc_motor_pwm.ChangeDutyCycle(speed)
        if duration:
            time.sleep(duration)
            self._dc_motor_stop()

    def servo_sequence(self, servo_num):
        """Run sequence for selected servo (1 or 2)."""
        servo_pwm = self.servo1_pwm if servo_num == 1 else self.servo2_pwm

        # Move servo to 0°
        self._set_servo_angle(servo_pwm, 0)
        time.sleep(2)

        # Return servo to 90°
        self._set_servo_angle(servo_pwm, 90)
        time.sleep(0.5)

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
if __name__ == "__main__":
    controller = MotorController()
    try:
        print("Running sequence for Servo 1")
        controller.servo_sequence(servo_num=1)
        
        time.sleep(1)
        
        print("Running sequence for Servo 2")
        controller.servo_sequence(servo_num=2)
    finally:
        controller.cleanup()
