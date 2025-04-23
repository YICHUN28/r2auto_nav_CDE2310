import RPi.GPIO as GPIO
import time
import rclpy
import rcipy.node import Node
from std_msgs.msg import Float32.Bool
import smbus2
import numpy as np

class MotorController(Node):
  def __init__(self):
    super().__init__("motorandtemp")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    self.proxsub = self.create_subscription(Bool, "/proxcheck",self.proxchecker,10)
    self.counter = 0

    self.servo1_pin = 12
    self.servo2_pin = 16

    self.ENA = 13
    self.IN1 = 24
    self.IN2 = 23

    self.servo_pwm_freq = 50
    
    GPIO.setup(self.servo1_pin, GPIO. OUT)
    GPIO.setup(self.servo2_pin, GPIO. OUT)
    self.servo1_pwm = GPIO.PWM(self.servo1_pin, self.servo_pwm_freq)
    self.servo2_pwm = GPIO.PWM(self.servo2_pin, self.servo_pwm_freq)

    GPIO.setup(self.ENA, GPIO.OUT)
    GPIO.setup(self.IN1, GPIO.OUT)
    GPIO.setup(self.IN2, GPIO.OUT)
    self.dc_motor_pwm = GPIO.PWM(self.ENA, 1000)
    self.dc_motor_pwm.start(0)

    self.initialize_motors()

    self.MLX90614_ADDR = 0x5A
    self.MLX90614_TA = 0x06
    self.MLX90614_TOBJ1 = 0x07
    self.bus = smbus2.SMBus(1)
    
    self.temppub = self.create_publisher(Float32, '/temp_pub', 10)
    self.object_temp = Float32()

    self.object_temp.data = 0.0
    self.prox = Bool0)
    self.prox.data = False

  def read_terperature(self,register):
    data = self.bus.read.i2c_block_data(self.MLX90614_ADDR, register,2)
    raw_data = (data[1] << 8) + data[0]
    temperature = (raw_data * 0.02) - 273.15
    return temperature
  def proxchecker(self,msg):
    self.prox = msg
  def fn(self):
    try:
      while True:
        ambient_temp = self.read_temperature(self.MLX90614_TA)
        object_temp = self.read_temperature(Self.MLX90614_TOBJ1)
        
        self.object_temp.data = object_temp
        print (f"Ambient Temp: {ambient_temp:.2f} degrees Celsius")
        print (f"Object Temp: {object_temp:.2f} degrees Celsius")
        print (f"\n")
        self.temppub.publish(self.object_temp)
        time.sleep(0.3)
        
        if object_temp > 30.5: 
          if self.counter == 0:
            print ("missiles away")
            self.launch_1()
            time.sleep (20)
            continue
          elif self.counter == 1:
            print ("missles away second time")
            self.launch_2()
            time.sleep(20)
            continue
          elif self.counter == 2:
            print ("wah bonus also")
            self.launch_3()
            time.sleep(20)
            continue
          elif self.counter == 3:
            continue
            
    except Keyboardinterrupt:
      print ("Exiting..")
    finally:
      GPIO.cleanup()

  def initialize_notors(self):
    self.servo1_pwm.start(12.5)
    self.servo2_pwm.start(7.5)    
    self._dc_motor_stop()
    time.sleep(0.2) #Allow time for servos to move
  
  def _dc_motor_stop(self):
    GPTO.output(self.IN1, GPIO.LOW)
    GP10.output(self.IN2, GPIO.LOW)
    self.dc_motor_pwm.ChangeDutyCycle(0)
  
  def _dc_motor_rotate(self, direction, speed, duration=None):
    """
    Rotate DC motor via L298N.
  - direction: 1 (forward), -1 (backward)
  - speed: PuM duty cycle (0-100)
  - duration: time in seconds (None = continuous)
  """
   if direction == 1: #Forward
      GPIO.output(self.IN1, GPIO.HIGH)
      GPIO.output(self.IN2, GPIO.LOW)
   else:
      GPIO.output(self.IN1, GPIO.LOW)
      GPIO.output(self.IN2, GPIO.HIGH)
      self.dc_motor_pwm.ChangeDutyCycle(100)
    if duration:
      time.sleep(duration)  #The motor will run for exactly that many
      self._dc_motor_stop()
      
  def stopper(self, servo_no):
    if servo_no == 1:
        servo_pwm = self.servo1_pwm
        servo_pwm.ChangeDutyCycle(7.5)
        time.sleep(0.3)
        servo_pwm.ChangeDutyCycle(12.5)
        time.sleep(0.3)
    else:
        servo_pwm = self.servo2_pwm
        servo_pwm.ChangeDutyCycle(12.5)
        time.sleep(0.3)
        servo_pwm.ChangeDutyCycle(7.5)
        time.sleep(0.3)
    
  def launch(self,stopper_no):
    self.stopper(stopper_no)
    time.sleep(0)
    self._dc_motor_rotate(direction=1, speed=100, duration=1.11)
    self._dc_motor_stop()
    time.sleep(0.2)
    self._dc_motor_rotate(direction=1, speed=100, duration=1.01)
  
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


# Example Usage
if __name__ == "__main__":       #With this line, the motor control code won't run on import.
  rclpy.init()
  controller = MotorController()
try:
  controller.fn()        
except KeyboardInterrupt:
  print("Program Interrupted")
  controller.destroy_node()
  rclpy.shutdown()
finally:
  controller.cleanup()
