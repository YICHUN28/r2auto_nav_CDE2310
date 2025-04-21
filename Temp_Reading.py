import smbus2
import time
import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Float32

class Tempsens(Node):
    def __init__(self):
        super().__init__("HeatSensor")
        
        # Temperature Sensor (MLX90614)
        self.MLX90614_ADDR = 0x5A
        self.MLX90614_TA = 0x06
        self.MLX90614_TOBJ1 = 0x07
        self.bus = smbus2.SMBus(1)  # I2C bus 1
        
        self.temppub = self.create_publisher(Float32, '/temp_pub',10)
        self.object_temp = Float32()
        self.object_temp.data = 0.0
        
    def read_temperature(self,register):
        data = self.bus.read_i2c_block_data(self.MLX90614_ADDR, register,2)
        raw_data = (data[1] << 8) + data[0]
        temperature = (raw_data * 0.02) - 273.15
        return temperature
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
        

if __name__ == "__main__":
    rclpy.init()
    tempclass = Tempsens()
    tempclass.fn()
    tempclass.destroy_node()
    rclpy.shutdown()
