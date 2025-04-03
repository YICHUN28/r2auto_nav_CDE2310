import smbus2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MLX90614(Node):
    def __init__(self):
        super().__init__("tempsensor")
        
        # Temperature Sensor (MLX90614)
        self.temp_sensor_address = 0x5A
        self.bus = smbus2.SMBus(1)  # I2C bus 1
        self.threshold = 40

        self.temppub = self.create_publisher(Float32, '/temp_pub',10)
        self.object_temp = Float32()
        self.object_temp.data = 0.0
    
    def read_temp(self):
        def read_temp(register):
            data = self.bus.read_word_data(self.temp_sensor_address, register)
            temp = (data * 0.02) - 273.15  # Convert to Celsius
            return round(temp, 2)

        object_temp = read_temp(0x07)  # Object temperature register
        ambient_temp = read_temp(0x06)  # Ambient temperature register
        return object_temp, ambient_temp  # Returns as a tuple

if __name__ == "__main__":
    try: 
        reader = MLX90614()
        while True:
            self.object_temp.data, ambient_temp = self.read_temp()
            print(f"Object Temp: {self.object_temp.data}°C | Ambient Temp: {ambient_temp}°C")
            self.temppub.publish(self.object_temp)
            if object_temp > threshold:
                print("Object temperature too high! Launching...")
                return "L" 
            time.sleep(0.3)  # Wait 1 second before checking again

    except KeyboardInterrupt:
        print("Temperature reading stopped.")

  
