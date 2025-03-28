import smbus
import time

class MLX90614:
    def __init__(self):
        # Temperature Sensor (MLX90614)
        self.temp_sensor_address = 0x5A
        self.bus = smbus.SMBus(1)  # I2C bus 1
    
    def read_temp(self):
        """Reads and returns (object temperature, ambient temperature) from MLX90614"""
        def read_temp(register):
            data = self.bus.read_word_data(self.temp_sensor_address, register)
            temp = (data * 0.02) - 273.15  # Convert to Celsius
            return round(temp, 2)

        object_temp = read_temp(0x07)  # Object temperature register
        ambient_temp = read_temp(0x06)  # Ambient temperature register
        return object_temp, ambient_temp  # Returns as a tuple

    def monitor_temp(self):
        """ Continuously check temperature and launch if above 40°C """
        while True:
            object_temp, ambient_temp = self.read_temp()
            print(f"Object Temp: {object_temp}°C | Ambient Temp: {ambient_temp}°C")

            if object_temp > 40:
              print("Object temperature too high! Launching...")
              return True
            else:
              return False

            time.sleep(1)  # Check temperature every second


if __name__ == "__main__":
    try: 
      Reader = MLX90614()
      Reader.monitor_temp()
    except KeyboardInterrupt:
        print("Temperature reading stopped.")
  
