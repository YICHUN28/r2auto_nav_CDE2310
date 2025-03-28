import smbus
import time

class MLX90614:
    def __init__(self):
        # Temperature Sensor (MLX90614)
        self.temp_sensor_address = 0x5A
        self.bus = smbus.SMBus(1)  # I2C bus 1
        self.threshold = 40
    
    def read_temp(self):
        def read_temp(register):
            data = self.bus.read_word_data(self.temp_sensor_address, register)
            temp = (data * 0.02) - 273.15  # Convert to Celsius
            return round(temp, 2)

        object_temp = read_temp(0x07)  # Object temperature register
        ambient_temp = read_temp(0x06)  # Ambient temperature register
        return object_temp, ambient_temp  # Returns as a tuple

    def monitor_temp(self,threshold=None):
        if threshold is None:
            threshold = self.threshold  # Use default if not provided

        while True:
            object_temp, ambient_temp = self.read_temp()
            print(f"Object Temp: {object_temp}°C | Ambient Temp: {ambient_temp}°C")

            if object_temp > threshold:
                print("Object temperature too high! Launching...")
                return "L"  # Exit and signal that launch should happen
            
            time.sleep(1)  # Wait 1 second before checking again


if __name__ == "__main__":
    try: 
        reader = MLX90614()
        while True:  
            if reader.monitor_temp(40) == "L":  # Only launches if temp > 40
                print("more than")
            else:
                print("less than")
    except KeyboardInterrupt:
        print("Temperature reading stopped.")

  
