from picamera import PiCamera
from time import sleep

class CameraModule:
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (1920, 1080)  # Set resolution (adjust as needed)
        self.camera.rotation = 0  # Rotate if needed (0, 90, 180, 270)

    def capture_image(self, filename="image.jpg"):
        """Captures an image and saves it with the given filename."""
        self.camera.start_preview()
        sleep(2)  # Allow camera to adjust exposure
        self.camera.capture(filename)
        self.camera.stop_preview()
        print(f"Image saved as {filename}")

    def close_camera(self):
        """Closes the camera properly."""
        self.camera.close()

# Example usage
if __name__ == "__main__":
    cam = CameraModule()
    try:
        cam.capture_image("test_image.jpg")  # Capture and save image
    except KeyboardInterrupt:
        print("Process interrupted")
    finally:
        cam.close_camera()
