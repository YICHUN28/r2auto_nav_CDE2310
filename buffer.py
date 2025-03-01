import rclpy
from rclpy.node import Node
import tf2_ros
from rclpy.duration import Duration

class TFDebugNode(Node):
    def __init__(self):
        super().__init__('tf_debugger')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0, self.check_transform)

    def check_transform(self):
        frames = self.tf_buffer.all_frames_as_yaml()
        self.get_logger().info(f"Available TF Frames:\n{frames}")

        if self.tf_buffer.can_transform("map", "odom", rclpy.time.Time()):
            self.get_logger().info("✅ Transform map -> odom is available!")
        else:
            self.get_logger().warn("❌ Transform map -> odom NOT available!")

def main():
    rclpy.init()
    node = TFDebugNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()