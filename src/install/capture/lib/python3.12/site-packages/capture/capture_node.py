import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')
        self.subscription = self.create_subscription(
            String,
            'data_processing_msg',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'data_processing_msg', 10)
        self.capture_keyword = "capture"  # Trigger word

    def listener_callback(self, msg: String):
        self.get_logger().info(f"Received: {msg.data}")
        if msg.data.strip().lower() == self.capture_keyword:
            self.get_logger().info("Trigger word detected. Running capture script...")
            file_path = self.run_capture_script()
            if file_path:
                response_msg = String()
                response_msg.data = file_path
                self.publisher.publish(response_msg)
                self.get_logger().info(f"Published file location: {file_path}")

    def run_capture_script(self):
        try:
            result = subprocess.run(
                ['python3', '/absolute/path/to/run_capture.py'],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            output_path = result.stdout.strip()
            self.get_logger().info(f"Capture script output: {output_path}")
            return output_path
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error running capture script: {e.stderr}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()