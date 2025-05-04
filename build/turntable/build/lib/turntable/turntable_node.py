import rclpy
import time
from rclpy.node import Node
from std_srvs.srv import Trigger
from turntable import turntable_api

class TurntableNode(Node):
    def __init__(self):
        super().__init__('turntable_node')
        self.srv = self.create_service(Trigger, 'rotate_turntable_once', self.handle_rotation)
        self.get_logger().info('Turntable service ready.')

    def handle_rotation(self, request, response):
        try:
            turntable_api.set_speed(60)
            turntable_api.start_rotation()

            while turntable_api.is_done().lower() != "yes":
                self.get_logger().info("Waiting for rotation to finish...")
                time.sleep(0.5)

            response.success = True
            response.message = "Rotation complete."
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurntableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()