# robotic_arm_node.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from robotic_arm_api import run_sequence

class RoboticArmNode(Node):
    def __init__(self):
        super().__init__('robotic_arm_node')
        self.srv = self.create_service(Trigger, 'run_arm_sequence', self.run_sequence_callback)
        self.get_logger().info('Robotic Arm Node is ready.')

    def run_sequence_callback(self, request, response):
        self.get_logger().info('Received request to run arm sequence...')
        try:
            run_sequence()
            response.success = True
            response.message = 'Sequence executed successfully.'
        except Exception as e:
            response.success = False
            response.message = f'Sequence failed: {e}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RoboticArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()