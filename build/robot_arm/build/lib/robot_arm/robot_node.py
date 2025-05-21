import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .robotic_arm_api import (
    init, moveto_base, move_into_scanning,
    rotate_faceup, rotate_faceside, arm
)

class RoboticArmNode(Node):
    def __init__(self):
        super().__init__('robotic_arm_node')
        self.get_logger().info('Robotic Arm Node has started.')

        # Subscribe to commands
        self.create_subscription(String, 'arm/command', self.command_callback, 10)

        # Command dispatch table
        self.command_map = {
            'init': init,
            'move_to_base': lambda: moveto_base(arm),
            'move_to_scanning': lambda: move_into_scanning(arm),
            'rotate_faceup': rotate_faceup,     # requires 3 args
            'rotate_faceside': rotate_faceside  # requires 3 args
        }

    def command_callback(self, msg: String):
        raw_input = msg.data.strip()
        self.get_logger().info(f"Received command: {raw_input}")

        try:
            # Split command and arguments (if any)
            if ':' in raw_input:
                command, arg_str = raw_input.split(':', 1)
                args = list(map(float, arg_str.split(',')))
            else:
                command = raw_input
                args = []

            handler = self.command_map.get(command)
            if not handler:
                self.get_logger().warn(f"Unknown command: '{command}'")
                return

            # Call the handler with or without arguments
            handler(*args)
            self.get_logger().info(f"Executed command: {command}")

        except Exception as e:
            self.get_logger().error(f"Error while executing '{raw_input}': {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RoboticArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()