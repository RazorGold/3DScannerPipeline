import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pymeshlab
import os

class MeshingNode(Node):
    def __init__(self):
        super().__init__('meshing_node')
        self.subscription = self.create_subscription(
            String,
            'ply_file_input',  # Subscribe to a file name
            self.listener_callback,
            10)
        self.get_logger().info("Meshing Node Initialized, waiting for .ply input...")

    def listener_callback(self, msg):
        input_file = msg.data
        if not os.path.exists(input_file):
            self.get_logger().error(f"File {input_file} does not exist.")
            return

        script_dir = os.path.dirname(os.path.realpath(__file__))
        mlx_script = os.path.join(script_dir, 'ball.mlx')
        output_file = os.path.join(script_dir, 'ball_output.ply')

        self.get_logger().info(f"Loading point cloud: {input_file}")
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(input_file)

        self.get_logger().info(f"Applying MLX script: {mlx_script}")
        ms.load_filter_script(mlx_script)
        ms.apply_filter_script()
        ms.set_current_mesh(ms.number_meshes() - 1)
        ms.save_current_mesh(output_file)

        self.get_logger().info(f"Mesh reconstruction complete. Output saved to: {output_file}")

def main(args=None):
    rclpy.init(args=args)
    node = MeshingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
