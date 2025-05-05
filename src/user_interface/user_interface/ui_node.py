# ui_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from threading import Thread
from flask import Flask, request, jsonify
from flask_cors import CORS

# Initialize Flask app
app = Flask(__name__)
CORS(app)  # Allow cross-origin requests if frontend is served from another port

@app.route('/command', methods=['POST'])
def handle_command():
    data = request.get_json()
    action = data.get('action')
    if not action:
        return jsonify({'error': 'Missing action'}), 400

    msg = String()
    msg.data = action

    if ros_node:
        ros_node.get_logger().info(f"[UI] Button action received: {action}")
        ros_node.publisher.publish(msg)
        return jsonify({'status': 'published'}), 200
    else:
        return jsonify({'error': 'ROS node not initialized'}), 500


class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')
        self.publisher = self.create_publisher(String, 'ui/command', 10)
        self.get_logger().info("UI Node started and Flask server running.")

        global ros_node
        ros_node = self  # Make the ROS publisher accessible to Flask


def run_flask():
    app.run(host='0.0.0.0', port=5000)  # Listen on all interfaces


def main(args=None):
    rclpy.init(args=args)

    node = UINode()
    flask_thread = Thread(target=run_flask, daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()