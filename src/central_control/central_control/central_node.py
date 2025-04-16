import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from enum import Enum

class ScannerState(Enum):
    IDLE = 'IDLE'
    CALIBRATION = 'CALIBRATION'
    SCANNING = 'SCANNING'
    PROCESSING = 'PROCESSING'
    EVALUATION = 'EVALUATION'
    ERROR = 'ERROR'

class Central_Node(Node):
    def __init__(self):
        super().__init__('central_node')
        self.state = ScannerState.IDLE
        self.get_logger().info(f"Central Control Node initialized. Current state: {self.state.value}")

        self.command_pub = self.create_publisher(String, 'central_control/command', 10)
        self.status_sub = self.create_subscription(String, 'nodes/status', self.status_callback, 10)
        self.robotic_arm_pub = self.create_publisher(String, 'robotic_arm/command', 10)
        self.capture_pub = self.create_publisher(String, 'capture/command', 10)

        # Create client for turntable service
        self.turntable_client = self.create_client(Trigger, 'rotate_turntable_once')
        while not self.turntable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Turntable service...')

        self.timer = self.create_timer(5.0, self.timer_callback)

    def status_callback(self, msg: String):
        self.get_logger().info(f"Received status update: {msg.data}")
        if "error" in msg.data.lower():
            self.state = ScannerState.ERROR
            self.get_logger().error("Error reported by a node. Transitioning to ERROR state.")

    def timer_callback(self):
        self.get_logger().info(f"Timer tick: Current state is {self.state.value}")
        
        if self.state == ScannerState.IDLE:
            self.get_logger().info("System is idle. Awaiting start command...")
            self.state = ScannerState.CALIBRATION
            self.get_logger().info("Transitioning from IDLE to CALIBRATION")
            self.start_calibration()

        elif self.state == ScannerState.CALIBRATION:
            self.get_logger().info("Calibration in progress...")
            self.state = ScannerState.SCANNING
            self.get_logger().info("Calibration complete. Transitioning to SCANNING")
            self.start_scanning()

        elif self.state == ScannerState.SCANNING:
            self.get_logger().info("Scanning in progress...")
            self.state = ScannerState.PROCESSING
            self.get_logger().info("Scanning complete. Transitioning to PROCESSING")
            self.start_processing()

        elif self.state == ScannerState.PROCESSING:
            self.get_logger().info("Processing data (Filtering, Alignment, Projection, Meshing)...")
            self.state = ScannerState.EVALUATION
            self.get_logger().info("Processing complete. Transitioning to EVALUATION")
            self.start_evaluation()

        elif self.state == ScannerState.EVALUATION:
            self.get_logger().info("Evaluating scan quality...")
            evaluation_success = True
            if evaluation_success:
                self.get_logger().info("Evaluation passed. Ready for next scan cycle.")
                self.state = ScannerState.IDLE
            else:
                self.get_logger().warn("Evaluation failed. Re-scanning required.")
                self.state = ScannerState.SCANNING

        elif self.state == ScannerState.ERROR:
            self.get_logger().error("System in ERROR state. Attempting recovery...")
            self.recover_from_error()
        
        status_msg = String()
        status_msg.data = f"Current State: {self.state.value}"
        self.command_pub.publish(status_msg)

    def start_calibration(self):
        """
        Initiates calibration.
        Sends calibration command to robotic arm and calls the turntable service.
        """
        self.get_logger().info("Starting calibration process...")

        # Send command to robotic arm
        calibration_command = String()
        calibration_command.data = "start_calibration"
        self.robotic_arm_pub.publish(calibration_command)

        # ✨ MODIFIED: Call turntable service
        req = Trigger.Request()
        future = self.turntable_client.call_async(req)

        def turntable_response_callback(future):
            try:
                result = future.result()
                if result.success:
                    self.get_logger().info(f"Turntable calibration succeeded: {result.message}")
                else:
                    self.get_logger().warn(f"Turntable calibration failed: {result.message}")
                    self.state = ScannerState.ERROR
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                self.state = ScannerState.ERROR

        future.add_done_callback(turntable_response_callback)

    def start_scanning(self):
        self.get_logger().info("Starting scanning process...")
        scan_command = String()
        scan_command.data = "start_scanning"
        self.capture_pub.publish(scan_command)

    def start_processing(self):
        self.get_logger().info("Starting data processing...")
        # Future additions: filtering, alignment, meshing

    def start_evaluation(self):
        self.get_logger().info("Starting evaluation process...")
        # Future additions: evaluation node calls

    def recover_from_error(self):
        self.get_logger().info("Attempting to recover from ERROR state...")
        self.state = ScannerState.IDLE
        self.get_logger().info("Recovery complete. Transitioning to IDLE state.")

def main(args=None):
    rclpy.init(args=args)
    central_control_node = Central_Node()
    
    try:
        rclpy.spin(central_control_node)
    except KeyboardInterrupt:
        central_control_node.get_logger().info("Central Control Node shutting down due to KeyboardInterrupt.")
    finally:
        central_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()