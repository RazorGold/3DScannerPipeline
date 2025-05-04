import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from enum import Enum
import threading

class ScannerState(Enum):
    IDLE = 'IDLE'
    CALIBRATION = 'CALIBRATION'
    SCANNING = 'SCANNING'
    PROCESSING = 'PROCESSING'
    EVALUATION = 'EVALUATION'
    ERROR = 'ERROR'

class CentralNode(Node):
    def __init__(self):
        super().__init__('central_node')
        self.state = ScannerState.IDLE
        self.paused = False

        # Publishers and Subscribers
        self.command_pub = self.create_publisher(String, 'central_control/command', 10)
        self.capture_pub = self.create_publisher(String, 'capture/command', 10)
        self.ui_sub = self.create_subscription(String, 'ui/start', self.ui_callback, 10)

        # Service Clients
        self.turntable_client = self.create_client(Trigger, 'rotate_turntable_once')
        self.robot_arm_client = self.create_client(Trigger, 'run_arm_sequence')

        # Wait for services
        self.wait_for_services()

        # Periodic timer
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Start input listener thread
        threading.Thread(target=self.input_thread, daemon=True).start()

        self.get_logger().info("Central Control Node initialized.")

    def wait_for_services(self):
        self.get_logger().info('Waiting for required services...')
        while not self.turntable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turntable service...')
        while not self.robot_arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robot arm service...')
        self.get_logger().info('All services available.')

def input_thread(self):
    state_order = [
        ScannerState.IDLE,
        ScannerState.CALIBRATION,
        ScannerState.SCANNING,
        ScannerState.PROCESSING,
        ScannerState.EVALUATION,
        ScannerState.IDLE  # After evaluation, loop back to idle
    ]

    while rclpy.ok():
        try:
            cmd = input(">> ").strip().lower()
            if cmd == 'pause':
                self.paused = True
                self.get_logger().info("System paused.")
            elif cmd == 'resume':
                self.paused = False
                self.get_logger().info("System resumed.")
            elif cmd == 'skip':
                try:
                    current_index = state_order.index(self.state)
                    next_state = state_order[(current_index + 1) % len(state_order)]
                    self.get_logger().info("Skip command received. Forcing transition: {self.state.value} → {next_state.value}")
                    self.state = next_state
                except ValueError:
                    self.get_logger().warn("Current state not in skip sequence. No action taken.")
        except EOFError:
            break

    def ui_callback(self, msg: String):
        if self.state == ScannerState.IDLE and msg.data.strip().lower() == 'start':
            self.get_logger().info("Start command received from UI. Transitioning to CALIBRATION.")
            self.state = ScannerState.CALIBRATION

    def timer_callback(self):
        if self.paused:
            return
        handler = getattr(self, f'handle_{self.state.name.lower()}', None)
        if handler:
            handler()

        status_msg = String()
        status_msg.data = f"Current State: {self.state.value}"
        self.command_pub.publish(status_msg)

    def handle_idle(self):
        self.get_logger().info("System is idle. Waiting for UI input...")

    def handle_calibration(self):
        self.get_logger().info("Checking if all nodes are responsive...")
        # Could extend to ping nodes via dedicated topics/services
        self.state = ScannerState.SCANNING
        self.get_logger().info("Calibration checks passed. Transitioning to SCANNING.")

    def handle_scanning(self):
        self.get_logger().info("Initiating scanning sequence...")
        
        arm_future = self.robot_arm_client.call_async(Trigger.Request())
        table_future = self.turntable_client.call_async(Trigger.Request())

        def on_both_complete():
            if arm_future.result().success and table_future.result().success:
                self.get_logger().info("Scanning complete. Transitioning to PROCESSING.")
                self.state = ScannerState.PROCESSING
            else:
                self.get_logger().error("Error during scanning. Entering ERROR state.")
                self.state = ScannerState.ERROR

        def check_futures():
            rclpy.spin_until_future_complete(self, arm_future)
            rclpy.spin_until_future_complete(self, table_future)
            on_both_complete()

        threading.Thread(target=check_futures, daemon=True).start()
        self.state = ScannerState.WAITING

    def handle_waiting(self):
        # Do nothing while waiting for async service completion
        pass

    def handle_processing(self):
        self.get_logger().info("Processing data...")
        # Stub: add detailed processing logic later
        self.state = ScannerState.EVALUATION

    def handle_evaluation(self):
        self.get_logger().info("Evaluating scan...")
        # Stub: implement evaluation logic later
        evaluation_passed = True
        self.state = ScannerState.IDLE if evaluation_passed else ScannerState.SCANNING

    def handle_error(self):
        self.get_logger().warn("System in ERROR state. Attempting recovery...")
        self.state = ScannerState.IDLE
        self.get_logger().info("Recovered to IDLE state.")

def main(args=None):
    rclpy.init(args=args)
    node = CentralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
