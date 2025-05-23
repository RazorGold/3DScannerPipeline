import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from enum import Enum
import threading
import time

class ScannerState(Enum):
    IDLE = 'IDLE'
    CALIBRATION = 'CALIBRATION'
    SCANNING = 'SCANNING'
    PROCESSING = 'PROCESSING'
    EVALUATION = 'EVALUATION'
    ERROR = 'ERROR'
    WAITING = 'WAITING'

class CentralNode(Node):
    def __init__(self):
        super().__init__('central_node')
        self.state = ScannerState.IDLE
        self.paused = False
        self._skip_wait = False
        self._waiting_for_faceup = False
        self._post_faceup_loop_started = False

        # Publishers and Subscribers
        self.command_pub = self.create_publisher(String, 'central_control/command', 10)
        self.capture_pub = self.create_publisher(String, 'capture/command', 10)
        self.ui_sub = self.create_subscription(String, 'ui/command', self.ui_callback, 10)
        self.control_sub = self.create_subscription(String, '/central_control/input', self.control_callback, 10)
        self.arm_pub = self.create_publisher(String, '/arm_command', 10)

        # Service Clients
        self.turntable_client = self.create_client(Trigger, 'rotate_turntable_once')

        self.wait_for_services()
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("Central Control Node initialized.")

    def wait_for_services(self):
        self.get_logger().info('Waiting for required services (type "skip" to override)...')
        while rclpy.ok():
            if self._skip_wait:
                self.get_logger().warn("Skipping service wait due to user command.")
                break
            if self.turntable_client.service_is_ready():
                self.arm_pub.publish(String(data='init'))
                self.get_logger().info("All required services are now available.")
                break
            rclpy.spin_once(self, timeout_sec=0.5)

    def control_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Received control command: {cmd}")
        state_order = [
            ScannerState.IDLE,
            ScannerState.CALIBRATION,
            ScannerState.SCANNING,
            ScannerState.PROCESSING,
            ScannerState.EVALUATION,
            ScannerState.IDLE
        ]

        if cmd == 'pause':
            self.paused = True
            self.get_logger().info("System paused (via topic).")
        elif cmd == 'resume':
            self.paused = False
            self.get_logger().info("System resumed (via topic).")
        elif cmd == 'skip':
            self._skip_wait = True
            try:
                idx = state_order.index(self.state)
                next_state = state_order[(idx + 1) % len(state_order)]
                self.get_logger().info(f"Skip command received. Forcing transition: {self.state.value} → {next_state.value}")
                self.state = next_state
            except ValueError:
                self.get_logger().warn("Current state not in state_order list.")
        else:
            self.get_logger().warn(f"Unknown command received: {cmd}")

    def ui_callback(self, msg: String):
        msg_data = msg.data.strip().lower()
        if self.state == ScannerState.IDLE and msg_data == 'start':
            self.get_logger().info("Start command received from UI. Transitioning to CALIBRATION.")
            self.state = ScannerState.CALIBRATION
        elif self._waiting_for_faceup and msg_data.startswith("rotate_faceup:"):
            self.arm_pub.publish(String(data=msg.data))
            self.get_logger().info(f"Sent rotate_faceup command with measurements: {msg.data}")
            self._waiting_for_faceup = False
            self._post_faceup_loop_started = True
            self.start_second_scan_loop()
        else:
            self.get_logger().info(f"Ignoring UI message: {msg.data} (Current state: {self.state.value})")

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
        self.state = ScannerState.SCANNING
        self.get_logger().info("Calibration checks passed. Transitioning to SCANNING.")

    def handle_scanning(self):
        def scan_cycle():
            self.arm_pub.publish(String(data='move_to_scanning'))
            self.get_logger().info("Sent move_to_scanning to arm.")
            time.sleep(1.0)

            self.capture_pub.publish(String(data='capture'))
            self.get_logger().info("Initial capture sent.")

            for i in range(24):
                self.get_logger().info(f"[Cycle 1] Step {i+1}/24: Rotating turntable...")
                future = self.turntable_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future)
                if not future.result().success:
                    self.get_logger().error("Turntable rotation failed.")
                    self.state = ScannerState.ERROR
                    return
                self.capture_pub.publish(String(data='capture'))
                time.sleep(0.2)

            self.get_logger().info("Waiting for rotate_faceup measurements...")
            self._waiting_for_faceup = True
            self.state = ScannerState.WAITING

        threading.Thread(target=scan_cycle, daemon=True).start()
        self.state = ScannerState.WAITING

    def start_second_scan_loop(self):
        def second_loop():
            time.sleep(1.0)
            self.capture_pub.publish(String(data='capture'))
            self.get_logger().info("Post-faceup: Initial capture.")

            for i in range(24):
                self.get_logger().info(f"[Cycle 2] Step {i+1}/24: Rotating turntable...")
                future = self.turntable_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, future)
                if not future.result().success:
                    self.get_logger().error("Turntable rotation failed.")
                    self.state = ScannerState.ERROR
                    return
                self.capture_pub.publish(String(data='capture'))
                time.sleep(0.2)

            self.get_logger().info("Scanning complete. Transitioning to PROCESSING.")
            self._post_faceup_loop_started = False
            self.state = ScannerState.PROCESSING

        threading.Thread(target=second_loop, daemon=True).start()
        self.state = ScannerState.WAITING

    def handle_waiting(self):
        pass

    def handle_processing(self):
        self.get_logger().info("Processing data...")
        self.state = ScannerState.EVALUATION

    def handle_evaluation(self):
        self.get_logger().info("Evaluating scan...")
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