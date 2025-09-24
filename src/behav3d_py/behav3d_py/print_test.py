#!/usr/bin/env python3
# run_print_keys.py
# Starts a PrintTime goal, press:
#   ENTER -> cancel the action
#   'u'   -> increase speed by +100 via /update_print_config
#
# Notes:
# - Code comments in English only.
# - Works on Linux/macOS (tty/termios) and Windows (msvcrt) for key input.

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from behav3d_interfaces.action import PrintTime
from behav3d_interfaces.srv import UpdatePrintConfig

# ---------------- Single-char input helpers ----------------
def _getch_blocking():
    """Return one character from stdin (blocking), cross-platform."""
    try:
        # POSIX (Linux/macOS)
        import tty, termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch
    except Exception:
        # Windows
        import msvcrt
        ch = msvcrt.getch()
        try:
            return ch.decode(errors="ignore")
        except Exception:
            return "\n"

# ---------------- Config ----------------
GOAL_DURATION_S = 60
GOAL_SPEED      = 120
USE_PREV_SPEED  = False
SPEED_STEP      = 100

# ---------------- Client helper ----------------
class Print_test:
    def __init__(self, node: Node):
        self.node = node
        self.client = ActionClient(node, PrintTime, "print")
        self.gh = None  # ClientGoalHandle

    def start(self, sec, speed, use_prev=False, feedback_cb=None) -> bool:
        self.client.wait_for_server()
        goal = PrintTime.Goal()
        goal.duration.sec = int(sec)
        goal.duration.nanosec = 0
        goal.speed = int(speed)
        goal.use_previous_speed = bool(use_prev)

        # Wire feedback callback here (correct rclpy pattern)
        fut = self.client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self.node, fut)
        self.gh = fut.result()
        if not self.gh or not self.gh.accepted:
            self.node.get_logger().error("Goal was rejected by server.")
            return False

        self.node.get_logger().info("Goal accepted")
        return True

    def cancel(self):
        if not self.gh:
            self.node.get_logger().warning("No goal to cancel")
            return
        fut = self.gh.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, fut)
        res = fut.result()
        if res:
            # return_code: 0 UNKNOWN, 1 REJECTED, 2 ACCEPTED
            self.node.get_logger().info(f"Cancel return_code={res.return_code} (2=ACCEPTED)")
        else:
            self.node.get_logger().warning("Cancel returned no response")

# ---------------- Orchestrator node ----------------
class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("run_print_keys")
        self.pt = Print_test(self)

        # Service client for /update_print_config
        self.cfg_cli = self.create_client(UpdatePrintConfig, "update_print_config")
        self.get_logger().info("Waiting for update_print_config service...")
        self.cfg_cli.wait_for_service()
        self.get_logger().info("Service available.")

        # Track our notion of current speed (start from requested goal)
        self.current_speed = int(GOAL_SPEED)

        # Start the action goal with feedback callback
        if not self.pt.start(GOAL_DURATION_S, GOAL_SPEED, USE_PREV_SPEED, feedback_cb=self._on_feedback):
            self.get_logger().error("Failed to start goal. Exiting.")
            rclpy.shutdown()
            return

        # Watch for final result asynchronously
        self._result_future = self.pt.gh.get_result_async()
        self._result_future.add_done_callback(self._on_result)

        # Controls instructions
        self.get_logger().info(
            "Controls:\n"
            "  Press 'u' to increase speed by +100 via /update_print_config\n"
            "  Press ENTER to cancel the print\n"
        )

        # Start key listener in a daemon thread
        threading.Thread(target=self._keys_loop, daemon=True).start()

    # ------------- Key loop -------------
    def _keys_loop(self):
        while rclpy.ok():
            ch = _getch_blocking()
            if ch in ("\r", "\n"):  # ENTER
                self.get_logger().warn("ENTER pressed -> requesting cancel…")
                self.pt.cancel()
            elif ch.lower() == "u":
                self.current_speed += SPEED_STEP
                self._set_speed(self.current_speed)
            elif ch.lower() == "d":
                new_speed = self.current_speed - SPEED_STEP
                if new_speed < 0:
                    self.get_logger().warn("Rejecting speed update: would go below 0")
                else:
                    self.current_speed = new_speed
                    self._set_speed(self.current_speed)

    # ------------- Service call -------------
    def _set_speed(self, speed_val: int):
        req = UpdatePrintConfig.Request()
        req.set_speed = True
        req.speed = int(speed_val)
        req.set_extrude = False
        req.extrude_on = False

        fut = self.cfg_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.success:
            self.get_logger().info(f"Speed updated to {speed_val}")
        else:
            msg = res.message if res else "no response"
            self.get_logger().error(f"Failed to update speed to {speed_val}: {msg}")

    # ------------- Callbacks -------------
    def _on_feedback(self, fb_msg):
        fb = fb_msg.feedback
        # Keep logs concise; uncomment for detailed streaming feedback:
        # self.get_logger().info(f"Feedback: elapsed_ms={fb.elapsed_ms} progress={fb.progress:.3f}")

    def _on_result(self, future):
        try:
            result_wrap = future.result()
        except Exception as e:
            self.get_logger().error(f"Error waiting result: {e}")
            self._shutdown()
            return

        result = result_wrap.result
        status = result_wrap.status  # 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        self.get_logger().info(
            f"Result: status={status} success={result.success} "
            f"elapsed_ms={result.elapsed_ms} reason='{result.reason}'"
        )
        self._shutdown()

    def _shutdown(self):
        try:
            self.destroy_node()
        finally:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = OrchestratorNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
