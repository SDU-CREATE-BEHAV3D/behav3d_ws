#!/usr/bin/env python3
# run_print_keys.py
# ENTER -> cancel the action
# 'u'   -> increase speed by +100 via /update_print_config
# 'd'   -> decrease speed by -100 (reject if below 0)

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from behav3d_interfaces.action import PrintTime
from behav3d_interfaces.srv import UpdatePrintConfig

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

GOAL_DURATION_S = 60
GOAL_SPEED      = 120
USE_PREV_SPEED  = False
SPEED_STEP      = 100

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("run_print_keys")

        # Action client + goal handle
        self.action_client = ActionClient(self, PrintTime, "print")
        self.gh = None

        # Config service client
        self.cfg_cli = self.create_client(UpdatePrintConfig, "update_print_config")
        self.get_logger().info("Waiting for update_print_config service...")
        self.cfg_cli.wait_for_service()
        self.get_logger().info("Service available.")

        # Track current speed (start with requested goal speed)
        self.current_speed = int(GOAL_SPEED)

        # Send goal with feedback
        self._send_goal(GOAL_DURATION_S, GOAL_SPEED, USE_PREV_SPEED)

        # Watch final result
        self._result_future = None  # set once goal accepted

        # Controls
        self.get_logger().info(
            "Controls:\n"
            "  'u' -> speed +100\n"
            "  'd' -> speed -100 (reject if < 0)\n"
            "  ENTER -> cancel\n"
        )

        # Start key listener
        threading.Thread(target=self._keys_loop, daemon=True).start()

    # ---- Action helpers ----
    def _send_goal(self, sec: int, speed: int, use_prev: bool):
        self.get_logger().info("Waiting for /print action server…")
        self.action_client.wait_for_server()

        goal = PrintTime.Goal()
        goal.duration.sec = int(sec)
        goal.duration.nanosec = 0
        goal.speed = int(speed)
        goal.use_previous_speed = bool(use_prev)

        fut = self.action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self.gh = future.result()
        if not self.gh or not self.gh.accepted:
            self.get_logger().error("Goal was rejected by server.")
            self._shutdown()
            return
        self.get_logger().info("Goal accepted")
        self._result_future = self.gh.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _cancel_goal(self):
        if not self.gh:
            self.get_logger().warning("No goal to cancel.")
            return
        fut = self.gh.cancel_goal_async()
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res:
            # return_code: 0 UNKNOWN, 1 REJECTED, 2 ACCEPTED
            self.get_logger().info(f"Cancel return_code={res.return_code} (2=ACCEPTED)")
        else:
            self.get_logger().warning("Cancel returned no response")

    # ---- Service helper ----
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

    # ---- Key handling ----
    def _keys_loop(self):
        while rclpy.ok():
            ch = _getch_blocking()
            if ch in ("\r", "\n"):  # ENTER
                self.get_logger().warn("ENTER pressed -> requesting cancel…")
                self._cancel_goal()
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

    # ---- Callbacks ----
    def _on_feedback(self, fb_msg):
        fb = fb_msg.feedback
        # Uncomment to see streaming feedback:
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user, shutting down…")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
if __name__ == "__main__":
    main()
