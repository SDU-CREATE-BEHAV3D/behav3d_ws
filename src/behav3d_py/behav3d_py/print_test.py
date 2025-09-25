#!/usr/bin/env python3
# run_print_keys.py (tty-safe)
# ENTER -> cancel
# 'u'   -> speed +100
# 'd'   -> speed -100 (reject if < 0)
# Ctrl+C -> cancel goal first, then shutdown

import sys
import os
import time
import threading
import atexit
import signal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from behav3d_interfaces.action import PrintTime
from behav3d_interfaces.srv import UpdatePrintConfig

GOAL_DURATION_S = 60
GOAL_SPEED      = 120
USE_PREV_SPEED  = False
SPEED_STEP      = 100

# -------- Terminal raw mode helper (POSIX) --------
class KeyReader:
    """Put TTY into cbreak mode once and restore it reliably on exit."""
    def __init__(self):
        self._posix = os.name == "posix"
        self._fd = None
        self._old = None
        self._stop = False
        self._thread = None
        self._cb = None

    def start(self, on_char):
        self._cb = on_char
        if self._posix:
            import termios, tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)  # nicer output than raw
            atexit.register(self._restore)
            signal.signal(signal.SIGINT, self._sigint_restore)
            signal.signal(signal.SIGTERM, self._sigint_restore)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        self._restore()

    def _sigint_restore(self, *args):
        self._restore()
        raise KeyboardInterrupt

    def _restore(self):
        if self._posix and self._old is not None and self._fd is not None:
            import termios
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            except Exception:
                pass
            self._old = None

    def _loop(self):
        if self._posix:
            while not self._stop:
                try:
                    ch = os.read(self._fd, 1)
                    if not ch:
                        continue
                    self._cb(ch.decode(errors="ignore"))
                except Exception:
                    break
        else:
            import msvcrt
            while not self._stop:
                if msvcrt.kbhit():
                    ch = msvcrt.getch()
                    try:
                        self._cb(ch.decode(errors="ignore"))
                    except Exception:
                        pass

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("run_print_keys")

        # Action client + goal handle
        self.action_client = ActionClient(self, PrintTime, "print")
        self.gh = None
        self._result_future = None

        # Service client
        self.cfg_cli = self.create_client(UpdatePrintConfig, "update_print_config")
        self.get_logger().info("Waiting for update_print_config service...")
        self.cfg_cli.wait_for_service()
        self.get_logger().info("Service available.")

        self.current_speed = int(GOAL_SPEED)

        # Send goal
        self._send_goal(GOAL_DURATION_S, GOAL_SPEED, USE_PREV_SPEED)

        self.get_logger().info(
            "Controls:\n"
            "  'u' -> speed +100\n"
            "  'd' -> speed -100 (reject if < 0)\n"
            "  ENTER -> cancel\n"
            "  Ctrl+C -> cancel then shutdown\n"
        )

        # Start key reader
        self._keys = KeyReader()
        self._keys.start(self._on_key)

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
            return None
        fut = self.gh.cancel_goal_async()
        return fut  # caller can wait on it

    def _cancel_goal_and_wait(self, timeout_s: float = 1.0):
        """Request cancel and give the server a short window to respond and finish."""
        if not self.gh:
            return
        cfut = self._cancel_goal()
        t0 = time.monotonic()
        # Wait for cancel response
        if cfut is not None:
            while not cfut.done() and (time.monotonic() - t0) < timeout_s:
                rclpy.spin_once(self, timeout_sec=0.05)
            if cfut.done():
                res = cfut.result()
                if res:
                    self.get_logger().info(f"Cancel return_code={res.return_code} (2=ACCEPTED)")
        # Then wait briefly for the result future to complete
        if self._result_future is not None:
            t1 = time.monotonic()
            while not self._result_future.done() and (time.monotonic() - t1) < timeout_s:
                rclpy.spin_once(self, timeout_sec=0.05)

    # ---- Service helper ----
    def _set_speed(self, speed_val: int):
        req = UpdatePrintConfig.Request()
        req.set_speed = True
        req.speed = int(speed_val)
        req.set_extrude = False
        req.extrude_on = False

        fut = self.cfg_cli.call_async(req)
        # short, cooperative wait so we don't block shutdown forever
        t0 = time.monotonic()
        while not fut.done() and (time.monotonic() - t0) < 1.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        res = fut.result() if fut.done() else None
        if res and res.success:
            self.get_logger().info(f"Speed updated to {speed_val}")
        else:
            msg = (res.message if res else "no response (timeout)")
            self.get_logger().error(f"Failed to update speed to {speed_val}: {msg}")

    # ---- Key handling ----
    def _on_key(self, ch: str):
        if ch in ("\r", "\n"):  # ENTER
            self.get_logger().warn("ENTER pressed -> requesting cancel…")
            self._cancel_goal_and_wait(timeout_s=1.0)
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
        pass  # keep quiet; uncomment to log feedback

    def _on_result(self, future):
        try:
            wrap = future.result()
        except Exception as e:
            self.get_logger().error(f"Error waiting result: {e}")
            self._shutdown()
            return
        result = wrap.result
        status = wrap.status
        self.get_logger().info(
            f"Result: status={status} success={result.success} "
            f"elapsed_ms={result.elapsed_ms} reason='{result.reason}'"
        )
        self._shutdown()

    def _shutdown(self):
        # stop key reader and restore tty BEFORE shutting ROS down
        try:
            if hasattr(self, "_keys") and self._keys:
                self._keys.stop()
        except Exception:
            pass
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
        # On Ctrl+C: cancel the active goal first, give it a short grace period.
        node.get_logger().info("Interrupted by user -> canceling goal then shutting down…")
        try:
            node._cancel_goal_and_wait(timeout_s=1.0)
        except Exception:
            pass
        node._shutdown()

if __name__ == "__main__":
    main()
