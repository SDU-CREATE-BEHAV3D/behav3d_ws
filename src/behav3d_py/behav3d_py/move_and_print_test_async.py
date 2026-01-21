#!/usr/bin/env python3
# For running this node first:
# ros2 launch behav3d_bringup print_move.launch.py robot_ip:=192.168.1.8 use_mock_hardware:=true

import rclpy
from rclpy.node import Node
import behav3d_commands


class MoveAndPrintAsyncTest(Node):
    """Demo: schedule motion + print as a parallel group after a gate input."""

    def __init__(self):
        super().__init__("move_and_print_test_async")
        self.session = behav3d_commands.Session(self)
        self._started = False
        self.create_timer(0.25, self._run_once)

    def _run_once(self):
        if self._started:
            return
        self._started = True

        self.session.motion.home(duration_s=1.0, on_done=self._on_command_done)
        self.session.motion.setSpd(0.2)
        self.session.motion.setEef("femto_color_optical_calib")
        self.session.motion.setLIN()
        self.session.util.input(prompt="Press ENTER to go to target...", on_done=self._on_ready)

    def _on_ready(self, res):
        if not res.get("ok", False):
            self.get_logger().error(f"[input] FAILED: {res.get('error')}")
            return

        # Group 1: goto + print_time (decoupled, dispatched together)
        group1 = [
            self.session.motion.goto(
                x=0.50,
                y=1.0,
                z=0.31,
                eef="extruder_tcp",
                vel_scale=0.1,
                accel_scale=0.1,
                exec=True,
                motion="LIN",
                on_done=self._on_command_done,
                enqueue=False,
            ),
            self.session.extruder.print_time(
                secs=1.5,
                speed=1200,
                use_previous_speed=False,
                on_done=self._on_command_done,
                enqueue=False,
            ),
        ]
        self.session.run_group(group1)

        # Group 2: goto + print_steps
        group2 = [
            self.session.motion.goto(
                x=0.50,
                y=1.0,
                z=0.96,
                eef="extruder_tcp",
                vel_scale=0.1,
                accel_scale=0.1,
                exec=True,
                motion="LIN",
                on_done=self._on_command_done,
                enqueue=False,
            ),
            self.session.extruder.print_steps(
                steps=1800,
                speed=600,
                use_previous_speed=False,
                on_done=self._on_command_done,
                enqueue=False,
            ),
        ]
        self.session.run_group(group2)

        self.session.extruder.print_steps(steps=2000, speed=500, on_done=self._on_command_done)
        self.session.motion.goto(
            x=-0.0500,
            y=1.30,
            z=0.31,
            eef="extruder_tcp",
            vel_scale=0.5,
            accel_scale=0.1,
            exec=True,
            on_done=self._on_command_done,
        )

        self.session.motion.home(duration_s=1.0, on_done=self._on_command_done)
        self.session.util.input(
            key="q",
            prompt="Type 'q' + ENTER to shutdown...",
            on_done=self._on_quit,
        )

    def _on_command_done(self, res):
        if not res.get("ok", False):
            self.get_logger().error(f"[{res['kind']} {res['phase']}] FAILED: {res['error']}")
        else:
            self.get_logger().info(f"[{res['kind']} {res['phase']}] OK {res['metrics']}")

    def _on_quit(self, res):
        if res.get("ok", False):
            self.get_logger().info("Shutdown requested. Stopping ROS...")
            rclpy.shutdown()
        else:
            self.get_logger().warning("Expected 'q'. Ignoring input; not shutting down.")
            self.session.util.input(
                key="q",
                prompt="Type 'q' + ENTER to shutdown...",
                on_done=self._on_quit,
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MoveAndPrintAsyncTest())


if __name__ == "__main__":
    main()
