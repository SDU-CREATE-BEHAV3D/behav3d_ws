#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator print_dots_sequence --ros-args -p yaml_path:=/home/lab/behav3d_ws/yaml/lamine_01.yaml
"""

from __future__ import annotations

import sys
import threading
import time

import rclpy
from rclpy.node import Node

from .src.print_dots_session import PrintDotsSession


class PrintDotsSequenceNode(Node):
    def __init__(self):
        super().__init__("print_dots_sequence")

        self.declare_parameter("yaml_path", "yaml/lamine_01.yaml")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("accel_scale", 0.05)
        self.declare_parameter("pre_dot_vel_scale", 0.10)
        self.declare_parameter("dot_vel_scale", 0.02)
        self.declare_parameter("approach_z_offset_m", 0.40)
        self.declare_parameter("dot_z_offset_m", 0.04)
        self.declare_parameter("dot_steps", 5000)
        self.declare_parameter("dot_speed", 1200)
        self.declare_parameter("dwell_s", 0.4)
        self.declare_parameter("timeout_s", 0.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("axis_length", 0.01)
        self.declare_parameter("axis_radius", 0.002)
        self.declare_parameter("clear_markers_before", True)
        self.declare_parameter("enable_pause_hotkey", True)

        self.session = PrintDotsSession(self)
        self._paused = threading.Event()
        self._pause_hotkey_enabled = bool(self.get_parameter("enable_pause_hotkey").value)
        self._keyboard_thread = None
        if self._pause_hotkey_enabled and sys.stdin and sys.stdin.isatty():
            self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self._keyboard_thread.start()
        elif self._pause_hotkey_enabled:
            self.get_logger().warn(
                "[print_dots_sequence] No TTY detected; pause hotkey disabled."
            )

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        yaml_path = str(self.get_parameter("yaml_path").value).strip()
        frame_id = str(self.get_parameter("frame_id").value).strip() or "world"
        accel_scale = float(self.get_parameter("accel_scale").value)
        pre_dot_vel_scale = float(self.get_parameter("pre_dot_vel_scale").value)
        dot_vel_scale = float(self.get_parameter("dot_vel_scale").value)
        approach_z_offset_m = float(self.get_parameter("approach_z_offset_m").value)
        dot_z_offset_m = float(self.get_parameter("dot_z_offset_m").value)
        dot_steps = int(self.get_parameter("dot_steps").value)
        dot_speed = int(self.get_parameter("dot_speed").value)
        dwell_s = float(self.get_parameter("dwell_s").value)
        timeout_s = None
        publish_markers = bool(self.get_parameter("publish_markers").value)
        axis_length = float(self.get_parameter("axis_length").value)
        axis_radius = float(self.get_parameter("axis_radius").value)
        clear_markers_before = bool(self.get_parameter("clear_markers_before").value)

        if not yaml_path:
            log.error("Parameter 'yaml_path' is empty. Set it with --ros-args -p yaml_path:=<path>.")
            rclpy.shutdown()
            return

        log.info(
            "Starting print dots sequence: "
            f"path='{yaml_path}', frame='{frame_id}', steps={dot_steps}, speed={dot_speed}, "
            f"pre_dot_v={pre_dot_vel_scale:.3f}, dot_v={dot_vel_scale:.3f}, accel={accel_scale:.3f}"
        )
        if self._pause_hotkey_enabled and sys.stdin and sys.stdin.isatty():
            log.info(
                "[print_dots_sequence] Hotkey enabled: type 'p' + ENTER to pause/resume."
            )

        try:
            self.session.run_sync(self.session.motion.home(enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setSpd(float(dot_vel_scale), enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setEef("extruder_tcp", enqueue=False), timeout_s=timeout_s)

            res = self.session.run_print_dots(
                yaml_path=yaml_path,
                frame_id=frame_id,
                dot_steps=dot_steps,
                dot_speed=dot_speed,
                approach_z_offset_m=approach_z_offset_m,
                dot_z_offset_m=dot_z_offset_m,
                pre_dot_vel_scale=pre_dot_vel_scale,
                dot_vel_scale=dot_vel_scale,
                dwell_s=dwell_s,
                timeout_s=timeout_s,
                pause_gate=self._wait_if_paused,
                publish_markers=publish_markers,
                axis_length=axis_length,
                axis_radius=axis_radius,
                clear_markers_before=clear_markers_before,
            )
            if not res.get("ok", False):
                log.error(
                    f"[print_dots_sequence] Dot print failed at stage={res.get('stage')}: "
                    f"{res.get('error', 'unknown')}"
                )
            else:
                log.info(
                    "[print_dots_sequence] Dot print completed: "
                    f"printed={res.get('printed')} / targets={res.get('targets')}"
                )

            self.session.run_sync(self.session.motion.home(enqueue=False), timeout_s=timeout_s)
        except Exception as exc:
            log.error(f"[print_dots_sequence] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    def _wait_if_paused(self) -> None:
        while self._paused.is_set() and rclpy.ok():
            time.sleep(0.1)

    def _keyboard_loop(self) -> None:
        log = self.get_logger()
        while rclpy.ok():
            try:
                text = input().strip().lower()
            except EOFError:
                return
            except Exception as exc:
                log.warn(f"[print_dots_sequence] Keyboard thread stopped: {exc}")
                return

            if text != "p":
                continue

            if self._paused.is_set():
                self._paused.clear()
                log.warn("[print_dots_sequence] Resumed by user (p).")
            else:
                self._paused.set()
                log.warn("[print_dots_sequence] Paused by user (p).")


def main(args=None):
    rclpy.init(args=args)
    node = PrintDotsSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
