#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator print_path_sequence --ros-args -p yaml_path:=/home/lab/behav3d_ws/yaml/lamine_01.yaml
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from .src.yaml_session import YamlSession


class PrintPathSequenceNode(Node):
    def __init__(self):
        super().__init__("print_path_sequence")

        self.declare_parameter("yaml_path", "yaml/lamine_01.yaml")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("accel_scale", 0.05)
        self.declare_parameter("print_vel_scale", 0.007)
        self.declare_parameter("approach_vel_scale", 0.10)
        self.declare_parameter("approach_z_offset_m", 0.40)
        self.declare_parameter("print_speed", 1000)
        self.declare_parameter("timeout_s", 60.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("axis_length", 0.05)
        self.declare_parameter("axis_radius", 0.003)
        self.declare_parameter("clear_markers_before", True)

        self.session = YamlSession(self)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        yaml_path = str(self.get_parameter("yaml_path").value).strip()
        frame_id = str(self.get_parameter("frame_id").value).strip() or "world"
        accel_scale = float(self.get_parameter("accel_scale").value)
        print_vel_scale = float(self.get_parameter("print_vel_scale").value)
        approach_vel_scale = float(self.get_parameter("approach_vel_scale").value)
        approach_z_offset_m = float(self.get_parameter("approach_z_offset_m").value)
        print_speed = int(self.get_parameter("print_speed").value)
        timeout_s = self._param_optional_float("timeout_s")
        publish_markers = bool(self.get_parameter("publish_markers").value)
        axis_length = float(self.get_parameter("axis_length").value)
        axis_radius = float(self.get_parameter("axis_radius").value)
        clear_markers_before = bool(self.get_parameter("clear_markers_before").value)

        if not yaml_path:
            log.error("Parameter 'yaml_path' is empty. Set it with --ros-args -p yaml_path:=<path>.")
            rclpy.shutdown()
            return

        log.info(
            "Starting print path sequence: "
            f"path='{yaml_path}', frame='{frame_id}', print_speed={print_speed}, "
            f"approach_v={approach_vel_scale:.3f}, print_v={print_vel_scale:.3f}, accel={accel_scale:.3f}"
        )

        try:
            self.session.run_sync(self.session.motion.home(enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setSpd(float(print_vel_scale), enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setEef("extruder_tcp", enqueue=False), timeout_s=timeout_s)

            res = self.session.run_yaml_print_path(
                yaml_path=yaml_path,
                frame_id=frame_id,
                print_speed=print_speed,
                approach_z_offset_m=approach_z_offset_m,
                approach_vel_scale=approach_vel_scale,
                print_vel_scale=print_vel_scale,
                timeout_s=timeout_s,
                publish_markers=publish_markers,
                axis_length=axis_length,
                axis_radius=axis_radius,
                clear_markers_before=clear_markers_before,
            )
            if not res.get("ok", False):
                log.error(
                    f"[print_path_sequence] Print path failed at stage={res.get('stage')}: "
                    f"{res.get('error', 'unknown')}"
                )
            else:
                log.info(f"[print_path_sequence] Print path completed over {res.get('targets')} targets.")

            self.session.run_sync(self.session.motion.home(enqueue=False), timeout_s=timeout_s)
        except TimeoutError:
            log.error("[print_path_sequence] Sequence timed out.")
        finally:
            rclpy.shutdown()

    def _param_optional_float(self, name: str) -> Optional[float]:
        val = float(self.get_parameter(name).value)
        if val <= 0.0:
            return None
        return val


def main(args=None):
    rclpy.init(args=args)
    node = PrintPathSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

