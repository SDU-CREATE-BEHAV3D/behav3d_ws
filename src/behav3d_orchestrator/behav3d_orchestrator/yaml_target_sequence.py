#!/usr/bin/env python3
#Run with ros2: run behav3d_orchestrator yaml_target_sequence --ros-args -p yaml_path:=/home/lab/behav3d_ws/yaml/lamine_2.yaml -p debug:=true

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from .src.yaml_session import YamlSession


class YamlTargetSequenceNode(Node):
    def __init__(self):
        super().__init__("yaml_target_sequence")

        self.declare_parameter("yaml_path", "yaml/lamine_2.yaml")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("debug", False)
        self.declare_parameter("vel_scale", 0.05)
        self.declare_parameter("accel_scale", 0.05)
        self.declare_parameter("timeout_s", 20.0)
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
        debug = bool(self.get_parameter("debug").value)
        vel_scale = float(self.get_parameter("vel_scale").value)
        accel_scale = float(self.get_parameter("accel_scale").value)
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
            "Starting YAML target sequence: "
            f"path='{yaml_path}', frame='{frame_id}', "
            f"v={vel_scale:.3f}, a={accel_scale:.3f}, debug={debug}"
        )

        try:
            targets = self.session.run_yaml_target_sequence(
                yaml_path=yaml_path,
                frame_id=frame_id,
                debug=debug,
                vel_scale=vel_scale,
                accel_scale=accel_scale,
                timeout_s=timeout_s,
                publish_markers=publish_markers,
                axis_length=axis_length,
                axis_radius=axis_radius,
                clear_markers_before=clear_markers_before,
            )
        except TimeoutError:
            log.error("YAML target sequence timed out.")
            rclpy.shutdown()
            return
        except Exception as exc:
            log.error(f"YAML target sequence failed: {exc}")
            rclpy.shutdown()
            return

        log.info(f"YAML target sequence complete. Loaded targets: {len(targets)}")
        try:
            self.session.run_sync(
                self.session.motion.home(enqueue=False),
                timeout_s=timeout_s,
            )
        except TimeoutError:
            log.error("Final home move timed out.")
        rclpy.shutdown()

    def _param_optional_float(self, name: str) -> Optional[float]:
        val = float(self.get_parameter(name).value)
        if val <= 0.0:
            return None
        return val


def main(args=None):
    rclpy.init(args=args)
    node = YamlTargetSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
