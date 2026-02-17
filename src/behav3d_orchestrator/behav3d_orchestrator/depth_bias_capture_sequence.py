#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator depth_bias_capture_sequence
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from .src.depth_bias_session import DepthBiasSession


class DepthBiasCaptureSequenceNode(Node):
    def __init__(self):
        super().__init__("depth_bias_capture_sequence")

        # Center point in mm (world), as provided by operator.
        self.declare_parameter("center_x_mm", -442.0)
        self.declare_parameter("center_y_mm", 868.0)
        self.declare_parameter("center_z_mm", -17.0)

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("eef_link", "femto_ir_optical_calib")
        self.declare_parameter("folder_root", "@session/depth_bias")

        self.declare_parameter("first_height_offset_m", 0.50)
        self.declare_parameter("height_step_m", 0.30)
        self.declare_parameter("captures_per_height", 15)

        self.declare_parameter("vel_scale", 0.05)
        self.declare_parameter("accel_scale", 0.05)

        self.declare_parameter("motion_timeout_s", 30.0)
        self.declare_parameter("capture_timeout_s", 8.0)
        self.declare_parameter("settle_s", 1.00)
        self.declare_parameter("capture_wait_s", 0.50)

        self.declare_parameter("home_before", True)
        self.declare_parameter("home_after", False)

        self.session = DepthBiasSession(self)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        center_x_mm = float(self.get_parameter("center_x_mm").value)
        center_y_mm = float(self.get_parameter("center_y_mm").value)
        center_z_mm = float(self.get_parameter("center_z_mm").value)
        frame_id = str(self.get_parameter("frame_id").value).strip() or "world"
        eef_link = str(self.get_parameter("eef_link").value).strip() or "femto_ir_optical_calib"
        folder_root = str(self.get_parameter("folder_root").value).strip() or "@session/depth_bias"
        first_height_offset_m = float(self.get_parameter("first_height_offset_m").value)
        height_step_m = float(self.get_parameter("height_step_m").value)
        captures_per_height = int(self.get_parameter("captures_per_height").value)
        vel_scale = float(self.get_parameter("vel_scale").value)
        accel_scale = float(self.get_parameter("accel_scale").value)
        motion_timeout_s = self._param_optional_float("motion_timeout_s")
        capture_timeout_s = self._param_optional_float("capture_timeout_s")
        settle_s = float(self.get_parameter("settle_s").value)
        capture_wait_s = float(self.get_parameter("capture_wait_s").value)
        home_before = bool(self.get_parameter("home_before").value)
        home_after = bool(self.get_parameter("home_after").value)

        log.info(
            "Starting depth-bias capture sequence: "
            f"center_mm=({center_x_mm:.1f}, {center_y_mm:.1f}, {center_z_mm:.1f}), "
            f"offset0={first_height_offset_m:.3f}m, step={height_step_m:.3f}m, "
            f"captures_per_height={captures_per_height}, eef='{eef_link}', folder_root='{folder_root}'"
        )

        try:
            summary = self.session.run_depth_bias_capture(
                center_x_mm=center_x_mm,
                center_y_mm=center_y_mm,
                center_z_mm=center_z_mm,
                first_height_offset_m=first_height_offset_m,
                height_step_m=height_step_m,
                captures_per_height=captures_per_height,
                folder_root=folder_root,
                eef_link=eef_link,
                frame_id=frame_id,
                vel_scale=vel_scale,
                accel_scale=accel_scale,
                motion_timeout_s=motion_timeout_s,
                capture_timeout_s=capture_timeout_s,
                settle_s=settle_s,
                capture_wait_s=capture_wait_s,
                home_before=home_before,
                home_after=home_after,
            )
        except TimeoutError:
            log.error("[depth_bias] Sequence timed out.")
            rclpy.shutdown()
            return
        except Exception as exc:
            log.error(f"[depth_bias] Sequence failed: {exc}")
            rclpy.shutdown()
            return

        for item in summary.get("heights", []):
            log.info(
                f"[depth_bias] {item.get('name')}: "
                f"z={float(item.get('target_z_m', 0.0)):.3f} "
                f"ok={int(item.get('captures_ok', 0))}/{int(item.get('captures_total', 0))} "
                f"folder='{item.get('folder', '')}'"
            )
        log.info(
            f"[depth_bias] Done. Total captures: "
            f"ok={int(summary.get('captures_ok', 0))} fail={int(summary.get('captures_fail', 0))}"
        )
        rclpy.shutdown()

    def _param_optional_float(self, name: str) -> Optional[float]:
        val = float(self.get_parameter(name).value)
        if val <= 0.0:
            return None
        return val


def main(args=None):
    rclpy.init(args=args)
    node = DepthBiasCaptureSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
