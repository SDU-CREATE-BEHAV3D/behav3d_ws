#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator scan_sequence --ros-args -p scan_type:=grid_sweep
"""

from __future__ import annotations

import math
import threading
from pathlib import PurePosixPath
from typing import Optional

import rclpy
from rclpy.node import Node

from behav3d_utils import target_builder as tb

from .src.scan_session import ScanSession


class ScanSequenceNode(Node):
    def __init__(self):
        super().__init__("scan_sequence")

        self.declare_parameter("scan_type", "grid_sweep")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("eef_link", "femto_color_optical_calib")
        self.declare_parameter("capture_folder", "@session/grid_sweep")
        self.declare_parameter("scan_folder", "")

        self.declare_parameter("motion", "LIN")
        self.declare_parameter("do_home", True)
        self.declare_parameter("home_after", False)
        self.declare_parameter("vel_scale", 0.05)
        self.declare_parameter("accel_scale", 0.05)
        self.declare_parameter("timeout_s", 0.0)
        self.declare_parameter("settle_s", 0.5)
        self.declare_parameter("debug", False)
        self.declare_parameter("prompt", "")

        self.declare_parameter("rgb", True)
        self.declare_parameter("depth", True)
        self.declare_parameter("ir", True)
        self.declare_parameter("pose", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("axis_length", 0.05)
        self.declare_parameter("axis_radius", 0.003)
        self.declare_parameter("clear_markers_before", True)
        self.declare_parameter("clear_markers_after", True)

        self.declare_parameter("grid_width", 0.50)
        self.declare_parameter("grid_height", 0.40)
        self.declare_parameter("grid_center_x", -0.20)
        self.declare_parameter("grid_center_y", 0.80)
        self.declare_parameter("grid_center_z", 0.0)
        self.declare_parameter("grid_z_off", 0.60)
        self.declare_parameter("grid_nx", 5)
        self.declare_parameter("grid_ny", 4)
        self.declare_parameter("grid_row_major", False)
        self.declare_parameter("grid_use_tf_orientation", True)

        self.declare_parameter("target_x", 0.0)
        self.declare_parameter("target_y", 0.80)
        self.declare_parameter("target_z", 0.0)
        self.declare_parameter("target_qx", 0.0)
        self.declare_parameter("target_qy", 0.0)
        self.declare_parameter("target_qz", 0.0)
        self.declare_parameter("target_qw", 1.0)
        self.declare_parameter("fibonacci_distance", 0.50)
        self.declare_parameter("fibonacci_cap_deg", 45.0)
        self.declare_parameter("fibonacci_samples", 12)
        self.declare_parameter("fibonacci_z_jitter", 0.0)
        self.declare_parameter("fibonacci_use_tf_target_orientation", False)
        self.declare_parameter("fibonacci_orientation_mode", "look_at")

        self.declare_parameter("half_center_x", 0.0)
        self.declare_parameter("half_center_y", 0.80)
        self.declare_parameter("half_center_z", 0.30)
        self.declare_parameter("half_radius", 0.40)
        self.declare_parameter("half_height", 0.20)
        self.declare_parameter("half_angle_min_deg", -90.0)
        self.declare_parameter("half_angle_max_deg", 90.0)
        self.declare_parameter("half_n_angle", 7)
        self.declare_parameter("half_n_height", 3)
        self.declare_parameter("half_row_major", False)
        self.declare_parameter("half_orientation_mode", "look_at")
        self.declare_parameter("half_use_line_axis", True)
        self.declare_parameter("half_axis_start_x", -0.27)
        self.declare_parameter("half_axis_start_y", 0.85)
        self.declare_parameter("half_axis_start_z", 0.55)
        self.declare_parameter("half_axis_end_x", -0.07)
        self.declare_parameter("half_axis_end_y", 0.85)
        self.declare_parameter("half_axis_end_z", 0.55)
        self.declare_parameter("half_n_axis", 3)
        self.declare_parameter("half_arc_center_dx", 0.0)
        self.declare_parameter("half_arc_center_dy", 0.0)
        self.declare_parameter("half_arc_center_dz", 1.0)
        self.declare_parameter("half_roll_deg", 0.0)

        self.declare_parameter("run_reconstruction", False)
        self.declare_parameter("reconstruct_device", "CPU:0")
        self.declare_parameter("reconstruct_request_timeout_s", 10.0)
        self.declare_parameter("wait_reconstruction_outputs", True)
        self.declare_parameter("color_to_depth_wait_timeout_s", 60.0)
        self.declare_parameter("tsdf_wait_timeout_s", 180.0)
        self.declare_parameter("update_world_mesh", False)
        self.declare_parameter("tsdf_center_crop_enable", False)
        self.declare_parameter("tsdf_center_crop_width", 300)
        self.declare_parameter("tsdf_center_crop_height", 290)
        self.declare_parameter("tsdf_center_crop_apply_to_depth", False)
        self.declare_parameter("tsdf_aabb_crop_enable", False)
        self.declare_parameter("tsdf_aabb_crop_min", [-0.25, -1.10, -1.00])
        self.declare_parameter("tsdf_aabb_crop_max", [0.30, -0.65, 0.50])

        self.session = ScanSession(self)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self) -> None:
        log = self.get_logger()
        scan_type = str(self.get_parameter("scan_type").value).strip().lower()
        capture_folder = str(self.get_parameter("capture_folder").value).strip()
        timeout_s = self._param_optional_float("timeout_s")

        if not capture_folder:
            capture_folder = f"@session/{scan_type}"

        log.info(
            "Starting scan sequence: "
            f"type='{scan_type}', capture_folder='{capture_folder}', "
            f"debug={bool(self.get_parameter('debug').value)}, "
            f"reconstruct={bool(self.get_parameter('run_reconstruction').value)}"
        )

        try:
            res = self._run_selected_scan(scan_type=scan_type, capture_folder=capture_folder, timeout_s=timeout_s)
            if not res.get("ok", False):
                log.error(
                    f"[scan_sequence] Scan failed at stage={res.get('stage')}: "
                    f"{res.get('error', 'unknown')}"
                )
                return

            log.info(
                "[scan_sequence] Scan complete: "
                f"targets={len(res.get('targets', []))}, "
                f"planned={res.get('plan_ok', 0)}, "
                f"executed={res.get('exec_ok', 0)}, "
                f"captures={res.get('captures_ok', 0)}"
            )

            if bool(self.get_parameter("run_reconstruction").value):
                scan_folder = self._resolve_scan_folder(capture_folder)
                rec = self.session.run_reconstruction_for_scan(
                    scan_folder=scan_folder,
                    reconstruct_device=str(self.get_parameter("reconstruct_device").value).strip() or "CPU:0",
                    reconstruct_request_timeout_s=float(self.get_parameter("reconstruct_request_timeout_s").value),
                    wait_reconstruction_outputs=bool(self.get_parameter("wait_reconstruction_outputs").value),
                    color_to_depth_wait_timeout_s=float(self.get_parameter("color_to_depth_wait_timeout_s").value),
                    tsdf_wait_timeout_s=float(self.get_parameter("tsdf_wait_timeout_s").value),
                    update_world_mesh=bool(self.get_parameter("update_world_mesh").value),
                    tsdf_center_crop_enable=bool(self.get_parameter("tsdf_center_crop_enable").value),
                    tsdf_center_crop_width=int(self.get_parameter("tsdf_center_crop_width").value),
                    tsdf_center_crop_height=int(self.get_parameter("tsdf_center_crop_height").value),
                    tsdf_center_crop_apply_to_depth=bool(self.get_parameter("tsdf_center_crop_apply_to_depth").value),
                    tsdf_aabb_crop_enable=bool(self.get_parameter("tsdf_aabb_crop_enable").value),
                    tsdf_aabb_crop_min=list(self.get_parameter("tsdf_aabb_crop_min").value),
                    tsdf_aabb_crop_max=list(self.get_parameter("tsdf_aabb_crop_max").value),
                )
                if not rec.get("ok", False):
                    log.error(
                        f"[scan_sequence] Reconstruction failed at stage={rec.get('stage')}: "
                        f"{rec.get('error', 'unknown')}"
                    )
                else:
                    log.info(
                        "[scan_sequence] Reconstruction complete: "
                        f"mesh='{rec.get('mesh_path', '')}', rgb_ply='{rec.get('rgb_ply_path', '')}'"
                    )

            if bool(self.get_parameter("home_after").value):
                self.session.run_sync(self.session.motion.home(enqueue=False), timeout_s=timeout_s)
        except Exception as exc:
            log.error(f"[scan_sequence] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    def _run_selected_scan(self, *, scan_type: str, capture_folder: str, timeout_s: Optional[float]) -> dict:
        common = self._common_scan_kwargs(timeout_s)
        frame_id = str(self.get_parameter("frame_id").value).strip() or "world"

        if scan_type in ("grid", "grid_sweep"):
            return self.session.run_grid_scan(
                capture_folder=capture_folder,
                width=float(self.get_parameter("grid_width").value),
                height=float(self.get_parameter("grid_height").value),
                center_x=float(self.get_parameter("grid_center_x").value),
                center_y=float(self.get_parameter("grid_center_y").value),
                center_z=float(self.get_parameter("grid_center_z").value),
                z_off=float(self.get_parameter("grid_z_off").value),
                nx=int(self.get_parameter("grid_nx").value),
                ny=int(self.get_parameter("grid_ny").value),
                row_major=bool(self.get_parameter("grid_row_major").value),
                frame_id=frame_id,
                use_tf_orientation=bool(self.get_parameter("grid_use_tf_orientation").value),
                **common,
            )

        if scan_type in ("fibonacci", "fib"):
            target = self._target_pose(frame_id)
            orientation_pose = None
            if bool(self.get_parameter("fibonacci_use_tf_target_orientation").value):
                target = self._target_with_current_eef_orientation(target, frame_id)
            orientation_mode = str(self.get_parameter("fibonacci_orientation_mode").value).strip() or "look_at"
            if orientation_mode.lower() in (
                "fixed",
                "current",
                "current_eef",
                "look_at_current_roll",
                "look_at_keep_roll",
                "look_at_min_roll",
            ):
                orientation_pose = self._current_eef_pose(frame_id)
            return self.session.run_fibonacci_scan(
                target=target,
                distance=float(self.get_parameter("fibonacci_distance").value),
                cap_rad=math.radians(float(self.get_parameter("fibonacci_cap_deg").value)),
                samples=int(self.get_parameter("fibonacci_samples").value),
                z_jitter=float(self.get_parameter("fibonacci_z_jitter").value),
                orientation_mode=orientation_mode,
                orientation_pose=orientation_pose,
                capture_folder=capture_folder,
                **common,
            )

        if scan_type in ("half_cylinder", "half-cylinder", "half"):
            half_orientation_mode = str(self.get_parameter("half_orientation_mode").value).strip() or "look_at"
            half_orientation_pose = None
            if half_orientation_mode.lower() in (
                "fixed",
                "current",
                "current_eef",
                "look_at_current_roll",
                "look_at_keep_roll",
                "look_at_min_roll",
            ):
                half_orientation_pose = self._current_eef_pose(frame_id)
            axis_start_xyz = None
            axis_end_xyz = None
            n_axis = None
            if bool(self.get_parameter("half_use_line_axis").value):
                axis_start_xyz = (
                    float(self.get_parameter("half_axis_start_x").value),
                    float(self.get_parameter("half_axis_start_y").value),
                    float(self.get_parameter("half_axis_start_z").value),
                )
                axis_end_xyz = (
                    float(self.get_parameter("half_axis_end_x").value),
                    float(self.get_parameter("half_axis_end_y").value),
                    float(self.get_parameter("half_axis_end_z").value),
                )
                n_axis = int(self.get_parameter("half_n_axis").value)
            return self.session.run_half_cylinder_scan(
                capture_folder=capture_folder,
                center_x=float(self.get_parameter("half_center_x").value),
                center_y=float(self.get_parameter("half_center_y").value),
                center_z=float(self.get_parameter("half_center_z").value),
                radius=float(self.get_parameter("half_radius").value),
                height=float(self.get_parameter("half_height").value),
                angle_min_deg=float(self.get_parameter("half_angle_min_deg").value),
                angle_max_deg=float(self.get_parameter("half_angle_max_deg").value),
                n_angle=int(self.get_parameter("half_n_angle").value),
                n_height=int(self.get_parameter("half_n_height").value),
                frame_id=frame_id,
                row_major=bool(self.get_parameter("half_row_major").value),
                orientation_mode=half_orientation_mode,
                orientation_pose=half_orientation_pose,
                axis_start_xyz=axis_start_xyz,
                axis_end_xyz=axis_end_xyz,
                n_axis=n_axis,
                arc_center_direction=(
                    float(self.get_parameter("half_arc_center_dx").value),
                    float(self.get_parameter("half_arc_center_dy").value),
                    float(self.get_parameter("half_arc_center_dz").value),
                ),
                roll_deg=float(self.get_parameter("half_roll_deg").value),
                **common,
            )

        raise ValueError("scan_type must be one of: grid_sweep, fibonacci, half_cylinder")

    def _target_with_current_eef_orientation(self, target, frame_id: str):
        tf_pose = self._current_eef_pose(frame_id)
        if tf_pose is not None:
            target.pose.orientation = tf_pose.pose.orientation
        else:
            self.get_logger().warn("[scan_sequence] TF target orientation unavailable; using target quaternion params.")
        return target

    def _current_eef_pose(self, frame_id: str):
        eef_link = str(self.get_parameter("eef_link").value).strip() or "femto_color_optical_calib"
        timeout_s = self._param_optional_float("timeout_s")
        res = self.session.run_sync(
            self.session.camera.get_pose(
                eef=eef_link,
                base_frame=str(frame_id or "world"),
                use_tf=True,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if res.get("ok", False) and "pose" in res:
            return res["pose"]
        return None

    def _common_scan_kwargs(self, timeout_s: Optional[float]) -> dict:
        prompt = str(self.get_parameter("prompt").value).strip() or None
        return {
            "motion": str(self.get_parameter("motion").value).strip() or "LIN",
            "eef_link": str(self.get_parameter("eef_link").value).strip() or "femto_color_optical_calib",
            "do_home": bool(self.get_parameter("do_home").value),
            "vel_scale": float(self.get_parameter("vel_scale").value),
            "accel_scale": float(self.get_parameter("accel_scale").value),
            "timeout_s": timeout_s,
            "settle_s": float(self.get_parameter("settle_s").value),
            "prompt": prompt,
            "debug": bool(self.get_parameter("debug").value),
            "rgb": bool(self.get_parameter("rgb").value),
            "depth": bool(self.get_parameter("depth").value),
            "ir": bool(self.get_parameter("ir").value),
            "pose": bool(self.get_parameter("pose").value),
            "publish_markers": bool(self.get_parameter("publish_markers").value),
            "axis_length": float(self.get_parameter("axis_length").value),
            "axis_radius": float(self.get_parameter("axis_radius").value),
            "clear_markers_before": bool(self.get_parameter("clear_markers_before").value),
            "clear_markers_after": bool(self.get_parameter("clear_markers_after").value),
        }

    def _target_pose(self, frame_id: str):
        target = tb.poseStamped(
            float(self.get_parameter("target_x").value),
            float(self.get_parameter("target_y").value),
            float(self.get_parameter("target_z").value),
            float(self.get_parameter("target_qx").value),
            float(self.get_parameter("target_qy").value),
            float(self.get_parameter("target_qz").value),
            float(self.get_parameter("target_qw").value),
            frame_id,
        )
        return target

    def _resolve_scan_folder(self, capture_folder: str) -> str:
        explicit = str(self.get_parameter("scan_folder").value).strip()
        if explicit:
            return explicit
        name = str(capture_folder or "").strip()
        if name.startswith("@session/"):
            name = name[len("@session/"):]
        return PurePosixPath(name).name or "grid_sweep"

    def _param_optional_float(self, name: str) -> Optional[float]:
        val = float(self.get_parameter(name).value)
        if val <= 0.0:
            return None
        return val


def main(args=None):
    rclpy.init(args=args)
    node = ScanSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
