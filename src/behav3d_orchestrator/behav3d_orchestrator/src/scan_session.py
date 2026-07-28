#!/usr/bin/env python3
from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

import numpy as np
import rclpy
from behav3d_utils import pose_from_xyz_and_z_axis
from behav3d_utils import target_builder as tb
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from .control_session import ControlAwareSession
from .scan_sequences import fibonacci, grid_sweep, half_cylinder, half_cylinder_side_caps


class ScanSession(ControlAwareSession):
    """
    Shared scan execution helpers.

    Scan patterns generate PoseStamped targets. The common executor handles
    plan -> exec -> settle/input -> capture for all scan types.
    """

    def run_scan_targets(
        self,
        *,
        targets: Sequence[PoseStamped],
        capture_folder: Optional[str],
        motion: str = "LIN",
        eef_link: Optional[str] = "femto_color_optical_calib",
        do_home: bool = False,
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        timeout_s: Optional[float] = None,
        settle_s: float = 0.5,
        prompt: Optional[str] = None,
        debug: bool = False,
        rgb: bool = True,
        depth: bool = True,
        ir: bool = True,
        pose: bool = True,
        publish_markers: bool = True,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
        clear_markers_after: bool = True,
    ) -> dict[str, Any]:
        log = self.node.get_logger()
        target_list = list(targets)
        if not target_list:
            return {
                "ok": True,
                "stage": "no_targets",
                "targets": [],
                "plan_ok": 0,
                "exec_ok": 0,
                "captures_ok": 0,
                "failed_targets": [],
            }

        if do_home:
            self.run_sync(self.motion.home(enqueue=False), timeout_s=timeout_s)

        if str(motion).strip().upper() == "PTP":
            self.run_sync(self.motion.setPTP(enqueue=False), timeout_s=timeout_s)
            motion_mode = "PTP"
        else:
            self.run_sync(self.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            motion_mode = "LIN"

        if eef_link:
            self.run_sync(self.motion.setEef(str(eef_link), enqueue=False), timeout_s=timeout_s)
        if vel_scale is not None:
            self.run_sync(self.motion.setSpd(float(vel_scale), enqueue=False), timeout_s=timeout_s)
        if accel_scale is not None:
            self.run_sync(self.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)

        if publish_markers:
            try:
                self.run_sync(
                    self.util.publish_targets(
                        target_list,
                        axis_length=float(axis_length),
                        axis_radius=float(axis_radius),
                        clear_before=bool(clear_markers_before),
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
            except TimeoutError:
                log.warn("[scan_session] publish_targets timed out; continuing.")

        plan_ok = 0
        exec_ok = 0
        captures_ok = 0
        failed_targets: list[dict[str, Any]] = []

        try:
            for i, ps in enumerate(target_list):
                try:
                    plan_res = self.run_sync(
                        self.motion.plan(
                            pose=ps,
                            motion=motion_mode,
                            vel_scale=vel_scale,
                            accel_scale=accel_scale,
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                except TimeoutError:
                    failed_targets.append({"index": i, "stage": "plan", "error": "timeout"})
                    log.warn(f"[scan_session] Plan timed out at target {i}.")
                    continue

                if not plan_res.get("ok", False):
                    err = str(plan_res.get("error", "unknown"))
                    failed_targets.append({"index": i, "stage": "plan", "error": err})
                    log.warn(f"[scan_session] Plan failed at target {i}: {err}")
                    continue
                plan_ok += 1

                if debug:
                    if prompt:
                        self.run_sync(self.util.input(prompt=prompt, enqueue=False), timeout_s=None)
                    continue

                try:
                    exec_res = self.run_sync(self.motion.exec(enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    failed_targets.append({"index": i, "stage": "exec", "error": "timeout"})
                    log.warn(f"[scan_session] Exec timed out at target {i}.")
                    continue

                if not exec_res.get("ok", False):
                    err = str(exec_res.get("error", "unknown"))
                    failed_targets.append({"index": i, "stage": "exec", "error": err})
                    log.warn(f"[scan_session] Exec failed at target {i}: {err}")
                    continue
                exec_ok += 1

                if float(settle_s) > 0.0:
                    try:
                        self.run_sync(
                            self.util.wait(float(settle_s), enqueue=False),
                            timeout_s=timeout_s,
                        )
                    except TimeoutError:
                        log.warn(f"[scan_session] Settle wait timed out at target {i}; continuing.")

                if prompt:
                    self.run_sync(self.util.input(prompt=prompt, enqueue=False), timeout_s=None)

                try:
                    cap_res = self.run_sync(
                        self.camera.capture(
                            rgb=bool(rgb),
                            depth=bool(depth),
                            ir=bool(ir),
                            pose=bool(pose),
                            folder=capture_folder,
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                except TimeoutError:
                    failed_targets.append({"index": i, "stage": "capture", "error": "timeout"})
                    log.warn(f"[scan_session] Capture timed out at target {i}.")
                    continue

                if not cap_res.get("ok", False):
                    err = str(cap_res.get("error", "unknown"))
                    failed_targets.append({"index": i, "stage": "capture", "error": err})
                    log.warn(f"[scan_session] Capture failed at target {i}: {err}")
                    continue
                captures_ok += 1
        finally:
            if publish_markers and clear_markers_after:
                try:
                    self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    log.warn("[scan_session] delete_markers timed out.")

        ok = captures_ok > 0 or (debug and plan_ok > 0)
        if ok:
            stage = "done"
            error = ""
        elif plan_ok <= 0:
            stage = "plan"
            error = "No targets planned successfully."
        elif exec_ok <= 0:
            stage = "exec"
            error = "No planned targets executed successfully."
        else:
            stage = "capture"
            error = "No captures completed successfully."

        return {
            "ok": ok,
            "stage": stage,
            "error": error,
            "targets": target_list,
            "plan_ok": plan_ok,
            "exec_ok": exec_ok,
            "captures_ok": captures_ok,
            "failed_targets": failed_targets,
            "capture_folder": capture_folder,
        }

    def build_grid_targets(
        self,
        *,
        target: Optional[PoseStamped] = None,
        width: float = 1.0,
        height: float = 0.5,
        center_x: float = 0.0,
        center_y: float = 0.80,
        center_z: float = 0.0,
        z_off: float = 0.40,
        nx: int = 12,
        ny: int = 5,
        row_major: bool = False,
        frame_id: str = "world",
        eef_link: Optional[str] = "femto_color_optical_calib",
        use_tf_orientation: bool = True,
        timeout_s: Optional[float] = None,
    ) -> list[PoseStamped]:
        log = self.node.get_logger()
        if int(nx) < 2 or int(ny) < 2:
            log.warn("[scan_session:grid] nx/ny must be >= 2. No targets generated.")
            return []

        if target is not None:
            if not isinstance(target, PoseStamped):
                raise TypeError("target must be geometry_msgs.msg.PoseStamped")
            center = target
        else:
            center = tb.worldXY(float(center_x), float(center_y), float(center_z), str(frame_id or "world"))

        if target is None and use_tf_orientation and eef_link:
            try:
                pose_res = self.run_sync(
                    self.camera.get_pose(
                        eef=str(eef_link),
                        base_frame=str(frame_id or "world"),
                        use_tf=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if pose_res.get("ok", False) and "pose" in pose_res:
                    center = tb.setTargetOrigin(pose_res["pose"], center_x, center_y, center_z)
                else:
                    log.warn("[scan_session:grid] Using default orientation; TF pose not available.")
            except TimeoutError:
                log.warn("[scan_session:grid] Using default orientation; TF lookup timed out.")

        if target is None and not use_tf_orientation:
            center = pose_from_xyz_and_z_axis(
                xyz_m=(float(center_x), float(center_y), float(center_z)),
                z_axis=(0.0, 0.0, -1.0),
                frame_id=str(frame_id or "world"),
            )
            center = tb.rotate_euler(center, (0.0, 0.0, 180.0), degrees=True)

        return grid_sweep.build_targets_from_center(
            center=center,
            width=float(width),
            height=float(height),
            z_off=float(z_off),
            nx=int(nx),
            ny=int(ny),
            row_major=bool(row_major),
        )

    def run_grid_scan(self, **kwargs: Any) -> dict[str, Any]:
        capture_folder = kwargs.pop("capture_folder", "@session/grid_sweep")
        do_home = bool(kwargs.pop("do_home", False))
        timeout_s = kwargs.get("timeout_s", None)
        if do_home:
            self.run_sync(self.motion.home(enqueue=False), timeout_s=timeout_s)
        targets = self.build_grid_targets(
            target=kwargs.pop("target", None),
            width=kwargs.pop("width", 1.0),
            height=kwargs.pop("height", 0.5),
            center_x=kwargs.pop("center_x", 0.0),
            center_y=kwargs.pop("center_y", 0.80),
            center_z=kwargs.pop("center_z", 0.0),
            z_off=kwargs.pop("z_off", 0.40),
            nx=kwargs.pop("nx", 12),
            ny=kwargs.pop("ny", 5),
            row_major=kwargs.pop("row_major", False),
            frame_id=kwargs.pop("frame_id", "world"),
            eef_link=kwargs.get("eef_link", "femto_color_optical_calib"),
            use_tf_orientation=kwargs.pop("use_tf_orientation", True),
            timeout_s=timeout_s,
        )
        return self.run_scan_targets(targets=targets, capture_folder=capture_folder, **kwargs)

    @staticmethod
    def build_fibonacci_targets(
        *,
        target: PoseStamped,
        distance: float,
        cap_rad: float,
        samples: int,
        z_jitter: float = 0.0,
        order_start_xyz: Optional[Sequence[float]] = None,
        orientation_mode: str = "look_at",
        orientation_pose: Optional[PoseStamped] = None,
    ) -> list[PoseStamped]:
        if not isinstance(target, PoseStamped):
            raise TypeError("target must be geometry_msgs.msg.PoseStamped")
        if int(samples) <= 0:
            return []

        return fibonacci.build_targets(
            target=target,
            distance=float(distance),
            cap_rad=float(cap_rad),
            samples=int(samples),
            z_jitter=float(z_jitter),
            order_start_xyz=order_start_xyz,
            orientation_mode=str(orientation_mode or "look_at"),
            orientation_pose=orientation_pose,
        )

    def run_fibonacci_scan(
        self,
        *,
        target: PoseStamped,
        distance: float,
        cap_rad: float,
        samples: int,
        capture_folder: Optional[str],
        z_jitter: float = 0.0,
        order_start_xyz: Optional[Sequence[float]] = None,
        orientation_mode: str = "look_at",
        orientation_pose: Optional[PoseStamped] = None,
        **kwargs: Any,
    ) -> dict[str, Any]:
        do_home = bool(kwargs.pop("do_home", False))
        timeout_s = kwargs.get("timeout_s", None)
        if do_home:
            self.run_sync(self.motion.home(enqueue=False), timeout_s=timeout_s)
        targets = self.build_fibonacci_targets(
            target=target,
            distance=distance,
            cap_rad=cap_rad,
            samples=samples,
            z_jitter=z_jitter,
            order_start_xyz=order_start_xyz,
            orientation_mode=orientation_mode,
            orientation_pose=orientation_pose,
        )
        return self.run_scan_targets(targets=targets, capture_folder=capture_folder, **kwargs)

    @staticmethod
    def build_half_cylinder_targets(
        *,
        center_x: float,
        center_y: float,
        center_z: float,
        radius: float,
        height: float,
        angle_min_deg: float,
        angle_max_deg: float,
        n_angle: int,
        n_height: int,
        frame_id: str = "world",
        row_major: bool = False,
        orientation_mode: str = "look_at",
        orientation_pose: Optional[PoseStamped] = None,
        axis_start_xyz: Optional[Sequence[float]] = None,
        axis_end_xyz: Optional[Sequence[float]] = None,
        n_axis: Optional[int] = None,
        arc_center_direction: Sequence[float] = (0.0, 0.0, 1.0),
        roll_deg: float = 0.0,
    ) -> list[PoseStamped]:
        axis_start = _coerce_xyz_triplet(axis_start_xyz, fallback=(0.0, 0.0, 0.0)) if axis_start_xyz is not None else None
        axis_end = _coerce_xyz_triplet(axis_end_xyz, fallback=(0.0, 0.0, 0.0)) if axis_end_xyz is not None else None
        arc_center = _coerce_xyz_triplet(arc_center_direction, fallback=(0.0, 0.0, 1.0))
        return half_cylinder.build_targets(
            center_x=float(center_x),
            center_y=float(center_y),
            center_z=float(center_z),
            radius=float(radius),
            height=float(height),
            angle_min_deg=float(angle_min_deg),
            angle_max_deg=float(angle_max_deg),
            n_angle=int(n_angle),
            n_height=int(n_height),
            frame_id=str(frame_id or "world"),
            row_major=bool(row_major),
            orientation_mode=str(orientation_mode or "look_at"),
            orientation_pose=orientation_pose,
            axis_start_xyz=axis_start,
            axis_end_xyz=axis_end,
            n_axis=n_axis,
            arc_center_direction=arc_center,
            roll_deg=float(roll_deg),
        )

    def run_half_cylinder_scan(
        self,
        *,
        capture_folder: Optional[str],
        **kwargs: Any,
    ) -> dict[str, Any]:
        do_home = bool(kwargs.pop("do_home", False))
        timeout_s = kwargs.get("timeout_s", None)
        if do_home:
            self.run_sync(self.motion.home(enqueue=False), timeout_s=timeout_s)
        orientation_mode = str(kwargs.pop("orientation_mode", "look_at") or "look_at")
        orientation_pose = kwargs.pop("orientation_pose", None)
        axis_start_xyz = kwargs.pop("axis_start_xyz", None)
        axis_end_xyz = kwargs.pop("axis_end_xyz", None)
        n_axis = kwargs.pop("n_axis", None)
        arc_center_direction = kwargs.pop("arc_center_direction", (0.0, 0.0, 1.0))
        roll_deg = kwargs.pop("roll_deg", 0.0)
        targets = self.build_half_cylinder_targets(
            center_x=kwargs.pop("center_x"),
            center_y=kwargs.pop("center_y"),
            center_z=kwargs.pop("center_z"),
            radius=kwargs.pop("radius"),
            height=kwargs.pop("height"),
            angle_min_deg=kwargs.pop("angle_min_deg", -90.0),
            angle_max_deg=kwargs.pop("angle_max_deg", 90.0),
            n_angle=kwargs.pop("n_angle", 7),
            n_height=kwargs.pop("n_height", 3),
            frame_id=kwargs.pop("frame_id", "world"),
            row_major=kwargs.pop("row_major", False),
            orientation_mode=orientation_mode,
            orientation_pose=orientation_pose,
            axis_start_xyz=axis_start_xyz,
            axis_end_xyz=axis_end_xyz,
            n_axis=n_axis,
            arc_center_direction=arc_center_direction,
            roll_deg=float(roll_deg),
        )
        return self.run_scan_targets(targets=targets, capture_folder=capture_folder, **kwargs)

    @staticmethod
    def build_half_cylinder_side_caps_targets(
        *,
        axis_start_xyz: Sequence[float],
        axis_end_xyz: Sequence[float],
        radius: float,
        angle_min_deg: float,
        angle_max_deg: float,
        n_angle: int,
        n_axis: int,
        frame_id: str = "world",
        row_major: bool = False,
        orientation_mode: str = "look_at",
        orientation_pose: Optional[PoseStamped] = None,
        arc_center_direction: Sequence[float] = (0.0, 0.0, 1.0),
        roll_deg: float = 0.0,
        endcap_radius: Optional[float] = None,
        endcap_angle_min_deg: Optional[float] = None,
        endcap_angle_max_deg: Optional[float] = None,
        endcap_n_angle: Optional[int] = None,
        endcap_polar_min_deg: float = 15.0,
        endcap_polar_max_deg: float = 75.0,
        endcap_n_polar: int = 3,
        endcap_include_start: bool = True,
        endcap_include_end: bool = True,
        endcap_row_major: bool = False,
    ) -> list[PoseStamped]:
        axis_start = _coerce_xyz_triplet(axis_start_xyz, fallback=(0.0, 0.0, 0.0))
        axis_end = _coerce_xyz_triplet(axis_end_xyz, fallback=(0.0, 0.0, 0.0))
        arc_center = _coerce_xyz_triplet(arc_center_direction, fallback=(0.0, 0.0, 1.0))
        return half_cylinder_side_caps.build_targets(
            axis_start_xyz=axis_start,
            axis_end_xyz=axis_end,
            radius=float(radius),
            angle_min_deg=float(angle_min_deg),
            angle_max_deg=float(angle_max_deg),
            n_angle=int(n_angle),
            n_axis=int(n_axis),
            frame_id=str(frame_id or "world"),
            row_major=bool(row_major),
            orientation_mode=str(orientation_mode or "look_at"),
            orientation_pose=orientation_pose,
            arc_center_direction=arc_center,
            roll_deg=float(roll_deg),
            endcap_radius=endcap_radius,
            endcap_angle_min_deg=endcap_angle_min_deg,
            endcap_angle_max_deg=endcap_angle_max_deg,
            endcap_n_angle=endcap_n_angle,
            endcap_polar_min_deg=float(endcap_polar_min_deg),
            endcap_polar_max_deg=float(endcap_polar_max_deg),
            endcap_n_polar=int(endcap_n_polar),
            endcap_include_start=bool(endcap_include_start),
            endcap_include_end=bool(endcap_include_end),
            endcap_row_major=bool(endcap_row_major),
        )

    def run_half_cylinder_side_caps_scan(
        self,
        *,
        capture_folder: Optional[str],
        **kwargs: Any,
    ) -> dict[str, Any]:
        do_home = bool(kwargs.pop("do_home", False))
        timeout_s = kwargs.get("timeout_s", None)
        if do_home:
            self.run_sync(self.motion.home(enqueue=False), timeout_s=timeout_s)
        orientation_mode = str(kwargs.pop("orientation_mode", "look_at") or "look_at")
        orientation_pose = kwargs.pop("orientation_pose", None)
        axis_start_xyz = kwargs.pop("axis_start_xyz", None)
        axis_end_xyz = kwargs.pop("axis_end_xyz", None)
        if axis_start_xyz is None or axis_end_xyz is None:
            raise ValueError("half_cylinder_side_caps requires axis_start_xyz and axis_end_xyz")
        arc_center_direction = kwargs.pop("arc_center_direction", (0.0, 0.0, 1.0))
        roll_deg = kwargs.pop("roll_deg", 0.0)
        targets = self.build_half_cylinder_side_caps_targets(
            axis_start_xyz=axis_start_xyz,
            axis_end_xyz=axis_end_xyz,
            radius=kwargs.pop("radius"),
            angle_min_deg=kwargs.pop("angle_min_deg", -90.0),
            angle_max_deg=kwargs.pop("angle_max_deg", 90.0),
            n_angle=kwargs.pop("n_angle", 7),
            n_axis=kwargs.pop("n_axis", 3),
            frame_id=kwargs.pop("frame_id", "world"),
            row_major=kwargs.pop("row_major", False),
            orientation_mode=orientation_mode,
            orientation_pose=orientation_pose,
            arc_center_direction=arc_center_direction,
            roll_deg=float(roll_deg),
            endcap_radius=kwargs.pop("endcap_radius", None),
            endcap_angle_min_deg=kwargs.pop("endcap_angle_min_deg", None),
            endcap_angle_max_deg=kwargs.pop("endcap_angle_max_deg", None),
            endcap_n_angle=kwargs.pop("endcap_n_angle", None),
            endcap_polar_min_deg=kwargs.pop("endcap_polar_min_deg", 15.0),
            endcap_polar_max_deg=kwargs.pop("endcap_polar_max_deg", 75.0),
            endcap_n_polar=kwargs.pop("endcap_n_polar", 3),
            endcap_include_start=kwargs.pop("endcap_include_start", True),
            endcap_include_end=kwargs.pop("endcap_include_end", True),
            endcap_row_major=kwargs.pop("endcap_row_major", False),
        )
        return self.run_scan_targets(targets=targets, capture_folder=capture_folder, **kwargs)

    def run_reconstruction_for_scan(
        self,
        *,
        scan_folder: str,
        session_path: str = "@session",
        reconstruct_device: str = "CPU:0",
        reconstruct_request_timeout_s: float = 10.0,
        wait_reconstruction_outputs: bool = True,
        color_to_depth_wait_timeout_s: float = 60.0,
        tsdf_wait_timeout_s: float = 180.0,
        mesh_prefer: str = "mesh",
        mesh_update_wait_timeout_s: float = 45.0,
        mesh_update_request_timeout_s: float = 55.0,
        update_world_mesh: bool = True,
        tsdf_center_crop_enable: Optional[bool] = None,
        tsdf_center_crop_width: Optional[int] = None,
        tsdf_center_crop_height: Optional[int] = None,
        tsdf_center_crop_apply_to_depth: Optional[bool] = None,
        tsdf_aabb_crop_enable: Optional[bool] = None,
        tsdf_aabb_crop_min: Optional[Sequence[float]] = None,
        tsdf_aabb_crop_max: Optional[Sequence[float]] = None,
        tsdf_auto_object_crop_enable: Optional[bool] = None,
        tsdf_auto_object_min_height_m: float = 0.010,
        tsdf_auto_object_cluster_eps_m: float = 0.020,
        tsdf_auto_object_cluster_min_points: int = 30,
        tsdf_auto_object_neighbor_max_gap_m: float = 0.100,
        tsdf_auto_object_xy_margin_m: float = 0.020,
        tsdf_auto_object_top_margin_m: float = 0.010,
        tsdf_auto_object_table_below_margin_m: float = 0.010,
        tsdf_param_update_timeout_s: float = 8.0,
    ) -> dict[str, Any]:
        log = self.node.get_logger()

        c2d_start_ts = time.time()
        c2d_res = self.run_sync(
            self.camera.reconstruct_color_to_depth(
                use_latest=True,
                session_path=session_path,
                scan_folder=scan_folder,
                visualize=False,
                enqueue=False,
            ),
            timeout_s=float(reconstruct_request_timeout_s),
        )
        if not c2d_res.get("ok", False):
            return {"ok": False, "stage": "color_to_depth", "error": c2d_res.get("error", "unknown")}

        c2d_output = str(c2d_res.get("metrics", {}).get("output_path", "")).strip()
        if wait_reconstruction_outputs and c2d_output:
            self.wait_for_fresh_alignment_output(
                output_path=c2d_output,
                start_ts=c2d_start_ts,
                timeout_s=float(color_to_depth_wait_timeout_s),
            )

        if (
            tsdf_center_crop_enable is not None
            or tsdf_aabb_crop_enable is not None
            or tsdf_auto_object_crop_enable is not None
        ):
            try:
                self.set_tsdf_service_crop_params(
                    center_crop_enable=bool(tsdf_center_crop_enable),
                    center_crop_width=int(tsdf_center_crop_width or 0),
                    center_crop_height=int(tsdf_center_crop_height or 0),
                    center_crop_apply_to_depth=bool(tsdf_center_crop_apply_to_depth),
                    aabb_crop_enable=bool(tsdf_aabb_crop_enable),
                    aabb_crop_min=tsdf_aabb_crop_min or [-0.25, -1.10, -1.00],
                    aabb_crop_max=tsdf_aabb_crop_max or [0.30, -0.65, 0.50],
                    auto_object_crop_enable=bool(tsdf_auto_object_crop_enable),
                    auto_object_min_height_m=float(tsdf_auto_object_min_height_m),
                    auto_object_cluster_eps_m=float(tsdf_auto_object_cluster_eps_m),
                    auto_object_cluster_min_points=int(tsdf_auto_object_cluster_min_points),
                    auto_object_neighbor_max_gap_m=float(
                        tsdf_auto_object_neighbor_max_gap_m
                    ),
                    auto_object_xy_margin_m=float(tsdf_auto_object_xy_margin_m),
                    auto_object_top_margin_m=float(tsdf_auto_object_top_margin_m),
                    auto_object_table_below_margin_m=float(
                        tsdf_auto_object_table_below_margin_m
                    ),
                    timeout_s=float(tsdf_param_update_timeout_s),
                )
            except Exception as exc:
                log.warn(f"[scan_session] Failed to update TSDF crop params; continuing. reason='{exc}'")

        tsdf_start_ts = time.time()
        tsdf_res = self.run_sync(
            self.camera.reconstruct_tsdf_cropped(
                use_latest=True,
                session_path=session_path,
                scan_folder=scan_folder,
                visualize=False,
                device=str(reconstruct_device),
                enqueue=False,
            ),
            timeout_s=float(reconstruct_request_timeout_s),
        )
        if not tsdf_res.get("ok", False):
            return {"ok": False, "stage": "tsdf", "error": tsdf_res.get("error", "unknown")}

        mesh_path = str(tsdf_res.get("metrics", {}).get("mesh_path", "")).strip()
        rgb_ply_path = str(tsdf_res.get("metrics", {}).get("rgb_ply_path", "")).strip()
        if not rgb_ply_path:
            rgb_ply_path = str(tsdf_res.get("metrics", {}).get("output_path", "")).strip()

        if wait_reconstruction_outputs and mesh_path:
            self.wait_for_fresh_file_output(
                output_path=mesh_path,
                start_ts=tsdf_start_ts,
                timeout_s=float(tsdf_wait_timeout_s),
            )

        mesh_res: dict[str, Any] = {}
        if update_world_mesh:
            mesh_res = self.run_sync(
                self.camera.update_world_mesh(
                    use_latest=True,
                    session_path=session_path,
                    mesh_path=mesh_path,
                    ply_path=rgb_ply_path,
                    prefer=str(mesh_prefer),
                    wait_timeout_s=float(mesh_update_wait_timeout_s),
                    enqueue=False,
                ),
                timeout_s=float(mesh_update_request_timeout_s),
            )
            if not mesh_res.get("ok", False):
                return {"ok": False, "stage": "update_world_mesh", "error": mesh_res.get("error", "unknown")}

        return {
            "ok": True,
            "stage": "done",
            "scan_folder": scan_folder,
            "mesh_path": mesh_path,
            "rgb_ply_path": rgb_ply_path,
            "color_to_depth": c2d_res,
            "tsdf": tsdf_res,
            "world_mesh": mesh_res,
        }

    def set_tsdf_service_crop_params(
        self,
        *,
        center_crop_enable: bool,
        center_crop_width: int,
        center_crop_height: int,
        center_crop_apply_to_depth: bool,
        aabb_crop_enable: bool,
        aabb_crop_min: Sequence[float],
        aabb_crop_max: Sequence[float],
        auto_object_crop_enable: bool,
        auto_object_min_height_m: float,
        auto_object_cluster_eps_m: float,
        auto_object_cluster_min_points: int,
        auto_object_neighbor_max_gap_m: float,
        auto_object_xy_margin_m: float,
        auto_object_top_margin_m: float,
        auto_object_table_below_margin_m: float,
        timeout_s: float,
    ) -> None:
        timeout = max(0.1, float(timeout_s))
        crop_min = _coerce_xyz_triplet(aabb_crop_min, fallback=(-0.25, -1.10, -1.00))
        crop_max = _coerce_xyz_triplet(aabb_crop_max, fallback=(0.30, -0.65, 0.50))

        client = AsyncParameterClient(self.node, "/tsdf_cropped_service")
        if hasattr(client, "wait_for_services"):
            ready = bool(client.wait_for_services(timeout_sec=timeout))
        elif hasattr(client, "wait_for_service"):
            ready = bool(client.wait_for_service(timeout_sec=timeout))
        else:
            ready = bool(client.services_are_ready()) if hasattr(client, "services_are_ready") else False
        if not ready:
            raise RuntimeError("Parameter service for /tsdf_cropped_service not available.")

        fut = client.set_parameters(
            [
                Parameter("center_crop_enable", value=bool(center_crop_enable)),
                Parameter("center_crop_width", value=int(center_crop_width)),
                Parameter("center_crop_height", value=int(center_crop_height)),
                Parameter("center_crop_apply_to_depth", value=bool(center_crop_apply_to_depth)),
                Parameter("aabb_crop_enable", value=bool(aabb_crop_enable)),
                Parameter("aabb_crop_min", value=[float(crop_min[0]), float(crop_min[1]), float(crop_min[2])]),
                Parameter("aabb_crop_max", value=[float(crop_max[0]), float(crop_max[1]), float(crop_max[2])]),
                Parameter(
                    "auto_object_crop_enable",
                    value=bool(auto_object_crop_enable),
                ),
                Parameter(
                    "auto_object_min_height_m",
                    value=float(auto_object_min_height_m),
                ),
                Parameter(
                    "auto_object_cluster_eps_m",
                    value=float(auto_object_cluster_eps_m),
                ),
                Parameter(
                    "auto_object_cluster_min_points",
                    value=int(auto_object_cluster_min_points),
                ),
                Parameter(
                    "auto_object_neighbor_max_gap_m",
                    value=float(auto_object_neighbor_max_gap_m),
                ),
                Parameter(
                    "auto_object_xy_margin_m",
                    value=float(auto_object_xy_margin_m),
                ),
                Parameter(
                    "auto_object_top_margin_m",
                    value=float(auto_object_top_margin_m),
                ),
                Parameter(
                    "auto_object_table_below_margin_m",
                    value=float(auto_object_table_below_margin_m),
                ),
            ]
        )
        deadline = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            time.sleep(0.05)
        if not fut.done():
            raise TimeoutError("Timed out while setting TSDF crop parameters.")

        response = fut.result()
        results = list(response.results) if hasattr(response, "results") else list(response or [])
        failed = [str(r.reason) for r in results if not bool(getattr(r, "successful", False))]
        if failed:
            raise RuntimeError(f"Failed to set TSDF crop parameters: {'; '.join(failed)}")

    def wait_for_fresh_alignment_output(self, *, output_path: str, start_ts: float, timeout_s: float) -> None:
        out_dir = Path(output_path)

        def _fresh_alignment_exists() -> bool:
            if not out_dir.exists() or not out_dir.is_dir():
                return False
            for pattern in ("color_in_depth*.png", "color_in_depth*.jpg", "*.npy"):
                for p in out_dir.glob(pattern):
                    try:
                        if p.stat().st_mtime >= (float(start_ts) - 0.25):
                            return True
                    except OSError:
                        continue
            return False

        self.wait_for_fresh_file_output(
            output_path=str(out_dir),
            start_ts=start_ts,
            timeout_s=timeout_s,
            exists_fn=_fresh_alignment_exists,
        )

    def wait_for_fresh_file_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
        exists_fn: Optional[Callable[[], bool]] = None,
    ) -> None:
        deadline = time.time() + max(0.1, float(timeout_s))
        out_path = Path(output_path)

        def _fresh_file_exists() -> bool:
            if exists_fn is not None:
                return bool(exists_fn())
            if not out_path.exists():
                return False
            try:
                return out_path.stat().st_mtime >= float(start_ts)
            except Exception:
                return True

        while rclpy.ok() and time.time() < deadline:
            if _fresh_file_exists():
                return
            time.sleep(0.1)

        raise TimeoutError(
            f"Timed out waiting for reconstruction output '{output_path}' "
            f"to be updated after start_ts={start_ts:.3f}."
        )

def _coerce_xyz_triplet(values: Sequence[float], *, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
    try:
        arr = np.asarray(values, dtype=float).reshape(-1)
        if arr.shape[0] >= 3 and np.all(np.isfinite(arr[:3])):
            return (float(arr[0]), float(arr[1]), float(arr[2]))
    except Exception:
        pass
    return (float(fallback[0]), float(fallback[1]), float(fallback[2]))
