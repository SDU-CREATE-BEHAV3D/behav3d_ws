#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from .yaml_session import YamlSession


TOOL_PLUS_Z_POINTS_OUTWARD = False


class PrintScanDotsSession(YamlSession):
    """
    Session helpers for print+scan over YAML targets.
    Keeps YAML parsing in YamlSession and scan-enabled print behavior here.
    """

    def run_print_dots(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
        dot_steps: int = 5000,
        dot_speed: int = 1200,
        simulate_printing: bool = False,
        approach_z_offset_m: float = 0.40,
        dot_z_offset_m: float = 0.04,
        pre_dot_vel_scale: float = 0.10,
        dot_vel_scale: float = 0.03,
        dot_accel_scale: float = 0.05,
        dwell_s: float = 0.4,
        timeout_s: Optional[float] = None,
        pause_gate: Optional[Callable[[], None]] = None,
        publish_markers: bool = True,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
        enable_scan: bool = True,
        scan_every_n: int = 7,
        scan_eef_link: str = "femto_color_optical_calib",
        scan_distance_m: float = 0.55,
        scan_cap_rad_deg: float = 28.0,
        scan_samples: int = 16,
        scan_settle_s: float = 0.3,
        scan_z_jitter_m: float = 0.0,
        scan_motion: str = "LIN",
        scan_vel_scale: float = 0.05,
        scan_accel_scale: float = 0.05,
        scan_timeout_s: Optional[float] = None,
        scan_capture_folder_prefix: str = "@session/print_scan",
        scan_publish_targets: bool = True,
        scan_axis_length: float = 0.05,
        scan_axis_radius: float = 0.003,
        scan_clear_markers_before: bool = True,
        scan_clear_markers_after: bool = False,
        run_reconstruction: bool = True,
        reconstruct_device: str = "CPU:0",
        reconstruct_request_timeout_s: float = 10.0,
        color_to_depth_wait_timeout_s: float = 60.0,
        tsdf_wait_timeout_s: float = 180.0,
        mesh_update_wait_timeout_s: float = 45.0,
        mesh_update_request_timeout_s: float = 55.0,
        mesh_prefer: str = "mesh",
    ) -> dict:
        """
        Dot print behavior:
        1) Parse ordered poses from YAML.
        2) Move to first target + approach_z_offset_m at pre_dot_vel_scale.
        3) For each target:
           - go to target + dot_z_offset_m
           - go to target
           - extrude print_steps
           - wait dwell_s
           - lift back to target + dot_z_offset_m
           - every `scan_every_n` printed targets, run a Fibonacci scan around the
             midpoint target of that block
        """
        log = self.node.get_logger()
        targets = self.parse_yaml_targets(yaml_path=yaml_path, frame_id=frame_id)
        if len(targets) < 1:
            raise ValueError("Need at least 1 target to print dots.")
        if simulate_printing:
            log.warn("[print_dots] simulate_printing=True: extruder commands will be skipped.")

        first = targets[0]
        first_approach = self._with_z_offset(first, float(approach_z_offset_m))

        if publish_markers:
            self.run_sync(
                self.util.publish_targets(
                    targets,
                    axis_length=float(axis_length),
                    axis_radius=float(axis_radius),
                    clear_before=bool(clear_markers_before),
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )

        try:
            if callable(pause_gate):
                pause_gate()

            res = self.run_sync(
                self.motion.goto(
                    pose=first_approach,
                    motion="LIN",
                    vel_scale=float(pre_dot_vel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": "approach_first", "error": res.get("error", "unknown")}

            printed = 0
            for i, target in enumerate(targets):
                if callable(pause_gate):
                    pause_gate()

                above = self._with_z_offset(target, float(dot_z_offset_m))

                above_vel = float(pre_dot_vel_scale) if i == 0 else float(dot_vel_scale)
                res = self.run_sync(
                    self.motion.goto(
                        pose=above,
                        motion="LIN",
                        vel_scale=above_vel,
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not res.get("ok", False):
                    return {
                        "ok": False,
                        "stage": f"goto_above_{i}",
                        "error": res.get("error", "unknown"),
                    }

                res = self.run_sync(
                    self.motion.goto(
                        pose=target,
                        motion="LIN",
                        vel_scale=float(dot_vel_scale),
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not res.get("ok", False):
                    return {
                        "ok": False,
                        "stage": f"goto_target_{i}",
                        "error": res.get("error", "unknown"),
                    }

                if not simulate_printing:
                    res = self.run_sync(
                        self.extruder.print_steps(
                            steps=int(dot_steps),
                            speed=int(dot_speed),
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                    if not res.get("ok", False):
                        return {
                            "ok": False,
                            "stage": f"extrude_{i}",
                            "error": res.get("error", "unknown"),
                        }

                if callable(pause_gate):
                    pause_gate()

                if float(dwell_s) > 0.0:
                    res = self.run_sync(
                        self.util.wait(float(dwell_s), enqueue=False),
                        timeout_s=timeout_s,
                    )
                    if not res.get("ok", False):
                        return {
                            "ok": False,
                            "stage": f"wait_{i}",
                            "error": res.get("error", "unknown"),
                        }

                res = self.run_sync(
                    self.motion.goto(
                        pose=above,
                        motion="LIN",
                        vel_scale=float(dot_vel_scale),
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not res.get("ok", False):
                    return {
                        "ok": False,
                        "stage": f"lift_{i}",
                        "error": res.get("error", "unknown"),
                    }

                printed += 1

                if enable_scan and scan_every_n > 0 and (printed % scan_every_n == 0):
                    block_start = printed - scan_every_n
                    block_mid = block_start + (scan_every_n // 2)
                    if block_mid < 0 or block_mid >= len(targets):
                        log.warn(
                            f"[print_dots] Scan midpoint index out of range: {block_mid}. "
                            "Skipping scan trigger."
                        )
                        continue

                    center_target = targets[block_mid]
                    capture_folder = f"{scan_capture_folder_prefix.rstrip('/')}_{printed:03d}"
                    scan_folder = self._scan_folder_from_capture_folder(capture_folder)
                    log.info(
                        f"[print_dots] Triggering scan after printed={printed}. "
                        f"Block [{block_start + 1}..{printed}] midpoint={block_mid + 1} "
                        f"folder='{capture_folder}'"
                    )

                    # Switch motion defaults for scan
                    self.run_sync(
                        self.motion.setEef(str(scan_eef_link), enqueue=False),
                        timeout_s=scan_timeout_s,
                    )
                    self.run_sync(
                        self.motion.setAcc(float(scan_accel_scale), enqueue=False),
                        timeout_s=scan_timeout_s,
                    )
                    self.run_sync(
                        self.motion.setSpd(float(scan_vel_scale), enqueue=False),
                        timeout_s=scan_timeout_s,
                    )
                    if str(scan_motion).strip().upper() == "PTP":
                        self.run_sync(self.motion.setPTP(enqueue=False), timeout_s=scan_timeout_s)
                    else:
                        self.run_sync(self.motion.setLIN(enqueue=False), timeout_s=scan_timeout_s)

                    if callable(pause_gate):
                        pause_gate()

                    scan_res = self.run_fib_scan_blocking(
                        target=center_target,
                        distance=float(scan_distance_m),
                        cap_rad=math.radians(float(scan_cap_rad_deg)),
                        samples=int(scan_samples),
                        capture_folder=capture_folder,
                        settle_s=float(scan_settle_s),
                        prompt=None,
                        debug=False,
                        z_jitter=float(scan_z_jitter_m),
                        motion=str(scan_motion),
                        vel_scale=float(scan_vel_scale),
                        accel_scale=float(scan_accel_scale),
                        timeout_s=scan_timeout_s,
                        publish_targets=bool(scan_publish_targets),
                        axis_length=float(scan_axis_length),
                        axis_radius=float(scan_axis_radius),
                        clear_markers_before=bool(scan_clear_markers_before),
                        clear_markers_after=bool(scan_clear_markers_after),
                    )

                    if int(scan_res.get("captures_ok", 0)) <= 0:
                        return {
                            "ok": False,
                            "stage": f"scan_after_{printed}",
                            "error": "scan captured 0 viewpoints",
                            "printed": printed,
                            "targets": len(targets),
                        }

                    # Restore print motion defaults
                    self.run_sync(
                        self.motion.setEef("extruder_tcp", enqueue=False),
                        timeout_s=timeout_s,
                    )
                    self.run_sync(
                        self.motion.setAcc(float(dot_accel_scale), enqueue=False),
                        timeout_s=timeout_s,
                    )
                    self.run_sync(
                        self.motion.setSpd(float(dot_vel_scale), enqueue=False),
                        timeout_s=timeout_s,
                    )
                    self.run_sync(
                        self.motion.setLIN(enqueue=False),
                        timeout_s=timeout_s,
                    )

                    if run_reconstruction:
                        recon_res = self.run_reconstruction_pipeline(
                            capture_folder=capture_folder,
                            device=str(reconstruct_device),
                            request_timeout_s=float(reconstruct_request_timeout_s),
                            color_wait_timeout_s=float(color_to_depth_wait_timeout_s),
                            tsdf_wait_timeout_s=float(tsdf_wait_timeout_s),
                            mesh_update_wait_timeout_s=float(mesh_update_wait_timeout_s),
                            mesh_update_request_timeout_s=float(mesh_update_request_timeout_s),
                            mesh_prefer=str(mesh_prefer),
                        )
                        if not recon_res.get("ok", False):
                            return {
                                "ok": False,
                                "stage": f"reconstruct_after_{printed}",
                                "error": recon_res.get("error", "reconstruction failed"),
                                "scan_folder": scan_folder,
                                "printed": printed,
                                "targets": len(targets),
                            }

            return {"ok": True, "stage": "done", "targets": len(targets), "printed": printed}
        finally:
            if publish_markers:
                try:
                    self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    self.node.get_logger().warn("[print_dots] delete_markers timed out.")

    @staticmethod
    def _with_z_offset(ps: PoseStamped, z_offset_m: float) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = ps.header.frame_id
        out.header.stamp = ps.header.stamp
        out.pose.position.x = float(ps.pose.position.x)
        out.pose.position.y = float(ps.pose.position.y)
        out.pose.position.z = float(ps.pose.position.z) + float(z_offset_m)
        out.pose.orientation.x = float(ps.pose.orientation.x)
        out.pose.orientation.y = float(ps.pose.orientation.y)
        out.pose.orientation.z = float(ps.pose.orientation.z)
        out.pose.orientation.w = float(ps.pose.orientation.w)
        return out

    def run_reconstruction_pipeline(
        self,
        *,
        capture_folder: str,
        device: str,
        request_timeout_s: float,
        color_wait_timeout_s: float,
        tsdf_wait_timeout_s: float,
        mesh_update_wait_timeout_s: float,
        mesh_update_request_timeout_s: float,
        mesh_prefer: str,
    ) -> Dict[str, Any]:
        log = self.node.get_logger()
        scan_folder = self._scan_folder_from_capture_folder(capture_folder)
        log.info(
            f"[print_dots] Reconstruction for scan_folder='{scan_folder}' "
            f"(device='{device}')"
        )

        try:
            c2d_res = self.run_color_to_depth_reconstruct(
                use_latest=True,
                session_path="@session",
                scan_folder=scan_folder,
                visualize=False,
                timeout_s=float(request_timeout_s),
                wait_for_outputs=True,
                wait_timeout_s=float(color_wait_timeout_s),
            )
        except TimeoutError:
            return {"ok": False, "error": "color_to_depth request timed out"}

        if not c2d_res.get("ok", False):
            return {"ok": False, "error": f"color_to_depth failed: {c2d_res.get('error')}"}

        try:
            tsdf_res = self.run_tsdf_cropped_reconstruct(
                use_latest=True,
                session_path="@session",
                scan_folder=scan_folder,
                visualize=False,
                device=str(device),
                timeout_s=float(request_timeout_s),
                wait_for_outputs=True,
                wait_timeout_s=float(tsdf_wait_timeout_s),
            )
        except TimeoutError:
            return {"ok": False, "error": "tsdf_cropped request timed out"}

        if not tsdf_res.get("ok", False):
            return {"ok": False, "error": f"tsdf_cropped failed: {tsdf_res.get('error')}"}

        mesh_path = str(tsdf_res.get("metrics", {}).get("mesh_path", "")).strip()
        rgb_ply_path = str(tsdf_res.get("metrics", {}).get("rgb_ply_path", "")).strip()
        if not rgb_ply_path:
            rgb_ply_path = str(tsdf_res.get("metrics", {}).get("output_path", "")).strip()

        try:
            mesh_res = self.run_update_world_mesh(
                use_latest=True,
                session_path="@session",
                mesh_path=mesh_path,
                ply_path=rgb_ply_path,
                prefer=str(mesh_prefer),
                wait_timeout_s=float(mesh_update_wait_timeout_s),
                timeout_s=float(mesh_update_request_timeout_s),
            )
        except TimeoutError:
            return {"ok": False, "error": "update_world_mesh request timed out"}

        if not mesh_res.get("ok", False):
            return {"ok": False, "error": f"update_world_mesh failed: {mesh_res.get('error')}"}

        published_path = str(mesh_res.get("metrics", {}).get("published_path", "")).strip()
        published_kind = str(mesh_res.get("metrics", {}).get("published_kind", "")).strip()
        log.info(
            f"[print_dots] Reconstruction completed for '{scan_folder}'. "
            f"World mesh updated ({published_kind}): {published_path}"
        )
        return {
            "ok": True,
            "scan_folder": scan_folder,
            "metrics": {
                "published_path": published_path,
                "published_kind": published_kind,
                "mesh_path": mesh_path,
                "rgb_ply_path": rgb_ply_path,
            },
        }

    def run_tsdf_cropped_reconstruct(
        self,
        *,
        use_latest: bool = True,
        session_path: str = "",
        scan_folder: str = "manual_caps",
        visualize: bool = False,
        device: str = "CPU:0",
        timeout_s: Optional[float] = None,
        wait_for_outputs: bool = False,
        wait_timeout_s: float = 180.0,
    ) -> Dict[str, Any]:
        start_ts = time.time()
        res = self.run_sync(
            self.camera.reconstruct_tsdf_cropped(
                use_latest=use_latest,
                session_path=session_path,
                scan_folder=scan_folder,
                visualize=visualize,
                device=device,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if not wait_for_outputs or not res.get("ok", False):
            return res

        metrics = res.get("metrics", {})
        output_path = str(metrics.get("mesh_path", "")).strip()
        if not output_path:
            output_path = str(metrics.get("output_path", "")).strip()
        if output_path:
            self._wait_for_fresh_file_output(
                output_path=output_path,
                start_ts=start_ts,
                timeout_s=float(wait_timeout_s),
            )
        return res

    def run_color_to_depth_reconstruct(
        self,
        *,
        use_latest: bool = True,
        session_path: str = "",
        scan_folder: str = "manual_caps",
        visualize: bool = False,
        timeout_s: Optional[float] = None,
        wait_for_outputs: bool = False,
        wait_timeout_s: float = 30.0,
    ) -> Dict[str, Any]:
        start_ts = time.time()
        res = self.run_sync(
            self.camera.reconstruct_color_to_depth(
                use_latest=use_latest,
                session_path=session_path,
                scan_folder=scan_folder,
                visualize=visualize,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if not wait_for_outputs or not res.get("ok", False):
            return res

        output_path = str(res.get("metrics", {}).get("output_path", ""))
        if output_path:
            self._wait_for_fresh_alignment_output(
                output_path=output_path,
                start_ts=start_ts,
                timeout_s=float(wait_timeout_s),
            )
        return res

    def run_update_world_mesh(
        self,
        *,
        use_latest: bool = True,
        session_path: str = "",
        mesh_path: str = "",
        ply_path: str = "",
        prefer: str = "mesh",
        wait_timeout_s: float = 60.0,
        timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        return self.run_sync(
            self.camera.update_world_mesh(
                use_latest=use_latest,
                session_path=session_path,
                mesh_path=mesh_path,
                ply_path=ply_path,
                prefer=prefer,
                wait_timeout_s=wait_timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )

    def _wait_for_fresh_alignment_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ) -> Dict[str, Any]:
        out_dir = Path(output_path)

        def _fresh_alignment_exists() -> bool:
            if not out_dir.exists() or not out_dir.is_dir():
                return False
            for path in out_dir.glob("color_in_depth*.png"):
                try:
                    if path.stat().st_mtime >= (start_ts - 0.25):
                        return True
                except OSError:
                    continue
            return False

        return self.run_sync(
            self.util.wait_until(
                predicate=_fresh_alignment_exists,
                period_s=0.5,
                timeout_s=timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s + 1.0,
        )

    def _wait_for_fresh_file_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ) -> Dict[str, Any]:
        out_path = Path(output_path)

        def _fresh_file_exists() -> bool:
            try:
                if not out_path.exists() or not out_path.is_file():
                    return False
                st = out_path.stat()
                if st.st_size <= 0:
                    return False
                return st.st_mtime >= (start_ts - 0.25)
            except OSError:
                return False

        return self.run_sync(
            self.util.wait_until(
                predicate=_fresh_file_exists,
                period_s=0.5,
                timeout_s=timeout_s,
                enqueue=False,
            ),
            timeout_s=timeout_s + 1.0,
        )

    def run_fib_scan_blocking(
        self,
        *,
        target: PoseStamped,
        distance: float,
        cap_rad: float,
        samples: int,
        capture_folder: Optional[str],
        settle_s: float = 0.2,
        prompt: Optional[str] = None,
        debug: bool = False,
        z_jitter: float = 0.0,
        motion: str = "LIN",
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        timeout_s: Optional[float] = None,
        publish_targets: bool = True,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
        clear_markers_after: bool = False,
    ) -> Dict[str, int]:
        log = self.node.get_logger()

        targets = self._build_fib_targets(
            target=target,
            distance=float(distance),
            cap_rad=float(cap_rad),
            samples=int(samples),
            z_jitter=float(z_jitter),
        )
        targets = self._order_targets_by_nearest_neighbor(targets=targets)
        if not targets:
            return {"ok": True, "targets": 0, "plan_ok": 0, "exec_ok": 0, "captures_ok": 0}

        if publish_targets:
            try:
                self.run_sync(
                    self.util.publish_targets(
                        targets,
                        axis_length=axis_length,
                        axis_radius=axis_radius,
                        clear_before=clear_markers_before,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
            except TimeoutError:
                log.warn("[print_dots:fib_scan] publish_targets timed out; continuing.")

        plan_ok = 0
        exec_ok = 0
        captures_ok = 0

        for i, ps in enumerate(targets):
            try:
                plan_res = self.run_sync(
                    self.motion.plan(
                        pose=ps,
                        motion=motion,
                        vel_scale=vel_scale,
                        accel_scale=accel_scale,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
            except TimeoutError:
                log.warn(f"[print_dots:fib_scan] Plan timeout at viewpoint {i}.")
                continue

            if not plan_res.get("ok", False):
                log.warn(
                    f"[print_dots:fib_scan] Plan failed at viewpoint {i}; "
                    f"error={plan_res.get('error')}"
                )
                continue
            plan_ok += 1

            try:
                exec_res = self.run_sync(
                    self.motion.exec(enqueue=False),
                    timeout_s=timeout_s,
                )
            except TimeoutError:
                log.warn(f"[print_dots:fib_scan] Exec timeout at viewpoint {i}.")
                continue

            if not exec_res.get("ok", False):
                log.warn(f"[print_dots:fib_scan] Exec failed at viewpoint {i}.")
                continue
            exec_ok += 1

            if settle_s > 0.0:
                try:
                    self.run_sync(self.util.wait(float(settle_s), enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    log.warn(f"[print_dots:fib_scan] Settle wait timeout at viewpoint {i}.")

            if prompt:
                self.run_sync(self.util.input(prompt=prompt, enqueue=False))
            elif debug:
                self.run_sync(
                    self.util.input(
                        prompt=f"Press ENTER to capture at viewpoint {i}...",
                        enqueue=False,
                    )
                )

            try:
                cap_res = self.run_sync(
                    self.camera.capture(
                        rgb=True,
                        depth=True,
                        ir=True,
                        pose=True,
                        folder=capture_folder,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
            except TimeoutError:
                log.warn(f"[print_dots:fib_scan] Capture timeout at viewpoint {i}.")
                continue

            if not cap_res.get("ok", False):
                log.warn(f"[print_dots:fib_scan] Capture failed at viewpoint {i}.")
                continue
            captures_ok += 1

        if publish_targets and clear_markers_after:
            try:
                self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
            except TimeoutError:
                log.warn("[print_dots:fib_scan] delete_markers timed out.")

        return {
            "ok": True,
            "targets": len(targets),
            "plan_ok": plan_ok,
            "exec_ok": exec_ok,
            "captures_ok": captures_ok,
        }

    @staticmethod
    def _build_fib_targets(
        *,
        target: PoseStamped,
        distance: float,
        cap_rad: float,
        samples: int,
        z_jitter: float,
    ) -> List[PoseStamped]:
        if samples <= 0:
            return []

        p_t = np.array(
            [target.pose.position.x, target.pose.position.y, target.pose.position.z],
            dtype=float,
        )
        q_t = np.array(
            [
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ],
            dtype=float,
        )
        qn = np.linalg.norm(q_t)
        R_t = R.identity() if qn < 1e-12 else R.from_quat(q_t / qn)

        dirs_local = _fibonacci_cap_dirs_np(cap_rad, samples)
        poses_world: List[PoseStamped] = []
        rz_adjust = R.from_rotvec([0.0, 0.0, -np.pi / 2])

        for i in range(samples):
            d = dirs_local[i]
            p_loc = distance * d

            z_axis = d if TOOL_PLUS_Z_POINTS_OUTWARD else -d
            x_guess = np.array([1.0, 0.0, 0.0], dtype=float)
            x_axis = x_guess - np.dot(x_guess, z_axis) * z_axis
            nx = np.linalg.norm(x_axis)
            if nx < 1e-9:
                y_guess = np.array([0.0, 1.0, 0.0], dtype=float)
                x_axis = y_guess - np.dot(y_guess, z_axis) * z_axis
                nx = np.linalg.norm(x_axis)
                if nx < 1e-12:
                    x_axis = _any_orthonormal(z_axis)
                    nx = np.linalg.norm(x_axis)
            x_axis /= nx
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= max(np.linalg.norm(y_axis), 1e-12)

            R_loc = R.from_matrix(np.column_stack((x_axis, y_axis, z_axis))) * rz_adjust
            p_w = p_t + R_t.apply(p_loc)
            R_w = R_t * R_loc

            if z_jitter > 0.0:
                ray_dir = p_t - p_w
                n_ray = np.linalg.norm(ray_dir)
                if n_ray > 1e-9:
                    ray_dir /= n_ray
                    delta = np.random.uniform(-z_jitter, +z_jitter)
                    p_w = p_w + delta * ray_dir

            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.pose.position.x = float(p_w[0])
            ps.pose.position.y = float(p_w[1])
            ps.pose.position.z = float(p_w[2])
            q_w = R_w.as_quat()
            ps.pose.orientation.x = float(q_w[0])
            ps.pose.orientation.y = float(q_w[1])
            ps.pose.orientation.z = float(q_w[2])
            ps.pose.orientation.w = float(q_w[3])
            poses_world.append(ps)

        return poses_world

    @staticmethod
    def _order_targets_by_nearest_neighbor(*, targets: List[PoseStamped]) -> List[PoseStamped]:
        if len(targets) <= 2:
            return targets

        points = np.array(
            [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z] for ps in targets],
            dtype=float,
        )
        remaining = set(range(len(targets)))
        ordered_indices: List[int] = []

        current_idx = 0
        ordered_indices.append(current_idx)
        remaining.remove(current_idx)
        current = points[current_idx]

        while remaining:
            rem_idx = np.array(sorted(remaining), dtype=int)
            d2 = np.sum((points[rem_idx] - current) ** 2, axis=1)
            next_idx = int(rem_idx[int(np.argmin(d2))])
            ordered_indices.append(next_idx)
            remaining.remove(next_idx)
            current = points[next_idx]

        return [targets[i] for i in ordered_indices]

    @staticmethod
    def _scan_folder_from_capture_folder(capture_folder: str) -> str:
        folder = str(capture_folder or "").strip()
        if not folder:
            return "manual_caps"
        if folder.startswith("@session/"):
            folder = folder[len("@session/"):]
        folder = folder.strip("/")
        return folder or "manual_caps"


def _fibonacci_cap_dirs_np(cap_rad: float, n: int) -> np.ndarray:
    if n <= 0:
        return np.zeros((0, 3), dtype=float)

    cos_cap = float(np.cos(cap_rad))
    i = np.arange(n, dtype=float)
    z = cos_cap + (1.0 - cos_cap) * ((i + 0.5) / n)
    r_xy = np.sqrt(np.maximum(0.0, 1.0 - z * z))

    golden = (1.0 + np.sqrt(5.0)) * 0.5
    lon = 2.0 * np.pi * (i + 0.5) / golden

    x = r_xy * np.cos(lon)
    y = r_xy * np.sin(lon)
    dirs = np.stack([x, y, z], axis=1)
    norms = np.linalg.norm(dirs, axis=1, keepdims=True)
    norms = np.where(norms < 1e-12, 1.0, norms)
    return dirs / norms


def _any_orthonormal(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    idx = np.argmin(np.abs(v))
    basis = np.zeros(3, dtype=float)
    basis[idx] = 1.0
    u = basis - np.dot(basis, v) * v
    n = np.linalg.norm(u)
    return u / (n if n > 1e-12 else 1.0)
