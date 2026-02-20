#!/usr/bin/env python3
# Demo: blocking Fibonacci scan cycles with run_sync and reconstruction updates.

from __future__ import annotations

import math
import os
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from .src.custom_session import MySession


TOOL_PLUS_Z_POINTS_OUTWARD = False


class FibReconstructSession(MySession):
    """
    Combines:
    - Blocking Fibonacci scan orchestration (plan + exec per viewpoint)
    - Reconstruction/world-mesh helpers from MySession
    """

    def run_fib_scan_blocking(
        self,
        *,
        target: PoseStamped,
        distance: float,
        cap_rad: float,
        samples: int,
        capture_folder: Optional[str],
        order_start_xyz: Optional[List[float]] = None,
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
    ) -> Dict[str, Any]:
        log = self.node.get_logger()

        if not isinstance(target, PoseStamped):
            raise TypeError("target must be geometry_msgs.msg.PoseStamped in 'world'.")
        if target.header.frame_id not in ("world", "", None):
            raise ValueError("target.header.frame_id must be 'world'.")

        targets = self._build_fib_targets(
            target=target,
            distance=float(distance),
            cap_rad=float(cap_rad),
            samples=int(samples),
            z_jitter=float(z_jitter),
        )
        targets = self._order_targets_by_nearest_neighbor(
            targets=targets,
            order_start_xyz=order_start_xyz,
        )
        if not targets:
            return {
                "ok": True,
                "targets": [],
                "plan_ok": 0,
                "exec_ok": 0,
                "captures_ok": 0,
            }

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
                log.warn("[fib_scan_blocking] publish_targets timed out; continuing.")

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
                log.warn(
                    f"[fib_scan_blocking] Plan timed out at viewpoint {i} "
                    f"(timeout_s={timeout_s}); skipping."
                )
                continue
            if not plan_res.get("ok", False):
                log.warn(
                    f"[fib_scan_blocking] Plan failed at viewpoint {i}; "
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
                log.warn(
                    f"[fib_scan_blocking] Exec timed out at viewpoint {i} "
                    f"(timeout_s={timeout_s}); skipping capture."
                )
                continue
            if not exec_res.get("ok", False):
                log.warn(f"[fib_scan_blocking] Exec failed at viewpoint {i}; skipping capture.")
                continue
            exec_ok += 1

            if settle_s > 0.0:
                try:
                    self.run_sync(
                        self.util.wait(float(settle_s), enqueue=False),
                        timeout_s=timeout_s,
                    )
                except TimeoutError:
                    log.warn(
                        f"[fib_scan_blocking] Settle wait timed out at viewpoint {i} "
                        f"(timeout_s={timeout_s}); continuing."
                    )

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
                log.warn(
                    f"[fib_scan_blocking] Capture timed out at viewpoint {i} "
                    f"(timeout_s={timeout_s}); skipping."
                )
                continue
            if not cap_res.get("ok", False):
                log.warn(f"[fib_scan_blocking] Capture failed at viewpoint {i}.")
                continue
            captures_ok += 1

        if publish_targets and clear_markers_after:
            try:
                self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
            except TimeoutError:
                log.warn("[fib_scan_blocking] delete_markers timed out.")

        return {
            "ok": True,
            "targets": targets,
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
    def _order_targets_by_nearest_neighbor(
        *,
        targets: List[PoseStamped],
        order_start_xyz: Optional[List[float]] = None,
    ) -> List[PoseStamped]:
        if len(targets) <= 2:
            return targets

        points = np.array(
            [
                [ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
                for ps in targets
            ],
            dtype=float,
        )
        remaining = set(range(len(targets)))
        ordered_indices: List[int] = []

        if order_start_xyz is not None and len(order_start_xyz) == 3:
            start = np.asarray(order_start_xyz, dtype=float).reshape(3)
            d2 = np.sum((points - start) ** 2, axis=1)
            current_idx = int(np.argmin(d2))
        else:
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


class FibScanSequenceDemo(Node):
    def __init__(self):
        super().__init__("fib_scan_sequence")

        self.declare_parameter("iterations", -1)  # -1 = forever
        self.declare_parameter("do_home", True)
        self.declare_parameter("motion_mode", "LIN")
        self.declare_parameter("eef_link", "femto_color_optical_calib")
        self.declare_parameter("vel_scale", 0.05)
        self.declare_parameter("accel_scale", 0.05)

        self.declare_parameter("target_xyz", [0.0, 0.80, 0.15])
        self.declare_parameter("target_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("use_tf_target_orientation", False)

        self.declare_parameter("distance_m", 0.55)
        self.declare_parameter("cap_rad_deg", 65.0)
        self.declare_parameter("samples", 16)
        self.declare_parameter("settle_s", 0.3)
        self.declare_parameter("z_jitter_m", 0.02)
        self.declare_parameter("debug", False)
        self.declare_parameter("capture_prompt", "")
        self.declare_parameter("publish_targets", True)
        self.declare_parameter("axis_length", 0.05)
        self.declare_parameter("axis_radius", 0.003)
        self.declare_parameter("clear_markers_before", True)
        self.declare_parameter("clear_markers_after", False)
        self.declare_parameter("scan_step_timeout_s", 30.0)
        self.declare_parameter("auto_convert_mm", True)

        self.declare_parameter("scan_folder_prefix", "fib_scan")
        self.declare_parameter("ask_continue", True)
        self.declare_parameter(
            "continue_prompt",
            "Type 'c' + ENTER to capture another Fibonacci scan, or ENTER to stop.",
        )
        self.declare_parameter("continue_key", "c")

        self.declare_parameter("run_reconstruction", True)
        self.declare_parameter("reconstruct_visualize", False)
        self.declare_parameter("reconstruct_device", "CPU:0")
        self.declare_parameter("reconstruct_request_timeout_s", 10.0)
        self.declare_parameter("color_to_depth_wait_timeout_s", 60.0)
        self.declare_parameter("tsdf_wait_timeout_s", 180.0)
        self.declare_parameter("mesh_update_wait_timeout_s", 45.0)
        self.declare_parameter("mesh_update_request_timeout_s", 55.0)
        self.declare_parameter("mesh_prefer", "mesh")

        self.session = FibReconstructSession(self)
        self._folder_fallback_idx = 0
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        do_home = bool(self.get_parameter("do_home").value)
        motion_mode = str(self.get_parameter("motion_mode").value).strip().upper()
        if motion_mode not in ("PTP", "LIN"):
            log.warn(f"Invalid motion_mode='{motion_mode}'. Falling back to LIN.")
            motion_mode = "LIN"

        if do_home:
            self.session.run_sync(self.session.motion.home(enqueue=False))
        self.session.run_sync(
            self.session.motion.setSpd(float(self.get_parameter("vel_scale").value), enqueue=False)
        )
        self.session.run_sync(
            self.session.motion.setAcc(float(self.get_parameter("accel_scale").value), enqueue=False)
        )
        self.session.run_sync(
            self.session.motion.setEef(str(self.get_parameter("eef_link").value), enqueue=False)
        )
        if motion_mode == "LIN":
            self.session.run_sync(self.session.motion.setLIN(enqueue=False))
        else:
            self.session.run_sync(self.session.motion.setPTP(enqueue=False))

        log.info("Starting loop: Fibonacci scan -> optional reconstruction -> optional continue prompt.")

        iterations = int(self.get_parameter("iterations").value)
        cycle = 0

        while rclpy.ok() and (iterations < 0 or cycle < iterations):
            cycle += 1

            scan_folder_prefix = str(self.get_parameter("scan_folder_prefix").value).strip() or "fib_scan"
            scan_folder = self._next_scan_folder_name(scan_folder_prefix)
            capture_folder = f"@session/{scan_folder}"

            target_xyz = self._param_list("target_xyz", 3)
            target_rpy_deg = self._param_list("target_rpy_deg", 3)

            if bool(self.get_parameter("auto_convert_mm").value):
                target_xyz = self._coerce_xyz_m("target_xyz", target_xyz)

            distance_m = self._coerce_length_m(
                "distance_m",
                float(self.get_parameter("distance_m").value),
            )
            z_jitter_m = self._coerce_length_m(
                "z_jitter_m",
                float(self.get_parameter("z_jitter_m").value),
            )
            scan_step_timeout_s = float(self.get_parameter("scan_step_timeout_s").value)

            target_ps = self._target_pose_world(target_xyz, target_rpy_deg)
            if bool(self.get_parameter("use_tf_target_orientation").value):
                try:
                    pose_res = self.session.run_sync(
                        self.session.camera.get_pose(
                            eef=str(self.get_parameter("eef_link").value),
                            base_frame="world",
                            use_tf=True,
                            enqueue=False,
                        ),
                        timeout_s=scan_step_timeout_s,
                    )
                    if pose_res.get("ok", False) and "pose" in pose_res:
                        tf_pose: PoseStamped = pose_res["pose"]
                        target_ps.pose.orientation = tf_pose.pose.orientation
                    else:
                        log.warn(f"[cycle {cycle}] TF orientation not available; using target_rpy_deg.")
                except TimeoutError:
                    log.warn(f"[cycle {cycle}] TF orientation lookup timed out; using target_rpy_deg.")

            order_start_xyz = self._maybe_current_eef_xyz(timeout_s=scan_step_timeout_s)
            if order_start_xyz is not None:
                log.info(
                    f"[cycle {cycle}] Nearest-neighbor ordering start xyz(m)={order_start_xyz}"
                )

            capture_prompt = str(self.get_parameter("capture_prompt").value).strip() or None

            log.info(f"[cycle {cycle}] Running Fibonacci scan in folder: {capture_folder}")
            log.info(
                f"[cycle {cycle}] target_xyz(m)={target_xyz} distance_m={distance_m:.4f} "
                f"cap_rad_deg={float(self.get_parameter('cap_rad_deg').value):.3f} "
                f"samples={int(self.get_parameter('samples').value)}"
            )

            scan_res = self.session.run_fib_scan_blocking(
                target=target_ps,
                distance=distance_m,
                cap_rad=math.radians(float(self.get_parameter("cap_rad_deg").value)),
                samples=int(self.get_parameter("samples").value),
                capture_folder=capture_folder,
                order_start_xyz=order_start_xyz,
                settle_s=float(self.get_parameter("settle_s").value),
                prompt=capture_prompt,
                debug=bool(self.get_parameter("debug").value),
                z_jitter=z_jitter_m,
                motion=motion_mode,
                publish_targets=bool(self.get_parameter("publish_targets").value),
                axis_length=float(self.get_parameter("axis_length").value),
                axis_radius=float(self.get_parameter("axis_radius").value),
                clear_markers_before=bool(self.get_parameter("clear_markers_before").value),
                clear_markers_after=bool(self.get_parameter("clear_markers_after").value),
                timeout_s=scan_step_timeout_s,
            )

            targets = scan_res.get("targets", [])
            if targets:
                z_vals = [float(ps.pose.position.z) for ps in targets]
                log.info(
                    f"[cycle {cycle}] target z-range for scan: [{min(z_vals):.3f}, {max(z_vals):.3f}]"
                )
                if min(z_vals) < -0.10:
                    log.warn(
                        f"[cycle {cycle}] Some scan targets are below z=-0.10 m. "
                        "If planning fails, disable TF target orientation or adjust target pose/distance."
                    )

            captures_ok = int(scan_res.get("captures_ok", 0))
            exec_ok = int(scan_res.get("exec_ok", 0))
            plan_ok = int(scan_res.get("plan_ok", 0))
            requested = len(targets)
            log.info(
                f"[cycle {cycle}] Fibonacci scan completed: requested={requested} "
                f"plan_ok={plan_ok} exec_ok={exec_ok} captures_ok={captures_ok}"
            )

            if captures_ok <= 0:
                log.error(f"[cycle {cycle}] No captures succeeded; aborting.")
                rclpy.shutdown()
                return

            if bool(self.get_parameter("run_reconstruction").value):
                req_timeout_s = float(self.get_parameter("reconstruct_request_timeout_s").value)

                try:
                    c2d_res = self.session.run_color_to_depth_reconstruct(
                        use_latest=True,
                        session_path="@session",
                        scan_folder=scan_folder,
                        visualize=bool(self.get_parameter("reconstruct_visualize").value),
                        timeout_s=req_timeout_s,
                        wait_for_outputs=True,
                        wait_timeout_s=float(self.get_parameter("color_to_depth_wait_timeout_s").value),
                    )
                except TimeoutError:
                    log.error(f"[cycle {cycle}] color_to_depth request timed out.")
                    rclpy.shutdown()
                    return

                if not c2d_res.get("ok", False):
                    log.error(f"[cycle {cycle}] color_to_depth failed: {c2d_res.get('error')}")
                    rclpy.shutdown()
                    return

                try:
                    tsdf_res = self.session.run_tsdf_cropped_reconstruct(
                        use_latest=True,
                        session_path="@session",
                        scan_folder=scan_folder,
                        visualize=bool(self.get_parameter("reconstruct_visualize").value),
                        device=str(self.get_parameter("reconstruct_device").value),
                        timeout_s=req_timeout_s,
                        wait_for_outputs=True,
                        wait_timeout_s=float(self.get_parameter("tsdf_wait_timeout_s").value),
                    )
                except TimeoutError:
                    log.error(f"[cycle {cycle}] tsdf_cropped request timed out.")
                    rclpy.shutdown()
                    return

                if not tsdf_res.get("ok", False):
                    log.error(f"[cycle {cycle}] tsdf_cropped failed: {tsdf_res.get('error')}")
                    rclpy.shutdown()
                    return

                mesh_path = str(tsdf_res.get("metrics", {}).get("mesh_path", "")).strip()
                rgb_ply_path = str(tsdf_res.get("metrics", {}).get("rgb_ply_path", "")).strip()

                try:
                    mesh_res = self.session.run_update_world_mesh(
                        use_latest=True,
                        session_path="@session",
                        mesh_path=mesh_path,
                        ply_path=rgb_ply_path,
                        prefer=str(self.get_parameter("mesh_prefer").value),
                        wait_timeout_s=float(self.get_parameter("mesh_update_wait_timeout_s").value),
                        timeout_s=float(self.get_parameter("mesh_update_request_timeout_s").value),
                    )
                except TimeoutError:
                    log.error(f"[cycle {cycle}] update_world_mesh request timed out.")
                    rclpy.shutdown()
                    return

                if not mesh_res.get("ok", False):
                    log.error(f"[cycle {cycle}] update_world_mesh failed: {mesh_res.get('error')}")
                    rclpy.shutdown()
                    return

                published_path = mesh_res.get("metrics", {}).get("published_path", "")
                published_kind = mesh_res.get("metrics", {}).get("published_kind", "")
                log.info(f"[cycle {cycle}] World mesh updated ({published_kind}): {published_path}")

            log.info(f"[cycle {cycle}] Pipeline completed for scan folder '{scan_folder}'.")

            if bool(self.get_parameter("ask_continue").value) and (
                iterations < 0 or cycle < iterations
            ):
                if not self._ask_continue():
                    break

        rclpy.shutdown()

    def _ask_continue(self) -> bool:
        prompt = str(self.get_parameter("continue_prompt").value)
        continue_key = str(self.get_parameter("continue_key").value).strip().lower()
        res = self.session.run_sync(
            self.session.util.input(
                prompt=prompt,
                enqueue=False,
            )
        )
        value = str(res.get("metrics", {}).get("value", "")).strip().lower()
        return bool(continue_key) and value == continue_key

    def _next_scan_folder_name(self, prefix: str) -> str:
        session_dir = self._resolve_active_session_dir()
        if session_dir is not None and session_dir.exists():
            idx = 0
            while True:
                name = f"{prefix}_{idx:02d}"
                if not (session_dir / name).exists():
                    return name
                idx += 1

        name = f"{prefix}_{self._folder_fallback_idx:02d}"
        self._folder_fallback_idx += 1
        return name

    def _param_list(self, name: str, length: int) -> List[float]:
        val = self.get_parameter(name).get_parameter_value()
        if val.string_array_value:
            arr = [float(x) for x in val.string_array_value]
        elif val.double_array_value:
            arr = [float(x) for x in val.double_array_value]
        elif val.integer_array_value:
            arr = [float(x) for x in val.integer_array_value]
        else:
            raise ValueError(f"Parameter '{name}' must be an array with {length} values.")
        if len(arr) != length:
            raise ValueError(f"Parameter '{name}' must have {length} values.")
        return arr

    def _coerce_xyz_m(self, name: str, xyz: List[float]) -> List[float]:
        max_abs = max(abs(v) for v in xyz)
        if max_abs > 5.0:
            scaled = [v / 1000.0 for v in xyz]
            self.get_logger().warn(
                f"Parameter '{name}' looks like millimeters ({xyz}); using meters {scaled}."
            )
            return scaled
        return xyz

    def _coerce_length_m(self, name: str, value: float) -> float:
        if abs(value) > 5.0:
            scaled = value / 1000.0
            self.get_logger().warn(
                f"Parameter '{name}' looks like millimeters ({value}); using meters {scaled}."
            )
            return scaled
        return value

    @staticmethod
    def _target_pose_world(target_xyz: List[float], target_rpy_deg: List[float]) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = float(target_xyz[0])
        ps.pose.position.y = float(target_xyz[1])
        ps.pose.position.z = float(target_xyz[2])

        qx, qy, qz, qw = R.from_euler("xyz", target_rpy_deg, degrees=True).as_quat()
        ps.pose.orientation.x = float(qx)
        ps.pose.orientation.y = float(qy)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        return ps

    @staticmethod
    def _resolve_active_session_dir() -> Optional[Path]:
        active = os.environ.get("BEHAV3D_ACTIVE_SESSION", "").strip()
        if active:
            p = Path(active).expanduser()
            if p.exists():
                return p.resolve()

        root_env = os.environ.get("BEHAV3D_CAPTURES_ROOT", "").strip()
        captures_root = (
            Path(root_env).expanduser() if root_env else (Path.home() / "behav3d_ws" / "captures")
        )
        if not captures_root.exists():
            return None

        subdirs = [d for d in captures_root.iterdir() if d.is_dir()]
        if not subdirs:
            return None
        return max(subdirs, key=lambda d: d.stat().st_mtime).resolve()

    def _maybe_current_eef_xyz(self, *, timeout_s: Optional[float]) -> Optional[List[float]]:
        try:
            pose_res = self.session.run_sync(
                self.session.camera.get_pose(
                    eef=str(self.get_parameter("eef_link").value),
                    base_frame="world",
                    use_tf=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
        except TimeoutError:
            return None

        if not pose_res.get("ok", False) or "pose" not in pose_res:
            return None

        ps: PoseStamped = pose_res["pose"]
        return [
            float(ps.pose.position.x),
            float(ps.pose.position.y),
            float(ps.pose.position.z),
        ]


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FibScanSequenceDemo())


if __name__ == "__main__":
    main()


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
