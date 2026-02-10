#!/usr/bin/env python3
from __future__ import annotations

import random
import time
from pathlib import Path
from typing import Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped

from .grid_sweep_session import GridSweepSession


class Loop0Session(GridSweepSession):
    """
    Minimal closed-loop baseline using the existing Session API.
    No new interfaces; all commands are executed via run_sync on Session commands.
    """

    def run_loop0_cycle(
        self,
        *,
        scan_pose: Sequence[float],
        bbox_min: Sequence[float],
        bbox_max: Sequence[float],
        capture_folder: str,
        ground_offset_m: float,
        safety_offset_mm: float,
        motion_timeout_s: float,
        capture_timeout_s: float,
        reconstruct_timeout_s: float,
        print_timeout_s: float,
        print_steps: int,
        print_speed: int,
        reconstruct_points_frame: str = "ur20_base_link",
    ) -> bool:
        log = self.node.get_logger()

        # 1) Move to scan pose
        log.info("[Loop0] Moving to scan pose")
        try:
            res = self.run_sync(
                self.motion.goto(
                    x=float(scan_pose[0]),
                    y=float(scan_pose[1]),
                    z=float(scan_pose[2]),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=motion_timeout_s,
            )
        except TimeoutError:
            log.error(f"[Loop0] Move to scan pose timed out after {motion_timeout_s:.1f}s")
            return False
        if not res or not res.get("ok", False):
            log.error(f"[Loop0] Move to scan pose failed: {res.get('error') if res else 'no response'}")
            return False

        # 2) Capture via shared GridSweepSession library flow (2x2 over 20cm x 20cm)
        sweep_targets = self.run_grid_sweep(
            width=0.20,
            height=0.20,
            center_x=0.0,
            center_y=0.75,
            center_z=0.0,
            z_off=float(scan_pose[2]),
            nx=2,
            ny=2,
            debug=False,
            capture_folder=str(capture_folder),
            eef_link="femto_color_optical_calib",
            use_tf_orientation=True,
            do_home=False,
            timeout_s=max(float(motion_timeout_s), float(capture_timeout_s)),
            publish_markers=True,
            clear_markers_before=True,
        )
        log.info(f"[Loop0] Grid sweep capture done. Targets traversed: {len(sweep_targets)}")
        if len(sweep_targets) <= 0:
            log.error("[Loop0] Grid sweep capture failed; aborting cycle.")
            return False

        # 3) Reconstruction pipeline
        scan_folder = self._scan_folder_from_capture_folder(capture_folder)
        session_path = "@session"
        mesh_path = ""
        rgb_ply_path = ""
        c2d_wait_timeout_s = max(60.0, float(reconstruct_timeout_s))
        tsdf_wait_timeout_s = max(180.0, float(reconstruct_timeout_s))

        log.info(
            f"[Loop0] Reconstruction pipeline start: scan_folder='{scan_folder}' "
            f"timeout={reconstruct_timeout_s:.1f}s"
        )
        c2d_start_ts = time.time()
        try:
            c2d_res = self.run_sync(
                self.camera.reconstruct_color_to_depth(
                    use_latest=False,
                    session_path=session_path,
                    scan_folder=scan_folder,
                    visualize=False,
                    enqueue=False,
                ),
                timeout_s=reconstruct_timeout_s,
            )
        except TimeoutError:
            log.warn(f"[Loop0] color_to_depth request timed out after {reconstruct_timeout_s:.1f}s")
            c2d_res = None

        if not c2d_res or not c2d_res.get("ok", False):
            log.warn("[Loop0] color_to_depth failed; using bbox fallback for Z.")
        else:
            c2d_output = str(c2d_res.get("metrics", {}).get("output_path", "")).strip()
            if c2d_output:
                log.info(
                    f"[Loop0] Waiting for fresh color_to_depth outputs (timeout={c2d_wait_timeout_s:.1f}s)"
                )
                ready_c2d = self._wait_for_fresh_alignment_output(
                    output_path=c2d_output,
                    start_ts=c2d_start_ts,
                    timeout_s=c2d_wait_timeout_s,
                )
                if not ready_c2d:
                    log.warn("[Loop0] Fresh color_to_depth output not detected within wait timeout.")

            tsdf_start_ts = time.time()
            try:
                tsdf_res = self.run_sync(
                    self.camera.reconstruct_tsdf_cropped(
                        use_latest=False,
                        session_path=session_path,
                        scan_folder=scan_folder,
                        visualize=False,
                        device="CPU:0",
                        enqueue=False,
                    ),
                    timeout_s=reconstruct_timeout_s,
                )
            except TimeoutError:
                log.warn(f"[Loop0] tsdf_cropped request timed out after {reconstruct_timeout_s:.1f}s")
                tsdf_res = None

            if not tsdf_res or not tsdf_res.get("ok", False):
                log.warn("[Loop0] tsdf_cropped failed; using bbox fallback for Z.")
            else:
                mesh_path = str(tsdf_res.get("metrics", {}).get("mesh_path", "")).strip()
                rgb_ply_path = str(tsdf_res.get("metrics", {}).get("rgb_ply_path", "")).strip()
                if not rgb_ply_path:
                    rgb_ply_path = str(tsdf_res.get("metrics", {}).get("output_path", "")).strip()

                if mesh_path:
                    log.info(
                        f"[Loop0] Waiting for fresh TSDF mesh output (timeout={tsdf_wait_timeout_s:.1f}s)"
                    )
                    ready_mesh = self._wait_for_fresh_file_output(
                        output_path=mesh_path,
                        start_ts=tsdf_start_ts,
                        timeout_s=tsdf_wait_timeout_s,
                    )
                    if not ready_mesh:
                        log.warn("[Loop0] Fresh TSDF mesh not detected within wait timeout.")

                if rgb_ply_path:
                    log.info(
                        f"[Loop0] Waiting for fresh TSDF RGB PLY output (timeout={tsdf_wait_timeout_s:.1f}s)"
                    )
                    ready_ply = self._wait_for_fresh_file_output(
                        output_path=rgb_ply_path,
                        start_ts=tsdf_start_ts,
                        timeout_s=tsdf_wait_timeout_s,
                    )
                    if not ready_ply:
                        log.warn("[Loop0] Fresh TSDF RGB PLY not detected within wait timeout.")

                # Mirror custom_sequence behavior: update RViz mesh after TSDF.
                try:
                    mesh_res = self.run_sync(
                        self.camera.update_world_mesh(
                            use_latest=False,
                            session_path=session_path,
                            mesh_path=mesh_path,
                            ply_path=rgb_ply_path,
                            prefer="mesh",
                            wait_timeout_s=45.0,
                            enqueue=False,
                        ),
                        timeout_s=55.0,
                    )
                    if mesh_res and mesh_res.get("ok", False):
                        published = str(mesh_res.get("metrics", {}).get("published_path", "")).strip()
                        self.node.get_logger().info(f"[Loop0] World mesh updated: {published}")
                    else:
                        self.node.get_logger().warn(
                            f"[Loop0] World mesh update failed: "
                            f"{(mesh_res or {}).get('error', 'no response')}"
                        )
                except TimeoutError:
                    self.node.get_logger().warn("[Loop0] World mesh update request timed out.")

        # 4) Select candidate deposition point (random in bbox, near ground)
        sample_frame = str(reconstruct_points_frame or "world").strip() or "world"
        frame_tf = self._lookup_frame_transform(
            source_frame=sample_frame,
            target_frame="world",
        )
        if frame_tf is None and sample_frame != "world":
            log.warn(
                f"[Loop0] Could not resolve TF {sample_frame} -> world; "
                "sampling PLY in native coordinates."
            )

        xyz_from_ply = self._sample_point_from_ply(
            ply_path=rgb_ply_path,
            bbox_min=bbox_min,
            bbox_max=bbox_max,
            frame_tf=frame_tf,
        ) if rgb_ply_path else None

        if xyz_from_ply is not None:
            x, y, z_mesh = xyz_from_ply
            z_ground = float(z_mesh) + float(ground_offset_m)
            log.info(
                f"[Loop0] Sampled deposition candidate from PLY: "
                f"x={x:.3f} y={y:.3f} z_mesh={z_mesh:.3f}"
            )
        else:
            x, y, z_ground = self._select_point(bbox_min, bbox_max, ground_offset_m)
            if rgb_ply_path:
                z_est = self._estimate_z_from_ply_xy(
                    ply_path=rgb_ply_path,
                    x=float(x),
                    y=float(y),
                    radius_m=0.005,
                    min_neighbors=25,
                    frame_tf=frame_tf,
                )
                if z_est is not None:
                    z_ground = z_est + float(ground_offset_m)
                    log.info(f"[Loop0] Z estimated from PLY: z={z_est:.4f} -> z_ground={z_ground:.4f}")
                else:
                    log.warn("[Loop0] PLY Z estimate unavailable; using bbox fallback.")
        log.info(f"[Loop0] Selected point: {x:.3f} {y:.3f} {z_ground:.3f}")

        # 5) Safety offset (input in mm)
        safety_offset_m = max(0.0, float(safety_offset_mm)) / 1000.0
        z_target = z_ground + safety_offset_m

        # Publish selected deposition target marker in RViz.
        self._publish_print_target_marker(x=float(x), y=float(y), z=float(z_target))

        # Manual gate before the deposition move.
        self.run_sync(
            self.util.input(
                prompt="[Loop0] Press ENTER to continue to deposition point...",
                enqueue=False,
            )
        )

        # 6) Move to deposition pose
        self.run_sync(self.motion.setEef("extruder_tcp", enqueue=False))
        log.info("[Loop0] Moving to deposition pose")
        try:
            res = self.run_sync(
                self.motion.goto(
                    x=float(x),
                    y=float(y),
                    z=float(z_target),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=motion_timeout_s,
            )
        except TimeoutError:
            log.error(f"[Loop0] Move to deposition pose timed out after {motion_timeout_s:.1f}s")
            res = None
        if not res or not res.get("ok", False):
            log.warn("[Loop0] Deposit move failed; returning to scan pose.")
            try:
                self.run_sync(
                    self.motion.goto(
                        x=float(scan_pose[0]),
                        y=float(scan_pose[1]),
                        z=float(scan_pose[2]),
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=motion_timeout_s,
                )
            except TimeoutError:
                log.error(f"[Loop0] Return to scan pose timed out after {motion_timeout_s:.1f}s")
            return False

        # 7) Deposit (real or mock)
        self.run_sync(
            self.util.input(
                prompt="[Loop0] Press ENTER to start extrusion...",
                enqueue=False,
            )
        )
        log.info("[Loop0] Depositing (real or mock)")
        try:
            res = self.run_sync(
                self.extruder.print_steps(
                    steps=int(print_steps),
                    speed=int(print_speed),
                    enqueue=False,
                ),
                timeout_s=print_timeout_s,
            )
        except TimeoutError:
            log.error(f"[Loop0] Deposit timed out after {print_timeout_s:.1f}s")
            res = None
        if not res or not res.get("ok", False):
            log.warn("[Loop0] Deposit failed or unavailable; treated as mock/no-op.")

        # 8) Return to scan pose
        log.info("[Loop0] Returning to scan pose")
        try:
            self.run_sync(
                self.motion.goto(
                    x=float(scan_pose[0]),
                    y=float(scan_pose[1]),
                    z=float(scan_pose[2]),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=motion_timeout_s,
            )
        except TimeoutError:
            log.error(f"[Loop0] Return to scan pose timed out after {motion_timeout_s:.1f}s")

        # 9) Cycle complete
        log.info("[Loop0] Cycle complete")
        return True

    @staticmethod
    def _select_point(
        bbox_min: Sequence[float],
        bbox_max: Sequence[float],
        ground_offset_m: float,
    ) -> Tuple[float, float, float]:
        x = random.uniform(min(bbox_min[0], bbox_max[0]), max(bbox_min[0], bbox_max[0]))
        y = random.uniform(min(bbox_min[1], bbox_max[1]), max(bbox_min[1], bbox_max[1]))
        z_ground = min(bbox_min[2], bbox_max[2]) + float(ground_offset_m)
        return x, y, z_ground

    @staticmethod
    def _scan_folder_from_capture_folder(capture_folder: str) -> str:
        folder = str(capture_folder or "").strip()
        if folder.startswith("@session/"):
            sub = folder[len("@session/"):].strip("/")
            return sub if sub else "manual_caps"
        if folder in ("@session", "@"):
            return "manual_caps"
        cleaned = folder.strip("/")
        if "/" in cleaned:
            cleaned = cleaned.split("/")[-1]
        return cleaned or "manual_caps"

    def _wait_for_fresh_alignment_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ) -> bool:
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

        try:
            res = self.run_sync(
                self.util.wait_until(
                    predicate=_fresh_alignment_exists,
                    period_s=0.5,
                    timeout_s=timeout_s,
                    enqueue=False,
                ),
                timeout_s=timeout_s + 1.0,
            )
            return bool(res and res.get("ok", False))
        except TimeoutError:
            return False

    def _wait_for_fresh_file_output(
        self,
        *,
        output_path: str,
        start_ts: float,
        timeout_s: float,
    ) -> bool:
        out_path = Path(output_path)

        def _fresh_file_exists() -> bool:
            try:
                if not out_path.exists() or not out_path.is_file():
                    return False
                st = out_path.stat()
                if st.st_size <= 0:
                    return False
                if st.st_mtime < (start_ts - 0.25):
                    return False
                return (time.time() - st.st_mtime) >= 0.4
            except OSError:
                return False

        try:
            res = self.run_sync(
                self.util.wait_until(
                    predicate=_fresh_file_exists,
                    period_s=0.5,
                    timeout_s=timeout_s,
                    enqueue=False,
                ),
                timeout_s=timeout_s + 1.0,
            )
            return bool(res and res.get("ok", False))
        except TimeoutError:
            return False

    def _sample_point_from_ply(
        self,
        *,
        ply_path: str,
        bbox_min: Sequence[float],
        bbox_max: Sequence[float],
        frame_tf=None,
    ) -> Optional[Tuple[float, float, float]]:
        pts = self._load_points_from_ply(ply_path)
        if pts is None:
            return None

        try:
            import numpy as np
        except Exception:
            return None

        pts = self._transform_points_to_world(pts, frame_tf)

        x_min = float(min(bbox_min[0], bbox_max[0]))
        x_max = float(max(bbox_min[0], bbox_max[0]))
        y_min = float(min(bbox_min[1], bbox_max[1]))
        y_max = float(max(bbox_min[1], bbox_max[1]))

        mask = (
            (pts[:, 0] >= x_min)
            & (pts[:, 0] <= x_max)
            & (pts[:, 1] >= y_min)
            & (pts[:, 1] <= y_max)
        )
        valid = pts[mask] if bool(np.any(mask)) else pts
        if valid.size == 0:
            return None

        idx = random.randrange(valid.shape[0])
        chosen = valid[idx]
        return (float(chosen[0]), float(chosen[1]), float(chosen[2]))

    def _estimate_z_from_ply_xy(
        self,
        *,
        ply_path: str,
        x: float,
        y: float,
        radius_m: float,
        min_neighbors: int,
        frame_tf=None,
    ) -> Optional[float]:
        try:
            import numpy as np
        except Exception:
            return None

        pts = self._load_points_from_ply(ply_path)
        if pts is None:
            return None

        pts = self._transform_points_to_world(pts, frame_tf)

        dx = pts[:, 0] - float(x)
        dy = pts[:, 1] - float(y)
        radius = max(1e-6, float(radius_m))
        r2 = radius * radius
        mask = (dx * dx + dy * dy) <= r2

        neighbors = int(np.count_nonzero(mask))
        if neighbors < int(min_neighbors):
            # One expansion pass before giving up.
            radius *= 1.7
            r2 = radius * radius
            mask = (dx * dx + dy * dy) <= r2
            neighbors = int(np.count_nonzero(mask))
            if neighbors < int(min_neighbors):
                return None

        z_vals = pts[mask, 2]
        if z_vals.size == 0:
            return None

        z_med = float(np.median(z_vals))
        abs_dev = np.abs(z_vals - z_med)
        mad = float(np.median(abs_dev))
        if mad > 1e-9:
            sigma = 1.4826 * mad
            keep = abs_dev <= (2.5 * sigma)
            if int(np.count_nonzero(keep)) >= max(8, min_neighbors // 2):
                z_vals = z_vals[keep]

        return float(np.median(z_vals))

    def _load_points_from_ply(self, ply_path: str):
        try:
            import numpy as np
            import open3d as o3d
        except Exception as exc:
            self.node.get_logger().warn(f"[Loop0] Open3D/Numpy import failed for PLY read: {exc}")
            return None

        p = Path(ply_path).expanduser()
        if not p.is_file():
            return None

        for _ in range(6):
            pcd = o3d.io.read_point_cloud(str(p))
            arr = np.asarray(pcd.points)
            if arr.size > 0:
                return arr
            time.sleep(0.25)
        return None

    def _publish_print_target_marker(self, *, x: float, y: float, z: float) -> None:
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0

        try:
            self.run_sync(
                self.util.publish_targets(
                    [ps],
                    axis_length=0.07,
                    axis_radius=0.004,
                    clear_before=True,
                    enqueue=False,
                ),
                timeout_s=2.0,
            )
        except TimeoutError:
            self.node.get_logger().warn("[Loop0] publish_targets timed out for print target marker.")

    def _lookup_frame_transform(self, *, source_frame: str, target_frame: str):
        if not source_frame or source_frame == target_frame:
            return None
        try:
            import numpy as np
        except Exception:
            return None

        try:
            res = self.run_sync(
                self.camera.get_pose(
                    eef=str(source_frame),
                    base_frame=str(target_frame),
                    use_tf=True,
                    enqueue=False,
                ),
                timeout_s=2.0,
            )
        except TimeoutError:
            return None

        if not res or not res.get("ok", False) or "pose" not in res:
            return None

        q = res["pose"].pose.orientation
        p = res["pose"].pose.position
        rot = self._quat_to_rot(float(q.x), float(q.y), float(q.z), float(q.w))
        trans = np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float64)

        self.node.get_logger().info(
            f"[Loop0] Using TF transform for PLY points: {source_frame} -> {target_frame}"
        )
        return (rot, trans)

    @staticmethod
    def _quat_to_rot(qx: float, qy: float, qz: float, qw: float):
        import numpy as np

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _transform_points_to_world(pts, frame_tf):
        if frame_tf is None:
            return pts
        rot, trans = frame_tf
        return (pts @ rot.T) + trans
