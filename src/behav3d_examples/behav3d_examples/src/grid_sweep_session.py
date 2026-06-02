#!/usr/bin/env python3
from __future__ import annotations

from typing import List, Optional

from geometry_msgs.msg import PoseStamped

from behav3d_commands.session import Session
from behav3d_utils import target_builder as tb


class GridSweepSession(Session):
    """
    Grid sweep session using current behav3d_commands semantics.
    - Prefer LIN motions
    - Use plan + exec for each target
    - Capture only after a successful exec
    - Optional debug prompt before capture
    """

    def run_grid_sweep(
        self,
        *,
        target: Optional[PoseStamped] = None,
        width: float = 1.0,
        height: float = 0.5,
        center_x: float = 0.0,
        center_y: float = 0.80,
        center_z: float = 20.0,
        z_off: float = 0.40,
        nx: int = 12,
        ny: int = 5,
        row_major: bool = False,
        frame_id: str = "world",
        eef_link: Optional[str] = "femto_color_optical_calib",
        use_tf_orientation: bool = True,
        debug: bool = False,
        capture_folder: Optional[str] = "@session/grid_sweep",
        do_home: bool = True,
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        timeout_s: Optional[float] = None,
        publish_markers: bool = True,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
    ) -> List[PoseStamped]:
        log = self.node.get_logger()

        if nx < 2 or ny < 2:
            log.warn("[grid_sweep] nx/ny must be >= 2. No targets generated.")
            return []

        if do_home:
            self.run_sync(self.motion.home(enqueue=False))

        # Prefer LIN for the sweep
        self.run_sync(self.motion.setLIN(enqueue=False))
        if eef_link:
            self.run_sync(self.motion.setEef(eef_link, enqueue=False))

        if target is not None:
            if not isinstance(target, PoseStamped):
                raise TypeError("target must be geometry_msgs.msg.PoseStamped")
            if target.header.frame_id not in (frame_id, "", None):
                log.warn("[grid_sweep] target frame_id differs; using target as-is.")
            center = target
        else:
            center = tb.worldXY(center_x, center_y, center_z, frame_id)

        if target is None and use_tf_orientation and eef_link:
            try:
                pose_res = self.run_sync(
                    self.camera.get_pose(
                        eef=eef_link,
                        base_frame=frame_id,
                        use_tf=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if pose_res.get("ok", False) and "pose" in pose_res:
                    center = tb.setTargetOrigin(pose_res["pose"], center_x, center_y, center_z)
                else:
                    log.warn("[grid_sweep] Using default orientation (TF pose not available).")
            except TimeoutError:
                log.warn("[grid_sweep] Using default orientation (TF lookup timed out).")
        targets = self._sweep_zigzag(
            center=center,
            width=float(width),
            height=float(height),
            z_off=float(z_off),
            nx=int(nx),
            ny=int(ny),
            row_major=bool(row_major),
        )

        if not targets:
            log.warn("[grid_sweep] No targets generated.")
            return []

        if publish_markers:
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
                log.warn("[grid_sweep] publish_targets timed out; continuing without markers.")

        for i, ps in enumerate(targets):
            plan_res = self.run_sync(
                self.motion.plan(
                    pose=ps,
                    motion="LIN",
                    vel_scale=vel_scale,
                    accel_scale=accel_scale,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not plan_res.get("ok", False):
                log.warn(f"[grid_sweep] Plan failed at target {i}; skipping.")
                continue

            exec_res = self.run_sync(self.motion.exec(enqueue=False), timeout_s=timeout_s)
            if not exec_res.get("ok", False):
                log.warn(f"[grid_sweep] Exec failed at target {i}; skipping capture.")
                continue

            if debug:
                self.run_sync(
                    self.util.input(
                        prompt=f"Press ENTER to capture at target {i}...",
                        enqueue=False,
                    )
                )

            self.run_sync(
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

            self.run_sync(
                self.util.wait(0.5, enqueue=False),
                timeout_s=timeout_s,
            )

        if publish_markers:
            try:
                self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
            except TimeoutError:
                log.warn("[grid_sweep] delete_markers timed out.")

        return targets

    @staticmethod
    def _sweep_zigzag(
        *,
        center: PoseStamped,
        width: float,
        height: float,
        z_off: float,
        nx: int,
        ny: int,
        row_major: bool,
    ) -> List[PoseStamped]:
        out: List[PoseStamped] = []

        dx = width / float(nx - 1)
        dy = height / float(ny - 1)

        cx = float(center.pose.position.x)
        cy = float(center.pose.position.y)
        cz = float(center.pose.position.z)
        qx = float(center.pose.orientation.x)
        qy = float(center.pose.orientation.y)
        qz = float(center.pose.orientation.z)
        qw = float(center.pose.orientation.w)

        for j in range(ny):
            row: List[PoseStamped] = []
            for i in range(nx):
                x = -0.5 * width + i * dx
                y = -0.5 * height + j * dy
                # Keep XYZ in world frame; keep orientation from the EEF.
                row.append(
                    tb.poseStamped(
                        cx + x,
                        cy + y,
                        cz + float(z_off),
                        qx,
                        qy,
                        qz,
                        qw,
                        center.header.frame_id,
                    )
                )

            if not row_major and (j % 2 == 1):
                row.reverse()

            out.extend(row)

        return out
