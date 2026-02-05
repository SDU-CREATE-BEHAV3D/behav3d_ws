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
        width: float = 1.0,
        height: float = 0.5,
        center_x: float = 0.0,
        center_y: float = 0.75,
        center_z: float = 0.0,
        z_off: float = 0.75,
        nx: int = 10,
        ny: int = 5,
        row_major: bool = False,
        frame_id: str = "world",
        eef_link: Optional[str] = "femto_ir_optical_calib",
        use_tf_orientation: bool = True,
        debug: bool = False,
        capture_folder: Optional[str] = "@session/grid_sweep",
        do_home: bool = True,
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        timeout_s: Optional[float] = None,
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

        center = tb.worldXY(center_x, center_y, center_z, frame_id)
        if use_tf_orientation and eef_link:
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

        # Canonical local pose (mirrors C++ flipTargetAxes(worldXY(...), false, true))
        p_base_local = tb.flipTargetAxes(tb.worldXY(0.0, 0.0, 0.0, center.header.frame_id), False, True)

        dx = width / float(nx - 1)
        dy = height / float(ny - 1)

        for j in range(ny):
            row: List[PoseStamped] = []
            for i in range(nx):
                x = -0.5 * width + i * dx
                y = -0.5 * height + j * dy
                p_local = tb.translate(p_base_local, [x, y, -float(z_off)])
                row.append(tb.changeBasis(center, p_local))

            if not row_major and (j % 2 == 1):
                row.reverse()

            out.extend(row)

        return out
