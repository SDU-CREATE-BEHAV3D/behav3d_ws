#!/usr/bin/env python3
from __future__ import annotations

import random
from typing import Sequence, Tuple

from behav3d_commands.session import Session


class Loop0Session(Session):
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
        safety_offset_m: float,
        motion_timeout_s: float,
        capture_timeout_s: float,
        reconstruct_timeout_s: float,
        print_timeout_s: float,
        print_steps: int,
        print_speed: int,
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

        # 2) Capture
        log.info("[Loop0] Triggering capture")
        try:
            res = self.run_sync(
                self.camera.capture(
                    rgb=True,
                    depth=True,
                    ir=True,
                    pose=True,
                    folder=str(capture_folder),
                    enqueue=False,
                ),
                timeout_s=capture_timeout_s,
            )
        except TimeoutError:
            log.error(f"[Loop0] Capture timed out after {capture_timeout_s:.1f}s")
            return False
        if not res or not res.get("ok", False):
            log.error(f"[Loop0] Capture failed: {res.get('error') if res else 'no response'}")
            return False

        # 3) Reconstruct / wait for geometry
        log.info("[Loop0] Waiting for reconstruction")
        try:
            res = self.run_sync(
                self.camera.reconstruct(use_latest=True, session_path="", enqueue=False),
                timeout_s=reconstruct_timeout_s,
            )
        except TimeoutError:
            log.error(f"[Loop0] Reconstruct timed out after {reconstruct_timeout_s:.1f}s")
            res = None
        if res and res.get("ok", False):
            log.info("[Loop0] Geometry received")
        else:
            log.warn("[Loop0] Geometry not available; continuing with placeholder.")

        # 4) Select candidate deposition point (random in bbox, near ground)
        x, y, z_ground = self._select_point(bbox_min, bbox_max, ground_offset_m)
        log.info(f"[Loop0] Selected point: {x:.3f} {y:.3f} {z_ground:.3f}")

        # 5) Safety offset
        z_target = z_ground + float(safety_offset_m)

        # 6) Move to deposition pose
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
