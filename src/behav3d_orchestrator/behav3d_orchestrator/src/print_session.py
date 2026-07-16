#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from typing import Callable, Optional, Sequence

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

from .yaml_session import TargetSegment, YamlSession


class PrintSession(YamlSession):
    """
    Shared print execution helpers for parsed YAML targets.
    """

    def run_print_dots_targets(
        self,
        *,
        targets: Sequence[PoseStamped],
        dot_steps: int = 11000,
        dot_speed: int = 1200,
        approach_z_offset_m: float = 0.40,
        dot_z_offset_m: float = 0.04,
        pre_dot_vel_scale: float = 0.10,
        dot_vel_scale: float = 0.05,
        accel_scale: float = 0.05,
        dwell_s: float = 0.4,
        post_dot_retract_steps: int = 0,
        post_dot_retract_speed: int = 0,
        eef_link: str = "extruder_tcp",
        timeout_s: Optional[float] = None,
        pause_gate: Optional[Callable[[], None]] = None,
        publish_markers: bool = False,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
    ) -> dict:
        log = self.node.get_logger()
        target_list = list(targets)
        if len(target_list) < 1:
            return {"ok": False, "stage": "parse_targets", "error": "Need at least 1 target."}

        def _pause() -> None:
            if callable(pause_gate):
                pause_gate()

        if publish_markers:
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

        try:
            self.run_sync(self.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            self.run_sync(self.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)
            self.run_sync(self.motion.setSpd(float(dot_vel_scale), enqueue=False), timeout_s=timeout_s)
            if str(eef_link).strip():
                self.run_sync(self.motion.setEef(str(eef_link).strip(), enqueue=False), timeout_s=timeout_s)

            _pause()

            first_approach = self._with_z_offset(target_list[0], float(approach_z_offset_m))
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
            _pause()
            if not res.get("ok", False):
                log.warn(
                    "[print_session:dots] Failed initial approach; continuing target-by-target. "
                    f"error='{res.get('error', 'unknown')}'"
                )

            printed = 0
            skipped = 0
            failed_targets: list[dict] = []
            printed_targets: list[PoseStamped] = []
            for i, target in enumerate(target_list):
                _pause()

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
                _pause()
                if not res.get("ok", False):
                    skipped += 1
                    err = str(res.get("error", "unknown"))
                    failed_targets.append({"index": int(i), "stage": f"goto_above_{i}", "error": err})
                    log.warn(f"[print_session:dots] Skip target {i}: goto_above failed. error='{err}'")
                    continue

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
                _pause()
                if not res.get("ok", False):
                    skipped += 1
                    err = str(res.get("error", "unknown"))
                    failed_targets.append({"index": int(i), "stage": f"goto_target_{i}", "error": err})
                    log.warn(f"[print_session:dots] Skip target {i}: goto_target failed. error='{err}'")
                    continue

                res = self.run_sync(
                    self.extruder.print_steps(
                        steps=int(dot_steps),
                        speed=int(dot_speed),
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                _pause()
                if not res.get("ok", False):
                    skipped += 1
                    err = str(res.get("error", "unknown"))
                    failed_targets.append({"index": int(i), "stage": f"extrude_{i}", "error": err})
                    log.warn(f"[print_session:dots] Skip target {i}: extrude failed. error='{err}'")
                    try:
                        self.run_sync(
                            self.motion.goto(
                                pose=above,
                                motion="LIN",
                                vel_scale=float(dot_vel_scale),
                                exec=True,
                                enqueue=False,
                            ),
                            timeout_s=timeout_s,
                        )
                    except Exception:
                        pass
                    continue

                if int(post_dot_retract_steps) > 0 and int(post_dot_retract_speed) > 0:
                    res = self.run_sync(
                        self.extruder.print_steps(
                            steps=int(post_dot_retract_steps),
                            speed=int(post_dot_retract_speed),
                            reverse=True,
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                    _pause()
                    if not res.get("ok", False):
                        err = str(res.get("error", "unknown"))
                        failed_targets.append({"index": int(i), "stage": f"retract_{i}", "error": err})
                        log.warn(f"[print_session:dots] Target {i} printed but retract failed. error='{err}'")

                if float(dwell_s) > 0.0:
                    res = self.run_sync(self.util.wait(float(dwell_s), enqueue=False), timeout_s=timeout_s)
                    _pause()
                    if not res.get("ok", False):
                        log.warn(
                            f"[print_session:dots] Dwell wait failed at target {i}; continuing. "
                            f"error='{res.get('error', 'unknown')}'"
                        )

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
                _pause()
                if not res.get("ok", False):
                    err = str(res.get("error", "unknown"))
                    failed_targets.append({"index": int(i), "stage": f"lift_{i}", "error": err})
                    log.warn(f"[print_session:dots] Target {i} printed but lift failed. error='{err}'")

                printed += 1
                printed_targets.append(target)

            return {
                "ok": True,
                "stage": "done",
                "targets": len(target_list),
                "printed": printed,
                "skipped": skipped,
                "failed_targets": failed_targets,
                "printed_targets": printed_targets,
            }
        finally:
            if publish_markers:
                try:
                    self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    log.warn("[print_session:dots] delete_markers timed out.")

    def run_print_dots(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
        **kwargs,
    ) -> dict:
        targets = self.parse_yaml_targets(yaml_path=yaml_path, frame_id=frame_id)
        return self.run_print_dots_targets(targets=targets, **kwargs)

    def run_print_path_targets(
        self,
        *,
        targets: Sequence[PoseStamped],
        print_speed: int = 800,
        approach_z_offset_m: float = 0.40,
        approach_vel_scale: float = 0.10,
        print_vel_scale: float = 0.01,
        accel_scale: float = 0.05,
        eef_link: str = "extruder_tcp",
        timeout_s: Optional[float] = None,
        publish_markers: bool = False,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
    ) -> dict:
        log = self.node.get_logger()
        target_list = list(targets)
        if len(target_list) < 2:
            return {"ok": False, "stage": "parse_targets", "error": "Need at least 2 targets to print a path."}

        first = target_list[0]
        approach = self._with_z_offset(first, float(approach_z_offset_m))
        marker_targets = [approach] + target_list

        extruder_on = False
        try:
            self.run_sync(self.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            self.run_sync(self.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)
            self.run_sync(self.motion.setSpd(float(print_vel_scale), enqueue=False), timeout_s=timeout_s)
            if str(eef_link).strip():
                self.run_sync(self.motion.setEef(str(eef_link).strip(), enqueue=False), timeout_s=timeout_s)

            if publish_markers:
                self.run_sync(
                    self.util.publish_targets(
                        marker_targets,
                        axis_length=float(axis_length),
                        axis_radius=float(axis_radius),
                        clear_before=bool(clear_markers_before),
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )

            res = self.run_sync(
                self.motion.goto(
                    pose=approach,
                    motion="LIN",
                    vel_scale=float(approach_vel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": "approach_first", "error": res.get("error", "unknown")}

            res = self.run_sync(
                self.motion.goto(
                    pose=first,
                    motion="LIN",
                    vel_scale=float(print_vel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                return {"ok": False, "stage": "goto_first", "error": res.get("error", "unknown")}

            on_res = self.run_sync(
                self.extruder.setExtruder(True, speed=int(print_speed), enqueue=False),
                timeout_s=timeout_s,
            )
            if not on_res.get("ok", False):
                return {"ok": False, "stage": "extruder_on", "error": on_res.get("error", "unknown")}
            extruder_on = True

            for i in range(1, len(target_list)):
                ps = target_list[i]
                plan_res = self.run_sync(
                    self.motion.plan(
                        pose=ps,
                        motion="LIN",
                        vel_scale=float(print_vel_scale),
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not plan_res.get("ok", False):
                    return {"ok": False, "stage": f"plan_segment_{i}", "error": plan_res.get("error", "unknown")}

                exec_res = self.run_sync(self.motion.exec(enqueue=False), timeout_s=timeout_s)
                if not exec_res.get("ok", False):
                    return {"ok": False, "stage": f"exec_segment_{i}", "error": exec_res.get("error", "unknown")}

            off_res = self.run_sync(self.extruder.setExtruder(False, enqueue=False), timeout_s=timeout_s)
            extruder_on = False
            if not off_res.get("ok", False):
                return {"ok": False, "stage": "extruder_off", "error": off_res.get("error", "unknown")}

            return {"ok": True, "stage": "done", "targets": len(target_list)}
        finally:
            if extruder_on:
                try:
                    self.run_sync(self.extruder.setExtruder(False, enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    log.warn("[print_session:path] Failed to force extruder OFF after error/timeout.")
            if publish_markers:
                try:
                    self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    log.warn("[print_session:path] delete_markers timed out.")

    def run_print_segments(
        self,
        *,
        segments: Sequence[TargetSegment],
        print_speed: int = 800,
        approach_z_offset_m: float = 0.40,
        travel_z_offset_m: float = 0.04,
        approach_vel_scale: float = 0.10,
        travel_vel_scale: float = 0.10,
        print_vel_scale: float = 0.01,
        accel_scale: float = 0.05,
        target_print_speed_mm_s: float = 0.0,
        post_segment_wait_s: float = 0.0,
        post_segment_retract_s: float = 0.0,
        post_segment_retract_speed: int = 0,
        eef_link: str = "extruder_tcp",
        timeout_s: Optional[float] = None,
        publish_markers: bool = False,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
        pause_gate: Optional[Callable[[], None]] = None,
    ) -> dict:
        log = self.node.get_logger()
        segment_list = list(segments)
        if not segment_list:
            return {"ok": False, "stage": "parse_segments", "error": "Need at least 1 segment."}

        def _pause() -> None:
            if callable(pause_gate):
                pause_gate()

        flattened: list[PoseStamped] = []
        for seg in segment_list:
            flattened.append(seg.start)
            flattened.append(seg.end)

        if publish_markers:
            self.run_sync(
                self.util.publish_targets(
                    flattened,
                    axis_length=float(axis_length),
                    axis_radius=float(axis_radius),
                    clear_before=bool(clear_markers_before),
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )

        printed = 0
        failed_segments: list[dict] = []
        printed_targets: list[PoseStamped] = []
        extruder_on = False

        def _step_timeout(extra_s: float) -> Optional[float]:
            if timeout_s is None:
                return None
            return max(float(timeout_s), float(extra_s))

        try:
            self.run_sync(self.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            self.run_sync(self.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)
            self.run_sync(self.motion.setSpd(float(print_vel_scale), enqueue=False), timeout_s=timeout_s)
            if str(eef_link).strip():
                self.run_sync(self.motion.setEef(str(eef_link).strip(), enqueue=False), timeout_s=timeout_s)

            _pause()

            first_start = segment_list[0].start
            first_approach = self._with_z_offset(first_start, float(approach_z_offset_m))
            res = self.run_sync(
                self.motion.goto(
                    pose=first_approach,
                    motion="LIN",
                    vel_scale=float(approach_vel_scale),
                    exec=True,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            _pause()
            if not res.get("ok", False):
                return {"ok": False, "stage": "approach_first_segment", "error": res.get("error", "unknown")}

            for pos, seg in enumerate(segment_list):
                _pause()
                start_travel = self._with_z_offset(seg.start, float(travel_z_offset_m))
                end_travel = self._with_z_offset(seg.end, float(travel_z_offset_m))

                res = self.run_sync(
                    self.motion.goto(
                        pose=start_travel,
                        motion="LIN",
                        vel_scale=float(travel_vel_scale),
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                _pause()
                if not res.get("ok", False):
                    failed_segments.append(
                        {
                            "index": int(seg.index),
                            "position": int(pos),
                            "stage": "goto_start_travel",
                            "error": str(res.get("error", "unknown")),
                        }
                    )
                    log.warn(
                        f"[print_session:segments] Segment {seg.index} failed moving above start. "
                        f"error='{res.get('error', 'unknown')}'"
                    )
                    continue

                res = self.run_sync(
                    self.motion.goto(
                        pose=seg.start,
                        motion="LIN",
                        vel_scale=float(travel_vel_scale),
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                _pause()
                if not res.get("ok", False):
                    failed_segments.append(
                        {
                            "index": int(seg.index),
                            "position": int(pos),
                            "stage": "goto_segment_start",
                            "error": str(res.get("error", "unknown")),
                        }
                    )
                    log.warn(
                        f"[print_session:segments] Segment {seg.index} failed descending to start. "
                        f"error='{res.get('error', 'unknown')}'"
                    )
                    continue

                plan_res = self._plan_print_segment(
                    segment=seg,
                    seed_vel_scale=float(print_vel_scale),
                    seed_accel_scale=float(accel_scale),
                    target_print_speed_mm_s=float(target_print_speed_mm_s),
                    timeout_s=timeout_s,
                )
                _pause()
                if not plan_res.get("ok", False):
                    res = {
                        "ok": False,
                        "stage": str(plan_res.get("stage", "plan_segment")),
                        "error": plan_res.get("error", "unknown"),
                    }
                else:
                    metrics = plan_res.get("metrics", {})
                    if float(target_print_speed_mm_s) > 0.0:
                        log.info(
                            "[print_session:segments] Tuned segment "
                            f"{seg.index}: target_speed={float(target_print_speed_mm_s):.3f}mm/s "
                            f"distance={float(metrics.get('distance_m', 0.0)):.6f}m "
                            f"target_duration={float(metrics.get('target_duration_s', 0.0)):.3f}s "
                            f"planned_duration={float(metrics.get('planned_duration_s', 0.0)):.3f}s "
                            f"error={float(metrics.get('duration_error_s', 0.0)):+.3f}s "
                            f"v={float(metrics.get('vel_scale', print_vel_scale)):.5f} "
                            f"a={float(metrics.get('accel_scale', accel_scale)):.5f} "
                            f"attempts={int(metrics.get('attempts', 1))}"
                        )
                    on_res = self.run_sync(
                        self.extruder.setExtruder(
                            True,
                            speed=int(print_speed),
                            reverse=False,
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                    _pause()
                    if not on_res.get("ok", False):
                        res = {"ok": False, "stage": "extruder_on", "error": on_res.get("error", "unknown")}
                    else:
                        extruder_on = True
                        exec_res = self.run_sync(self.motion.exec(enqueue=False), timeout_s=timeout_s)
                        _pause()
                        if not exec_res.get("ok", False):
                            res = {"ok": False, "stage": "exec_segment", "error": exec_res.get("error", "unknown")}
                        else:
                            res = self.run_sync(
                                self.extruder.setExtruder(False, enqueue=False),
                                timeout_s=timeout_s,
                            )
                            _pause()
                            if not res.get("ok", False):
                                res = {"ok": False, "stage": "extruder_off", "error": res.get("error", "unknown")}
                            else:
                                extruder_on = False
                                res = {"ok": True}

                                if float(post_segment_wait_s) > 0.0:
                                    wait_res = self.run_sync(
                                        self.util.wait(float(post_segment_wait_s), enqueue=False),
                                        timeout_s=_step_timeout(float(post_segment_wait_s) + 2.0),
                                    )
                                    _pause()
                                    if not wait_res.get("ok", False):
                                        res = {
                                            "ok": False,
                                            "stage": "post_segment_wait",
                                            "error": wait_res.get("error", "unknown"),
                                        }

                                if (
                                    res.get("ok", False)
                                    and float(post_segment_retract_s) > 0.0
                                    and int(post_segment_retract_speed) > 0
                                ):
                                    retract_res = self.run_sync(
                                        self.extruder.print_time(
                                            secs=float(post_segment_retract_s),
                                            speed=int(post_segment_retract_speed),
                                            reverse=True,
                                            enqueue=False,
                                        ),
                                        timeout_s=_step_timeout(float(post_segment_retract_s) + 3.0),
                                    )
                                    _pause()
                                    if not retract_res.get("ok", False):
                                        res = {
                                            "ok": False,
                                            "stage": "post_segment_retract",
                                            "error": retract_res.get("error", "unknown"),
                                        }

                if not res.get("ok", False):
                    if extruder_on:
                        try:
                            self.run_sync(self.extruder.setExtruder(False, enqueue=False), timeout_s=timeout_s)
                        except TimeoutError:
                            log.warn("[print_session:segments] Failed to force extruder OFF after segment error.")
                        extruder_on = False
                    failed_segments.append(
                        {
                            "index": int(seg.index),
                            "position": int(pos),
                            "stage": str(res.get("stage", "unknown")),
                            "error": str(res.get("error", "unknown")),
                        }
                    )
                    log.warn(
                        f"[print_session:segments] Segment {seg.index} failed at stage={res.get('stage')}. "
                        f"error='{res.get('error', 'unknown')}'"
                    )
                    continue

                lift_res = self.run_sync(
                    self.motion.goto(
                        pose=end_travel,
                        motion="LIN",
                        vel_scale=float(travel_vel_scale),
                        exec=True,
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                _pause()
                if not lift_res.get("ok", False):
                    failed_segments.append(
                        {
                            "index": int(seg.index),
                            "position": int(pos),
                            "stage": "lift_segment_end",
                            "error": str(lift_res.get("error", "unknown")),
                        }
                    )
                    log.warn(
                        f"[print_session:segments] Segment {seg.index} printed but lift failed. "
                        f"error='{lift_res.get('error', 'unknown')}'"
                    )

                printed += 1
                printed_targets.extend([seg.start, seg.end])

            return {
                "ok": printed > 0,
                "stage": "done" if printed > 0 else "all_segments_failed",
                "segments": len(segment_list),
                "targets": 2 * len(segment_list),
                "printed": printed,
                "skipped": len(segment_list) - printed,
                "failed_segments": failed_segments,
                "printed_targets": printed_targets,
            }
        finally:
            if publish_markers:
                try:
                    self.run_sync(self.util.delete_markers(enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    log.warn("[print_session:segments] delete_markers timed out.")
            if extruder_on:
                try:
                    self.run_sync(self.extruder.setExtruder(False, enqueue=False), timeout_s=timeout_s)
                except TimeoutError:
                    log.warn("[print_session:segments] Failed to force extruder OFF at cleanup.")

    def _plan_print_segment(
        self,
        *,
        segment: TargetSegment,
        seed_vel_scale: float,
        seed_accel_scale: float,
        target_print_speed_mm_s: float,
        timeout_s: Optional[float],
    ) -> dict:
        if float(target_print_speed_mm_s) <= 0.0:
            plan_res = self.run_sync(
                self.motion.plan(
                    pose=segment.end,
                    motion="LIN",
                    vel_scale=float(seed_vel_scale),
                    accel_scale=float(seed_accel_scale),
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not plan_res.get("ok", False):
                return {"ok": False, "stage": "plan_segment", "error": plan_res.get("error", "unknown")}
            return {"ok": True, "stage": "plan_segment", "metrics": dict(plan_res.get("metrics", {}))}

        return self._plan_print_segment_for_target_speed(
            segment=segment,
            seed_vel_scale=seed_vel_scale,
            seed_accel_scale=seed_accel_scale,
            target_print_speed_mm_s=target_print_speed_mm_s,
            timeout_s=timeout_s,
        )

    def _plan_print_segment_for_target_speed(
        self,
        *,
        segment: TargetSegment,
        seed_vel_scale: float,
        seed_accel_scale: float,
        target_print_speed_mm_s: float,
        timeout_s: Optional[float],
    ) -> dict:
        max_iterations = 5
        relative_tolerance = 0.05
        min_scale = 0.001
        max_scale = 1.0

        distance_m = self._euclidean_distance(segment.start, segment.end)
        target_duration_s = distance_m / (float(target_print_speed_mm_s) * 1e-3)
        vel = self._clamp(float(seed_vel_scale), min_scale, max_scale)
        accel = self._clamp(float(seed_accel_scale), min_scale, max_scale)

        best: Optional[dict] = None
        converged = False
        limit_reached = False
        for attempt_i in range(max_iterations):
            attempt = self._plan_print_segment_attempt(
                segment=segment,
                vel_scale=vel,
                accel_scale=accel,
                timeout_s=timeout_s,
            )
            if not attempt.get("ok", False):
                return attempt

            planned_duration_s = float(attempt["metrics"]["planned_duration_s"])
            duration_error_s = planned_duration_s - target_duration_s
            relative_error = (
                abs(duration_error_s) / target_duration_s if target_duration_s > 1e-9 else 0.0
            )
            attempt["metrics"].update(
                {
                    "distance_m": distance_m,
                    "target_duration_s": target_duration_s,
                    "duration_error_s": duration_error_s,
                    "relative_error": relative_error,
                    "target_print_speed_mm_s": float(target_print_speed_mm_s),
                    "attempts": attempt_i + 1,
                }
            )

            if best is None or abs(duration_error_s) < abs(float(best["metrics"]["duration_error_s"])):
                best = attempt

            if relative_error <= relative_tolerance:
                converged = True
                break

            if target_duration_s <= 1e-9 or planned_duration_s <= 1e-9:
                break

            ratio = planned_duration_s / target_duration_s
            next_vel = self._clamp(vel * ratio, min_scale, max_scale)
            next_accel = self._clamp(accel * ratio, min_scale, max_scale)
            if abs(next_vel - vel) < 1e-9 and abs(next_accel - accel) < 1e-9:
                limit_reached = True
                break
            vel = next_vel
            accel = next_accel

        if best is None:
            return {
                "ok": False,
                "stage": "plan_segment_target_speed",
                "error": "target-speed segment planning produced no valid plan",
            }

        metrics = best["metrics"]
        metrics["converged"] = bool(converged)
        metrics["limit_reached"] = bool(limit_reached)
        if not converged:
            return {
                "ok": False,
                "stage": "plan_segment_target_speed",
                "error": (
                    f"could not converge to target speed {target_print_speed_mm_s:.3f}mm/s "
                    f"within {100.0 * relative_tolerance:.1f}% "
                    f"(best_duration={float(metrics['planned_duration_s']):.3f}s, "
                    f"target_duration={target_duration_s:.3f}s, "
                    f"v={float(metrics['vel_scale']):.5f}, a={float(metrics['accel_scale']):.5f}, "
                    f"limit_reached={limit_reached})"
                ),
                "metrics": metrics,
            }

        setattr(self.motion, "_planned_jt", best["trajectory"])
        setattr(self.motion, "_planned_meta", metrics)
        return {"ok": True, "stage": "plan_segment_target_speed", "metrics": metrics}

    def _plan_print_segment_attempt(
        self,
        *,
        segment: TargetSegment,
        vel_scale: float,
        accel_scale: float,
        timeout_s: Optional[float],
    ) -> dict:
        plan_t0 = time.monotonic()
        plan_res = self.run_sync(
            self.motion.plan(
                pose=segment.end,
                motion="LIN",
                vel_scale=float(vel_scale),
                accel_scale=float(accel_scale),
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        plan_wall_s = time.monotonic() - plan_t0
        if not plan_res.get("ok", False):
            return {"ok": False, "stage": "plan_segment", "error": plan_res.get("error", "unknown")}

        jt = getattr(self.motion, "_planned_jt", None)
        metrics = dict(plan_res.get("metrics", {}))
        metrics.update(
            {
                "plan_wall_s": plan_wall_s,
                "planned_duration_s": self._trajectory_duration_s(jt),
                "trajectory_points": len(jt.points) if isinstance(jt, JointTrajectory) else 0,
                "joint_path_rad": self._joint_path_distance(jt),
                "vel_scale": float(vel_scale),
                "accel_scale": float(accel_scale),
            }
        )
        return {
            "ok": True,
            "stage": "plan_segment",
            "trajectory": jt,
            "metrics": metrics,
        }

    def run_print_yaml_auto(
        self,
        *,
        yaml_path: str,
        frame_id: str = "world",
        **kwargs,
    ) -> dict:
        segments = self.parse_yaml_segments(yaml_path=yaml_path, frame_id=frame_id)
        if segments:
            return self.run_print_segments(segments=segments, **kwargs)
        targets = self.parse_yaml_targets(yaml_path=yaml_path, frame_id=frame_id)
        return self.run_print_dots_targets(targets=targets, **kwargs)

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

    @staticmethod
    def _euclidean_distance(a: PoseStamped, b: PoseStamped) -> float:
        dx = float(b.pose.position.x) - float(a.pose.position.x)
        dy = float(b.pose.position.y) - float(a.pose.position.y)
        dz = float(b.pose.position.z) - float(a.pose.position.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _trajectory_duration_s(jt: object) -> float:
        if not isinstance(jt, JointTrajectory) or not jt.points:
            return 0.0
        t = jt.points[-1].time_from_start
        return float(t.sec) + 1e-9 * float(t.nanosec)

    @staticmethod
    def _joint_path_distance(jt: object) -> float:
        if not isinstance(jt, JointTrajectory) or len(jt.points) < 2:
            return 0.0
        total = 0.0
        prev = list(jt.points[0].positions)
        for point in jt.points[1:]:
            cur = list(point.positions)
            if len(cur) != len(prev):
                prev = cur
                continue
            total += math.sqrt(sum((float(c) - float(p)) ** 2 for p, c in zip(prev, cur)))
            prev = cur
        return total

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        lo = min(float(low), float(high))
        hi = max(float(low), float(high))
        return max(lo, min(hi, float(value)))
