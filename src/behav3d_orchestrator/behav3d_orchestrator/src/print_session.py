#!/usr/bin/env python3
from __future__ import annotations

from typing import Callable, Optional, Sequence

from geometry_msgs.msg import PoseStamped

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

            if callable(pause_gate):
                pause_gate()

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

                if float(dwell_s) > 0.0:
                    res = self.run_sync(self.util.wait(float(dwell_s), enqueue=False), timeout_s=timeout_s)
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
        eef_link: str = "extruder_tcp",
        timeout_s: Optional[float] = None,
        publish_markers: bool = False,
        axis_length: float = 0.05,
        axis_radius: float = 0.003,
        clear_markers_before: bool = True,
    ) -> dict:
        log = self.node.get_logger()
        segment_list = list(segments)
        if not segment_list:
            return {"ok": False, "stage": "parse_segments", "error": "Need at least 1 segment."}

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
        try:
            self.run_sync(self.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            self.run_sync(self.motion.setAcc(float(accel_scale), enqueue=False), timeout_s=timeout_s)
            self.run_sync(self.motion.setSpd(float(print_vel_scale), enqueue=False), timeout_s=timeout_s)
            if str(eef_link).strip():
                self.run_sync(self.motion.setEef(str(eef_link).strip(), enqueue=False), timeout_s=timeout_s)

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
            if not res.get("ok", False):
                return {"ok": False, "stage": "approach_first_segment", "error": res.get("error", "unknown")}

            for pos, seg in enumerate(segment_list):
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

                plan_res = self.run_sync(
                    self.motion.plan(
                        pose=seg.end,
                        motion="LIN",
                        vel_scale=float(print_vel_scale),
                        enqueue=False,
                    ),
                    timeout_s=timeout_s,
                )
                if not plan_res.get("ok", False):
                    res = {"ok": False, "stage": "plan_segment", "error": plan_res.get("error", "unknown")}
                else:
                    on_res = self.run_sync(
                        self.extruder.setExtruder(True, speed=int(print_speed), enqueue=False),
                        timeout_s=timeout_s,
                    )
                    if not on_res.get("ok", False):
                        res = {"ok": False, "stage": "extruder_on", "error": on_res.get("error", "unknown")}
                    else:
                        extruder_on = True
                        exec_res = self.run_sync(self.motion.exec(enqueue=False), timeout_s=timeout_s)
                        if not exec_res.get("ok", False):
                            res = {"ok": False, "stage": "exec_segment", "error": exec_res.get("error", "unknown")}
                        else:
                            res = self.run_sync(
                                self.extruder.setExtruder(False, enqueue=False),
                                timeout_s=timeout_s,
                            )
                            if not res.get("ok", False):
                                res = {"ok": False, "stage": "extruder_off", "error": res.get("error", "unknown")}
                            else:
                                extruder_on = False
                                res = {"ok": True}

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
