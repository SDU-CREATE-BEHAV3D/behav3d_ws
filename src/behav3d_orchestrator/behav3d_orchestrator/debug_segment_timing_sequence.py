#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator debug_segment_timing_sequence

Optional:
ros2 run behav3d_orchestrator debug_segment_timing_sequence --ros-args \
  -p yaml_path:=/home/lab/behav3d_ws/captures/260429_175654/field_loop/cycle_0004/candidates/targets.yaml
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from .src.print_session import PrintSession
from .src.yaml_session import TargetSegment


DEFAULT_YAML_PATH = (
    "/home/lab/behav3d_ws/captures/260429_175654/"
    "field_loop/cycle_0004/candidates/targets.yaml"
)


class DebugSegmentTimingSequenceNode(Node):
    def __init__(self):
        super().__init__("debug_segment_timing_sequence")

        self.declare_parameter("yaml_path", DEFAULT_YAML_PATH)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("eef_link", "extruder_tcp")
        self.declare_parameter("vel_scale", 0.003)
        self.declare_parameter("accel_scale", 0.01)
        self.declare_parameter("target_speed_mm_s", 5.0)
        self.declare_parameter("tune_scales", True)
        self.declare_parameter("tune_max_iterations", 15)
        self.declare_parameter("tune_time_tolerance", 0.01)
        self.declare_parameter("min_vel_scale", 0.001)
        self.declare_parameter("max_vel_scale", 1.0)
        self.declare_parameter("min_accel_scale", 0.001)
        self.declare_parameter("max_accel_scale", 1.0)
        self.declare_parameter("execute_if_not_converged", False)
        self.declare_parameter("approach_vel_scale", 0.20)
        self.declare_parameter("approach_z_offset_m", 0.04)
        self.declare_parameter("home_duration_s", 10.0)
        self.declare_parameter("timeout_s", 0.0)
        self.declare_parameter("max_segments", 3)

        self.session = PrintSession(self)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        yaml_path = str(self.get_parameter("yaml_path").value).strip()
        frame_id = str(self.get_parameter("frame_id").value).strip() or "world"
        eef_link = str(self.get_parameter("eef_link").value).strip() or "extruder_tcp"
        vel_scale = float(self.get_parameter("vel_scale").value)
        accel_scale = float(self.get_parameter("accel_scale").value)
        target_speed_mm_s = float(self.get_parameter("target_speed_mm_s").value)
        tune_scales = bool(self.get_parameter("tune_scales").value)
        tune_max_iterations = int(self.get_parameter("tune_max_iterations").value)
        tune_time_tolerance = float(self.get_parameter("tune_time_tolerance").value)
        min_vel_scale = float(self.get_parameter("min_vel_scale").value)
        max_vel_scale = float(self.get_parameter("max_vel_scale").value)
        min_accel_scale = float(self.get_parameter("min_accel_scale").value)
        max_accel_scale = float(self.get_parameter("max_accel_scale").value)
        execute_if_not_converged = bool(self.get_parameter("execute_if_not_converged").value)
        approach_vel_scale = float(self.get_parameter("approach_vel_scale").value)
        approach_z_offset_m = float(self.get_parameter("approach_z_offset_m").value)
        home_duration_s = float(self.get_parameter("home_duration_s").value)
        timeout_raw = float(self.get_parameter("timeout_s").value)
        timeout_s: Optional[float] = None if timeout_raw <= 0.0 else timeout_raw
        max_segments = int(self.get_parameter("max_segments").value)

        try:
            segments = self.session.parse_yaml_segments(yaml_path=yaml_path, frame_id=frame_id)
            if max_segments > 0:
                segments = segments[:max_segments]
            if not segments:
                raise RuntimeError(f"No segments found in YAML: {yaml_path}")

            log.info(
                "[debug_segment_timing] Starting: "
                f"yaml='{yaml_path}', segments={len(segments)}, frame='{frame_id}', eef='{eef_link}', "
                f"target_speed={target_speed_mm_s:.3f}mm/s, tune_scales={tune_scales}, "
                f"initial_v={vel_scale:.4f}, initial_a={accel_scale:.4f}, "
                f"scale_limits=v[{min_vel_scale:.4f},{max_vel_scale:.4f}] "
                f"a[{min_accel_scale:.4f},{max_accel_scale:.4f}], "
                f"approach_v={approach_vel_scale:.4f}, "
                f"approach_z={approach_z_offset_m:.4f}m"
            )

            self.session.run_sync(
                self.session.motion.home(duration_s=home_duration_s, enqueue=False),
                timeout_s=timeout_s,
            )
            self.session.run_sync(self.session.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setEef(eef_link, enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setSpd(vel_scale, enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setAcc(accel_scale, enqueue=False), timeout_s=timeout_s)

            for pos, segment in enumerate(segments):
                self._move_to_segment_start(
                    segment=segment,
                    approach_z_offset_m=approach_z_offset_m,
                    approach_vel_scale=approach_vel_scale,
                    accel_scale=accel_scale,
                    timeout_s=timeout_s,
                )
                self._measure_segment(
                    position=pos,
                    segment=segment,
                    vel_scale=vel_scale,
                    accel_scale=accel_scale,
                    target_speed_mm_s=target_speed_mm_s,
                    tune_scales=tune_scales,
                    tune_max_iterations=tune_max_iterations,
                    tune_time_tolerance=tune_time_tolerance,
                    min_vel_scale=min_vel_scale,
                    max_vel_scale=max_vel_scale,
                    min_accel_scale=min_accel_scale,
                    max_accel_scale=max_accel_scale,
                    execute_if_not_converged=execute_if_not_converged,
                    timeout_s=timeout_s,
                )

            log.info("[debug_segment_timing] Done.")
        except Exception as exc:
            log.error(f"[debug_segment_timing] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    def _move_to_segment_start(
        self,
        *,
        segment: TargetSegment,
        approach_z_offset_m: float,
        approach_vel_scale: float,
        accel_scale: float,
        timeout_s: Optional[float],
    ) -> None:
        above_start = self._with_z_offset(segment.start, approach_z_offset_m)
        self._plan_and_exec(
            pose=above_start,
            vel_scale=approach_vel_scale,
            accel_scale=accel_scale,
            timeout_s=timeout_s,
        )
        self._plan_and_exec(
            pose=segment.start,
            vel_scale=approach_vel_scale,
            accel_scale=accel_scale,
            timeout_s=timeout_s,
        )

    def _measure_segment(
        self,
        *,
        position: int,
        segment: TargetSegment,
        vel_scale: float,
        accel_scale: float,
        target_speed_mm_s: float,
        tune_scales: bool,
        tune_max_iterations: int,
        tune_time_tolerance: float,
        min_vel_scale: float,
        max_vel_scale: float,
        min_accel_scale: float,
        max_accel_scale: float,
        execute_if_not_converged: bool,
        timeout_s: Optional[float],
    ) -> None:
        log = self.get_logger()
        distance_m = self._euclidean_distance(segment.start, segment.end)
        target_speed_m_s = max(1e-9, float(target_speed_mm_s) * 1e-3)
        target_duration_s = distance_m / target_speed_m_s if distance_m > 0.0 else 0.0

        vel = self._clamp(float(vel_scale), float(min_vel_scale), float(max_vel_scale))
        accel = self._clamp(float(accel_scale), float(min_accel_scale), float(max_accel_scale))
        iterations = max(1, int(tune_max_iterations) if tune_scales else 1)
        tolerance = max(0.0, float(tune_time_tolerance))

        best: Optional[dict] = None
        converged = False
        limit_reached = False
        for attempt_i in range(iterations):
            attempt = self._plan_segment_attempt(
                segment=segment,
                vel_scale=vel,
                accel_scale=accel,
                timeout_s=timeout_s,
            )
            planned_duration_s = float(attempt["planned_duration_s"])
            duration_error_s = planned_duration_s - target_duration_s
            relative_error = (
                abs(duration_error_s) / target_duration_s if target_duration_s > 1e-9 else 0.0
            )

            if best is None or abs(duration_error_s) < abs(float(best["duration_error_s"])):
                best = attempt
                best["duration_error_s"] = duration_error_s
                best["relative_error"] = relative_error

            log.info(
                "[debug_segment_timing:tune] "
                f"segment_pos={position} segment_index={segment.index} attempt={attempt_i + 1}/{iterations} "
                f"distance={distance_m:.6f}m target_speed={target_speed_mm_s:.3f}mm/s "
                f"target_duration={target_duration_s:.3f}s "
                f"planned_duration={planned_duration_s:.3f}s "
                f"error={duration_error_s:+.3f}s ({100.0 * relative_error:.1f}%) "
                f"v={vel:.5f} a={accel:.5f}"
            )

            if relative_error <= tolerance:
                converged = True
                break

            if target_duration_s <= 1e-9 or planned_duration_s <= 1e-9:
                break

            # Same Cartesian target; distance is only used to compute desired time.
            # If planned is too slow, ratio > 1 increases scales. If too fast, it decreases scales.
            ratio = planned_duration_s / target_duration_s
            next_vel = self._clamp(vel * ratio, float(min_vel_scale), float(max_vel_scale))
            next_accel = self._clamp(accel * ratio, float(min_accel_scale), float(max_accel_scale))
            if abs(next_vel - vel) < 1e-9 and abs(next_accel - accel) < 1e-9:
                limit_reached = True
                break
            vel = next_vel
            accel = next_accel

        if best is None:
            raise RuntimeError(f"segment index={segment.index} position={position} produced no valid plan")

        jt = best["trajectory"]
        setattr(self.session.motion, "_planned_jt", jt)
        setattr(self.session.motion, "_planned_meta", best.get("plan_metrics", {}))

        if not converged and not execute_if_not_converged:
            raise RuntimeError(
                f"segment index={segment.index} position={position} did not converge to "
                f"{target_speed_mm_s:.3f}mm/s within {100.0 * tolerance:.1f}% tolerance. "
                f"best_duration={float(best['planned_duration_s']):.3f}s "
                f"target_duration={target_duration_s:.3f}s "
                f"best_v={float(best['vel_scale']):.5f} best_a={float(best['accel_scale']):.5f} "
                f"limit_reached={limit_reached}. Set execute_if_not_converged:=true to execute anyway."
            )

        exec_t0 = time.monotonic()
        exec_res = self.session.run_sync(self.session.motion.exec(enqueue=False), timeout_s=timeout_s)
        exec_wall_s = time.monotonic() - exec_t0
        if not exec_res.get("ok", False):
            raise RuntimeError(
                f"segment index={segment.index} position={position} exec failed: "
                f"{exec_res.get('error', 'unknown')}"
            )

        log.info(
            "[debug_segment_timing] "
            f"segment_pos={position} segment_index={segment.index} "
            f"distance={distance_m:.6f}m ({1000.0 * distance_m:.2f}mm) "
            f"target_speed={target_speed_mm_s:.3f}mm/s "
            f"target_duration={target_duration_s:.3f}s "
            f"plan_wall={float(best['plan_wall_s']):.3f}s "
            f"exec_wall={exec_wall_s:.3f}s "
            f"planned_duration={float(best['planned_duration_s']):.3f}s "
            f"duration_error={float(best['duration_error_s']):+.3f}s "
            f"relative_error={100.0 * float(best['relative_error']):.1f}% "
            f"converged={converged} "
            f"limit_reached={limit_reached} "
            f"v={float(best['vel_scale']):.5f} a={float(best['accel_scale']):.5f} "
            f"trajectory_points={int(best['points'])} "
            f"joint_path={float(best['joint_path_rad']):.6f}rad"
        )

    def _plan_segment_attempt(
        self,
        *,
        segment: TargetSegment,
        vel_scale: float,
        accel_scale: float,
        timeout_s: Optional[float],
    ) -> dict:
        plan_t0 = time.monotonic()
        plan_res = self.session.run_sync(
            self.session.motion.plan(
                pose=segment.end,
                motion="LIN",
                vel_scale=vel_scale,
                accel_scale=accel_scale,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        plan_wall_s = time.monotonic() - plan_t0
        if not plan_res.get("ok", False):
            raise RuntimeError(
                f"segment index={segment.index} plan failed at v={vel_scale:.5f}, "
                f"a={accel_scale:.5f}: {plan_res.get('error', 'unknown')}"
            )

        jt = getattr(self.session.motion, "_planned_jt", None)
        planned_duration_s = self._trajectory_duration_s(jt)
        joint_path_rad = self._joint_path_distance(jt)
        points = len(jt.points) if isinstance(jt, JointTrajectory) else 0
        return {
            "trajectory": jt,
            "plan_metrics": dict(plan_res.get("metrics", {})),
            "plan_wall_s": plan_wall_s,
            "planned_duration_s": planned_duration_s,
            "joint_path_rad": joint_path_rad,
            "points": points,
            "vel_scale": float(vel_scale),
            "accel_scale": float(accel_scale),
        }

    def _plan_and_exec(
        self,
        *,
        pose: PoseStamped,
        vel_scale: float,
        accel_scale: float,
        timeout_s: Optional[float],
    ) -> None:
        plan_res = self.session.run_sync(
            self.session.motion.plan(
                pose=pose,
                motion="LIN",
                vel_scale=vel_scale,
                accel_scale=accel_scale,
                enqueue=False,
            ),
            timeout_s=timeout_s,
        )
        if not plan_res.get("ok", False):
            raise RuntimeError(f"approach plan failed: {plan_res.get('error', 'unknown')}")

        exec_res = self.session.run_sync(self.session.motion.exec(enqueue=False), timeout_s=timeout_s)
        if not exec_res.get("ok", False):
            raise RuntimeError(f"approach exec failed: {exec_res.get('error', 'unknown')}")

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
    def _clamp(value: float, low: float, high: float) -> float:
        lo = min(float(low), float(high))
        hi = max(float(low), float(high))
        return max(lo, min(hi, float(value)))

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


def main(args=None):
    rclpy.init(args=args)
    node = DebugSegmentTimingSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
