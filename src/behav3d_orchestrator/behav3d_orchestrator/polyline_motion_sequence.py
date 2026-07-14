#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_orchestrator polyline_motion_sequence --ros-args \
  -p yaml_path:=/home/lab/behav3d_ws/yaml/pilz_polyline_dummy.yaml
"""

from __future__ import annotations

import math
import threading

import rclpy
from rclpy.node import Node

from .src.yaml_session import YamlSession


class PolylineMotionSequenceNode(Node):
    def __init__(self):
        super().__init__("polyline_motion_sequence")

        self.declare_parameter("yaml_path", "yaml/pilz_polyline_dummy.yaml")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("eef_link", "extruder_tcp")
        self.declare_parameter("vel_scale", 0.05)
        self.declare_parameter("accel_scale", 0.05)
        self.declare_parameter("blend_radius", 0.005)
        self.declare_parameter("target_speed_mm_s", 0.0)
        self.declare_parameter("retime_constant_tcp_speed", False)
        self.declare_parameter("retime_min_dt_s", 0.001)
        self.declare_parameter("tcp_sample_spacing_mm", 2.0)
        self.declare_parameter("speed_tolerance", 0.05)
        self.declare_parameter("speed_max_iterations", 5)
        self.declare_parameter("speed_require_convergence", False)
        self.declare_parameter("tcp_speed_warn_fraction", 0.8)
        self.declare_parameter("move_to_polyline_start", True)
        self.declare_parameter("start_motion", "LIN")
        self.declare_parameter("start_vel_scale", -1.0)
        self.declare_parameter("start_accel_scale", -1.0)
        self.declare_parameter("home_before", True)
        self.declare_parameter("home_after", False)
        self.declare_parameter("timeout_s", 0.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("axis_length", 0.05)
        self.declare_parameter("axis_radius", 0.003)
        self.declare_parameter("clear_markers_before", True)

        self.session = YamlSession(self)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        yaml_path = str(self.get_parameter("yaml_path").value).strip()
        frame_id = str(self.get_parameter("frame_id").value).strip() or "world"
        eef_link = str(self.get_parameter("eef_link").value).strip()
        vel_scale = float(self.get_parameter("vel_scale").value)
        accel_scale = float(self.get_parameter("accel_scale").value)
        blend_radius = float(self.get_parameter("blend_radius").value)
        target_speed_mm_s = float(self.get_parameter("target_speed_mm_s").value)
        retime_constant_tcp_speed = bool(self.get_parameter("retime_constant_tcp_speed").value)
        retime_min_dt_s = float(self.get_parameter("retime_min_dt_s").value)
        tcp_sample_spacing_mm = float(self.get_parameter("tcp_sample_spacing_mm").value)
        speed_tolerance = float(self.get_parameter("speed_tolerance").value)
        speed_max_iterations = int(self.get_parameter("speed_max_iterations").value)
        speed_require_convergence = bool(self.get_parameter("speed_require_convergence").value)
        tcp_speed_warn_fraction = float(self.get_parameter("tcp_speed_warn_fraction").value)
        move_to_polyline_start = bool(self.get_parameter("move_to_polyline_start").value)
        start_motion = str(self.get_parameter("start_motion").value).strip().upper() or "LIN"
        start_vel_scale_param = float(self.get_parameter("start_vel_scale").value)
        start_accel_scale_param = float(self.get_parameter("start_accel_scale").value)
        start_vel_scale = vel_scale if start_vel_scale_param <= 0.0 else start_vel_scale_param
        start_accel_scale = accel_scale if start_accel_scale_param <= 0.0 else start_accel_scale_param
        home_before = bool(self.get_parameter("home_before").value)
        home_after = bool(self.get_parameter("home_after").value)
        timeout_param = float(self.get_parameter("timeout_s").value)
        timeout_s = None if timeout_param <= 0.0 else timeout_param
        publish_markers = bool(self.get_parameter("publish_markers").value)
        axis_length = float(self.get_parameter("axis_length").value)
        axis_radius = float(self.get_parameter("axis_radius").value)
        clear_markers_before = bool(self.get_parameter("clear_markers_before").value)

        if not yaml_path:
            log.error("Parameter 'yaml_path' is empty. Set it with --ros-args -p yaml_path:=<path>.")
            rclpy.shutdown()
            return

        try:
            polylines = self.session.parse_yaml_polylines(yaml_path=yaml_path, frame_id=frame_id)
            path = [pose for polyline in polylines for pose in polyline.poses]
            if not polylines:
                raise ValueError("YAML file does not contain any polylines.")

            log.info(
                "Starting polyline motion sequence: "
                f"path='{yaml_path}', polylines={len(polylines)}, poses={len(path)}, frame='{frame_id}', "
                f"eef='{eef_link}', v={vel_scale:.3f}, a={accel_scale:.3f}, blend={blend_radius:.4f}, "
                f"target_speed_mm_s={target_speed_mm_s:.3f}, retime_constant_tcp_speed={retime_constant_tcp_speed}, "
                f"tcp_sample_spacing_mm={tcp_sample_spacing_mm:.3f}, retime_min_dt_s={retime_min_dt_s:.4f}, "
                f"move_to_polyline_start={move_to_polyline_start}, start_motion={start_motion}, "
                f"start_v={start_vel_scale:.3f}, start_a={start_accel_scale:.3f}"
            )

            if publish_markers and clear_markers_before:
                clear_res = self.session.run_sync(
                    self.session.util.delete_markers(enqueue=False),
                    timeout_s=timeout_s,
                )
                if not clear_res.get("ok", False):
                    raise RuntimeError(f"Target marker cleanup failed: {clear_res.get('error', 'unknown')}")

            if home_before:
                self.session.run_sync(self.session.motion.home(enqueue=False), timeout_s=timeout_s)

            self.session.run_sync(self.session.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setSpd(vel_scale, enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setAcc(accel_scale, enqueue=False), timeout_s=timeout_s)
            if eef_link:
                self.session.run_sync(self.session.motion.setEef(eef_link, enqueue=False), timeout_s=timeout_s)

            executed = 0
            executed_polylines = 0
            skipped_polylines: list[int] = []
            for polyline in polylines:
                sequence_poses = list(polyline.poses)
                if publish_markers:
                    marker_res = self.session.run_sync(
                        self.session.util.publish_targets(
                            polyline.poses,
                            axis_length=axis_length,
                            axis_radius=axis_radius,
                            clear_before=True,
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                    if not marker_res.get("ok", False):
                        skipped_polylines.append(int(polyline.index))
                        log.error(
                            f"[polyline_motion] Skipping polyline index={polyline.index}: "
                            f"target marker publication failed: {marker_res.get('error', 'unknown')}"
                        )
                        continue
                    log.info(
                        f"[polyline_motion] Published active target markers for "
                        f"polyline index={polyline.index} poses={len(polyline.poses)}."
                    )

                if move_to_polyline_start:
                    if not sequence_poses:
                        skipped_polylines.append(int(polyline.index))
                        log.error(f"[polyline_motion] Skipping polyline index={polyline.index}: no poses")
                        continue

                    start_pose = sequence_poses[0]
                    log.info(
                        f"[polyline_motion] Moving to polyline index={polyline.index} start with {start_motion}: "
                        f"({start_pose.pose.position.x:.3f}, "
                        f"{start_pose.pose.position.y:.3f}, "
                        f"{start_pose.pose.position.z:.3f})"
                    )
                    start_res = self.session.run_sync(
                        self.session.motion.goto(
                            pose=start_pose,
                            eef=eef_link,
                            vel_scale=start_vel_scale,
                            accel_scale=start_accel_scale,
                            motion=start_motion,
                            exec=True,
                            enqueue=False,
                        ),
                        timeout_s=timeout_s,
                    )
                    if not start_res.get("ok", False):
                        skipped_polylines.append(int(polyline.index))
                        log.error(
                            f"[polyline_motion] Skipping polyline index={polyline.index}: "
                            f"start move failed: {start_res.get('error', 'unknown')}"
                        )
                        continue

                    if len(sequence_poses) > 2:
                        sequence_poses = sequence_poses[1:]

                log.info(
                    f"[polyline_motion] Planning sequence for polyline index={polyline.index} "
                    f"poses={len(sequence_poses)}"
                )
                for i, pose in enumerate(sequence_poses):
                    log.info(
                        f"[polyline_motion] Polyline {polyline.index} pose {i + 1}/{len(sequence_poses)}: "
                        f"({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
                    )

                plan_res = self._plan_polyline_sequence(
                    poses=sequence_poses,
                    polyline_index=int(polyline.index),
                    eef_link=eef_link,
                    seed_vel_scale=vel_scale,
                    seed_accel_scale=accel_scale,
                    blend_radius=blend_radius,
                    target_speed_mm_s=target_speed_mm_s,
                    retime_constant_tcp_speed=retime_constant_tcp_speed,
                    retime_min_dt_s=retime_min_dt_s,
                    tcp_sample_spacing_m=max(0.0, tcp_sample_spacing_mm) * 1e-3,
                    speed_tolerance=speed_tolerance,
                    max_iterations=speed_max_iterations,
                    require_convergence=speed_require_convergence,
                    frame_id=frame_id,
                    tcp_speed_warn_fraction=tcp_speed_warn_fraction,
                    timeout_s=timeout_s,
                )
                if not plan_res.get("ok", False):
                    skipped_polylines.append(int(polyline.index))
                    log.error(
                        f"[polyline_motion] Skipping polyline index={polyline.index}: "
                        f"sequence plan failed: {plan_res.get('error', 'unknown')}"
                    )
                    continue
                metrics = dict(plan_res.get("metrics", {}))
                log.info(
                    f"[polyline_motion] Sequence plan metrics: "
                    f"points={int(metrics.get('points', 0))} "
                    f"duration={float(metrics.get('duration_s', 0.0)):.3f}s "
                    f"interior_zero_velocity_points={int(metrics.get('interior_zero_velocity_points', 0))} "
                    f"retimed={bool(metrics.get('tcp_speed_retimed', False))} "
                    f"fallback={bool(metrics.get('tcp_speed_retime_fallback', False))} "
                    f"tcp_speed[min/mean/max]="
                    f"{float(metrics.get('tcp_speed_min_mm_s', 0.0)):.3f}/"
                    f"{float(metrics.get('tcp_speed_mean_mm_s', 0.0)):.3f}/"
                    f"{float(metrics.get('tcp_speed_max_mm_s', 0.0)):.3f}mm/s "
                    f"tcp_low_samples={int(metrics.get('tcp_speed_low_sample_count', 0))}/"
                    f"{int(metrics.get('tcp_speed_sample_count', 0))}"
                )

                exec_res = self.session.run_sync(self.session.motion.exec(enqueue=False), timeout_s=timeout_s)
                if not exec_res.get("ok", False):
                    skipped_polylines.append(int(polyline.index))
                    log.error(
                        f"[polyline_motion] Skipping polyline index={polyline.index}: "
                        f"sequence exec failed: {exec_res.get('error', 'unknown')}"
                    )
                    continue
                executed += len(polyline.poses)
                executed_polylines += 1

            if home_after:
                self.session.run_sync(self.session.motion.home(enqueue=False), timeout_s=timeout_s)

            log.info(
                f"Polyline motion sequence complete. Executed polylines={executed_polylines}, "
                f"skipped={len(skipped_polylines)} indexes={skipped_polylines}, poses={executed}"
            )
        except Exception as exc:
            log.error(f"[polyline_motion_sequence] Sequence failed: {exc}")
        finally:
            rclpy.shutdown()

    def _plan_polyline_sequence(
        self,
        *,
        poses,
        polyline_index: int,
        eef_link: str,
        seed_vel_scale: float,
        seed_accel_scale: float,
        blend_radius: float,
        target_speed_mm_s: float,
        retime_constant_tcp_speed: bool,
        retime_min_dt_s: float,
        tcp_sample_spacing_m: float,
        speed_tolerance: float,
        max_iterations: int,
        require_convergence: bool,
        frame_id: str,
        tcp_speed_warn_fraction: float,
        timeout_s,
    ) -> dict:
        log = self.get_logger()
        tcp_speed_threshold_m_s = 0.0
        if float(target_speed_mm_s) > 0.0 and float(tcp_speed_warn_fraction) > 0.0:
            tcp_speed_threshold_m_s = float(target_speed_mm_s) * float(tcp_speed_warn_fraction) * 1e-3

        if float(target_speed_mm_s) <= 0.0 or bool(retime_constant_tcp_speed):
            return self.session.run_sync(
                self.session.motion.plan_sequence(
                    poses,
                    eef=eef_link,
                    vel_scale=float(seed_vel_scale),
                    accel_scale=float(seed_accel_scale),
                    blend_radius=float(blend_radius),
                    frame_id=frame_id,
                    tcp_speed_threshold_m_s=tcp_speed_threshold_m_s,
                    target_tcp_speed_m_s=(
                        float(target_speed_mm_s) * 1e-3 if bool(retime_constant_tcp_speed) else 0.0
                    ),
                    retime_min_dt_s=float(retime_min_dt_s),
                    tcp_sample_spacing_m=float(tcp_sample_spacing_m),
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )

        distance_m = self._polyline_length_m(poses)
        if distance_m <= 1e-9:
            return {"ok": False, "error": "polyline distance is zero"}

        target_duration_s = distance_m / (float(target_speed_mm_s) * 1e-3)
        vel = self._clamp(float(seed_vel_scale), 0.001, 1.0)
        accel = self._clamp(float(seed_accel_scale), 0.001, 1.0)
        tolerance = max(0.0, float(speed_tolerance))
        max_iter = max(1, int(max_iterations))

        best = None
        best_jt = None
        converged = False
        limit_reached = False

        for attempt_i in range(max_iter):
            plan_res = self.session.run_sync(
                self.session.motion.plan_sequence(
                    poses,
                    eef=eef_link,
                    vel_scale=vel,
                    accel_scale=accel,
                    blend_radius=float(blend_radius),
                    frame_id=frame_id,
                    tcp_speed_threshold_m_s=tcp_speed_threshold_m_s,
                    enqueue=False,
                ),
                timeout_s=timeout_s,
            )
            if not plan_res.get("ok", False):
                return plan_res

            metrics = dict(plan_res.get("metrics", {}))
            planned_duration_s = float(metrics.get("duration_s", 0.0))
            duration_error_s = planned_duration_s - target_duration_s
            relative_error = abs(duration_error_s) / target_duration_s if target_duration_s > 1e-9 else 0.0
            actual_speed_mm_s = distance_m / max(planned_duration_s, 1e-9) * 1000.0
            metrics.update(
                {
                    "distance_m": distance_m,
                    "target_duration_s": target_duration_s,
                    "duration_error_s": duration_error_s,
                    "relative_error": relative_error,
                    "target_speed_mm_s": float(target_speed_mm_s),
                    "actual_speed_mm_s": actual_speed_mm_s,
                    "vel_scale": vel,
                    "accel_scale": accel,
                    "attempts": attempt_i + 1,
                }
            )
            plan_res["metrics"] = metrics

            log.info(
                f"[polyline_motion] Speed tune polyline={polyline_index} attempt={attempt_i + 1}/{max_iter}: "
                f"target={float(target_speed_mm_s):.3f}mm/s actual={actual_speed_mm_s:.3f}mm/s "
                f"distance={distance_m:.4f}m target_duration={target_duration_s:.3f}s "
                f"planned_duration={planned_duration_s:.3f}s rel_error={relative_error:.3f} "
                f"v={vel:.5f} a={accel:.5f}"
            )

            if best is None or abs(duration_error_s) < abs(float(best["metrics"]["duration_error_s"])):
                best = plan_res
                best_jt = getattr(self.session.motion, "_planned_jt", None)

            if relative_error <= tolerance:
                converged = True
                best = plan_res
                best_jt = getattr(self.session.motion, "_planned_jt", None)
                break

            if planned_duration_s <= 1e-9:
                break

            ratio = planned_duration_s / target_duration_s
            next_vel = self._clamp(vel * ratio, 0.001, 1.0)
            next_accel = self._clamp(accel * ratio, 0.001, 1.0)
            if abs(next_vel - vel) < 1e-9 and abs(next_accel - accel) < 1e-9:
                limit_reached = True
                break
            vel = next_vel
            accel = next_accel

        if best is None:
            return {"ok": False, "error": "sequence speed tuning produced no valid plan"}

        best["metrics"]["converged"] = bool(converged)
        best["metrics"]["limit_reached"] = bool(limit_reached)
        if best_jt is not None:
            setattr(self.session.motion, "_planned_jt", best_jt)
            setattr(self.session.motion, "_planned_meta", best["metrics"])

        if require_convergence and not converged:
            return {
                "ok": False,
                "error": (
                    f"could not converge to {float(target_speed_mm_s):.3f}mm/s "
                    f"within {100.0 * tolerance:.1f}%"
                ),
                "metrics": best["metrics"],
            }

        if not converged:
            log.warn(
                f"[polyline_motion] Speed tune did not converge for polyline={polyline_index}; "
                "using best available Pilz sequence plan."
            )
        return best

    @staticmethod
    def _polyline_length_m(poses) -> float:
        total = 0.0
        for a, b in zip(poses, poses[1:]):
            dx = float(b.pose.position.x) - float(a.pose.position.x)
            dy = float(b.pose.position.y) - float(a.pose.position.y)
            dz = float(b.pose.position.z) - float(a.pose.position.z)
            total += math.sqrt(dx * dx + dy * dy + dz * dz)
        return total

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(float(lo), min(float(hi), float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = PolylineMotionSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
