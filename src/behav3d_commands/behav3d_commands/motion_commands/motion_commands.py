#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Any, Callable, Dict, Optional, Sequence

from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

from behav3d_interfaces.srv import PlanPilzPtp, PlanPilzLin, PlanPilzSequence

from behav3d_commands.command import Command, OnCommandDone
from behav3d_commands.queue import QueueItem, SessionQueue

OnPlanned = Callable[[JointTrajectory, Dict[str, Any]], None]


class MotionCommands:
    def __init__(
        self,
        node: Node,
        *,
        queue: Optional[SessionQueue] = None,
        controller_action: str = "/scaled_joint_trajectory_controller/follow_joint_trajectory",
    ):
        self._queue = queue
        self._node = node
        self._ctrl = ActionClient(node, FollowJointTrajectory, controller_action)

        self._ptp_cli = node.create_client(PlanPilzPtp, "/behav3d/plan_pilz_ptp")
        self._lin_cli = node.create_client(PlanPilzLin, "/behav3d/plan_pilz_lin")
        self._seq_cli = node.create_client(PlanPilzSequence, "/behav3d/plan_pilz_sequence")

        self._motion_mode = "PTP"
        self._default_eef = "extruder_tcp"
        self._default_vel_scale = 0.1
        self._default_accel_scale = 0.1

        self._joint_names = [
            "ur20_shoulder_pan_joint",
            "ur20_shoulder_lift_joint",
            "ur20_elbow_joint",
            "ur20_wrist_1_joint",
            "ur20_wrist_2_joint",
            "ur20_wrist_3_joint",
        ]

        home_deg = [-90.0, -120.0, 120.0, 0.0, 90.0, -180.0]
        self._home_rad = [math.radians(d) for d in home_deg]

        self._planned_jt: Optional[JointTrajectory] = None
        self._planned_meta: Optional[Dict[str, Any]] = None

    @property
    def motion_mode(self) -> str:
        return self._motion_mode

    @property
    def default_eef(self) -> str:
        return self._default_eef

    @property
    def default_vel_scale(self) -> float:
        return float(self._default_vel_scale)

    @property
    def default_accel_scale(self) -> float:
        return float(self._default_accel_scale)

    def register(self, router) -> None:
        router.register("home", self._handle_home)
        router.register("plan_motion", self._handle_plan_motion)
        router.register("plan_sequence", self._handle_plan_sequence)
        router.register("exec_motion", self._handle_exec_motion)
        router.register("goto", self._handle_goto)
        router.register("goto_sequence", self._handle_goto_sequence)
        router.register("set_motion_mode", self._handle_set_motion_mode)
        router.register("set_eef", self._handle_set_eef)
        router.register("set_spd", self._handle_set_spd)
        router.register("set_acc", self._handle_set_acc)

    def _queue_or_item(self, item: QueueItem, *, enqueue: bool):
        if enqueue:
            if self._queue is None:
                raise RuntimeError("MotionCommands requires a SessionQueue to enqueue items.")
            self._queue.enqueue(item)
            return None
        return item

    def home(
        self,
        *,
        duration_s: float = 10.0,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "home",
            {"duration_s": float(duration_s)},
            cmd_kind="home",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def goto(
        self,
        *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        pose: Any = None,
        rx: Optional[float] = None,
        ry: Optional[float] = None,
        rz: Optional[float] = None,
        eef: Optional[str] = None,
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        exec: bool = True,
        motion: Optional[str] = None,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "goto",
            {
                "pose": pose,
                "x": x,
                "y": y,
                "z": z,
                "rx": rx,
                "ry": ry,
                "rz": rz,
                "eef": eef,
                "vel_scale": vel_scale,
                "accel_scale": accel_scale,
                "exec": bool(exec),
                "motion": motion,
            },
            cmd_kind="goto",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def plan(
        self,
        *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        pose: Any = None,
        eef: Optional[str] = None,
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        motion: Optional[str] = None,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "plan_motion",
            {
                "pose": pose,
                "x": x,
                "y": y,
                "z": z,
                "eef": eef,
                "vel_scale": vel_scale,
                "accel_scale": accel_scale,
                "motion": motion,
            },
            cmd_kind="plan_motion",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def plan_sequence(
        self,
        poses: Sequence[PoseStamped],
        *,
        eef: Optional[str] = None,
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        blend_radius: float = 0.0,
        blend_radii: Optional[Sequence[float]] = None,
        frame_id: Optional[str] = None,
        tcp_speed_threshold_m_s: float = 0.0,
        target_tcp_speed_m_s: float = 0.0,
        retime_min_dt_s: float = 0.001,
        tcp_sample_spacing_m: float = 0.002,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "plan_sequence",
            {
                "poses": list(poses),
                "eef": eef,
                "vel_scale": vel_scale,
                "accel_scale": accel_scale,
                "blend_radius": float(blend_radius),
                "blend_radii": None if blend_radii is None else [float(v) for v in blend_radii],
                "frame_id": frame_id,
                "tcp_speed_threshold_m_s": float(tcp_speed_threshold_m_s),
                "target_tcp_speed_m_s": float(target_tcp_speed_m_s),
                "retime_min_dt_s": float(retime_min_dt_s),
                "tcp_sample_spacing_m": float(tcp_sample_spacing_m),
            },
            cmd_kind="plan_sequence",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def goto_sequence(
        self,
        poses: Sequence[PoseStamped],
        *,
        eef: Optional[str] = None,
        vel_scale: Optional[float] = None,
        accel_scale: Optional[float] = None,
        blend_radius: float = 0.0,
        blend_radii: Optional[Sequence[float]] = None,
        frame_id: Optional[str] = None,
        tcp_speed_threshold_m_s: float = 0.0,
        target_tcp_speed_m_s: float = 0.0,
        retime_min_dt_s: float = 0.001,
        tcp_sample_spacing_m: float = 0.002,
        exec: bool = True,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "goto_sequence",
            {
                "poses": list(poses),
                "eef": eef,
                "vel_scale": vel_scale,
                "accel_scale": accel_scale,
                "blend_radius": float(blend_radius),
                "blend_radii": None if blend_radii is None else [float(v) for v in blend_radii],
                "frame_id": frame_id,
                "tcp_speed_threshold_m_s": float(tcp_speed_threshold_m_s),
                "target_tcp_speed_m_s": float(target_tcp_speed_m_s),
                "retime_min_dt_s": float(retime_min_dt_s),
                "tcp_sample_spacing_m": float(tcp_sample_spacing_m),
                "exec": bool(exec),
            },
            cmd_kind="goto_sequence",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def exec(
        self,
        *,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "exec_motion",
            {},
            cmd_kind="exec_motion",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def setPTP(
        self,
        *,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "set_motion_mode",
            {"mode": "PTP"},
            cmd_kind="setPTP",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def setLIN(
        self,
        *,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "set_motion_mode",
            {"mode": "LIN"},
            cmd_kind="setLIN",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def setEef(
        self,
        name: str,
        *,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "set_eef",
            {"eef": str(name)},
            cmd_kind="setEef",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def setSpd(
        self,
        val: float,
        *,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "set_spd",
            {"vel_scale": float(val)},
            cmd_kind="setSpd",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def setAcc(
        self,
        val: float,
        *,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "set_acc",
            {"accel_scale": float(val)},
            cmd_kind="setAcc",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def build_home_trajectory(self, *, duration_s: float) -> JointTrajectory:
        jt = JointTrajectory()
        jt.joint_names = list(self._joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(self._home_rad)

        sec = int(duration_s)
        nsec = int((duration_s - sec) * 1e9)
        pt.time_from_start.sec = sec
        pt.time_from_start.nanosec = nsec

        jt.points.append(pt)
        return jt

    def plan_motion(
        self,
        *,
        ps: PoseStamped,
        eef: Optional[str],
        vel_scale: Optional[float],
        accel_scale: Optional[float],
        motion: Optional[str],
        cmd: Command,
        on_planned: OnPlanned,
        finish_on_plan: bool = True,
    ) -> None:
        motion_eff = self._normalize_motion(motion or self._motion_mode, cmd, phase="plan")
        if motion_eff is None:
            return

        eef_eff = self._default_eef if eef is None else str(eef)
        vel_eff = self._clamp_scale(self._default_vel_scale if vel_scale is None else float(vel_scale))
        accel_eff = self._clamp_scale(self._default_accel_scale if accel_scale is None else float(accel_scale))

        if motion_eff == "PTP":
            cli = self._ptp_cli
            Req = PlanPilzPtp.Request
            label = "PLAN(PTP)"
        else:
            cli = self._lin_cli
            Req = PlanPilzLin.Request
            label = "PLAN(LIN)"

        if not cli.wait_for_service(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="plan", error=f"{motion_eff.lower()} planner not available")
            return

        req = Req()
        req.group_name = "ur_arm"
        req.eef_link = eef_eff
        req.velocity_scale = float(vel_eff)
        req.accel_scale = float(accel_eff)
        req.preview_only = True
        req.target = ps

        self._node.get_logger().info(
            f"{label}: planning to ({ps.pose.position.x:.3f}, {ps.pose.position.y:.3f}, {ps.pose.position.z:.3f}) "
            f"eef={eef_eff} v={vel_eff:.3f} a={accel_eff:.3f}"
        )

        fut = cli.call_async(req)

        def _on_planned(fr):
            resp = fr.result()
            if resp is None:
                cmd.finish_flag(ok=False, phase="plan", error="planning failed (no response)")
                return

            if not resp.success:
                moveit_error_code = int(getattr(resp, "moveit_error_code", 0))
                cmd.finish_flag(
                    ok=False,
                    phase="plan",
                    error=f"planning failed (moveit_error_code={moveit_error_code})",
                    metrics={"moveit_error_code": moveit_error_code},
                )
                return

            jt: JointTrajectory = resp.trajectory.joint_trajectory
            if len(jt.points) == 0:
                cmd.finish_flag(ok=False, phase="plan", error="empty trajectory")
                return

            meta = {
                "points": len(jt.points),
                "mode": motion_eff,
                "eef": eef_eff,
                "vel_scale": vel_eff,
                "accel_scale": accel_eff,
            }
            try:
                on_planned(jt, meta)
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="plan", error=f"on_planned exception: {exc}")
                return

            if finish_on_plan:
                cmd.finish_flag(ok=True, phase="plan", metrics=meta, planned_only=True)

        fut.add_done_callback(_on_planned)

    def plan_sequence_motion(
        self,
        *,
        poses: Sequence[PoseStamped],
        eef: Optional[str],
        vel_scale: Optional[float],
        accel_scale: Optional[float],
        blend_radius: float,
        blend_radii: Optional[Sequence[float]],
        frame_id: Optional[str],
        tcp_speed_threshold_m_s: float,
        target_tcp_speed_m_s: float,
        retime_min_dt_s: float,
        tcp_sample_spacing_m: float,
        cmd: Command,
        on_planned: OnPlanned,
        finish_on_plan: bool = True,
    ) -> None:
        pose_list = list(poses)
        if len(pose_list) < 2:
            cmd.finish_flag(ok=False, phase="plan", error="pilz sequence requires at least 2 poses")
            return
        if any(not isinstance(ps, PoseStamped) for ps in pose_list):
            cmd.finish_flag(ok=False, phase="plan", error="pilz sequence poses must be PoseStamped")
            return

        eef_eff = self._default_eef if eef is None else str(eef)
        vel_eff = self._clamp_scale(self._default_vel_scale if vel_scale is None else float(vel_scale))
        accel_eff = self._clamp_scale(self._default_accel_scale if accel_scale is None else float(accel_scale))
        radii = self._sequence_blend_radii(
            len(pose_list),
            blend_radius=float(blend_radius),
            blend_radii=blend_radii,
        )

        if not self._seq_cli.wait_for_service(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="plan", error="pilz sequence planner not available")
            return

        req = PlanPilzSequence.Request()
        req.group_name = "ur_arm"
        frame_eff = str(frame_id or pose_list[0].header.frame_id or "world")
        req.frame_id = frame_eff
        req.eef_link = eef_eff
        req.velocity_scale = float(vel_eff)
        req.accel_scale = float(accel_eff)
        req.preview_only = True
        req.targets = [ps.pose for ps in pose_list]
        req.blend_radii = list(radii)
        req.tcp_speed_threshold_m_s = max(0.0, float(tcp_speed_threshold_m_s))
        req.target_tcp_speed_m_s = max(0.0, float(target_tcp_speed_m_s))
        req.retime_min_dt_s = max(0.0, float(retime_min_dt_s))
        req.tcp_sample_spacing_m = max(0.0, float(tcp_sample_spacing_m))

        frame_ids = sorted({str(ps.header.frame_id or "") for ps in pose_list})
        self._node.get_logger().info(
            f"PLAN(SEQ): planning {len(pose_list)} poses eef={eef_eff} "
            f"v={vel_eff:.3f} a={accel_eff:.3f} blend={float(blend_radius):.4f} "
            f"frame={frame_eff} pose_frames={frame_ids}"
        )

        fut = self._seq_cli.call_async(req)

        def _on_planned(fr):
            resp = fr.result()
            if resp is None:
                cmd.finish_flag(ok=False, phase="plan", error="sequence planning failed (no response)")
                return
            if not resp.success:
                message = str(getattr(resp, "message", "") or "sequence planning failed")
                cmd.finish_flag(ok=False, phase="plan", error=message)
                return

            jt: JointTrajectory = resp.trajectory
            if len(jt.points) == 0:
                cmd.finish_flag(ok=False, phase="plan", error="empty sequence trajectory")
                return

            meta = {
                "points": len(jt.points),
                "poses": len(pose_list),
                "mode": "SEQ",
                "eef": eef_eff,
                "vel_scale": vel_eff,
                "accel_scale": accel_eff,
                "blend_radius": float(blend_radius),
                "blend_radii": list(radii),
                "frame_ids": frame_ids,
                "duration_s": self._trajectory_duration_s(jt),
                "interior_zero_velocity_points": self._interior_zero_velocity_points(jt),
                "tcp_path_length_m": float(getattr(resp, "tcp_path_length_m", 0.0)),
                "tcp_duration_s": float(getattr(resp, "tcp_duration_s", 0.0)),
                "tcp_speed_min_mm_s": 1000.0 * float(getattr(resp, "tcp_speed_min_m_s", 0.0)),
                "tcp_speed_mean_mm_s": 1000.0 * float(getattr(resp, "tcp_speed_mean_m_s", 0.0)),
                "tcp_speed_max_mm_s": 1000.0 * float(getattr(resp, "tcp_speed_max_m_s", 0.0)),
                "tcp_speed_sample_count": int(getattr(resp, "tcp_speed_sample_count", 0)),
                "tcp_speed_low_sample_count": int(getattr(resp, "tcp_speed_low_sample_count", 0)),
                "tcp_speed_threshold_mm_s": 1000.0 * max(0.0, float(tcp_speed_threshold_m_s)),
                "tcp_speed_retimed": bool(getattr(resp, "tcp_speed_retimed", False)),
                "tcp_speed_retime_fallback": bool(getattr(resp, "tcp_speed_retime_fallback", False)),
                "tcp_sample_spacing_mm": 1000.0 * max(0.0, float(tcp_sample_spacing_m)),
                "message": str(getattr(resp, "message", "") or ""),
            }
            self._node.get_logger().info(
                "PLAN(SEQ): trajectory "
                f"points={meta['points']} duration={meta['duration_s']:.3f}s "
                f"interior_zero_velocity_points={meta['interior_zero_velocity_points']} "
                f"tcp_speed[min/mean/max]="
                f"{meta['tcp_speed_min_mm_s']:.3f}/"
                f"{meta['tcp_speed_mean_mm_s']:.3f}/"
                f"{meta['tcp_speed_max_mm_s']:.3f}mm/s "
                f"tcp_low_samples={meta['tcp_speed_low_sample_count']}/{meta['tcp_speed_sample_count']} "
                f"retimed={meta['tcp_speed_retimed']} "
                f"fallback={meta['tcp_speed_retime_fallback']} "
                f"tcp_sample_spacing={meta['tcp_sample_spacing_mm']:.3f}mm"
            )
            try:
                on_planned(jt, meta)
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="plan", error=f"on_planned exception: {exc}")
                return

            if finish_on_plan:
                cmd.finish_flag(ok=True, phase="plan", metrics=meta, planned_only=True)

        fut.add_done_callback(_on_planned)

    def exec_follow_traj(self, jt: JointTrajectory, *, cmd: Command) -> None:
        if not self._ctrl.wait_for_server(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="exec", error="controller not available")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self._node.get_logger().info(
            f"{cmd.kind.upper()}: sending trajectory "
            f"({len(jt.points)} pts, duration={self._trajectory_duration_s(jt):.3f}s, "
            f"interior_zero_velocity_points={self._interior_zero_velocity_points(jt)})..."
        )
        send_fut = self._ctrl.send_goal_async(goal)

        def _on_goal_sent(f):
            handle = f.result()
            if not handle or not handle.accepted:
                cmd.finish_flag(ok=False, phase="exec", error="exec goal rejected")
                return

            res_fut = handle.get_result_async()
            res_fut.add_done_callback(_on_result)

        def _on_result(rf):
            res = rf.result().result
            err = int(getattr(res, "error_code", -999))
            is_ok = (err == 0)

            cmd.finish_flag(
                ok=is_ok,
                phase="exec",
                metrics={"error_code": err},
                error=None if is_ok else f"exec failed (error_code={err})",
            )

        send_fut.add_done_callback(_on_goal_sent)

    def set_motion_mode(self, mode: str, *, cmd: Command) -> None:
        new_mode = self._normalize_motion(mode, cmd)
        if new_mode is None:
            return
        old = self._motion_mode
        self._motion_mode = new_mode
        cmd.finish_flag(ok=True, phase="exec", metrics={"old": old, "new": new_mode})

    def set_default_eef(self, eef: str, *, cmd: Command) -> None:
        old = self._default_eef
        self._default_eef = str(eef)
        cmd.finish_flag(ok=True, phase="exec", metrics={"old": old, "new": self._default_eef})

    def set_default_vel_scale(self, val: float, *, cmd: Command) -> None:
        old = float(self._default_vel_scale)
        new = self._clamp_scale(float(val))
        self._default_vel_scale = new
        cmd.finish_flag(ok=True, phase="exec", metrics={"old": old, "new": new})

    def set_default_accel_scale(self, val: float, *, cmd: Command) -> None:
        old = float(self._default_accel_scale)
        new = self._clamp_scale(float(val))
        self._default_accel_scale = new
        cmd.finish_flag(ok=True, phase="exec", metrics={"old": old, "new": new})

    def _handle_home(self, payload: Dict[str, Any], cmd: Command) -> None:
        duration_s = float(payload.get("duration_s", 10.0))
        jt = self.build_home_trajectory(duration_s=duration_s)
        self.exec_follow_traj(jt, cmd=cmd)

    def _handle_plan_motion(self, payload: Dict[str, Any], cmd: Command) -> None:
        ps = self._pose_from_payload(payload)

        def _store_plan(jt: JointTrajectory, meta: Dict[str, Any]) -> None:
            self._planned_jt = jt
            self._planned_meta = meta

        self.plan_motion(
            ps=ps,
            eef=payload.get("eef"),
            vel_scale=payload.get("vel_scale"),
            accel_scale=payload.get("accel_scale"),
            motion=payload.get("motion"),
            cmd=cmd,
            on_planned=_store_plan,
        )

    def _handle_plan_sequence(self, payload: Dict[str, Any], cmd: Command) -> None:
        def _store_plan(jt: JointTrajectory, meta: Dict[str, Any]) -> None:
            self._planned_jt = jt
            self._planned_meta = meta

        self.plan_sequence_motion(
            poses=payload.get("poses", []),
            eef=payload.get("eef"),
            vel_scale=payload.get("vel_scale"),
            accel_scale=payload.get("accel_scale"),
            blend_radius=float(payload.get("blend_radius", 0.0)),
            blend_radii=payload.get("blend_radii"),
            frame_id=payload.get("frame_id"),
            tcp_speed_threshold_m_s=float(payload.get("tcp_speed_threshold_m_s", 0.0)),
            target_tcp_speed_m_s=float(payload.get("target_tcp_speed_m_s", 0.0)),
            retime_min_dt_s=float(payload.get("retime_min_dt_s", 0.001)),
            tcp_sample_spacing_m=float(payload.get("tcp_sample_spacing_m", 0.002)),
            cmd=cmd,
            on_planned=_store_plan,
        )

    def _handle_exec_motion(self, payload: Dict[str, Any], cmd: Command) -> None:
        jt = self._planned_jt
        if jt is None:
            cmd.finish_flag(ok=False, phase="exec", error="no stored motion plan (call plan_motion first)")
            return
        self.exec_follow_traj(jt, cmd=cmd)

    def _handle_goto(self, payload: Dict[str, Any], cmd: Command) -> None:
        ps = self._pose_from_payload(payload)
        do_exec = bool(payload.get("exec", True))

        def _on_planned(jt: JointTrajectory, meta: Dict[str, Any]) -> None:
            self._planned_jt = jt
            self._planned_meta = meta
            if not do_exec:
                cmd.finish_flag(ok=True, phase="plan", metrics=meta, planned_only=True)
                return
            self.exec_follow_traj(jt, cmd=cmd)

        self.plan_motion(
            ps=ps,
            eef=payload.get("eef"),
            vel_scale=payload.get("vel_scale"),
            accel_scale=payload.get("accel_scale"),
            motion=payload.get("motion"),
            cmd=cmd,
            on_planned=_on_planned,
            finish_on_plan=False,
        )

    def _handle_goto_sequence(self, payload: Dict[str, Any], cmd: Command) -> None:
        do_exec = bool(payload.get("exec", True))

        def _on_planned(jt: JointTrajectory, meta: Dict[str, Any]) -> None:
            self._planned_jt = jt
            self._planned_meta = meta
            if not do_exec:
                cmd.finish_flag(ok=True, phase="plan", metrics=meta, planned_only=True)
                return
            self.exec_follow_traj(jt, cmd=cmd)

        self.plan_sequence_motion(
            poses=payload.get("poses", []),
            eef=payload.get("eef"),
            vel_scale=payload.get("vel_scale"),
            accel_scale=payload.get("accel_scale"),
            blend_radius=float(payload.get("blend_radius", 0.0)),
            blend_radii=payload.get("blend_radii"),
            frame_id=payload.get("frame_id"),
            tcp_speed_threshold_m_s=float(payload.get("tcp_speed_threshold_m_s", 0.0)),
            target_tcp_speed_m_s=float(payload.get("target_tcp_speed_m_s", 0.0)),
            retime_min_dt_s=float(payload.get("retime_min_dt_s", 0.001)),
            tcp_sample_spacing_m=float(payload.get("tcp_sample_spacing_m", 0.002)),
            cmd=cmd,
            on_planned=_on_planned,
            finish_on_plan=False,
        )

    def _handle_set_motion_mode(self, payload: Dict[str, Any], cmd: Command) -> None:
        self.set_motion_mode(payload["mode"], cmd=cmd)

    def _handle_set_eef(self, payload: Dict[str, Any], cmd: Command) -> None:
        self.set_default_eef(payload["eef"], cmd=cmd)

    def _handle_set_spd(self, payload: Dict[str, Any], cmd: Command) -> None:
        self.set_default_vel_scale(payload["vel_scale"], cmd=cmd)

    def _handle_set_acc(self, payload: Dict[str, Any], cmd: Command) -> None:
        self.set_default_accel_scale(payload["accel_scale"], cmd=cmd)

    def _pose_from_payload(self, payload: Dict[str, Any]) -> PoseStamped:
        ps = payload.get("pose")
        if isinstance(ps, PoseStamped):
            return ps

        x = payload.get("x")
        y = payload.get("y")
        z = payload.get("z")
        if x is None or y is None or z is None:
            raise ValueError("pose or x/y/z required for motion command")

        ps = PoseStamped()
        ps.header.frame_id = str(payload.get("frame_id", "world"))
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)

        rx = payload.get("rx")
        ry = payload.get("ry")
        rz = payload.get("rz")
        if rx is not None or ry is not None or rz is not None:
            r = 0.0 if rx is None else float(rx)
            p = 0.0 if ry is None else float(ry)
            y_ = 0.0 if rz is None else float(rz)
            qx, qy, qz, qw = self._quat_from_euler(r, p, y_)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
        else:
            ps.pose.orientation.w = 1.0

        return ps

    def _normalize_motion(self, mode: str, cmd: Command, *, phase: str = "exec") -> Optional[str]:
        new_mode = str(mode).upper()
        if new_mode not in ("PTP", "LIN"):
            cmd.finish_flag(ok=False, phase=phase, error=f"unknown motion mode '{new_mode}'")
            return None
        return new_mode

    @staticmethod
    def _clamp_scale(val: float) -> float:
        return max(0.0, min(1.0, float(val)))

    @staticmethod
    def _sequence_blend_radii(
        count: int,
        *,
        blend_radius: float,
        blend_radii: Optional[Sequence[float]],
    ) -> list[float]:
        if blend_radii is not None:
            radii = [max(0.0, float(v)) for v in blend_radii]
            if len(radii) < count:
                radii.extend([0.0] * (count - len(radii)))
            elif len(radii) > count:
                radii = radii[:count]
        else:
            radii = [max(0.0, float(blend_radius)) for _ in range(count)]
        if radii:
            radii[-1] = 0.0
        return radii

    @staticmethod
    def _trajectory_duration_s(jt: JointTrajectory) -> float:
        if not isinstance(jt, JointTrajectory) or not jt.points:
            return 0.0
        t = jt.points[-1].time_from_start
        return float(t.sec) + 1e-9 * float(t.nanosec)

    @staticmethod
    def _interior_zero_velocity_points(jt: JointTrajectory, *, eps: float = 1e-6) -> int:
        if not isinstance(jt, JointTrajectory) or len(jt.points) <= 2:
            return 0

        count = 0
        for point in jt.points[1:-1]:
            velocities = list(point.velocities)
            if velocities and all(abs(float(v)) <= eps for v in velocities):
                count += 1
        return count

    @staticmethod
    def _quat_from_euler(roll: float, pitch: float, yaw: float):
        """RPY (rad) -> quaternion (x,y,z,w), extrinsic XYZ (ROS-style)."""
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        w = cr*cp*cy + sr*sp*sy
        x = sr*cp*cy - cr*sp*sy
        y = cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*cp*cy
        return x, y, z, w
