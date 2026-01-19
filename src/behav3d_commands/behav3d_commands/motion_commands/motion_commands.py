#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Callable, Dict, Any

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

from behav3d_interfaces.srv import PlanPilzPtp, PlanPilzLin

from behav3d_commands.command import Command

OnPlanned = Callable[[JointTrajectory, Dict[str, Any]], None]


class MotionCommands:
    def __init__(
        self,
        node: Node,
        *,
        controller_action: str = "/scaled_joint_trajectory_controller/follow_joint_trajectory",
    ):
        self._node = node
        self._ctrl = ActionClient(node, FollowJointTrajectory, controller_action)

        # Pilz planning services
        self._ptp_cli = node.create_client(PlanPilzPtp, "/behav3d/plan_pilz_ptp")
        self._lin_cli = node.create_client(PlanPilzLin, "/behav3d/plan_pilz_lin")

        # DEFAULT
        self._motion_mode = "PTP"  # default
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
        ps,
        eef: str,
        vel_scale: float,
        accel_scale: float,
        motion: str,
        cmd: Command,
        on_planned: OnPlanned,
    ) -> None:
        motion = (motion or self._motion_mode).upper()

        if motion == "PTP":
            cli = self._ptp_cli
            Req = PlanPilzPtp.Request
            label = "PLAN(PTP)"
        elif motion == "LIN":
            cli = self._lin_cli
            Req = PlanPilzLin.Request
            label = "PLAN(LIN)"
        else:
            cmd.finish_flag(ok=False, phase="plan", error=f"unknown motion mode '{motion}'")
            return

        if not cli.wait_for_service(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="plan", error=f"{motion.lower()} planner not available")
            return

        req = Req()
        req.group_name = "ur_arm"
        req.eef_link = eef
        req.velocity_scale = float(vel_scale)
        req.accel_scale = float(accel_scale)
        req.preview_only = True
        req.target = ps

        self._node.get_logger().info(
            f"{label}: planning to ({ps.pose.position.x:.3f}, {ps.pose.position.y:.3f}, {ps.pose.position.z:.3f}) eef={eef}..."
        )

        fut = cli.call_async(req)

        def _on_planned(fr):
            resp = fr.result()
            if resp is None or not resp.success:
                cmd.finish_flag(ok=False, phase="plan", error="planning failed")
                return

            jt: JointTrajectory = resp.trajectory.joint_trajectory
            if len(jt.points) == 0:
                cmd.finish_flag(ok=False, phase="plan", error="empty trajectory")
                return

            # Store the plan internally via the injected callback
            on_planned(
                jt,
                {"points": len(jt.points), "mode": motion, "eef": eef},
            )

            # Finish the PLAN command (no execution here)
            cmd.finish_flag(
                ok=True,
                phase="plan",
                planned_only=True,
                metrics={"points": len(jt.points), "mode": motion},
            )

        fut.add_done_callback(_on_planned)

    def exec_follow_traj(self, jt, *, cmd: Command) -> None:
        if not self._ctrl.wait_for_server(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="exec", error="controller not available")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self._node.get_logger().info(f"{cmd.kind.upper()}: sending trajectory ({len(jt.points)} pts)...")
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


# ---------------- HELPERS ----------------

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