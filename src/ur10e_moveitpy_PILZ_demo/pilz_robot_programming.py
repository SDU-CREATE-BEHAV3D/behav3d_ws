
#!/usr/bin/env python3
"""
Pilz‑style motion programming API for MoveIt 2 using moveit_py.

This module provides a ROS 2–native re‑implementation of the
``pilz_industrial_motion`` *programming* layer that existed for ROS 1.
Only PTP (joint) and LIN (Cartesian) motions are supported—CIRC and
gripper commands were intentionally left out per user request.

Typical usage
-------------
```python
from pilz_motion_controller import Robot, Ptp, Lin, Sequence

robot = Robot(group="ur_manipulator")

# Move in joint space
robot.move(Ptp(goal=[0, -90, 90, -90, -90, 0], vel_scale=0.3))

# Single Cartesian move
robot.move(Lin(goal=my_pose, vel_scale=0.2))

# Cartesian sequence with blending
robot.move(Sequence([
    Lin(goal=pose_a),
    Lin(goal=pose_b),
    Lin(goal=pose_c),
], blend_radius=0.05))
```
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence as Seq, Union

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped
from moveit_py.pose_goal_builder import create_pose_goal
from moveit.planning import PlanRequestParameters
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_msgs.srv import GetMotionSequence
from moveit_msgs.msg import MotionSequenceItem, MotionSequenceRequest

# =============================================================================
# Basic helpers
# =============================================================================

def degrees_to_radians(deg: List[float]) -> List[float]:
    return [math.radians(v) for v in deg]


def radians_to_degrees(rad: List[float]) -> List[float]:
    return [math.degrees(v) for v in rad]


# =============================================================================
# High‑level motion primitives
# =============================================================================

@dataclass
class Ptp:
    """Point‑to‑point (joint‑space) motion."""
    goal: Union[List[float], RobotState]
    vel_scale: Optional[float] = None
    acc_scale: Optional[float] = None


@dataclass
class Lin:
    """Cartesian (TCP) motion to an absolute pose."""
    goal: PoseStamped
    vel_scale: Optional[float] = None
    acc_scale: Optional[float] = None


@dataclass
class LinRel:
    """Cartesian motion expressed as a frame‑relative offset (extrinsic RPY)."""
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    droll: float = 0.0    # [rad]
    dpitch: float = 0.0   # [rad]
    dyaw: float = 0.0     # [rad]
    vel_scale: Optional[float] = None
    acc_scale: Optional[float] = None


@dataclass
class Sequence:
    """Ordered collection of LIN/LinRel motions executed with optional blending."""
    items: Seq[Union[Lin, LinRel]]
    blend_radius: Union[float, List[float]] = 0.05
    vel_scale: Optional[float] = None
    acc_scale: Optional[float] = None


# =============================================================================
# Internal: thin MoveItPy wrapper
# =============================================================================

class _PilzMotionController(Node):
    def __init__(
        self,
        *,
        group: str,
        root_link: str,
        eef_link: str,
        default_lin_scaling: float,
        default_ptp_scaling: float,
        default_acc_scaling: float,
        node_name: str = "pilz_motion",
    ):
        super().__init__(node_name)

        self.group = group
        self.root_link = root_link
        self.eef_link = eef_link

        self.default_lin_scaling = default_lin_scaling
        self.default_ptp_scaling = default_ptp_scaling
        self.default_acc_scaling = default_acc_scaling

        # MoveItPy spins internally
        self.robot = MoveItPy(node_name=f"{node_name}_moveit")
        self.planning_component = self.robot.get_planning_component(group)

    # --------------------------------------------------------------------- #
    # Planning helpers
    # --------------------------------------------------------------------- #

    def _plan_single(
        self,
        *,
        planner_id: str,
        vel: float,
        acc: float,
        pose_goal: Optional[PoseStamped] = None,
        robot_state_goal: Optional[RobotState] = None,
    ):
        self.planning_component.set_start_state_to_current_state()

        if pose_goal:
            self.planning_component.set_goal_state(
                pose_stamped_msg=pose_goal, pose_link=self.eef_link
            )
        elif robot_state_goal:
            self.planning_component.set_goal_state(robot_state=robot_state_goal)
        else:
            raise ValueError("Either pose_goal or robot_state_goal must be given.")

        params = PlanRequestParameters(self.robot, "pilz_lin")
        params.planner_id = planner_id
        params.max_velocity_scaling_factor = vel
        params.max_acceleration_scaling_factor = acc

        return self.planning_component.plan(single_plan_parameters=params)

    def _plan_sequence(
        self,
        pose_goals: List[PoseStamped],
        vel: float,
        acc: float,
        blend_radius: Union[float, List[float]],
        timeout: float = 5.0,
    ) -> Optional[RobotTrajectory]:
        # Validate radii / way‑points
        if len(pose_goals) < 2:
            self.get_logger().error("Need at least two way‑points for a sequence.")
            return None

        radii: List[float]
        if isinstance(blend_radius, list):
            radii = blend_radius
        else:
            radii = [float(blend_radius)] * (len(pose_goals) - 1)

        if len(radii) != len(pose_goals) - 1:
            self.get_logger().error("blend_radius list must have len(pose_goals)-1 elements.")
            return None

        # Build request
        seq_req = MotionSequenceRequest()
        for idx, pose in enumerate(pose_goals):
            item = MotionSequenceItem()
            if idx < len(radii):
                item.blend_radius = radii[idx]

            mpr = item.req
            mpr.group_name = self.group
            mpr.planner_id = "LIN"
            mpr.max_velocity_scaling_factor = vel
            mpr.max_acceleration_scaling_factor = acc
            mpr.allowed_planning_time = 5.0
            mpr.goal_constraints.append(
                create_pose_goal(link_name=self.eef_link, target_pose=pose)
            )
            seq_req.items.append(item)

        client = self.create_client(GetMotionSequence, "/plan_sequence_path")
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/plan_sequence_path service not available.")
            return None

        future = client.call_async(GetMotionSequence.Request(request=seq_req))
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()
        if (
            resp is None
            or resp.response.error_code.val != resp.response.error_code.SUCCESS
        ):
            self.get_logger().error("Sequence planning failed.")
            return None

        return resp.response.planned_trajectory

    # --------------------------------------------------------------------- #
    # Execution helpers
    # --------------------------------------------------------------------- #

    @staticmethod
    def _apply_time_parameterization(traj: RobotTrajectory) -> None:
        traj.apply_totg_time_parameterization(
            max_velocity_scaling_factor=1.0,
            max_acceleration_scaling_factor=1.0,
            path_tolerance=0.01,
            resample_dt=0.01,
        )

    def _execute(self, traj: Optional[RobotTrajectory]) -> bool:
        if traj is None:
            return False
        self._apply_time_parameterization(traj)
        self.robot.execute(traj, controllers=[])
        return True

    # --------------------------------------------------------------------- #
    # Public helpers (called by Robot wrapper)
    # --------------------------------------------------------------------- #

    def move_ptp(
        self,
        goal: Union[List[float], RobotState],
        vel: Optional[float],
        acc: Optional[float],
    ) -> bool:
        vel = self.default_ptp_scaling if vel is None else vel
        acc = self.default_acc_scaling if acc is None else acc

        if isinstance(goal, list):
            rs = RobotState(self.robot.get_robot_model())
            rs.set_joint_group_positions(self.group, goal)
            rs.update()
            robot_state_goal = rs
        else:
            robot_state_goal = goal

        plan = self._plan_single(
            planner_id="PTP",
            robot_state_goal=robot_state_goal,
            vel=vel,
            acc=acc,
        )
        return self._execute(plan.trajectory if plan else None)

    def move_lin(
        self,
        goal: PoseStamped,
        vel: Optional[float],
        acc: Optional[float],
    ) -> bool:
        vel = self.default_lin_scaling if vel is None else vel
        acc = self.default_acc_scaling if acc is None else acc

        plan = self._plan_single(
            planner_id="LIN",
            pose_goal=goal,
            vel=vel,
            acc=acc,
        )
        return self._execute(plan.trajectory if plan else None)

    def move_sequence(
        self,
        goals: List[PoseStamped],
        blend_radius: Union[float, List[float]],
        vel: Optional[float],
        acc: Optional[float],
    ) -> bool:
        vel = self.default_lin_scaling if vel is None else vel
        acc = self.default_acc_scaling if acc is None else acc

        traj = self._plan_sequence(goals, vel, acc, blend_radius)
        return self._execute(traj)

    # --------------------------------------------------------------------- #
    # Current state helpers
    # --------------------------------------------------------------------- #

    def current_robot_state(self) -> RobotState:
        return self.planning_component.get_start_state()

    def current_pose(self) -> PoseStamped:
        rs = self.current_robot_state()
        tf = rs.get_global_link_transform(self.eef_link)
        q = tf.rotation.quaternion

        ps = PoseStamped()
        ps.header.frame_id = self.root_link
        ps.pose.position.x = tf.translation[0]
        ps.pose.position.y = tf.translation[1]
        ps.pose.position.z = tf.translation[2]
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps


# =============================================================================
# Public: user‑facing Robot wrapper
# =============================================================================

class Robot:
    """
    High‑level facade that mirrors ``pilz_robot_programming.Robot`` (ROS 1).
    """

    def __init__(
        self,
        *,
        group: str = "ur_manipulator",
        root_link: str = "base_link",
        eef_link: str = "tool0",
        default_lin_scaling: float = 0.5,
        default_ptp_scaling: float = 0.5,
        default_acc_scaling: float = 0.5,
        node_name: str = "pilz_robot",
    ):
        rclpy.init()

        self._ctrl = _PilzMotionController(
            group=group,
            root_link=root_link,
            eef_link=eef_link,
            default_lin_scaling=default_lin_scaling,
            default_ptp_scaling=default_ptp_scaling,
            default_acc_scaling=default_acc_scaling,
            node_name=f"{node_name}_ctrl",
        )
        self._user_node = Node(node_name)

        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self._ctrl)
        self._exec.add_node(self._user_node)

        import threading
        self._spin_thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin_thread.start()

    # ------------------------------------------------------------------ #
    # Motion API
    # ------------------------------------------------------------------ #

    def move(self, cmd: Union[Ptp, Lin, LinRel, Sequence]) -> bool:
        """Execute a single motion command."""
        if isinstance(cmd, Ptp):
            return self._ctrl.move_ptp(cmd.goal, cmd.vel_scale, cmd.acc_scale)

        if isinstance(cmd, Lin):
            return self._ctrl.move_lin(cmd.goal, cmd.vel_scale, cmd.acc_scale)

        if isinstance(cmd, LinRel):
            pose = self._ctrl.current_pose()
            self._apply_rel_offset(pose, cmd)
            return self._ctrl.move_lin(pose, cmd.vel_scale, cmd.acc_scale)

        if isinstance(cmd, Sequence):
            waypoints: List[PoseStamped] = []
            for item in cmd.items:
                if isinstance(item, Lin):
                    waypoints.append(item.goal)
                elif isinstance(item, LinRel):
                    p = waypoints[-1] if waypoints else self._ctrl.current_pose()
                    p = self._deepcopy_pose(p)
                    self._apply_rel_offset(p, item)
                    waypoints.append(p)
                else:
                    raise TypeError("Sequence items must be Lin or LinRel.")
            return self._ctrl.move_sequence(
                waypoints, cmd.blend_radius, cmd.vel_scale, cmd.acc_scale
            )

        raise TypeError(f"Unsupported motion type: {type(cmd)}")

    # Convenience wrappers ------------------------------------------------ #

    def ptp(
        self,
        goal: Union[List[float], RobotState],
        vel_scale: Optional[float] = None,
        acc_scale: Optional[float] = None,
    ):
        return self.move(Ptp(goal=goal, vel_scale=vel_scale, acc_scale=acc_scale))

    def lin(
        self,
        goal: PoseStamped,
        vel_scale: Optional[float] = None,
        acc_scale: Optional[float] = None,
    ):
        return self.move(Lin(goal=goal, vel_scale=vel_scale, acc_scale=acc_scale))

    # Internal utilities -------------------------------------------------- #

    @staticmethod
    def _deepcopy_pose(p: PoseStamped) -> PoseStamped:
        from copy import deepcopy as _dc
        return _dc(p)

    @staticmethod
    def _apply_rel_offset(pose: PoseStamped, offset: LinRel) -> None:
        pose.pose.position.x += offset.dx
        pose.pose.position.y += offset.dy
        pose.pose.position.z += offset.dz

        import tf_transformations as tft
        q = [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        r_orig = tft.euler_from_quaternion(q)
        r_new = (
            r_orig[0] + offset.droll,
            r_orig[1] + offset.dpitch,
            r_orig[2] + offset.dyaw,
        )
        q_new = tft.quaternion_from_euler(*r_new)
        pose.pose.orientation.x = q_new[0]
        pose.pose.orientation.y = q_new[1]
        pose.pose.orientation.z = q_new[2]
        pose.pose.orientation.w = q_new[3]

    # ------------------------------------------------------------------ #
    # Shutdown
    # ------------------------------------------------------------------ #

    def shutdown(self):
        self._exec.shutdown()
        self._ctrl.destroy_node()
        self._user_node.destroy_node()
        rclpy.shutdown()
        self._spin_thread.join()


# =============================================================================
# Quick demo when invoked directly
# =============================================================================

def _demo():
    import sys
    from copy import deepcopy as _dc

    rclpy.logging.get_logger("demo").info("Starting Pilz‑style MoveIt 2 demo …")
    robot = Robot()
    try:
        # Home pose via PTP
        robot.ptp(degrees_to_radians([45, -120, 120, -90, -90, 0]))

        # Draw a 20 cm square
        center = robot._ctrl.current_pose()
        square: List[PoseStamped] = []
        r = 0.20
        for dx, dy in [(-r, -r), (-r, r), (r, r), (r, -r)]:
            p = _dc(center)
            p.pose.position.x += dx
            p.pose.position.y += dy
            square.append(p)
        robot.move(Sequence([Lin(goal=p) for p in square], blend_radius=0.05))
    except KeyboardInterrupt:
        pass
    finally:
        robot.shutdown()
        rclpy.logging.get_logger("demo").info("Demo finished.")
        sys.exit(0)


if __name__ == "__main__":
    _demo()
