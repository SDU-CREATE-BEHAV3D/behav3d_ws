#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory

from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from geometry_msgs.msg import Pose, PoseStamped

from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MotionSequenceItem,
    MotionSequenceRequest,
    OrientationConstraint,
    PositionConstraint
)
from moveit_msgs.msg import MoveItErrorCodes, PlanningOptions


from typing import List, Optional, Union

from dataclasses import dataclass

@dataclass
class MotionDefaults:
    ptp_scaling: float = 0.5
    linear_scaling: float = 0.5
    acceleration_scaling: float = 0.5
    max_cartesian_speed: float = 0.1
    constraint_position_tolerance: float = 0.001
    constraint_orientation_tolerance: float = 0.01
    blend_radius_default: float = 0.001
    totg_path_tolerance: float = 0.01
    totg_resample_dt: float = 0.01
    planning_timeout: float = 10.0
    planning_pipeline: str = "pilz_industrial_motion_planner"
    default_planner_ptp: str = "PTP"
    default_planner_lin: str = "LIN"

class PilzMotionController(Node):
    """High‑level wrapper around MoveItPy and PILZ industrial planners."""
    def __init__(
            self,
            *,
            node_name: str = "pilz_motion",
            group: str = "ur_manipulator",
            root_link: str = "base_link",
            eef_link: str = "tool0",
        ):
        
        super().__init__(node_name)
        self.defaults = MotionDefaults()

        self.group = group
        self.root_link = root_link
        self.eef_link = eef_link

        # Start MoveItPy (spins its own executor)
        self.robot = MoveItPy(node_name=f"{node_name}_moveit")
        self.planning_component = self.robot.get_planning_component(self.group)

        # Action client for Pilz MoveGroupSequence interface
        self.sequence_client = ActionClient(self, MoveGroupSequence, "/sequence_move_group")
        while not self.sequence_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for /sequence_move_group action server...")

        self.get_logger().info("PilzMotionController initialised.")

    # === Planning helpers ===
    def _build_constraints(
            self,
            pose_goal: PoseStamped,
            frame_id: str,
            link: str,
            pos_tolerance: float = 0.001,
            ori_tolerance: float = 0.01
        ) -> Constraints:
        
        pc = PositionConstraint()
        pc.header = Header(frame_id=frame_id)
        pc.link_name = link
        # A tiny spherical region around the goal
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE,
                                dimensions=[pos_tolerance])
        vol = BoundingVolume(primitives=[sphere],
                            primitive_poses=[Pose(position=pose_goal.pose.position)])
        pc.constraint_region = vol
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = Header(frame_id=frame_id)
        oc.link_name = link
        oc.orientation = pose_goal.pose.orientation
        oc.absolute_x_axis_tolerance = ori_tolerance
        oc.absolute_y_axis_tolerance = ori_tolerance
        oc.absolute_z_axis_tolerance = ori_tolerance
        oc.weight = 1.0

        return Constraints(
            position_constraints=[pc],
            orientation_constraints=[oc],
        )


    def _build_motion_plan_request(
            self,
            pose_goal: PoseStamped,
            vel: float,
            acc: float,
            planner_id: str = "LIN",
            blend_radius: float = 0.001,
            ori_tolerance: float = 0.01,
        ) -> MotionPlanRequest:

        req = MotionPlanRequest()
        req.pipeline_id = self.defaults.planning_pipeline
        req.planner_id = planner_id
        req.allowed_planning_time = self.defaults.planning_timeout
        req.group_name = self.group
        req.max_acceleration_scaling_factor = acc
        req.max_velocity_scaling_factor = vel
        req.max_cartesian_speed = self.defaults.max_cartesian_speed
        req.goal_constraints.append(
            self._build_constraints(
                pose_goal,
                frame_id=self.root_link,
                link=self.eef_link,
                pos_tolerance=blend_radius,
                ori_tolerance=ori_tolerance,
            )
        )
        return req

    def _build_motion_sequence_request(
            self,
            pose_goals: List[PoseStamped],
            vel: float,
            acc: float,
            planner_id: str = "LIN",
            blend_radius: float = 0.001,
            ori_tolerance: float = 0.01,
        ) -> MotionSequenceRequest | None:

        if len(pose_goals) < 2:
            self.get_logger().error("Need at least two way‑points for a sequence.")
            return None

        msr = MotionSequenceRequest()

        for idx, pose in enumerate(pose_goals):
            item = MotionSequenceItem(
                blend_radius=blend_radius,
                req=self._build_motion_plan_request(
                    pose_goal=pose,
                    planner_id=planner_id,
                    vel=vel,
                    acc=acc,
                    blend_radius=blend_radius,
                    ori_tolerance=ori_tolerance,
                ),
            )
            msr.items.append(item)
        msr.items[-1].blend_radius = 0.0    #Last waypoint should have blend_radius = 0.0

        return msr
    
    # -------------------------------------------------------
    # Public planning API
    # -------------------------------------------------------
    def plan_ptp(
            self,
            target: PoseStamped | RobotState,
            *,
            vel: Optional[float] = None,
            acc: Optional[float] = None,
            timeout: Optional[float] = None,
        ) -> Optional[List[RobotTrajectory]]:
        """Plan a point‑to‑point (joint‑space) motion and return the trajectory."""
        vel        = vel        or self.defaults.ptp_scaling
        acc        = acc        or self.defaults.acceleration_scaling
        timeout    = timeout    or self.defaults.planning_timeout

        if isinstance(target, PoseStamped):
            robot_state_goal = self.compute_ik(target, timeout=timeout)
            if robot_state_goal is None:
                self.get_logger().error("IK failed; cannot plan PTP")
                return None
        else:
            robot_state_goal = target

        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(robot_state=robot_state_goal)

        params = PlanRequestParameters(self.robot, "pilz_lin")  #TODO: Implement pilz_ptp?
        params.planner_id = "PTP"
        params.max_velocity_scaling_factor = vel
        params.max_acceleration_scaling_factor = acc
        params.allowed_planning_time = timeout

        result = self.planning_component.plan(single_plan_parameters=params)
        return [result.trajectory] if result else None

    def plan_lin(
            self,
            target: PoseStamped | RobotState,
            *,
            vel: Optional[float] = None,
            acc: Optional[float] = None,
            timeout: Optional[float] = None,
        ) -> Optional[List[RobotTrajectory]]:
        """Plan a Cartesian (linear) motion and return the trajectory."""
        # Returns: list with a single RobotTrajectory for compatibility with execute_trajectory
        vel        = vel        or self.defaults.linear_scaling
        acc        = acc        or self.defaults.acceleration_scaling
        timeout    = timeout    or self.defaults.planning_timeout

        if isinstance(target, RobotState):
            pose_goal = self.compute_fk(target)
        else:
            pose_goal = target

        # --- build and solve the plan directly ---
        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=self.eef_link)

        params = PlanRequestParameters(self.robot, "pilz_lin")
        params.planner_id = "LIN"
        params.max_velocity_scaling_factor = vel
        params.max_acceleration_scaling_factor = acc
        params.allowed_planning_time = timeout

        result = self.planning_component.plan(single_plan_parameters=params)
        return [result.trajectory] if result else None
    
    def plan_sequence(
            self,
            msr: MotionSequenceRequest,
        ) -> Optional[List[RobotTrajectory]]:
        
        if msr is None:
            return None

        goal = MoveGroupSequence.Goal()
        goal.request = msr

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True

        send_goal_future = self.sequence_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Sequence goal rejected by action server.")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_msg = result_future.result().result  # MoveGroupSequence.Result
        response = result_msg.response

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Sequence planning failed (code={response.error_code.val})."
            )
            return None
        else:
            self.get_logger().info(f"MotionSequence was planned successfully in {response.planning_time} seconds.")
        return list(response.planned_trajectories)
    
    # -------------------------------------------------------
    # Public execution API
    # -------------------------------------------------------
    def execute_trajectory(
            self, 
            traj: Optional[Union[RobotTrajectory, List[RobotTrajectory]]],
            *, 
            apply_totg: bool = True, 
            publish_markers: bool = False,
        ) -> bool:
       
        if traj is None:
            return False

        # Handle sequences transparently
        if isinstance(traj, list):
            for t in traj:
                if not self.execute_trajectory(t, apply_totg=apply_totg, publish_markers=publish_markers):
                    return False
            return True

        # Convert message ↔ core representation if necessary
        if isinstance(traj, RobotTrajectory):
            core_traj = traj
        else:  # assume a ROS message
            core_traj = RobotTrajectory(self.robot.get_robot_model())
            start_state = self.planning_component.get_start_state()
            core_traj.set_robot_trajectory_msg(start_state, traj)
            core_traj.joint_model_group_name = self.group

        if apply_totg:
            core_traj.apply_totg_time_parameterization(
                velocity_scaling_factor=1.0,
                acceleration_scaling_factor=1.0,
                path_tolerance=self.defaults.totg_path_tolerance,
                resample_dt=self.defaults.totg_resample_dt,
            )

        # TODO: publish ghost arm to RViz
        self.robot.execute(core_traj, controllers=[])
        return True

    # === IK/FK helpers ===

    def compute_ik(
            self,
            pose: PoseStamped,
            *,
            timeout: float = 0.1
        ) -> Optional[RobotState]:
       
        current_rs = self.planning_component.get_start_state()
        rs = RobotState(self.robot.get_robot_model())
        rs.set_joint_group_positions(
            self.group,
            current_rs.get_joint_group_positions(self.group)
        )
        rs.update()
        # Solve IK
        success = rs.set_from_ik(
            self.group,
            pose.pose,
            self.eef_link,
            timeout=timeout
        )
        if not success:
            return None
        rs.update()
        return rs

    def compute_fk(self, state: RobotState) -> PoseStamped:
        pose = state.get_pose(self.eef_link)
        ps = PoseStamped()
        ps.header.frame_id = self.root_link
        ps.pose = pose
        return ps

    # === Diagnostics & reachability ===
    
    def is_reachable(
            self,
            pose: PoseStamped
            ) -> bool:
        """Return True if pose has an IK solution and is collision‑free."""
        rs = self.compute_ik(pose)
        if rs is None:
            return False  # no IK solution

        # Collision check via PlanningComponent if available
        try:
            return bool(self.planning_component.is_state_valid(robot_state=rs))
        except AttributeError:
            # Fallback: assume reachable if IK worked
            return True

    # -------------------------------------------------------
    # Generic single‑target motion primitive
    # -------------------------------------------------------
    def go_to(
        self,
        target: PoseStamped | RobotState,
        *,
        motion_type: str = "PTP",
        planner_id: Optional[str] = None,
        vel: Optional[float] = None,
        acc: Optional[float] = None,
        timeout: float = 10.0,
    ) -> bool:
      
        motion_type = motion_type.upper()
        planner_id = planner_id or motion_type

        if motion_type == "PTP":
            traj = self.plan_ptp(
                target,
                planner_id=planner_id,
                vel=vel,
                acc=acc,
                timeout=timeout,
            )
        elif motion_type == "LIN":
            traj = self.plan_lin(
                target,
                planner_id=planner_id,
                vel=vel,
                acc=acc,
                timeout=timeout,
            )
        else:
            raise ValueError("motion_type must be 'LIN' or 'PTP'")

        return self.execute_trajectory(traj)

    # -------------------------------------------------------
    # Multi‑waypoint Cartesian (LIN) trajectory
    # -------------------------------------------------------
    def run_sequence(
        self,
        targets: List[PoseStamped],
        *,
        motion_type: str = "LIN",
        blend_radius: float = 0.001,
        vel: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:

        assert len(targets) >= 2

        motion_type = motion_type.upper()
        if motion_type != "LIN":
            raise ValueError("motion_type must be 'LIN'")

        vel = vel or self.defaults.linear_scaling
        acc = acc or self.defaults.acceleration_scaling

        msr = self._build_motion_sequence_request(
            pose_goals=targets,
            vel=vel,
            acc=acc,
            planner_id="LIN",
            blend_radius=blend_radius,
        )

        traj_list = self.plan_sequence(msr)
        if traj_list is None:
            return False

        return self.execute_trajectory(traj_list, apply_totg=True)