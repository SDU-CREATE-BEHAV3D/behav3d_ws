#!/usr/bin/env python3
          
# =============================================================================
#   ____  _____ _   _    ___     _______ ____  
#  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
#  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
#  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
#  |____/|_____|_| |_/_/   \_\_/  |____/|____/ 
#                                               
#                                               
# Author: Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
# Maintainers:
#   - Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
#   - Lucas José Helle <luh@iti.sdu.dk>
# Institute: University of Southern Denmark (Syddansk Universitet)
# Date: 2025-07
# =============================================================================

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.logging import LoggingSeverity

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
    PositionConstraint,
    WorkspaceParameters
)
from moveit_msgs.msg import MoveItErrorCodes, PlanningOptions


from typing import List, Optional, Union

from dataclasses import dataclass

@dataclass
class WorkspaceDefaults:
    frame_id = "base_link"
    min_corner_x = -2.0
    min_corner_y = -0.25
    min_corner_z =  0.0
    max_corner_x =  2.0
    max_corner_y =  2.0
    max_corner_z =  2.0


@dataclass
class MotionDefaults:
    ptp_scaling: float = 0.5
    linear_scaling: float = 0.5
    acceleration_scaling: float = 0.5
    max_cartesian_speed: float = 0.1
    constraint_position_tolerance: float = 0.0001       # 0.1 mm
    constraint_orientation_tolerance: float = 0.001     # ~0.06 deg
    blend_radius_default: float = 0.001
    totg_path_tolerance: float = 0.01
    totg_resample_dt: float = 0.01
    planning_timeout: float = 10.0
    planning_pipeline: str = "pilz_industrial_motion_planner"
    default_planner_ptp: str = "PTP"
    default_planner_lin: str = "LIN"

class PilzMotionController(Node):
    """MoveIt/PILZ motion controller exposing high-level planning and execution API"""
    def __init__(
            self,
            *,
            node_name: str = "pilz_motion",
            group: str = "ur_arm",
            root_link: str = "ur10e_base_link",
            eef_link: str = "ur10e_tool0",
            debug: bool = False,
        ):
        
        super().__init__(node_name)
        # Configure logger level based on debug flag
        if debug:
            self.get_logger().set_level(LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(LoggingSeverity.INFO)
        
        self.motion_defaults = MotionDefaults()
        self.workspace_defaults = WorkspaceDefaults()

        self.workspace_parameters = WorkspaceParameters()
        self.workspace_parameters.header.frame_id   = self.workspace_defaults.frame_id
        self.workspace_parameters.min_corner.x      = self.workspace_defaults.min_corner_x
        self.workspace_parameters.min_corner.y      = self.workspace_defaults.min_corner_y
        self.workspace_parameters.min_corner.z      = self.workspace_defaults.min_corner_z
        self.workspace_parameters.max_corner.x      = self.workspace_defaults.max_corner_x
        self.workspace_parameters.max_corner.y      = self.workspace_defaults.max_corner_y
        self.workspace_parameters.max_corner.z      = self.workspace_defaults.max_corner_z

        self.group = group
        self.root_link = root_link
        self.eef_link = eef_link
        # Log initial parameters
        self.get_logger().debug(
            f"PilzMotionController initialised with "
            f"node_name={node_name}, group={group}, root_link={root_link}, "
            f"eef_link={eef_link}, debug={debug}"
        )

        self.robot = MoveItPy(node_name=f"{node_name}_moveit")
        self.planning_component = self.robot.get_planning_component(self.group)
        self.planning_component.set_workspace(
            self.workspace_defaults.min_corner_x,
            self.workspace_defaults.min_corner_y,
            self.workspace_defaults.min_corner_z,
            self.workspace_defaults.max_corner_x,
            self.workspace_defaults.max_corner_y,
            self.workspace_defaults.max_corner_z,
        )


        self.sequence_client = ActionClient(self, MoveGroupSequence, "/sequence_move_group")
        while not self.sequence_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for /sequence_move_group action server...")


    # --- Planning helpers ---
    def _build_constraints(
            self,
            pose_goal: PoseStamped,
            frame_id: str,
            link: str,
            pos_tolerance: float = 0.001,
            ori_tolerance: float = 0.01
        ) -> Constraints:
        """Return a tight position/orientation constraint around *pose_goal*."""
        self.get_logger().debug(
            f"_build_constraints: frame={frame_id}, link={link}, "
            f"pos_tol={pos_tolerance}, ori_tol={ori_tolerance}"
        )
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
        """Create a :class:`MotionPlanRequest` for the PILZ planner."""
        req = MotionPlanRequest()
        req.workspace_parameters = self.workspace_parameters
        req.pipeline_id = self.motion_defaults.planning_pipeline
        req.planner_id = planner_id
        req.allowed_planning_time = self.motion_defaults.planning_timeout
        req.group_name = self.group
        req.max_acceleration_scaling_factor = acc
        req.max_velocity_scaling_factor = vel
        req.max_cartesian_speed = self.motion_defaults.max_cartesian_speed
        req.goal_constraints.append(
            self._build_constraints(
                pose_goal,
                frame_id=self.root_link,
                link=self.eef_link,
                pos_tolerance=blend_radius,
                ori_tolerance=ori_tolerance,
            )
        )
        self.get_logger().debug(
            f"_build_motion_plan_request: pipeline={req.pipeline_id}, "
            f"planner_id={req.planner_id}, time={req.allowed_planning_time}s, "
            f"vel_scale={req.max_velocity_scaling_factor}, acc_scale={req.max_acceleration_scaling_factor}"
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
        """Build a :class:`MotionSequenceRequest` from a list of way‑points."""
        self.get_logger().debug(
            f"_build_motion_sequence_request: {len(pose_goals)} way-points, "
            f"vel={vel}, acc={acc}, planner_id={planner_id}, "
            f"blend_radius={blend_radius}"
        )
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
        msr.items[-1].blend_radius = 0.0 #Last waypoint should have blend_radius = 0.0

        return msr
    
    def _to_core_trajectory(self, traj_msg):
        """Convert a ROS-level RobotTrajectory message into a MoveItPy RobotTrajectory object."""
        self.get_logger().debug(
            f"_to_core_trajectory: Converting ROS msg to RobotTrajectory"        )
        core_traj = RobotTrajectory(self.robot.get_robot_model())
        start_state = self.planning_component.get_start_state()
        core_traj.set_robot_trajectory_msg(start_state, traj_msg)
        core_traj.joint_model_group_name = self.group
        return core_traj

    def _concat_traj_msgs(self, traj_msgs: List) -> RobotTrajectory:
        """Concatenate multiple moveit_msgs/RobotTrajectory segments into one core RobotTrajectory. (Emulates the missing C++ `append()` function.)
        """
        self.get_logger().debug(
            f"_concat_traj_msgs: Concatenating {len(traj_msgs)} segments."
        )
        from copy import deepcopy
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        assert traj_msgs, "traj_msgs must contain at least one segment"

        # Prepare an empty RobotTrajectory message
        combined_msg = type(traj_msgs[0])()
        combined_msg.joint_trajectory = JointTrajectory()
        combined_msg.joint_trajectory.joint_names = deepcopy(
            traj_msgs[0].joint_trajectory.joint_names
        )

        t_offset = 0.0
        for seg in traj_msgs:
            for pt in seg.joint_trajectory.points:
                new_pt = JointTrajectoryPoint()
                new_pt.positions = deepcopy(pt.positions)
                new_pt.velocities = deepcopy(pt.velocities)
                new_pt.accelerations = deepcopy(pt.accelerations)
                new_pt.effort = deepcopy(pt.effort)

                # Convert Duration to float seconds
                pt_time = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                cumulative_time = t_offset + pt_time

                # Back to Duration msg
                new_pt.time_from_start = Duration()
                new_pt.time_from_start.sec = int(cumulative_time)
                new_pt.time_from_start.nanosec = int(
                    (cumulative_time - int(cumulative_time)) * 1e9
                )

                combined_msg.joint_trajectory.points.append(new_pt)

            # Advance offset by the last point's time
            last_pt_time = (
                seg.joint_trajectory.points[-1].time_from_start.sec
                + seg.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
            )
            t_offset += last_pt_time

        self.get_logger().debug(
            f"_concat_traj_msgs: Combined trajectory has "
            f"{len(combined_msg.joint_trajectory.points)} points."
        )
        return self._to_core_trajectory(combined_msg)

    def _build_sequence_trajectory(
        self,
        traj_msgs: List,
        *,
        combine: bool = True,
    ):
        """Return either a single concatenated RobotTrajectory or a list of per‑segment ones.

        *combine = True* → returns one continuous RobotTrajectory  
        *combine = False* → returns ``List[RobotTrajectory]`` for per‑segment execution
        """
        if combine:
            return self._concat_traj_msgs(traj_msgs)
        else:
            return [self._to_core_trajectory(msg) for msg in traj_msgs]
    
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
        ) -> Optional[RobotTrajectory]:
        """Plan a joint‑space (PTP) move and return a trajectory or ``None``."""
        self.get_logger().debug(
            f"plan_ptp: target_type={type(target).__name__}, vel={vel}, acc={acc}, timeout={timeout}"
        )
        vel        = vel        or self.motion_defaults.ptp_scaling
        acc        = acc        or self.motion_defaults.acceleration_scaling
        timeout    = timeout    or self.motion_defaults.planning_timeout

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
        params.planning_time = timeout

        result = self.planning_component.plan(single_plan_parameters=params)
        if result:
            self.get_logger().debug(
                f"plan_ptp: planned in {result.planning_time:.3f}s."
            )
            return result.trajectory
        return None

    def plan_lin(
            self,
            target: PoseStamped | RobotState,
            *,
            vel: Optional[float] = None,
            acc: Optional[float] = None,
            timeout: Optional[float] = None,
        ) -> Optional[RobotTrajectory]:
        """Plan a Cartesian (LIN) move and return a trajectory or ``None``."""
        self.get_logger().debug(
            f"plan_lin: target_type={type(target).__name__}, "
            f"vel={vel}, acc={acc}, timeout={timeout}"
        )
        vel        = vel        or self.motion_defaults.linear_scaling
        acc        = acc        or self.motion_defaults.acceleration_scaling
        timeout    = timeout    or self.motion_defaults.planning_timeout

        if isinstance(target, RobotState):
            pose_goal = self.compute_fk(target)
        else:
            pose_goal = target

        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=self.eef_link)

        params = PlanRequestParameters(self.robot, "pilz_lin")
        params.planner_id = "LIN"
        params.max_velocity_scaling_factor = vel
        params.max_acceleration_scaling_factor = acc
        params.planning_time = timeout

        result = self.planning_component.plan(single_plan_parameters=params)
        if result:
            self.get_logger().debug(
                f"plan_lin: planned "
                # f"{len(result.trajectory.joint_trajectory.points)} points "
                f"in {result.planning_time:.3f}s."
            )
            return result.trajectory
        return None
    
    def plan_sequence(
        self,
        targets: List[PoseStamped],
        *,
        vel: Optional[float] = None,
        acc: Optional[float] = None,
        blend_radius: float = None,
        ori_tolerance: float = None,
    ) -> Optional[RobotTrajectory]:
        """Plan a multi-waypoint motion sequence through *targets* and return a RobotTrajectory or None."""
        vel = vel or self.motion_defaults.linear_scaling
        acc = acc or self.motion_defaults.acceleration_scaling
        planner_id = "LIN"
        blend_radius = blend_radius or self.motion_defaults.blend_radius_default
        ori_tolerance = ori_tolerance or self.motion_defaults.constraint_orientation_tolerance

        self.get_logger().debug(
            f"plan_sequence: {len(targets)} way-points, "
            f"vel={vel}, acc={acc}, blend_radius={blend_radius}"
        )

        msr = self._build_motion_sequence_request(
            pose_goals=targets,
            vel=vel,
            acc=acc,
            planner_id=planner_id,
            blend_radius=blend_radius,
            ori_tolerance=ori_tolerance,
        )
        if msr is None:
            return None

        goal = MoveGroupSequence.Goal()
        goal.request = msr
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True

        self.get_logger().debug("plan_sequence: sending goal to action server…")
        send_goal_future = self.sequence_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Sequence goal rejected by action server.")
            return None
        
        self.get_logger().debug("plan_sequence: goal accepted by action server.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_msg = result_future.result().result
        response = result_msg.response

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Sequence planning failed (code={response.error_code.val})."
            )
            return None
        self.get_logger().info(
            f"MotionSequence was planned successfully in {response.planning_time} seconds."
        )

        return self._build_sequence_trajectory(
            response.planned_trajectories,
            combine=True  # set False if you want per‑segment execution
        )
    
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
        """Execute a trajectory, applying TOTG when requested."""
        self.get_logger().debug(
            "execute_trajectory: received "
            f"{'list of ' if isinstance(traj, list) else ''}trajectory."
        )
        if traj is None:
            return False

        # Handle list‑of‑segments transparently
        if isinstance(traj, list):
            for t in traj:
                if not self.execute_trajectory(
                    t,
                    apply_totg=apply_totg,
                    publish_markers=publish_markers,
                ):
                    return False
                self.get_logger().debug("execute_trajectory: executed segment successfully.")
            return True
        
        vel = self.motion_defaults.linear_scaling
        acc = self.motion_defaults.acceleration_scaling

        # Single RobotTrajectory branch
        if apply_totg:
            traj.apply_totg_time_parameterization(
                velocity_scaling_factor=vel,
                acceleration_scaling_factor=acc,
                path_tolerance=self.motion_defaults.totg_path_tolerance,
                resample_dt=self.motion_defaults.totg_resample_dt,
            )

        self.get_logger().debug(
            f"execute_trajectory: apply_totg={apply_totg}, "
            # f"points={len(traj.joint_trajectory.points)}"
        )
        try:
            self.robot.execute(traj, controllers=[])
        except Exception as e:
            self.get_logger().error(f"Execution failed: {e}")
            return False
        self.get_logger().debug("execute_trajectory: execution completed.")
        return True

    # --- IK/FK helpers ---

    def get_pose(
            self,
    ) -> PoseStamped:
        # TODO: Implement a function that returns the curent pose of the move group
        return
    
    def get_tcp_vel(
            self,
    ):
        # TODO: Implement a function that returns the current velocity and direction vector
        return

    def compute_ik(
            self,
            pose: PoseStamped,
            *,
            timeout: float = 0.1
        ) -> Optional[RobotState]:
        """Return a :class:`RobotState` that reaches *pose* or ``None`` on failure."""
        self.get_logger().debug("compute_ik: solving IK…")
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
            self.get_logger().error("IK solution failed.")
            return None
        else:
            self.get_logger().debug("compute_ik: IK solution found.")
        rs.update()
        return rs

    def compute_fk(self, state: RobotState) -> PoseStamped:
        self.get_logger().debug("compute_fk: computing FK.")
        """Return the end‑effector pose for *state*."""
        pose = state.get_pose(self.eef_link)
        ps = PoseStamped()
        ps.header.frame_id = self.root_link
        ps.pose = pose
        return ps

    # --- Diagnostics & reachability ---
    
    def is_reachable(
            self,
            pose: PoseStamped
        ) -> bool:
        """Quick reachability test (IK + optional collision check)."""
        rs = self.compute_ik(pose)
        if rs is None:
            return False  # no IK solution

        # Collision check via PlanningComponent if available
        try:
            valid = bool(self.planning_component.is_state_valid(robot_state=rs))
            self.get_logger().debug(f"is_reachable: collision check returned {valid}")
            return valid
        except AttributeError:
            self.get_logger().warning("is_reachable: no collision‑check interface, assuming reachable")
            return True

    # -------------------------------------------------------
    # Generic single‑target motion primitive
    # -------------------------------------------------------
    def go_to(
        self,
        target: PoseStamped | RobotState,
        *,
        motion_type: str = "PTP",
        vel: Optional[float] = None,
        acc: Optional[float] = None,
        timeout: float = 10.0,
    ) -> bool:
        """Plan **and** execute one LIN or PTP move."""
        motion_type = motion_type.upper()
        self.get_logger().debug(
            f"go_to: starting {motion_type} move to {target} "
            f"with vel={vel}, acc={acc}, timeout={timeout}"
        )

        if motion_type == "PTP":
            traj = self.plan_ptp(
                target,
                vel=vel,
                acc=acc,
                timeout=timeout,
            )
        elif motion_type == "LIN":
            traj = self.plan_lin(
                target,
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
        blend_radius: float = 0.001,
        vel: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        """Plan and execute a blended LIN trajectory through *targets*."""
        assert len(targets) >= 2
        self.get_logger().debug(f"run_sequence: planning blended LIN through {len(targets)} points")

        vel = vel or self.motion_defaults.linear_scaling
        acc = acc or self.motion_defaults.acceleration_scaling

        traj_list = self.plan_sequence(
            targets=targets,
            vel=vel,
            acc=acc,
            blend_radius=blend_radius
        )
        if traj_list is None:
            return False

        return self.execute_trajectory(traj_list, apply_totg=True)