#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory

from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, String


from geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    PoseStamped
)

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


from typing import List, Optional

from trajectory_msgs.msg import JointTrajectoryPoint


def degrees_to_radians(degrees : List[float]):
    """Convert a list of joint angles from degrees to radians."""
    return [math.radians(degree) for degree in degrees]

def radians_to_degrees(radians : List[float]):
    """Convert a list of joint angles from radians to degrees."""
    return [math.degrees(radian) for radian in radians]

class PilzMotionController(Node):
    """High‑level wrapper around MoveItPy and PILZ industrial planners."""
    def __init__(
        self,
        *,
        group: str = "ur_manipulator",
        root_link: str = "base_link",
        eef_link: str = "tool0",
        home_pose: Optional[PoseStamped | RobotState | None] = None,
        default_lin_scaling: float = 0.5,
        default_ptp_scaling: float = 0.5,
        default_acc_scaling: float = 0.5,
        node_name: str = "pilz_motion",
    ):
        """Instantiate a ``PilzMotionController``.

        Parameters
        ----------
        group : str
            MoveIt planning group name.
        root_link : str
            Name of the robot’s root link.
        eef_link : str
            End‑effector link name.
        home_pose : Optional[PoseStamped | RobotState | None]
            Desired home position.  If ``None``, a built‑in joint preset is used.
        default_lin_scaling : float
            Default velocity scaling for LIN moves.
        default_ptp_scaling : float
            Default velocity scaling for PTP moves.
        default_acc_scaling : float
            Default acceleration scaling factor.
        node_name : str
            ROS 2 node name.
        """
        super().__init__(node_name)

        self.group = group
        self.root_link = root_link
        self.eef_link = eef_link

        self.default_lin_scaling = default_lin_scaling
        self.default_ptp_scaling = default_ptp_scaling
        self.default_acc_scaling = default_acc_scaling

        # Start MoveItPy (spins its own executor)
        self.robot = MoveItPy(node_name=f"{node_name}_moveit")
        self.planning_component = self.robot.get_planning_component(self.group)

        # Action client for Pilz MoveGroupSequence interface
        self.sequence_client = ActionClient(self, MoveGroupSequence, "/sequence_move_group")
        while not self.sequence_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for /sequence_move_group action server...")


        # === Home Pose ===
        if home_pose is None:
            joint_deg = [90., -120., 120., -90., -90., 0.]
            self.home_state = self.RobotState_from_joints(joint_deg)
        elif isinstance(home_pose, RobotState):
            self.home_state = home_pose
        elif isinstance(home_pose, PoseStamped):
            rs = self.compute_ik(home_pose)
            if rs is None:
                raise RuntimeError("home_pose PoseStamped unreachable (IK failed)")
            self.home_state = rs
        else:
            raise TypeError("home_pose must be PoseStamped, RobotState, or None")

        self.get_logger().info("PilzMotionController initialised.")

    # === Planning helpers ===

    def _plan_target(
        self,
        *,
        planner_id: str,
        vel: float,
        acc: float,
        pose_goal: Optional[PoseStamped] = None,
        robot_state_goal: Optional[RobotState] = None,
    ):
        """Plan a single PTP or LIN motion toward a pose or joint‑state target."""
        self.planning_component.set_start_state_to_current_state()

        if pose_goal:
            self.planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=self.eef_link)
        elif robot_state_goal:
            self.planning_component.set_goal_state(robot_state=robot_state_goal)
        else:
            raise ValueError("Either pose_goal or robot_state_goal must be supplied.")

        params = PlanRequestParameters(self.robot, "pilz_lin")
        params.planner_id = planner_id
        params.max_velocity_scaling_factor = vel
        params.max_acceleration_scaling_factor = acc

        return self.planning_component.plan(single_plan_parameters=params)

    def _build_constraints(self,
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
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = planner_id            # “LIN”, “PTP”, or “CIRC”
        req.allowed_planning_time = 10.0
        req.group_name = self.group
        req.max_acceleration_scaling_factor = acc
        req.max_velocity_scaling_factor = vel
        req.max_cartesian_speed = 0.1   #m/s
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
    
    # === Execution helpers ===
    
    def _execute_target(self, result) -> bool:
        """Time‑parameterize and execute a single‑segment trajectory."""
        if not result:
            return False
        traj = result.trajectory
        # self._apply_time_parameterization(traj)
        self.robot.execute(traj, controllers=[])
        return True
    
    def _plan_sequence(
        self,
        msr: MotionSequenceRequest,
        and_execute: bool = False,
    ) -> Optional[List[RobotTrajectory]]:
        
        if msr is None:
            return None

        goal = MoveGroupSequence.Goal()
        goal.request = msr

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = not and_execute

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

    def _execute_sequence(
        self,
        trajs: List[RobotTrajectory],
        *,
        apply_totg: bool = True,
    ) -> bool:
        if not trajs:
            return False

        for traj in trajs:
            traj_core = RobotTrajectory(self.robot.get_robot_model())
            start_state = self.planning_component.get_start_state()
            traj_core.set_robot_trajectory_msg(start_state, traj)
            traj_core.joint_model_group_name = self.group

            if apply_totg:
                ret = traj_core.apply_totg_time_parameterization(
                    velocity_scaling_factor=1.0,
                    acceleration_scaling_factor=1.0,
                    path_tolerance=0.01,
                    resample_dt=0.01,
                )
                if ret:
                    self.get_logger().info("TOTG parameterization is SUCCESSFUL!")
                else:
                    self.get_logger().error("TOTG parameterization FAILED!")

            # Empty list → default controller
            self.robot.execute(traj_core, controllers=[])
        return True

    # === IK/FK helpers ===

    def compute_ik(self, pose: PoseStamped) -> Optional[RobotState]:
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
            timeout=0.1
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
    
    def is_reachable(self, pose: PoseStamped) -> bool:
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
    def go_to_target(
        self,
        target: PoseStamped | RobotState,
        *,
        motion_type: str = "PTP",
        vel: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        """Plan and execute one motion toward ``target``.

        The function automatically chooses IK or FK depending on the
        combination of ``motion_type`` and the target’s data type.
        """
        motion_type = motion_type.upper()

        if vel is None:
            vel = self.default_lin_scaling if motion_type == "LIN" else self.default_ptp_scaling
        if acc is None:
            acc = self.default_acc_scaling

        if motion_type == "PTP":
            if isinstance(target, RobotState):
                robot_state_goal = target
            else:
                robot_state_goal = self.compute_ik(target)
                self.get_logger().debug("Converted PoseStamped to RobotState via IK for PTP target.")
                if robot_state_goal is None:
                    self.get_logger().error("IK failed; cannot plan PTP")
                    return False
            plan = self._plan_target(
                planner_id="PTP", vel=vel, acc=acc, robot_state_goal=robot_state_goal
            )
        elif motion_type == "LIN":
            if isinstance(target, PoseStamped):
                pose_goal = target
            else:
                pose_goal = self.compute_fk(target)
                self.get_logger().debug("Converted RobotState to PoseStamped via FK for LIN target")
            plan = self._plan_target(
                planner_id="LIN", vel=vel, acc=acc, pose_goal=pose_goal
            )
        else:
            raise ValueError("motion_type must be 'LIN' or 'PTP'")

        return self._execute_target(plan)

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
        """
        Plan a blended LIN motion through the given way‑points,
        apply Time‑Optimal Trajectory Generation, and execute it.

        Returns
        -------
        bool
            ``True`` on successful execution, otherwise ``False``.
        """
        assert len(targets) >= 2

        motion_type = motion_type.upper()
        if motion_type != "LIN":
            raise ValueError("motion_type must be 'LIN'")

        if vel is None:
            vel = self.default_lin_scaling
        if acc is None:
            acc = self.default_acc_scaling

        msr = self._build_motion_sequence_request(
            pose_goals=targets,
            vel=vel,
            acc=acc,
            planner_id="LIN",
            blend_radius=blend_radius,
        )

        # 1) Plan only – do not execute yet
        trajs = self._plan_sequence(msr, and_execute=False)
        if trajs is None:
            return False

        # 2) TOTG + 3) Execute
        return self._execute_sequence(trajs, apply_totg=True)


    def RobotState_from_joints(self, joints: List[float], radians = False) -> RobotState:
        """Create a RobotState from a 6-element list [J1, J2, J3, J4, J5, J6]."""
        
        assert(len(joints) == 6)

        if not radians:
            joints = degrees_to_radians(joints)
        
        robot_state = RobotState(self.robot.get_robot_model())
        robot_state.set_joint_group_positions(self.group, joints)
        robot_state.update()

        return robot_state

    @staticmethod
    def PoseStamped_from_xyzq(
        xyzq: List[float],
        frame_id: str
    ) -> PoseStamped:
        """Build a PoseStamped from a 7-element list [x, y, z, qx, qy, qz, qw]."""

        assert(len(xyzq) == 7)

        pose = PoseStamped()
        pose.header.frame_id = frame_id

        x, y, z, qx, qy, qz, qw = xyzq
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

class PilzDemo(Node):
    """Remote‑control demo node that maps textual commands on ``/user_input`` to
    high‑level motions executed by a :class:`PilzMotionController` instance."""

    def __init__(self, controller: PilzMotionController):
        super().__init__("pilz_remote")
        self.ctrl = controller
        self.create_subscription(
            String,
            "user_input",
            self.cb,
            10,
        )
        self.get_logger().info(
            "PilzDemo ready. Commands: "
            "'home', 'draw_line', 'draw_square', 'draw_square_seq', 'draw_circle', 'draw_circle_seq', 'quit'"
        )

    # ------------------------------------------------------------------
    # Command dispatcher
    # ------------------------------------------------------------------
    def cb(self, msg: String):
        """Dispatch text commands received on ``/user_input``."""
        cmd = msg.data.strip().lower()
        dispatch = {
            "home": self.home,
            "draw_square": self.draw_square,
            "draw_square_seq": self.draw_square_seq,
            "draw_circle": self.draw_circle,
            "draw_circle_seq": self.draw_circle_seq,
            "draw_line": self.draw_line,
            "quit": self._quit,
        }
        func = dispatch.get(cmd)
        if func is None:
            self.get_logger().info(f"Unknown command '{cmd}'")
        else:
            func()

    # ------------------------------------------------------------------
    # Command implementations
    # ------------------------------------------------------------------
    def home(self):
        self.ctrl.go_to_target(self.ctrl.home_state, motion_type="PTP")

    def draw_square(self, *, side: float = 0.4, z_fixed: float = 0.4):
        from copy import deepcopy as _dc

        self.home()

        home_orientation = self.ctrl.compute_fk(self.ctrl.home_state).pose.orientation
        
        center = PoseStamped()
        center.pose.position.x = 0.0
        center.pose.position.y = 0.6
        center.pose.position.z = z_fixed
        center.pose.orientation = home_orientation

        base = _dc(center.pose)
        half = side / 2.0

        # Corner sequence (BL → TL → TR → BR → BL)
        offsets = [(-half, -half), (-half, half), (half, half), (half, -half), (-half, -half)]

        for dx, dy in offsets:
            ps = PoseStamped()
            ps.header.frame_id = self.ctrl.root_link
            ps.pose = _dc(base)
            ps.pose.position.x += dx
            ps.pose.position.y += dy
            self.ctrl.go_to_target(ps, motion_type="LIN")

        self.home()

    def draw_square_seq(
        self,
        *,
        side: float = 0.4,
        z_fixed: float = 0.4,
        blend_radius: float = 0.001,
    ):
        from copy import deepcopy as _dc

        self.home()

        home_orientation = self.ctrl.compute_fk(self.ctrl.home_state).pose.orientation
        
        center = PoseStamped()
        center.pose.position.x = 0.0
        center.pose.position.y = 0.6
        center.pose.position.z = z_fixed
        center.pose.orientation = home_orientation
        
        base = _dc(center.pose)
        half = side / 2.0

        waypoints = []
        offsets = [(-half, -half), (-half, half), (half, half), (half, -half), (-half, -half)]

        for dx, dy in offsets:
            ps = PoseStamped()
            ps.header.frame_id = self.ctrl.root_link
            ps.pose = _dc(base)
            ps.pose.position.x += dx
            ps.pose.position.y += dy
            waypoints.append(ps)

        self.ctrl.run_sequence(
            waypoints,
            motion_type="LIN",
            blend_radius=blend_radius,
        )

        self.home()

    def draw_circle(
        self,
        *,
        radius: float = 0.3,
        z_fixed: float = 0.4,
        divisions: int = 36,
    ):
        """Draw a discretised circle using successive single‑segment LIN motions."""
        from copy import deepcopy as _dc
        import math

        # Start from the home pose
        self.home()

        # Use the end‑effector orientation of the home pose
        home_orientation = self.ctrl.compute_fk(self.ctrl.home_state).pose.orientation

        # Define the circle’s centre relative to the base_link
        center = PoseStamped()
        center.pose.position.x = 0.0
        center.pose.position.y = 0.7
        center.pose.position.z = z_fixed
        center.pose.orientation = home_orientation

        base = _dc(center.pose)

        # Step around the circle
        for i in range(divisions+1):
            angle = 2.0 * math.pi * i / divisions
            dx = radius * math.cos(angle)
            dy = radius * math.sin(angle)

            ps = PoseStamped()
            ps.header.frame_id = self.ctrl.root_link
            ps.pose = _dc(base)
            ps.pose.position.x += dx
            ps.pose.position.y += dy

            # Execute a linear move to the waypoint
            self.ctrl.go_to_target(ps, motion_type="LIN")

        # Return to home after completing the circle
        self.home()

    def draw_circle_seq(
        self,
        *,
        radius: float = 0.3,
        z_fixed: float = 0.4,
        divisions: int = 36,
        blend_radius: float = 0.001,
    ):
        """Draw a discretised circle using a single blended LIN trajectory."""
        from copy import deepcopy as _dc
        import math

        self.home()

        home_orientation = self.ctrl.compute_fk(self.ctrl.home_state).pose.orientation

        # Circle centre
        center = PoseStamped()
        center.pose.position.x = 0.0
        center.pose.position.y = 0.7
        center.pose.position.z = z_fixed
        center.pose.orientation = home_orientation

        base = _dc(center.pose)

        waypoints = []
        # Include divisions+1 waypoints so that the last one coincides with the first,
        # producing a closed curve when blending.
        for i in range(divisions+1):
            angle = 2.0 * math.pi * i / divisions
            dx = radius * math.cos(angle)
            dy = radius * math.sin(angle)

            ps = PoseStamped()
            ps.header.frame_id = self.ctrl.root_link
            ps.pose = _dc(base)
            ps.pose.position.x += dx
            ps.pose.position.y += dy
            waypoints.append(ps)

        # Execute the whole circle as one blended trajectory
        self.ctrl.run_sequence(
            waypoints,
            motion_type="LIN",
            blend_radius=blend_radius,
        )

        self.home()

    def draw_line(self):
        
        home_orientation = self.ctrl.compute_fk(self.ctrl.home_state).pose.orientation

        start = PoseStamped()
        start.pose.position.x = -0.2
        start.pose.position.y = 0.2
        start.pose.position.z = 0.4
        start.pose.orientation = home_orientation

        end = PoseStamped()
        end.pose.position.x = 0.2
        end.pose.position.y = 0.6
        end.pose.position.z = 0.8
        end.pose.orientation = home_orientation

        self.home()
        self.ctrl.go_to_target(start, motion_type="PTP")
        self.ctrl.go_to_target(end, motion_type="LIN")
        self.home()

    # ------------------------------------------------------------------
    # Miscellaneous helpers
    # ------------------------------------------------------------------
    def _quit(self):
        rclpy.shutdown()

def main():
    rclpy.init()
    controller = PilzMotionController()
    remote     = PilzDemo(controller)
    executor   = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(remote)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        controller.destroy_node()
        remote.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()