"""Pilz Motion Controller utilities and teleoperation helpers."""
#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped
from moveit_py.pose_goal_builder import create_pose_goal
from moveit.planning import PlanRequestParameters
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_msgs.srv import GetMotionSequence
from moveit_msgs.msg import MotionPlanRequest, MotionSequenceItem, MotionSequenceRequest
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

from typing import List, Optional


def degrees_to_radians(degrees : List[float]):
    """Convert a list of joint angles from degrees to radians."""
    return [math.radians(degree) for degree in degrees]

def radians_to_degrees(radians : List[float]):
    """Convert a list of joint angles from radians to degrees."""
    return [math.degrees(radian) for radian in radians]

class PilzMotionController(Node):
    """High‑level wrapper around MoveItPy and PILZ industrial planners.
    """
    def __init__(
        self,
        *,
        group: str = "ur_manipulator",
        root_link: str = "base_link",
        eef_link: str = "tool0",
        home_pose: Optional[PoseStamped | RobotState | None],
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

        # === Home Pose ===
        if home_pose is None:
            joint_deg = [45., -120., 120., -90., -90., 0.]
            self.home_state = self.RobotState_from_joints(angles_to_radians(joint_deg))
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

    def _plan_sequence(
        self,
        pose_goals: List[PoseStamped],
        vel: float,
        acc: float,
        blend_radius: float | List[float] = 0.0,
        timeout: float = 5.0,
    ) -> Optional[RobotTrajectory]:
        """Plan a Pilz LIN MotionSequence.

        Parameters
        ----------
        pose_goals : list[PoseStamped]
            Way‑points the TCP should visit (≥ 2, same frame_id).
        vel / acc : float
            Cartesian velocity/acceleration scaling factors (0…1).
        blend_radius : float | list[float]
            • If a single float, the same radius is used between every
              consecutive pair of segments.
            • If a list, it must contain ``len(pose_goals)‑1`` values.
              ``0.0`` forces a complete stop between segments.
        timeout : float
            Seconds to wait for ``/plan_sequence_path`` to appear.

        Returns
        -------
        moveit.core.robot_trajectory.RobotTrajectory | None
            A ready‑to‑execute trajectory on success, otherwise *None*.
        """
        # --- Validate input -------------------------------------------------
        if len(pose_goals) < 2:
            self.get_logger().error("Need at least two way‑points for a sequence.")
            return None

        if isinstance(blend_radius, list):
            radii = blend_radius
        else:
            radii = [float(blend_radius)] * (len(pose_goals) - 1)

        if len(radii) != len(pose_goals) - 1:
            self.get_logger().error(
                "blend_radius list must have len(pose_goals)‑1 entries."
            )
            return None

        # --- Build MotionSequenceRequest -----------------------------------
        seq_req = MotionSequenceRequest()

        for idx, pose in enumerate(pose_goals):
            item = MotionSequenceItem()

            # Apply blending except for the very last segment
            if idx < len(radii):
                item.blend_radius = radii[idx]

            req = item.req
            req.group_name = self.group
            req.planner_id = "LIN"
            req.max_velocity_scaling_factor = vel
            req.max_acceleration_scaling_factor = acc
            req.allowed_planning_time = 5.0

            # Cart. goal: let MoveItPy build the constraints boilerplate
            req.goal_constraints.append(
                create_pose_goal(link_name=self.eef_link, target_pose=pose)
            )

            seq_req.items.append(item)

        # --- Call the Pilz sequence service ---------------------------------
        client = self.create_client(GetMotionSequence, "/plan_sequence_path")
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/plan_sequence_path service not available.")
            return None

        future = client.call_async(GetMotionSequence.Request(request=seq_req))
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()
        if resp is None:
            self.get_logger().error("No response from /plan_sequence_path.")
            return None

        if resp.response.error_code.val != resp.response.error_code.SUCCESS:
            self.get_logger().error(
                f"Sequence planning failed (code={resp.response.error_code.val})."
            )
            return None

        return resp.response.planned_trajectory

    # === Execution helpers ===

    def _apply_time_parameterization(self, traj: RobotTrajectory) -> None:
        """Apply time parameterization (TOTG) to the trajectory."""
        traj.apply_totg_time_parameterization(1.0, 1.0, path_tolerance=0.01, resample_dt=0.01)
    
    def _execute_target(self, result) -> bool:
        """Time‑parameterize and execute a single‑segment trajectory."""
        if not result:
            return False
        traj = result.trajectory
        self._apply_time_parameterization(traj)
        self.robot.execute(traj, controllers=[])
        return True
    
    def _execute_sequence(self, traj: RobotTrajectory) -> bool:
        """Execute a planned RobotTrajectory (sequence)."""
        if traj is None:
            return False
        self._apply_time_parameterization(traj)
        self.robot.execute(traj, controllers=[])
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
        tf = state.get_global_link_transform(self.eef_link)
        q  = tf.rotation.quaternion
        return self.PoseStamped_from_xyzq(
            tf.translation[0], tf.translation[1], tf.translation[2],
            q[0], q[1], q[2], q[3],
            self.root_link,
        )

    # === Utility & waypoint generators ===
    def make_square(
        self,
        *,
        center_pose: Optional[PoseStamped] | None = None,
        r: float = 0.2,
        z_fixed: float = 0.5,
    ) -> List[PoseStamped]:
        from copy import deepcopy as _dc

        if center_pose is None:
            self.planning_component.set_start_state_to_current_state()
            start_ps = self.planning_component.get_start_state().get_pose(self.eef_link)
            base_pose = _dc(start_ps)
        else:
            base_pose = _dc(center_pose.pose)

        base_pose.position.z = z_fixed

        wp = []
        # bottom‑left
        p0 = _dc(base_pose)
        p0.position.x -= r
        p0.position.y -= r
        wp.append(p0)
        # top‑left
        p1 = _dc(base_pose)
        p1.position.x -= r
        p1.position.y += r
        wp.append(p1)
        # top‑right
        p2 = _dc(base_pose)
        p2.position.x += r
        p2.position.y += r
        wp.append(p2)
        # bottom‑right
        p3 = _dc(base_pose)
        p3.position.x += r
        p3.position.y -= r
        wp.append(p3)

        # Wrap into PoseStamped messages
        waypoints = []
        for pose in wp:
            ps = PoseStamped()
            ps.header.frame_id = self.root_link
            ps.pose = pose
            waypoints.append(ps)
        return waypoints

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
    def go_to_trajectory(
        self,
        targets: List[PoseStamped],
        *,
        motion_type: str = "LIN",
        blend_radius: float = 0.1,
        vel: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        """Execute a list of waypoints as a multi-waypoint (LIN) trajectory."""
        motion_type = motion_type.upper()
        if motion_type != "LIN":
            raise ValueError("motion_type must be 'LIN'")
        if vel is None:
            vel = self.default_lin_scaling
        if acc is None:
            acc = self.default_acc_scaling
        if len(targets) == 1:
            return self.go_to_target(
                targets[0], motion_type=motion_type, vel=vel, acc=acc
            )
        # Plan and execute a multi-waypoint (LIN) trajectory
        traj = self._plan_sequence(targets, vel, acc, blend_radius)
        return self._execute_sequence(traj)


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

class PilzRemote(Node):
    """Minimal remote control node that listens to ``/user_input`` and calls the controller."""
    def __init__(self, controller: PilzMotionController):
        super().__init__("pilz_remote")
        self.ctrl = controller
        self.create_subscription(
            String,
            "user_input",
            self.cb,
            10,
        )
        self.get_logger().info("PilzRemote ready. Use 'h','l','q' on /user_input.")

    def cb(self, msg: String):
        """Handle single‑character remote control commands received on ``/user_input``."""
        cmd = msg.data.strip().lower()
        if cmd == "h":
            self.ctrl.go_to_target(self.ctrl.home_state, motion_type="PTP")
        elif cmd == "l":
            waypoints = self.ctrl.make_square(r=0.2, z_fixed=0.5)
            self.ctrl.go_to_trajectory(waypoints)
        elif cmd == "q":
            self.get_logger().info("Quit; shutting down.")
            rclpy.shutdown()
        else:
            self.get_logger().info(f"Unknown command '{cmd}'")

def main():
    rclpy.init()
    controller = PilzMotionController()
    remote     = PilzRemote(controller)
    executor   = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(remote)
    executor.spin()
    executor.shutdown()
    controller.destroy_node()
    remote.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
