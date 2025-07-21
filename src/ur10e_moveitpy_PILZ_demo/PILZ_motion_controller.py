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
from moveit.planning import PlanRequestParameters as PRP
from moveit.core.robot_trajectory import RobotTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.srv import GetMotionSequence
from moveit_msgs.msg import MotionPlanRequest, MotionSequenceItem, MotionSequenceRequest

from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.kinematic_constraints import construct_link_constraint

from typing import List, Optional

GROUP     = "ur_manipulator"
ROOT_LINK = "base_link"
EEF_LINK = "tool0"

def angles_to_radians(angles : List[float]):
    return [math.radians(angle) for angle in angles]

class PilzMotionController(Node):
    def __init__(
        self,
        *,
        group: str = GROUP,
        root_link: str = ROOT_LINK,
        eef_link: str = EEF_LINK,
        home_pose: Optional[PoseStamped | RobotState | None] = None,
        default_lin_scaling: float = 0.5,
        default_ptp_scaling: float = 0.5,
        default_acc_scaling: float = 0.5,
        node_name: str = "pilz_motion",
    ):
        super().__init__(node_name)

        # ─── Home pose ───────────────────────────────────────────────────────
        if home_pose is None:
            joint_deg = [90., -120., 120., -90., -90., 0.]
            self.home_state = self.RobotState_from_joints(angles_to_radians(joint_deg))
        elif isinstance(home_pose, PoseStamped):
            rs = self.compute_ik(home_pose)
            if rs is None:
                raise RuntimeError("home_pose PoseStamped unreachable (IK failed)")
            self.home_state = rs
        elif isinstance(home_pose, RobotState):
            self.home_state = home_pose
        else:
            raise TypeError("home_pose must be PoseStamped, RobotState, or None")

        self.group = group
        self.root_link = root_link
        self.eef_link = eef_link

        self.default_lin_scaling = default_lin_scaling
        self.default_ptp_scaling = default_ptp_scaling
        self.default_acc_scaling = default_acc_scaling

        # Start MoveItPy (spins its own executor)
        self.robot = MoveItPy(node_name=f"{node_name}_moveit")
        self.pc     = self.robot.get_planning_component(self.group)

        self.get_logger().info("PilzMotionController initialised.")

    # ─── Planning helpers ───────────────────────────────────────────────────

    def _plan_target(
        self,
        *,
        planner_id: str,
        vel: float,
        acc: float,
        pose_goal: Optional[PoseStamped] = None,
        robot_state_goal: Optional[RobotState] = None,
    ):
        self.pc.set_start_state_to_current_state()

        if pose_goal:
            self.pc.set_goal_state(pose_stamped_msg=pose_goal, pose_link=self.eef_link)
        elif robot_state_goal:
            self.pc.set_goal_state(robot_state=robot_state_goal)
        else:
            raise ValueError("Either pose_goal or robot_state_goal must be supplied.")

        params = PRP(self.robot, "pilz_industrial_motion_planner")
        params.planner_id = planner_id
        params.max_velocity_scaling_factor = vel
        params.max_acceleration_scaling_factor = acc

        return self.pc.plan(single_plan_parameters=params)

    def _plan_sequence(
        self,
        pose_goals: List[PoseStamped],
        vel: float,
        acc: float,
        blend_radius: float = 0.0,
    ) -> RobotTrajectory:
        seq_req = MotionSequenceRequest()
        for idx, pose in enumerate(pose_goals):
            item = MotionSequenceItem()
            item.blend_radius = blend_radius
            mpr = MotionPlanRequest()
            mpr.group_name = self.group
            mpr.planner_id = "LIN"
            mpr.max_velocity_scaling_factor = vel
            mpr.max_acceleration_scaling_factor = acc
            if idx == 0:
                mpr.start_state = self.pc.get_start_state().to_msg()
            # Build goal_constraints from pose
            goal = construct_link_constraint(
                self.pc.get_start_state(),
                self.eef_link,
                pose.pose,
                1e-3,  # pos tolerance [m]
                1e-2,  # rot tolerance [rad]
            )
            mpr.goal_constraints.append(goal)
            item.req = mpr
            seq_req.items.append(item)
        # Call the service
        client = self.create_client(GetMotionSequence, "plan_sequence_path")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("plan_sequence_path service not available")
            return None
        srv_req = GetMotionSequence.Request()
        srv_req.request = seq_req
        future = client.call_async(srv_req)
        rclpy.spin_until_future_complete(self, future)
        if (
            future.result() is None
            or future.result().response.error_code.val != future.result().response.error_code.SUCCESS
        ):
            self.get_logger().error("Sequence planning failed")
            return None
        trajectory = future.result().response.planned_trajectory
        return trajectory

    # ─── Execution helpers ───────────────────────────────────────────────────

    def _apply_time_parameterization(self, traj: RobotTrajectory) -> None:
        """Apply time parameterization (TOTG) to the trajectory."""
        traj.apply_totg_time_parameterization(1.0, 1.0, path_tolerance=0.01, resample_dt=0.01)
    
    def _execute_target(self, result) -> bool:
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

    # ─── IK/FK helpers ───────────────────────────────────────────────────

    def compute_ik(self, pose: PoseStamped) -> Optional[RobotState]:
        """Compute IK for the end-effector; return RobotState or None."""
        # Seed from current state for better IK results
        current_rs = self.pc.get_start_state()
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

    # ─── Utility & waypoint generators ──────────────────────────────────────
    def make_square(
        self,
        *,
        center_pose: Optional[PoseStamped] | None = None,
        r: float = 0.2,
        z_fixed: float = 0.5,
    ) -> List[PoseStamped]:
        import copy as _cp

        if center_pose is None:
            self.pc.set_start_state_to_current_state()
            start_ps = self.pc.get_start_state().get_pose(self.eef_link)
            base_pose = _cp.deepcopy(start_ps)
        else:
            base_pose = _cp.deepcopy(center_pose.pose)

        base_pose.position.z = z_fixed

        wp = []
        # bottom‑left
        p0 = _cp.deepcopy(base_pose)
        p0.position.x -= r; p0.position.y -= r; wp.append(p0)
        # top‑left
        p1 = _cp.deepcopy(base_pose)
        p1.position.x -= r; p1.position.y += r; wp.append(p1)
        # top‑right
        p2 = _cp.deepcopy(base_pose)
        p2.position.x += r; p2.position.y += r; wp.append(p2)
        # bottom‑right
        p3 = _cp.deepcopy(base_pose)
        p3.position.x += r; p3.position.y -= r; wp.append(p3)

        # Wrap into PoseStamped messages
        waypoints = []
        for pose in wp:
            ps = PoseStamped()
            ps.header.frame_id = self.root_link
            ps.pose = pose
            waypoints.append(ps)
        return waypoints

    # ─── Diagnostics & reachability ─────────────────────────────────────────
    def is_reachable(self, pose: PoseStamped) -> bool:
        """Return True if pose has an IK solution and is collision‑free."""
        rs = self.compute_ik(pose)
        if rs is None:
            return False  # no IK solution

        # Collision check via PlanningComponent if available
        try:
            return bool(self.pc.is_state_valid(robot_state=rs))
        except AttributeError:
            # Fallback: assume reachable if IK worked
            return True


    # -------------------------------------------------------
    # Generic single‑target motion primitive
    # -------------------------------------------------------
    def go_to_target(
        self,
        target: PoseStamped,
        *,
        motion_type: str = "LIN",
        vel: Optional[float] = None,
        acc: Optional[float] = None,
    ) -> bool:
        """Plan & execute a single target with either LIN or PTP."""
        motion_type = motion_type.upper()
        if motion_type not in ("LIN", "PTP"):
            raise ValueError("motion_type must be 'LIN' or 'PTP'")

        if vel is None:
            vel = self.default_lin_scaling if motion_type == "LIN" else self.default_ptp_scaling
        if acc is None:
            acc = self.default_acc_scaling

        result = self._plan_target(
            planner_id=motion_type,
            vel=vel,
            acc=acc,
            pose_goal=target,
        )
        ok = self._execute(result)
        self.get_logger().info(
            f"go_to_target({motion_type}) {'executed' if ok else 'planning failed'}."
        )
        return ok

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


    # ----------- 

    def RobotState_from_joints(self, joints: List[float]) -> RobotState:
        """Create a RobotState from joint values."""
        rs = RobotState(self.robot.get_robot_model())
        rs.set_joint_group_positions(self.group, joints)
        rs.update()
        return rs

    @staticmethod
    def PoseStamped_from_xyzq(
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        frame_id: str
    ) -> PoseStamped:
        """Build a PoseStamped from XYZ and quaternion."""
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

class PilzTeleop(Node):
    def __init__(self, controller: PilzMotionController):
        super().__init__("pilz_teleop")
        self.ctrl = controller
        self.create_subscription(
            String,
            "user_input",
            self.cb,
            10,
        )
        self.get_logger().info("PilzTeleop ready. Use 'h','l','q' on /user_input.")

    def cb(self, msg: String):
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
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
