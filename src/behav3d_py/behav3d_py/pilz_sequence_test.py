#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from behav3d_py.commands import Commands
from geometry_msgs.msg import Pose, PoseStamped
from behav3d_interfaces.srv import PlanPilzLin
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from rclpy.action import ActionClient


class PilzSequenceTest(Node):
    def __init__(self):
        super().__init__("pilz_sequence_test")
        self.cmd = Commands(self)

        # Clients
        self.lin_cli = self.create_client(PlanPilzLin, "/behav3d/plan_pilz_lin")
        self.ctrl_ac = ActionClient(self, FollowJointTrajectory,
                                    "/scaled_joint_trajectory_controller/follow_joint_trajectory")

        self.eef_link = "extruder_tcp"
        self.vel_scale = 0.10
        self.acc_scale = 0.10
        self.target = self._mk_pose(-0.1965, 0.9550, -0.0440)

        self._started = False
        self.create_timer(0.2, self._kickoff)

    def _mk_pose(self, x, y, z):
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose = Pose()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        return ps

    def _kickoff(self):
        if self._started:
            return
        self._started = True
        self.cmd.home(duration_s=2.0, on_move_done=self._on_home_done)

    def _on_home_done(self, res):
        if not res["ok"]:
            self.get_logger().error("HOME failed")
            rclpy.shutdown()
            return
        self.get_logger().info("HOME done, planning LIN…")
        self._plan_lin_and_modify(self.target)

    def _plan_lin_and_modify(self, pose_stamped):
        if not self.lin_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("plan_pilz_lin not available")
            rclpy.shutdown()
            return
        req = PlanPilzLin.Request()
        req.group_name = "ur_arm"
        req.eef_link = self.eef_link
        req.velocity_scale = self.vel_scale
        req.accel_scale = self.acc_scale
        req.preview_only = True
        req.target = pose_stamped
        fut = self.lin_cli.call_async(req)
        fut.add_done_callback(self._on_lin_planned)

    def _on_lin_planned(self, future):
        resp = future.result()
        if not resp or not resp.success:
            self.get_logger().error("LIN planning failed")
            rclpy.shutdown()
            return

        jt: JointTrajectory = resp.trajectory.joint_trajectory
        self.get_logger().info(f"Original traj has {len(jt.points)} points")

        # Log original
        for i, pt in enumerate(jt.points):
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            self.get_logger().info(f"  ORIG PT {i:03d}: t={t:.3f}")

        # Modify: add +1s from PT 5 onward
        for i, pt in enumerate(jt.points):
            if i >= 15:
                t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 + 4.0
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t - int(t)) * 1e9)

        # Log modified
        self.get_logger().info("Modified trajectory (+1s from PT 5 onward):")
        for i, pt in enumerate(jt.points):
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            self.get_logger().info(f"  MOD PT {i:03d}: t={t:.3f}")

        # Send to controller
        if not self.ctrl_ac.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Controller action not available")
            rclpy.shutdown()
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        send_fut = self.ctrl_ac.send_goal_async(goal)
        send_fut.add_done_callback(self._on_exec_sent)

    def _on_exec_sent(self, fut):
        handle = fut.result()
        if not handle or not handle.accepted:
            self.get_logger().error("Trajectory goal rejected")
            rclpy.shutdown()
            return
        self.get_logger().info("Trajectory accepted, waiting for result…")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self._on_exec_done)

    def _on_exec_done(self, fut):
        res = fut.result().result
        self.get_logger().info(f"Execution finished, error_code={res.error_code}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = PilzSequenceTest()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
