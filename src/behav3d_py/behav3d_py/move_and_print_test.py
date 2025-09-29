#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import Pose, PoseStamped
from behav3d_interfaces.srv import PlanPilzPtp

class MoveAndPrintTest(Node):
    def __init__(self):
        super().__init__('move_and_print_test')

        # Controller action server for UR
        self.controller_action_name = '/scaled_joint_trajectory_controller/follow_joint_trajectory'

        # Joint names for UR20
        self.joint_names = [
            'ur20_shoulder_pan_joint',
            'ur20_shoulder_lift_joint',
            'ur20_elbow_joint',
            'ur20_wrist_1_joint',
            'ur20_wrist_2_joint',
            'ur20_wrist_3_joint',
        ]
        # Service client to Motion Bridge (Pilz PTP planning)
        self.ptp_cli = self.create_client(PlanPilzPtp, '/behav3d/plan_pilz_ptp')

        # Simple state: 0 = go home, 1 = plan+execute with EEF
        self._stage = 0
        # Home configuration in degrees (adjust as needed)
        home_deg = [-90.0, -120.0, 120.0, 0.0, 90.0, -180.0]
        self.home_rad = [math.radians(d) for d in home_deg]

        # Action client
        self.client = ActionClient(self, FollowJointTrajectory, self.controller_action_name)

        self.get_logger().info('wena choro — ready to send home position')

        # Timer to try sending the goal once the action server is up
        self.create_timer(0.5, self._try_send_once)
        self._sent = False

    def _try_send_once(self):
        if self._sent:
            return
        if not self.client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('waiting for controller action server...')
            return

        self._sent = True
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        # Single trajectory point at target positions
        pt = JointTrajectoryPoint()
        pt.positions = self.home_rad
        pt.time_from_start.sec = 10  #

        goal.trajectory.points.append(pt)

        self.get_logger().info('sending home trajectory...')
        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_sent_cb)

    def _goal_sent_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('goal rejected by controller')
            rclpy.shutdown()
            return
        self.get_logger().info('goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'controller result: error_code={result.error_code}')

        if self._stage == 0:
            # Home done -> plan next move using a specific EEF
            self._stage = 1
            self.get_logger().info('planning PTP to target pose using EEF "extruder_tcp"...')
            self._plan_ptp_with_eef()
        else:
            # Second execution finished -> shutdown
            rclpy.shutdown()

    def _plan_ptp_with_eef(self):
        # Wait for bridge service to be available
        if not self.ptp_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('plan_pilz_ptp service not available')
            rclpy.shutdown()
            return

        # Build a target PoseStamped (adjust as needed)
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.pose = Pose()
        ps.pose.position.x = -0.0500
        ps.pose.position.y = 1.30
        ps.pose.position.z = 0.31
        ps.pose.orientation.w = 1.0  # simple orientation

        # Fill request
        req = PlanPilzPtp.Request()
        req.group_name = 'ur_arm'
        req.eef_link = 'extruder_tcp'         # <-- specific EEF
        req.velocity_scale = 0.1
        req.accel_scale = 0.1
        req.preview_only = True               # plan only; we execute ourselves
        req.target = ps

        future = self.ptp_cli.call_async(req)
        future.add_done_callback(self._on_ptp_plan)

    def _on_ptp_plan(self, future):
        resp = future.result()
        if resp is None or not resp.success:
            self.get_logger().error('planning failed or no response')
            rclpy.shutdown()
            return

        # Motion Bridge returns a RobotTrajectory; we need the JointTrajectory
        jt = resp.trajectory.joint_trajectory
        if len(jt.points) == 0:
            self.get_logger().error('planned trajectory has no points')
            rclpy.shutdown()
            return

        # Send the planned joint trajectory to the controller (same action client)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self.get_logger().info('sending planned trajectory to controller...')
        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_sent_cb)

def main(args=None):
    rclpy.init(args=args)
    node = MoveAndPrintTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
