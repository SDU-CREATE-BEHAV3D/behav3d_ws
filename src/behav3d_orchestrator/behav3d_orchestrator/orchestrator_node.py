#!/usr/bin/env python3
import time
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from behav3d_interfaces.action import PlanAndExecute
from behav3d_interfaces.srv import PlanWithMoveIt
from geometry_msgs.msg import PoseStamped

class Orchestrator(Node):
    def __init__(self):
        super().__init__('behav3d_orchestrator')

        # Defaults (override via --ros-args -p key:=val)
        self.declare_parameter('group_name', 'ur_arm')
        self.declare_parameter('pipeline_id', 'ompl')
        self.declare_parameter('velocity_scale', 0.2)
        self.declare_parameter('accel_scale', 0.2)
        self.declare_parameter('eef_link', 'ur20_tool0')

        # Bridge client
        self.bridge_cli = self.create_client(PlanWithMoveIt, '/behav3d/plan_with_moveit')

        self._srv = ActionServer(
            self, PlanAndExecute, '/behav3d/plan_and_execute',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )
        self.get_logger().info('Orchestrator ready: /behav3d/plan_and_execute')

    def goal_cb(self, goal_request: PlanAndExecute.Goal):
        self.get_logger().info(
            f"Goal received: preview_only={goal_request.preview_only} "
            f"named_target={repr(getattr(goal_request, 'named_target', ''))}"
        )
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        # Feedback: planning
        fb = PlanAndExecute.Feedback()
        fb.phase = "planning"
        fb.percent_complete = 10.0
        goal_handle.publish_feedback(fb)

        # Ensure bridge service is up
        if not await self._wait_for_bridge():
            self.get_logger().error("Bridge service not available")
            result = PlanAndExecute.Result()
            result.success = False
            result.moveit_error_code = 99999
            goal_handle.abort()
            return result

        # Build request from params + goal
        req = PlanWithMoveIt.Request()
        req.group_name  = self.get_parameter('group_name').get_parameter_value().string_value
        req.pipeline_id = self.get_parameter('pipeline_id').get_parameter_value().string_value
        req.velocity_scale = float(self.get_parameter('velocity_scale').get_parameter_value().double_value)
        req.accel_scale    = float(self.get_parameter('accel_scale').get_parameter_value().double_value)
        req.eef_link       = self.get_parameter('eef_link').get_parameter_value().string_value

        # From action goal: prefer named_target; if a pose exists, use it
        named_target = getattr(goal_handle.request, 'named_target', '')
        if named_target:
            req.named_target = named_target
        else:
            # Optional: if your action has a PoseStamped field named 'pose'
            pose = getattr(goal_handle.request, 'pose', None)
            if isinstance(pose, PoseStamped):
                req.pose = pose
            else:
                self.get_logger().error("No target provided (named_target or pose)")
                result = PlanAndExecute.Result()
                result.success = False
                result.moveit_error_code = 99998
                goal_handle.abort()
                return result

        req.preview_only = bool(goal_handle.request.preview_only)

        # Call bridge
        try:
            self.get_logger().info(
                f"Calling bridge: group={req.group_name} planner={req.pipeline_id} "
                f"vel={req.velocity_scale} acc={req.accel_scale} "
                f"target={'named:' + req.named_target if req.named_target else 'pose'} "
                f"preview_only={req.preview_only}"
            )
            res = await self._call_bridge(req)
        except Exception as e:
            self.get_logger().exception(f"Bridge call failed: {e}")
            result = PlanAndExecute.Result()
            result.success = False
            result.moveit_error_code = 99997
            goal_handle.abort()
            return result

        # Feedback: executing (only for execute mode)
        if not req.preview_only:
            fb.phase = "executing"
            fb.percent_complete = 80.0
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)

        # Fill action result from bridge response
        out = PlanAndExecute.Result()
        out.success = bool(res.success)
        out.moveit_error_code = int(res.moveit_error_code)
        out.total_time_sec = float(res.total_time_sec)
        out.path_length = float(res.path_length)
        out.final_pose = res.final_pose

        if out.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return out

    async def _wait_for_bridge(self, timeout_sec: float = 5.0) -> bool:
        start = self.get_clock().now()
        while not self.bridge_cli.wait_for_service(timeout_sec=0.2):
            if (self.get_clock().now() - start).nanoseconds * 1e-9 > timeout_sec:
                return False
        return True

    async def _call_bridge(self, req: PlanWithMoveIt.Request) -> PlanWithMoveIt.Response:
        fut = self.bridge_cli.call_async(req)
        return await fut

def main():
    rclpy.init()
    node = Orchestrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
