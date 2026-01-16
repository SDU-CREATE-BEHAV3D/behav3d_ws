#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from behav3d_interfaces.srv import (
    PlanWithMoveIt,
    PlanCartesianPath,
    PlanPilzPtp,
    PlanPilzLin,
    PlanPilzSequence,
)

import yaml
import time
import math


class PoseSequenceRunner(Node):
    def __init__(self):
        super().__init__("pose_sequence_runner")

        # Declare parameters
        self.declare_parameter("poses_file", "")
        self.declare_parameter("planning_frame", "world")
        self.declare_parameter("preview_only", True)
        self.declare_parameter("action_timeout_sec", 20.0)

        # Clients
        self.moveit_client = self.create_client(PlanWithMoveIt, "/behav3d/plan_with_moveit")
        self.cartesian_client = self.create_client(PlanCartesianPath, "/behav3d/plan_cartesian_path")
        self.pilz_ptp_client = self.create_client(PlanPilzPtp, "/behav3d/plan_pilz_ptp")
        self.pilz_lin_client = self.create_client(PlanPilzLin, "/behav3d/plan_pilz_lin")
        self.pilz_seq_client = self.create_client(PlanPilzSequence, "/behav3d/plan_pilz_sequence")

    def run(self):
        poses_file = self.get_parameter("poses_file").get_parameter_value().string_value
        planning_frame = self.get_parameter("planning_frame").get_parameter_value().string_value
        preview_only = self.get_parameter("preview_only").get_parameter_value().bool_value
        action_timeout = self.get_parameter("action_timeout_sec").get_parameter_value().double_value

        # Load yaml
        with open(poses_file, "r") as f:
            data = yaml.safe_load(f)

        meta = data.get("meta", {})
        targets = data.get("targets", [])

        self.get_logger().info(f"[runner] Starting sequence with {len(targets)} targets")

        seq_buffer = []

        for idx, tgt in enumerate(targets, start=1):
            tname = tgt.get("name", f"target_{idx}")
            ttype = tgt.get("type", "named")

            # Collect pilz_lin items for blending
            if ttype == "pilz_lin":
                seq_buffer.append(tgt)
                continue
            else:
                # Flush buffered pilz_lin sequence if any
                if seq_buffer:
                    ok = self._send_pilz_sequence(meta, seq_buffer, planning_frame, preview_only, action_timeout)
                    if not ok:
                        self.get_logger().warn(f"[runner] Pilz sequence failed before {tname}; stopping")
                        return
                    seq_buffer = []

            self.get_logger().info(f"[runner] Executing {idx}/{len(targets)}: {tname}")

            ok = False
            if ttype == "named":
                ok = self._send_named(meta, tgt, planning_frame, preview_only, action_timeout)
            elif ttype == "cartesian_path":
                ok = self._send_cartesian(meta, tgt, planning_frame, preview_only, action_timeout)
            elif ttype == "pilz_ptp":
                ok = self._send_pilz_ptp(meta, tgt, planning_frame, preview_only, action_timeout)
            elif ttype == "pilz_lin":
                ok = True  # already handled by buffer
            elif ttype == "pilz_sequence":
                seq_items = tgt.get("targets") or tgt.get("points") or []
                if not seq_items:
                    self.get_logger().error("[runner] pilz_sequence has no 'targets' (or 'points')")
                    return
                ok = self._send_pilz_sequence(meta, seq_items, planning_frame, preview_only, action_timeout)

            else:
                self.get_logger().error(f"[runner] Unknown target type: {ttype}")

            if not ok:
                self.get_logger().warn(f"[runner] Target {tname} failed; stopping")
                return

            self.get_logger().info(f"[runner] Completed {idx}/{len(targets)}: {tname}")
            time.sleep(float(meta.get("hold_sec", 0.0)))

        # Flush tail sequence
        if seq_buffer:
            ok = self._send_pilz_sequence(meta, seq_buffer, planning_frame, preview_only, action_timeout)
            if not ok:
                self.get_logger().warn("[runner] Final Pilz sequence failed; stopping")
                return

    # --- Helpers ---

    def _send_named(self, meta, tgt, planning_frame, preview_only, timeout):
        if not self.moveit_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[runner] MoveIt service not available")
            return False

        req = PlanWithMoveIt.Request()
        req.frame_id = meta.get("frame_id", planning_frame)
        req.group_name = tgt.get("group_name", "ur_arm")
        req.eef_link = tgt.get("eef_link", "")
        req.pipeline_id = tgt.get("pipeline_id", "ompl")
        req.velocity_scale = float(tgt.get("velocity_scale", 0.2))
        req.accel_scale = float(tgt.get("accel_scale", 0.2))
        req.preview_only = preview_only
        req.named_target = tgt.get("named_target", "")

        fut = self.moveit_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False
        return bool(fut.result() and fut.result().success)

    def _send_cartesian(self, meta, tgt, planning_frame, preview_only, timeout):
        if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[runner] Cartesian service not available")
            return False

        cart = tgt.get("cartesian", {})

        req = PlanCartesianPath.Request()
        req.group_name = cart.get("group_name", "ur_arm")
        req.eef_link = cart.get("eef_link", "")
        req.velocity_scale = float(cart.get("velocity_scale", 0.2))
        req.accel_scale = float(cart.get("accel_scale", 0.2))
        req.preview_only = preview_only
        req.frame_id = meta.get("frame_id", planning_frame)
        req.max_step = float(cart.get("max_step", 0.005))
        req.jump_threshold = float(cart.get("jump_threshold", 0.0))
        req.avoid_collisions = bool(cart.get("avoid_collisions", False))

        waypoints = []
        for wp in cart.get("waypoints", []):
            ps = Pose()
            ps.position.x = float(wp["position"]["x"])
            ps.position.y = float(wp["position"]["y"])
            ps.position.z = float(wp["position"]["z"])
            ps.orientation = self._rpy_to_quat(wp["rpy"])
            waypoints.append(ps)
        req.waypoints = waypoints

        fut = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False
        return bool(fut.result() and fut.result().success)

    def _send_pilz_ptp(self, meta, tgt, planning_frame, preview_only, timeout):
        if not self.pilz_ptp_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[runner] Pilz PTP service not available")
            return False

        req = PlanPilzPtp.Request()
        req.group_name = tgt.get("group_name", "ur_arm")
        named = tgt.get("named_target", "")
        if named:
            req.eef_link = ""  # pure joint goal
        else:
            req.eef_link = tgt.get("eef_link", "")

        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        req.velocity_scale = float(tgt.get("velocity_scale", 0.2))
        req.accel_scale = float(tgt.get("accel_scale", 0.2))
        req.preview_only = preview_only
        req.named_target = named

        if not named and "target" in tgt:
            pose = PoseStamped()
            pose.header.frame_id = meta.get("frame_id", planning_frame)
            pose.pose.position.x = float(tgt["target"]["position"]["x"])
            pose.pose.position.y = float(tgt["target"]["position"]["y"])
            pose.pose.position.z = float(tgt["target"]["position"]["z"])
            pose.pose.orientation = self._rpy_to_quat(tgt["target"]["rpy"])
            req.target = pose

        fut = self.pilz_ptp_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False
        return bool(fut.result() and fut.result().success)

    def _send_pilz_lin(self, meta, tgt, planning_frame, preview_only, timeout):
        if not self.pilz_lin_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[runner] Pilz LIN service not available")
            return False

        req = PlanPilzLin.Request()
        req.group_name = tgt.get("group_name", "ur_arm")
        req.eef_link = tgt.get("eef_link", "")
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "LIN"
        req.velocity_scale = float(tgt.get("velocity_scale", 0.2))
        req.accel_scale = float(tgt.get("accel_scale", 0.2))
        req.preview_only = preview_only

        if "target" in tgt:
            pose = PoseStamped()
            pose.header.frame_id = meta.get("frame_id", planning_frame)
            pose.pose.position.x = float(tgt["target"]["position"]["x"])
            pose.pose.position.y = float(tgt["target"]["position"]["y"])
            pose.pose.position.z = float(tgt["target"]["position"]["z"])
            pose.pose.orientation = self._rpy_to_quat(tgt["target"]["rpy"])
            req.target = pose
        else:
            self.get_logger().error("[runner] pilz_lin needs a 'target' pose")
            return False

        fut = self.pilz_lin_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False
        return bool(fut.result() and fut.result().success)

    def _send_pilz_sequence(self, meta, tgts, planning_frame, preview_only, timeout):
        if not self.pilz_seq_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[runner] Pilz sequence service not available")
            return False

        req = PlanPilzSequence.Request()
        req.group_name = tgts[0].get("group_name", "ur_arm")
        req.eef_link = tgts[0].get("eef_link", "")
        req.velocity_scale = float(tgts[0].get("velocity_scale", 0.2))
        req.accel_scale = float(tgts[0].get("accel_scale", 0.2))
        req.preview_only = preview_only
        req.targets, req.blend_radii = [], []

        for i, tgt in enumerate(tgts):
            if "target" not in tgt:
                self.get_logger().error(f"[runner] sequence item {i} missing 'target'")
                return False
            pose = Pose()
            pose.position.x = float(tgt["target"]["position"]["x"])
            pose.position.y = float(tgt["target"]["position"]["y"])
            pose.position.z = float(tgt["target"]["position"]["z"])
            pose.orientation = self._rpy_to_quat(tgt["target"]["rpy"])
            req.targets.append(pose)
            req.blend_radii.append(float(tgt.get("blend_radius", 0.0)))

        fut = self.pilz_seq_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False
        return bool(fut.result() and fut.result().success)

    def _send_pilz_sequence_from_points(self, meta, tgt, planning_frame, preview_only, timeout):
        if not self.pilz_seq_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[runner] Pilz sequence service not available")
            return False

        req = PlanPilzSequence.Request()
        req.group_name = tgt.get("group_name", "ur_arm")
        req.eef_link = tgt.get("eef_link", "")
        req.velocity_scale = float(tgt.get("velocity_scale", 0.2))
        req.accel_scale = float(tgt.get("accel_scale", 0.2))
        req.preview_only = preview_only
        req.targets, req.blend_radii = [], []

        default_br = float(tgt.get("blend_radius", 0.0))
        for i, p in enumerate(tgt.get("points", [])):
            pose = Pose()
            pose.position.x = float(p["position"]["x"])
            pose.position.y = float(p["position"]["y"])
            pose.position.z = float(p["position"]["z"])
            pose.orientation = self._rpy_to_quat(p["rpy"])
            req.targets.append(pose)
            req.blend_radii.append(float(p.get("blend_radius", default_br)))

        fut = self.pilz_seq_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return False
        return bool(fut.result() and fut.result().success)

    def _rpy_to_quat(self, rpy):
        r, p, y = float(rpy["r"]), float(rpy["p"]), float(rpy["y"])
        cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
        cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
        cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main():
    rclpy.init()
    node = PoseSequenceRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
