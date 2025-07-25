#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped

from PILZ_motion_controller import PilzMotionController
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
        # === Home Pose Initialization ===
        joint_deg = [90.0, -120.0, 120.0, -90.0, -90.0, 0.0]
        self.home_state = self.RobotState_from_joints(joint_deg)

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
        self.ctrl.go_to_target(self.home_state, motion_type="PTP")

    def draw_square(self, *, side: float = 0.4, z_fixed: float = 0.4):
        from copy import deepcopy as _dc

        self.home()

        home_orientation = self.ctrl.compute_fk(self.home_state).pose.orientation
        
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

        home_orientation = self.ctrl.compute_fk(self.home_state).pose.orientation
        
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
        home_orientation = self.ctrl.compute_fk(self.home_state).pose.orientation

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

        home_orientation = self.ctrl.compute_fk(self.home_state).pose.orientation

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
        
        home_orientation = self.ctrl.compute_fk(self.home_state).pose.orientation

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


    def RobotState_from_joints(self, joints: List[float], radians: bool = False) -> RobotState:
        """Create a RobotState from a 6-element list [J1, J2, J3, J4, J5, J6]."""
        assert len(joints) == 6
        if not radians:
            joints = [math.radians(j) for j in joints]
        rs = RobotState(self.ctrl.robot.get_robot_model())
        rs.set_joint_group_positions(self.ctrl.group, joints)
        rs.update()
        return rs

    @staticmethod
    def PoseStamped_from_xyzq(xyzq: List[float], frame_id: str) -> PoseStamped:
        """Build a PoseStamped from a 7-element list [x, y, z, qx, qy, qz, qw]."""
        assert len(xyzq) == 7
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