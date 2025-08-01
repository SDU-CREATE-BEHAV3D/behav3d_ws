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

import rclpy
from rclpy.node import Node

# MoveIt‑Py imports – keep these lazy so the node can start without MoveIt if
# someone just wants Marker publishing.
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.planning import MoveItPy

from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import DisplayTrajectory

from builtin_interfaces.msg import Duration

class MotionVisualizer(Node):
    """ROS 2 node that converts *moveit_py* objects into RViz markers."""

    def __init__(self):
        super().__init__("pilz_motion_visualizer")

        # Standard DisplayTrajectory topic that RViz’s MotionPlanning panel listens to.
        self.ghost_pub = self.create_publisher(DisplayTrajectory,
                                               "/display_planned_path", 10)
        # Generic markers for trail / frames, use latched QoS so late RViz joins see them.
        self.marker_pub = self.create_publisher(MarkerArray,
                                                "/motion_markers", 10)

        # Convenience MoveItPy handle – mostly to sample TCP easily.
        self._moveit = MoveItPy(node_name="viz_moveit")
        self._tf_buffer = None  # could inject tf2 buffer for frame conversions

    # ---------------------------------------------------------------------
    # Public helpers
    # ---------------------------------------------------------------------
    def publish_ghost(self, traj: RobotTrajectory, ns: str = "ghost") -> None:
        """Publish a DisplayTrajectory so RViz shows the animated ghost robot."""
        if traj is None:
            self.get_logger().warning("publish_ghost: trajectory is None")
            return

        msg = DisplayTrajectory()
        msg.model_id = self._moveit.get_robot_model().get_name()
        msg.trajectory.append(traj.to_msg())
        # Use current state for start if available
        start_state = self._moveit.get_planning_scene().get_current_state()
        msg.trajectory_start = start_state.to_robot_state_msg()
        self.ghost_pub.publish(msg)
        self.get_logger().debug("Ghost trajectory published.")

    def publish_trail(self, traj: RobotTrajectory, dt: float = 0.02,
                       ns: str = "tcp_trail") -> None:
        """Sample the TCP along *traj* every *dt* seconds and draw a line strip."""
        if traj is None:
            return
        marker = Marker()
        marker.header.frame_id = traj.get_robot_state_at(0).get_root_link_name()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.001  # line width 1 mm
        marker.color.r = marker.color.g = 1.0
        marker.color.a = 1.0

        duration = traj.get_duration()  # total time in seconds
        t = 0.0
        while t <= duration:
            state = traj.get_robot_state_at(t)
            pose = state.get_global_link_transform(traj.joint_model_group_name)
            pt = Point(x=pose.translation.x,
                       y=pose.translation.y,
                       z=pose.translation.z)
            marker.points.append(pt)
            t += dt
        ma = MarkerArray(markers=[marker])
        self.marker_pub.publish(ma)
        self.get_logger().debug("TCP trail published with %d points" % len(marker.points))

    def publish_target_pose(self, pose: PoseStamped, ns: str = "target_pose") -> None:
        """Visualise a single target pose as coordinate axes (3 coloured arrows)."""
        ma = MarkerArray()
        axes = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]  # RGB
        for i, (r, g, b) in enumerate(axes):
            m = Marker()
            m.header = pose.header
            m.ns = ns
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = 0.005  # shaft diameter
            m.scale.y = 0.01   # head diameter
            m.scale.z = 0.0    # head length (ignored for ARROW)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0

            # Build arrow from origin to axis vector (0.1 m) in local frame.
            start = Point()  # (0,0,0)
            end = Point(x=0.1 * r, y=0.1 * g, z=0.1 * b)
            m.points = [start, end]
            ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.get_logger().debug("Target axes published.")


# -------------------------------------------------------------------------
# ROS entry‑point
# -------------------------------------------------------------------------

def main():
    rclpy.init()
    node = MotionVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()