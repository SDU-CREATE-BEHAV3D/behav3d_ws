# For running this node first:
# ros2 launch behav3d_bringup print_move.launch.py robot_ip:=192.168.1.8 use_mock_hardware:=true

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import behav3d_examples

import math
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

class HandeyeCaptureSeq(Node):
    """Demo: HOME → GOTO(plan+exec). Single on_move_done callback for all moves."""
    def __init__(self):
        super().__init__('handeye_capture_sequence')
        self.session = behav3d_examples.ScanSession(self)
        self._started = False
        self.create_timer(0.25, self._run_once)

    def _run_once(self):
        if self._started:
            return
        self._started = True

        # Hardcoded target pose in 'world' using XYZ + RPY (radians)
        px, py, pz = 0.00, 0.90, -0.07
        rx, ry, rz = 0.0, 0.0, math.radians(0)

        target_ps = PoseStamped()
        target_ps.header.frame_id = "world"
        target_ps.pose.position.x = px
        target_ps.pose.position.y = py
        target_ps.pose.position.z = pz

        qx, qy, qz, qw = R.from_euler("xyz", [rx, ry, rz], degrees=False).as_quat()
        target_ps.pose.orientation.x = float(qx)
        target_ps.pose.orientation.y = float(qy)
        target_ps.pose.orientation.z = float(qz)
        target_ps.pose.orientation.w = float(qw)

        self.session.motion.home(duration_s=1.0, on_done=self._on_move_done)
        self.session.motion.setAcc(0.35)
        self.session.motion.setSpd(0.35)
        self.session.motion.setEef("femto_color_optical_calib")
        self.session.motion.setLIN()
        self.session.util.input(prompt="Press ENTER to go to target...")
        self.session.fib_scan(
            target=target_ps,
            distance=0.42,
            cap_rad=math.radians(24),
            samples=16,
            folder="@session/scan_fib_simple",
            settle_s=0.2,
            z_jitter=0.20,
            prompt="Press ENTER to capture...",
            debug=False,
        )
        self.session.util.wait(1.0)

        # reconstruction scan
        # self.cmd.reconstruct(
        #     session_path="@session/scan_1",
        #     use_latest=True,
        #     on_done=lambda res: self.get_logger().info(f"Reconstruction request done: {res}"),
        # )        

        self.session.motion.home(duration_s=1.0, on_done=self._on_move_done)
        self.session.util.input(key="q",
                     prompt="Type 'q' + ENTER to shutdown...",
                     on_done=self._on_quit)


    def _on_move_done(self, res):
        # Unified callback for both HOME and GOTO
        if not res["ok"]:
            self.get_logger().error(f"[{res['kind']} {res['phase']}] FAILED: {res['error']}")
        else:
            self.get_logger().info(f"[{res['kind']} {res['phase']}] OK {res['metrics']}")

    def _on_pose(self, res):
        if not res.get("ok", False):
            self.get_logger().error(f"getPose failed: {res.get('error')}")
            return
        ps = res["pose"]
        p, q = ps.pose.position, ps.pose.orientation
        src = res["metrics"]["source"]
        base = res["base_frame"] or "(planning frame)"
        link = res["link"]
        self.get_logger().info(
            f"[{src}] {link} in {base}: "
            f"p=({p.x:.5f},{p.y:.5f},{p.z:.5f}) "
            f"q=({q.x:.5f},{q.y:.5f},{q.z:.5f},{q.w:.5f})"
        )
    def _on_quit(self, res):
        if res.get("ok", False):
            self.get_logger().info("Shutdown requested. Stopping ROS...")
            rclpy.shutdown()
        else:
            self.get_logger().warning("Expected 'q'. Ignoring input; not shutting down.")
            # Re-enqueue another input step so the FIFO waits again:
            self.session.util.input(
                key="q",
                prompt="Type 'q' + ENTER to shutdown...",
                on_done=self._on_quit
            )

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HandeyeCaptureSeq())


if __name__ == '__main__':
    main()
