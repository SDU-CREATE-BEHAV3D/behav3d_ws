# For running this node first:
# ros2 launch behav3d_bringup print_move.launch.py robot_ip:=192.168.1.8 use_mock_hardware:=true

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .commands import Commands   # Both in the same ROS2 Python package
from .macros import Macros     # Both in the same ROS2 Python package

import math
from scipy.spatial.transform import Rotation as R

class MoveAndPrintTest(Node):
    """Demo: HOME → GOTO(plan+exec). Single on_move_done callback for all moves."""
    def __init__(self):
        super().__init__('move_and_print_test')
        self.cmd = Commands(self)
        self._started = False
        self.create_timer(0.25, self._run_once)
        self.mac = Macros(self.cmd)

    def _run_once(self):
        if self._started:
            return
        self._started = True

        self.cmd.home(duration_s=1.0, on_move_done=self._on_move_done)
        self.cmd.setSpd(0.2)
        self.cmd.setEef("femto_color_optical_calib") 
        self.cmd.setLIN()
        self.cmd.input(prompt="Press ENTER to go to target...")   
        self.cmd.goto(x=0.50, y=1.0, z=0.31,eef="extruder_tcp",vel_scale=0.1, accel_scale=0.1,exec=True, motion="LIN", start_print={"secs": 1.5, "speed": 1200, "offset_s": 0, "sync": "accept"}, on_move_done=self._on_move_done)
        self.cmd.goto(x=0.50, y=1.0, z=0.96,eef="extruder_tcp",vel_scale=0.1, accel_scale=0.1,exec=True, motion="LIN", start_print={"steps": 1800, "speed": 600, "offset_s": 0, "sync": "accept"}, on_move_done=self._on_move_done)


        self.cmd.printSteps(steps=2000, speed=500, on_done=self._on_move_done)
        self.cmd.goto(x=-0.0500, y=1.30, z=0.31,eef="extruder_tcp",vel_scale=0.5, accel_scale=0.1,exec=True,on_move_done=self._on_move_done)

        self.cmd.home(duration_s=1.0, on_move_done=self._on_move_done)
        self.cmd.input(key="q",
                     prompt="Type 'q' + ENTER to shutdown...",
                     on_done=self._on_quit)

    # def _run_once(self):
    #     if self._started:
    #         return
    #     self._started = True
#
        # 1) SAMPLE COMMANDS HERE AMIGOS!!!
    #     self.cmd.home(duration_s=10.0, on_move_done=self._on_move_done)
    #     self.cmd.goto(x=-0.0500, y=1.30, z=0.31,eef="extruder_tcp",vel_scale=0.1, accel_scale=0.1,exec=True,on_move_done=self._on_move_done)
    #  #   self.cmd.goto(x=0.5, y=1.5, z=0.3, ry=1.57, exec=True)
    #     self.cmd.goto(x=0.5, y=1.5, z=0.3, exec=True)
    #     self.cmd.print(secs=1.2, speed=500, on_done=self._on_move_done) 
    #     self.cmd.LIN()
    #     self.cmd.goto(x=-0.0500, y=1.0, z=0.31,eef="extruder_tcp",vel_scale=0.1, accel_scale=0.1,exec=True,on_move_done=self._on_move_done)
    #     self.cmd.print(secs=1.2, speed=500, on_done=self._on_move_done) 
    #     self.cmd.goto(x=0.50, y=1.0, z=0.31,eef="extruder_tcp",vel_scale=0.1, accel_scale=0.1,exec=True, motion="LIN", start_print={"secs": 1.5, "speed": 1200, "offset_s": 0, "sync": "accept"}, on_move_done=self._on_move_done)
    #     self.cmd.home(duration_s=10.0, on_move_done=self._on_move_done)
        # 2) 3 Dots Sequence!!!

    #    self.cmd.home(duration_s=2.0, on_move_done=self._on_move_done)
    #    self.cmd.capture(rgb=True, depth=True, ir=True, pose=True)

     #   self.cmd.goto(x=-0.1965, y=0.955, z=-0.044,eef="extruder_tcp",vel_scale=0.9, accel_scale=0.1,exec=True,on_move_done=self._on_move_done)
        #self.cmd.print(secs=6.2, speed=900, on_done=self._on_move_done)
      #  self.cmd.capture(rgb=True, depth=True, ir=True, pose=True, folder="")
 
 #       self.cmd.LIN()
  #      self.cmd.SPD(0.1)
    #    self.cmd.goto(x=-0.1965, y=0.955, z=0.0, exec=True)
    #    self.cmd.capture(rgb=True, depth=True, ir=True, pose=True, folder="")
        # self.cmd.wait(5.0, on_done=self._on_move_done)
        # self.cmd.getPose("extruder_tcp", on_done=self._on_pose)
        #self.cmd.getPose("extruder_tcp", use_tf=True, on_done=self._on_pose)
        # self.cmd.getPose("femto_color_optical_calib", "ur20_tool0", on_done=self._on_pose)
        # self.cmd.getPose("femto_color_optical_calib", "ur20_tool0", use_tf=True, on_done=self._on_pose)
        # self.cmd.wait(45.0, on_done=self._on_move_done)

    #    self.cmd.goto(x=-0.205, y=0.955, z=0.0, exec=True)
 #       self.cmd.wait(1.0, on_done=self._on_move_done)
  #      self.cmd.getPose("extruder_tcp", on_done=self._on_pose)
   #     self.cmd.goto(x=-0.205, y=0.955, z=-0.044, exec=True)
        #self.cmd.print(secs=6.2, speed=900, on_done=self._on_move_done)
   #     self.cmd.goto(x=-0.205, y=0.955, z=0.0, exec=True) 
   #     self.cmd.goto(x=-0.215, y=0.955, z=0.0, exec=True) 
    #    self.cmd.goto(x=-0.215, y=0.955, z=-0.044, exec=True)
    #     self.cmd.capture(rgb=True, depth=True, ir=True, pose=True, folder="@session/folder_1")
    #     #self.cmd.print(secs=6.2, speed=900, on_done=self._on_move_done) 
    # #    self.cmd.home(duration_s=12.0, on_move_done=self._on_move_done)

    #     self.cmd.home(on_move_done=lambda r: self.get_logger().info("[home #1 done]"))

    #     self.cmd.input(prompt="Press ENTER to go to target...")
    #     self.cmd.capture(rgb=True, depth=True, ir=True, pose=True)

    #     self.cmd.goto(x=-0.1965, y=0.955, z=0.0, exec=True)
    #     self.cmd.capture(rgb=True, depth=True, ir=True, pose=True, folder="@session/folder_2")
    #     self.cmd.input(prompt="Press ENTER to capture (rgb+depth+ir)...")

    #     self.cmd.capture(rgb=True, depth=True, ir=True, pose=True)
   
    #     self.cmd.input(prompt="Press ENTER to go home...")

    #     self.cmd.home(on_move_done=lambda r: self.get_logger().info("[home #2 done]"))
    #     self.cmd.capture(rgb=True, depth=True, ir=True, pose=True, folder="@session/folder_1")

        
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
            self.cmd.input(
                key="q",
                prompt="Type 'q' + ENTER to shutdown...",
                on_done=self._on_quit
            )

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MoveAndPrintTest())


if __name__ == '__main__':
    main()
