# For running this node first:
# ros2 launch behav3d_bringup print_move.launch.py robot_ip:=192.168.1.8 use_mock_hardware:=true

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from .commands import Commands   # Both in the same ROS2 Python package


class MoveAndPrintTest(Node):
    """Demo: HOME → GOTO(plan+exec). Single on_move_done callback for all moves."""
    def __init__(self):
        super().__init__('move_and_print_test')
        self.cmd = Commands(self)
        self._started = False
        self.create_timer(0.25, self._run_once)

    def _run_once(self):
        if self._started:
            return
        self._started = True

        # 1) SAMPLE COMMANDS HERE AMIGOS!!!
    #     self.cmd.home(duration_s=10.0, on_move_done=self._on_move_done)
    #     self.cmd.goto(x=-0.0500, y=1.30, z=0.31,eef="extruder_tcp",vel_scale=0.1, accel_scale=0.1,exec=True,on_move_done=self._on_move_done)
    #  #   self.cmd.goto(x=0.5, y=1.5, z=0.3, ry=1.57, exec=True)
    #     self.cmd.goto(x=0.5, y=1.5, z=0.3, exec=True)
    #     self.cmd.print(secs=1.2, speed=500, on_done=self._on_move_done) 
    #     self.cmd.LIN()
    #     self.cmd.goto(x=-0.0500, y=1.0, z=0.31,eef="extruder_tcp",vel_scale=0.1, accel_scale=0.1,exec=True,on_move_done=self._on_move_done)
    #     self.cmd.print(secs=1.2, speed=500, on_done=self._on_move_done) 
    #     self.cmd.goto(x=0.50, y=1.0, z=0.31,eef="extruder_tcp",vel_scale=0.1, accel_scale=0.1,exec=True, motion="PTP", start_print={"secs": 1.5, "speed": 1200, "offset_s": 0, "sync": "accept"}, on_move_done=self._on_move_done)
    #     self.cmd.home(duration_s=10.0, on_move_done=self._on_move_done)
        # 2) 3 Dots Sequence!!!

        self.cmd.home(duration_s=10.0, on_move_done=self._on_move_done)
        self.cmd.goto(x=-0.1965, y=0.955, z=-0.044,eef="extruder_tcp",vel_scale=0.05, accel_scale=0.1,exec=True,on_move_done=self._on_move_done)
        self.cmd.print(secs=6.2, speed=900, on_done=self._on_move_done) 
        self.cmd.LIN()
        self.cmd.goto(x=-0.1965, y=0.955, z=0.0, exec=True)
        self.cmd.goto(x=-0.205, y=0.955, z=0.0, exec=True)
        self.cmd.goto(x=-0.205, y=0.955, z=-0.044, exec=True)
        self.cmd.print(secs=6.2, speed=900, on_done=self._on_move_done)
        self.cmd.goto(x=-0.205, y=0.955, z=0.0, exec=True) 
        self.cmd.goto(x=-0.215, y=0.955, z=0.0, exec=True) 
        self.cmd.goto(x=-0.215, y=0.955, z=-0.044, exec=True)
        self.cmd.print(secs=6.2, speed=900, on_done=self._on_move_done) 
        self.cmd.home(duration_s=12.0, on_move_done=self._on_move_done)

    def _on_move_done(self, res):
        # Unified callback for both HOME and GOTO
        if not res["ok"]:
            self.get_logger().error(f"[{res['kind']} {res['phase']}] FAILED: {res['error']}")
        else:
            self.get_logger().info(f"[{res['kind']} {res['phase']}] OK {res['metrics']}")



def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MoveAndPrintTest())


if __name__ == '__main__':
    main()
