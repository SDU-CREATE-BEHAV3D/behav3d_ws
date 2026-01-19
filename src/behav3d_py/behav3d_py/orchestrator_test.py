# For running this node first:
# ros2 launch behav3d_bringup print_move.launch.py robot_ip:=192.168.1.8 use_mock_hardware:=true

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .commands import Commands   # Both in the same ROS2 Python package
from .macros import Macros     # Both in the same ROS2 Python package
import behav3d_commands

import math
from scipy.spatial.transform import Rotation as R

class OrchestratorTest(Node):
    """Testing new cmds implementation."""
    def __init__(self):
        super().__init__('orchestrator_test')
        self.cmd = behav3d_commands.Commands(self)
        self.get_logger().info("behav3d_commands import OK")

        # 1) HOME
        self.cmd.home(duration_s=1.0, on_done=self._on_command_done)

        # 2) PLAN LIN to point A (plan only -> stored internally)
        self.cmd.plan_motion(
            x=0.50, y=1.0, z=0.31,
            eef="extruder_tcp",
            vel_scale=0.1,
            accel_scale=0.1,
            exec=False,          # IMPORTANT: plan only
            motion="LIN",
            on_done=self._on_command_done,
        )

        # 3) EXEC last planned motion
        self.cmd.exec_motion(on_done=self._on_command_done)

        # 4) PLAN LIN to point B
        self.cmd.plan_motion(x=0.50, y=1.0, z=0.96,  on_done=self._on_command_done,  )

        # 5) EXEC last planned motion
        self.cmd.exec_motion(on_done=self._on_command_done)

    
    def _on_command_done(self, res):
        # Unified callback for both HOME and GOTO
        if not res["ok"]:
            self.get_logger().error(f"[{res['kind']} {res['phase']}] FAILED: {res['error']}")
        else:
            self.get_logger().info(f"[{res['kind']} {res['phase']}] OK {res['metrics']}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OrchestratorTest())


if __name__ == '__main__':
    main()
