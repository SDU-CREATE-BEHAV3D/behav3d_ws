#!/usr/bin/env python3
# Demo: blocking sequencing with run_sync + run_group (queue-backed commands).

from __future__ import annotations

import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from .src.custom_session import MySession
from .src.grid_sweep_session import GridSweepSession

@dataclass(frozen=True)
class Target:
    x: float
    y: float
    z: float


class CustomSequenceDemo(Node):
    def __init__(self):
        super().__init__("session_sync_demo")
        self.session = MySession(self)
        self.grid_session = GridSweepSession(self)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        targets = [
            Target(0.20, 0.80, 0.31),
            Target(0.20, 0.80, 0.7),
        ]
       
      #  self.session.run_scan_session(targets)
        self.grid_session.run_grid_sweep(
            width=0.8,
            height=0.5,
            center_x=0.0,
            center_y=0.8,
            center_z=0.0,
            z_off=0.6,
            nx=3,
            ny=2,
            debug=True,
            capture_folder="@session/grid_sweep",
            eef_link="femto_color_optical_calib",
            use_tf_orientation=True,
        )

        self.session.run_sync(
            self.session.util.input(prompt="Press ENTER to start print demo...", enqueue=False)
        )
        self.session.run_disc_print_session(targets)
        self.session.run_sync(
            self.session.motion.home(enqueue=False)
        )
        self.session.run_sync(
            self.session.util.input(key="q", prompt="Type 'q' + ENTER to shutdown...", enqueue=False)
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CustomSequenceDemo())


if __name__ == "__main__":
    main()
