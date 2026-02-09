#!/usr/bin/env python3
# Demo: blocking sequencing with run_sync + run_group (queue-backed commands).

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node

from .src.custom_session import MySession
from .src.grid_sweep_session import GridSweepSession

class CustomSequenceDemo(Node):
    def __init__(self):
        super().__init__("session_sync_demo")
        self.session = MySession(self)
        self.grid_session = GridSweepSession(self)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()
        log.info("Starting grid sweep (12 targets) -> color_to_depth -> tsdf_grid_sweep pipeline.")

        sweep_targets = self.grid_session.run_grid_sweep(
            width=0.6,
            height=0.4,
            center_x=0.0,
            center_y=0.8,
            center_z=0.0,
            z_off=0.5,
            nx=4,
            ny=3,
            debug=False,
            capture_folder="@session/grid_sweep",
            eef_link="femto_color_optical_calib",
            use_tf_orientation=True,
        )
        log.info(f"Grid sweep completed. Captured targets: {len(sweep_targets)}")

        if len(sweep_targets) != 12:
            log.warn("Expected 12 targets from grid sweep (4x3). Continuing anyway.")

        try:
            c2d_res = self.session.run_color_to_depth_grid_sweep_reconstruct(
                use_latest=True,
                session_path="@session",
                scan_folder="grid_sweep",
                visualize=False,
                timeout_s=10.0,
                wait_for_outputs=True,
                wait_timeout_s=60.0,
            )
        except TimeoutError:
            log.error("color_to_depth request timed out.")
            rclpy.shutdown()
            return

        if not c2d_res.get("ok", False):
            log.error(f"color_to_depth failed: {c2d_res.get('error')}")
            rclpy.shutdown()
            return

        try:
            tsdf_res = self.session.run_tsdf_grid_sweep_reconstruct(
                use_latest=True,
                session_path="@session",
                scan_folder="grid_sweep",
                visualize=False,
                device="CPU:0",
                timeout_s=10.0,
            )
        except TimeoutError:
            log.error("tsdf_grid_sweep request timed out.")
            rclpy.shutdown()
            return

        if not tsdf_res.get("ok", False):
            log.error(f"tsdf_grid_sweep failed: {tsdf_res.get('error')}")
            rclpy.shutdown()
            return

        log.info("Pipeline requests submitted successfully.")
        self.session.run_sync(
            self.session.util.input(prompt="Press ENTER to shutdown...", enqueue=False)
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CustomSequenceDemo())


if __name__ == "__main__":
    main()
