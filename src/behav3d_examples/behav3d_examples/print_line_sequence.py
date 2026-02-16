#!/usr/bin/env python3
"""
Run with:
ros2 run behav3d_examples print_line_sequence
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node

from .src.print_line_session import PrintLineSession


class PrintLineSequence(Node):
    def __init__(self):
        super().__init__("print_line_sequence")
        self.session = PrintLineSession(self)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        log = self.get_logger()

        # Input points are provided in mm; convert to meters.
        p1_m = (718.0 / 1000.0, 540.0 / 1000.0, -72.3 / 1000.0)
        p2_m = (718.0 / 1000.0, 987.0 / 1000.0, -68.0 / 1000.0)

        timeout_s = 60.0
        try:
            self.session.run_sync(self.session.motion.home(enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setLIN(enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setAcc(0.05, enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setEef("extruder_tcp", enqueue=False), timeout_s=timeout_s)
            self.session.run_sync(self.session.motion.setSpd(0.01, enqueue=False), timeout_s=timeout_s)
            res = self.session.run_print_line(
                p1_xyz_m=p1_m,
                p2_xyz_m=p2_m,
                approach_z_offset_m=0.50,
                extruder_speed=1200,
                timeout_s=timeout_s,
            )
            if not res.get("ok", False):
                log.error(
                    f"[print_line_sequence] Line print failed at stage={res.get('stage')}: "
                    f"{res.get('error', 'unknown')}"
                )
            else:
                log.info("[print_line_sequence] Line print completed.")

            self.session.run_sync(self.session.motion.home(enqueue=False), timeout_s=timeout_s)
        except TimeoutError:
            log.error("[print_line_sequence] Sequence timed out.")
        finally:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PrintLineSequence()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()