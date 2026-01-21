#!/usr/bin/env python3
# Demo: blocking sequencing with run_sync + run_group (queue-backed commands).

from __future__ import annotations

import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from .src.custom_session import MySession


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    z: float


class CustomSequenceDemo(Node):
    def __init__(self):
        super().__init__("session_sync_demo")
        self.session = MySession(self)
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _run(self):
        targets = [
            Target(0.50, 1.00, 0.31),
            Target(0.50, 1.00, 0.96),
        ]

        self.session.run_scan_session(targets)
        self.session.run_sync(
            self.session.util.input(prompt="Press ENTER to start print demo...", enqueue=False)
        )
        self.session.run_disc_print_session(targets)
        self.session.run_sync(
            self.session.util.input(key="q", prompt="Type 'q' + ENTER to shutdown...", enqueue=False)
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CustomSequenceDemo())


if __name__ == "__main__":
    main()
