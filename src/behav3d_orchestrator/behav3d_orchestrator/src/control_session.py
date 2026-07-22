#!/usr/bin/env python3
from __future__ import annotations

import threading
from contextlib import contextmanager
from typing import Optional

import rclpy
from behav3d_commands.queue import QueueItem
from behav3d_commands.session import Session
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class ControlPauseGate:
    """Shared cooperative pause state for all sessions attached to one ROS node."""

    def __init__(self, node, *, topic: str = "/behav3d/control_state"):
        self._node = node
        self._condition = threading.Condition()
        self._paused = False

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._subscription = node.create_subscription(String, topic, self._on_state, qos)

    def _on_state(self, msg: String) -> None:
        state = str(msg.data or "").strip().lower()
        if state not in ("paused", "running"):
            self._node.get_logger().warn(
                f"[control_pause] Ignoring state '{state}'. Expected 'paused' or 'running'."
            )
            return

        with self._condition:
            paused = state == "paused"
            changed = paused != self._paused
            self._paused = paused
            self._condition.notify_all()

        if not changed:
            return
        if paused:
            self._node.get_logger().warn(
                "[control_pause] Global pause requested; waiting after the current command."
            )
        else:
            self._node.get_logger().info("[control_pause] Global resume requested.")

    def wait(self, label: str = "next command") -> bool:
        with self._condition:
            if not self._paused:
                return True
            self._node.get_logger().warn(f"[control_pause] Paused before {label}.")
            while rclpy.ok() and self._paused:
                self._condition.wait(timeout=0.25)
            if not rclpy.ok():
                return False

        self._node.get_logger().info(f"[control_pause] Resuming with {label}.")
        return True


def get_control_pause_gate(node) -> ControlPauseGate:
    """Return the single pause gate shared by every session on ``node``."""
    attribute = "_behav3d_control_pause_gate"
    gate = getattr(node, attribute, None)
    if gate is None:
        gate = ControlPauseGate(node)
        setattr(node, attribute, gate)
    return gate


class ControlAwareSession(Session):
    """Session that waits for the global control state before each command."""

    def __init__(self, node, **kwargs):
        super().__init__(node, **kwargs)
        self.control_pause = get_control_pause_gate(node)
        self._pause_deferral_depth = 0

    def wait_if_paused(self, label: str = "next command") -> bool:
        return self.control_pause.wait(label)

    @contextmanager
    def defer_pause(self):
        """Keep a safety-critical command group atomic before honoring pause."""
        self._pause_deferral_depth += 1
        try:
            yield
        finally:
            self._pause_deferral_depth -= 1

    @staticmethod
    def _must_run_while_paused(item: QueueItem) -> bool:
        return bool(
            item is not None
            and item.kind == "set_extruder"
            and not bool(item.payload.get("on", False))
        )

    def run_sync(
        self,
        item: QueueItem,
        *,
        timeout_s: Optional[float] = None,
    ) -> dict:
        if self._pause_deferral_depth == 0 and not self._must_run_while_paused(item):
            label = item.kind if item is not None else "next command"
            if not self.wait_if_paused(label):
                raise RuntimeError("ROS shutdown while waiting for global resume.")
        return super().run_sync(item, timeout_s=timeout_s)
