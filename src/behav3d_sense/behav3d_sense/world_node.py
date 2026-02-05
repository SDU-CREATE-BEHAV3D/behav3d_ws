#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class WorldNode(Node):
    """
    Phase 1 world-state cache (no reconstruction).
    Source of truth: latest capture session + manifest.yaml.
    """

    def __init__(self):
        super().__init__("world_node")

        env_root = os.environ.get("BEHAV3D_CAPTURES_ROOT", "")
        self.captures_root = Path(env_root).expanduser() if env_root else (Path.home() / "behav3d_ws" / "captures")

        self.declare_parameter("poll_period_s", 1.0)
        self.declare_parameter("session_path", "")  # optional explicit session path

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._pub = self.create_publisher(String, "/behav3d/world_state", qos)
        self._srv = self.create_service(Trigger, "/behav3d/get_world_state", self._handle_get_world_state)

        self._last_json: Optional[str] = None
        self._state: Dict[str, Any] = {}

        period = float(self.get_parameter("poll_period_s").value)
        self._timer = self.create_timer(period, self._poll)

        # Initial publish
        self._poll()
        self.get_logger().info("World node ready: /behav3d/world_state, /behav3d/get_world_state")

    def _handle_get_world_state(self, req: Trigger.Request, res: Trigger.Response):
        _ = req
        if not self._state:
            res.success = False
            res.message = "world_state empty (no sessions found)"
            return res
        res.success = True
        res.message = self._last_json or json.dumps(self._state, ensure_ascii=False)
        return res

    def _poll(self):
        state = self._build_state()
        if not state:
            return
        encoded = json.dumps(state, ensure_ascii=False)
        if encoded != self._last_json:
            self._last_json = encoded
            self._state = state
            msg = String()
            msg.data = encoded
            self._pub.publish(msg)

    def _build_state(self) -> Dict[str, Any]:
        session_path = str(self.get_parameter("session_path").value).strip()
        if session_path:
            session_dir = Path(session_path).expanduser().resolve()
        else:
            session_dir = self._get_latest_session()

        if session_dir is None or not session_dir.exists():
            self.get_logger().warn("No capture sessions found.")
            return {}

        manifest_path = session_dir / "manifest.yaml"
        data = {}
        if manifest_path.exists():
            try:
                with manifest_path.open("r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
            except Exception as exc:
                self.get_logger().warn(f"Failed to read manifest: {exc}")
                data = {}

        captures = data.get("captures", []) if isinstance(data, dict) else []
        last_cap = captures[-1] if captures else None

        state = {
            "captures_root": str(self.captures_root),
            "session_dir": str(session_dir),
            "manifest_path": str(manifest_path),
            "capture_count": len(captures),
            "last_capture": last_cap,
            "source": "manifest",
        }
        return state

    def _get_latest_session(self) -> Optional[Path]:
        if not self.captures_root.exists():
            return None
        subdirs = [d for d in self.captures_root.iterdir() if d.is_dir()]
        if not subdirs:
            return None
        return max(subdirs, key=lambda d: d.stat().st_mtime)


def main(args=None):
    rclpy.init(args=args)
    node = WorldNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
