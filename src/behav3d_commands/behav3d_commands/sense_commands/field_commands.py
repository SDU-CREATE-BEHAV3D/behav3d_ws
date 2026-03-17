#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Dict, Iterable, Optional

from rclpy.node import Node

try:
    from behav3d_interfaces.srv import InitFieldFromScan
except ImportError:
    # Fallback for stale rosidl __init__.py exports in incremental workspaces.
    from behav3d_interfaces.srv._init_field_from_scan import InitFieldFromScan

from behav3d_commands.command import Command, OnCommandDone
from behav3d_commands.queue import QueueItem, SessionQueue


class FieldCommands:
    def __init__(self, node: Node, *, queue: Optional[SessionQueue] = None):
        self._queue = queue
        self._node = node
        self._init_field_cli = node.create_client(InitFieldFromScan, "/behav3d/init_field_from_scan")

    def register(self, router) -> None:
        router.register("init_field_from_scan", self._handle_init_field_from_scan)

    def _queue_or_item(self, item: QueueItem, *, enqueue: bool):
        if enqueue:
            if self._queue is None:
                raise RuntimeError("FieldCommands requires a SessionQueue to enqueue items.")
            self._queue.enqueue(item)
            return None
        return item

    def init_field_from_scan(
        self,
        *,
        use_latest: bool = True,
        session_path: Optional[str] = "",
        scan_mesh_paths: Optional[Iterable[str]] = None,
        field_mesh_path: Optional[str] = "",
        state_output_dir: Optional[str] = "",
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "init_field_from_scan",
            {
                "use_latest": bool(use_latest),
                "session_path": (session_path or ""),
                "scan_mesh_paths": list(scan_mesh_paths or []),
                "field_mesh_path": (field_mesh_path or ""),
                "state_output_dir": (state_output_dir or ""),
            },
            cmd_kind="init_field_from_scan",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def _handle_init_field_from_scan(self, payload: Dict[str, Any], cmd: Command) -> None:
        if not self._init_field_cli.wait_for_service(timeout_sec=3.0):
            cmd.finish_flag(ok=False, phase="exec", error="init_field_from_scan service not available")
            return

        req = InitFieldFromScan.Request()
        req.session_path = str(payload.get("session_path", "")).strip()
        req.use_latest = bool(payload.get("use_latest", True))
        if req.session_path:
            req.use_latest = False

        raw_paths = payload.get("scan_mesh_paths", [])
        if raw_paths is None:
            raw_paths = []
        req.scan_mesh_paths = [str(p).strip() for p in raw_paths if str(p).strip()]

        req.field_mesh_path = str(payload.get("field_mesh_path", "")).strip()
        req.state_output_dir = str(payload.get("state_output_dir", "")).strip()

        self._node.get_logger().info(
            f"INIT_FIELD_FROM_SCAN: use_latest={req.use_latest} session_path='{req.session_path}' "
            f"scan_mesh_paths={len(req.scan_mesh_paths)} field_mesh_path='{req.field_mesh_path}' "
            f"state_output_dir='{req.state_output_dir}'"
        )

        fut = self._init_field_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"exception: {exc}")
                return

            ok = bool(getattr(resp, "success", False))
            msg = str(getattr(resp, "message", ""))
            cmd.finish_flag(
                ok=ok,
                phase="exec",
                metrics={
                    "message": msg,
                    "session_dir": str(getattr(resp, "session_dir", "")),
                    "resolved_scan_mesh_path": str(getattr(resp, "resolved_scan_mesh_path", "")),
                    "resolved_field_mesh_path": str(getattr(resp, "resolved_field_mesh_path", "")),
                    "field_state_path": str(getattr(resp, "field_state_path", "")),
                    "debug_field_ply_path": str(getattr(resp, "debug_field_ply_path", "")),
                    "offset_x": float(getattr(resp, "offset_x", 0.0)),
                    "offset_y": float(getattr(resp, "offset_y", 0.0)),
                    "offset_z": float(getattr(resp, "offset_z", 0.0)),
                    "scan_mesh_count": len(req.scan_mesh_paths),
                },
                error=None if ok else msg,
            )

        fut.add_done_callback(_on_resp)
