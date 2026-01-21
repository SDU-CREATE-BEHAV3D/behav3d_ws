#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Dict, Optional

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros

from geometry_msgs.msg import PoseStamped

from behav3d_interfaces.srv import Capture, GetLinkPose, ReconstructMesh

from behav3d_commands.command import Command, OnCommandDone
from behav3d_commands.queue import QueueItem, SessionQueue


class CameraCommands:
    def __init__(self, node: Node, *, queue: Optional[SessionQueue] = None):
        self._queue = queue
        self._node = node
        self._capture_cli = node.create_client(Capture, "/capture")
        self._pose_cli = node.create_client(GetLinkPose, "/behav3d/get_link_pose")
        self._reconstruct_cli = node.create_client(ReconstructMesh, "/reconstruct_mesh")

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

    def register(self, router) -> None:
        router.register("capture", self._handle_capture)
        router.register("get_pose", self._handle_get_pose)
        router.register("reconstruct", self._handle_reconstruct)

    def _queue_or_item(self, item: QueueItem, *, enqueue: bool):
        if enqueue:
            if self._queue is None:
                raise RuntimeError("CameraCommands requires a SessionQueue to enqueue items.")
            self._queue.enqueue(item)
            return None
        return item

    def capture(
        self,
        *,
        rgb: bool = False,
        depth: bool = False,
        ir: bool = False,
        pose: bool = False,
        folder: Optional[str] = None,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "capture",
            {
                "rgb": bool(rgb),
                "depth": bool(depth),
                "ir": bool(ir),
                "pose": bool(pose),
                "folder": folder,
            },
            cmd_kind="capture",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def get_pose(
        self,
        eef: str,
        base_frame: Optional[str] = "world",
        *,
        use_tf: bool = False,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "get_pose",
            {
                "link": str(eef),
                "base_frame": ("" if base_frame is None else str(base_frame)),
                "use_tf": bool(use_tf),
            },
            cmd_kind="get_pose",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def reconstruct(
        self,
        *,
        use_latest: bool = True,
        session_path: Optional[str] = "",
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "reconstruct",
            {"use_latest": bool(use_latest), "session_path": (session_path or "")},
            cmd_kind="reconstruct",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def _handle_capture(self, payload: Dict[str, Any], cmd: Command) -> None:
        if not self._capture_cli.wait_for_service(timeout_sec=2.0):
            cmd.finish_flag(ok=False, phase="exec", error="capture service not available")
            return

        req = Capture.Request()
        req.do_rgb = bool(payload.get("rgb", False))
        req.do_depth = bool(payload.get("depth", False))
        req.do_ir = bool(payload.get("ir", False))
        req.do_pose = bool(payload.get("pose", False))

        folder = payload.get("folder", None)
        if folder is None:
            req.set_folder = False
            req.folder = ""
        else:
            req.set_folder = True
            req.folder = str(folder)

        self._node.get_logger().info(
            f"CAPTURE: rgb={req.do_rgb} depth={req.do_depth} ir={req.do_ir} pose={req.do_pose} "
            f"set_folder={req.set_folder} folder='{req.folder}'"
        )

        fut = self._capture_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"exception: {exc}")
                return

            ok = bool(getattr(resp, "success", False))
            msg = getattr(resp, "message", "")
            cmd.finish_flag(
                ok=ok,
                phase="exec",
                metrics={"message": msg},
                error=None if ok else msg,
            )

        fut.add_done_callback(_on_resp)

    def _handle_get_pose(self, payload: Dict[str, Any], cmd: Command) -> None:
        link = payload.get("link")
        if not link:
            cmd.finish_flag(ok=False, phase="exec", error="get_pose requires 'link'")
            return

        base_frame = payload.get("base_frame", "world")
        base_frame = "" if base_frame is None else str(base_frame)
        use_tf = bool(payload.get("use_tf", False))

        if use_tf:
            target = base_frame if base_frame else "world"
            source = str(link)
            timeout = Duration(seconds=0.3)
            stamp = Time()

            try:
                ok = self._tf_buffer.can_transform(target, source, stamp, timeout)
            except Exception:
                ok = False

            if not ok:
                cmd.finish_flag(
                    ok=False,
                    phase="exec",
                    error=f"TF not available: {target} <- {source}",
                    metrics={"source": "tf", "base_frame": target, "link": source},
                )
                return

            try:
                tf = self._tf_buffer.lookup_transform(target, source, stamp, timeout)
            except Exception as exc:
                cmd.finish_flag(
                    ok=False,
                    phase="exec",
                    error=f"lookup_transform exception: {exc}",
                    metrics={"source": "tf", "base_frame": target, "link": source},
                )
                return

            ps = PoseStamped()
            ps.header = tf.header
            ps.header.frame_id = target
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = tf.transform.translation.z
            ps.pose.orientation = tf.transform.rotation

            cmd.finish_flag(
                ok=True,
                phase="exec",
                metrics={"source": "tf"},
                extra={"pose": ps, "base_frame": target, "link": source},
            )
            return

        if not self._pose_cli.wait_for_service(timeout_sec=2.0):
            cmd.finish_flag(
                ok=False,
                phase="exec",
                error="GetLinkPose service not available",
                metrics={"source": "moveit", "base_frame": base_frame, "link": str(link)},
            )
            return

        req = GetLinkPose.Request()
        req.base_frame = base_frame
        req.link = str(link)

        fut = self._pose_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as exc:
                cmd.finish_flag(
                    ok=False,
                    phase="exec",
                    error=f"exception: {exc}",
                    metrics={"source": "moveit", "base_frame": req.base_frame, "link": req.link},
                )
                return

            if not resp or not getattr(resp, "success", False):
                cmd.finish_flag(
                    ok=False,
                    phase="exec",
                    error=getattr(resp, "message", "no response / failed"),
                    metrics={"source": "moveit", "base_frame": req.base_frame, "link": req.link},
                )
                return

            cmd.finish_flag(
                ok=True,
                phase="exec",
                metrics={"source": "moveit"},
                extra={"pose": resp.pose, "base_frame": req.base_frame, "link": req.link},
            )

        fut.add_done_callback(_on_resp)

    def _handle_reconstruct(self, payload: Dict[str, Any], cmd: Command) -> None:
        if not self._reconstruct_cli.wait_for_service(timeout_sec=3.0):
            cmd.finish_flag(ok=False, phase="exec", error="reconstruct service not available")
            return

        req = ReconstructMesh.Request()
        req.use_latest = bool(payload.get("use_latest", True))
        req.session_path = str(payload.get("session_path", ""))

        self._node.get_logger().info(
            f"RECONSTRUCT: use_latest={req.use_latest} session_path='{req.session_path}'"
        )

        fut = self._reconstruct_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"exception: {exc}")
                return

            ok = bool(getattr(resp, "success", False))
            msg = getattr(resp, "message", "")
            cmd.finish_flag(
                ok=ok,
                phase="exec",
                metrics={"message": msg},
                error=None if ok else msg,
            )

        fut.add_done_callback(_on_resp)
