#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Dict, Optional

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros

from geometry_msgs.msg import PoseStamped

from behav3d_interfaces.srv import (
    Capture,
    ColorToDepth,
    GetLinkPose,
    TsdfCropped,
    UpdateWorldMesh,
)

from behav3d_commands.command import Command, OnCommandDone
from behav3d_commands.queue import QueueItem, SessionQueue


class CameraCommands:
    def __init__(self, node: Node, *, queue: Optional[SessionQueue] = None):
        self._queue = queue
        self._node = node
        self._capture_cli = node.create_client(Capture, "/capture")
        self._color_to_depth_cli = node.create_client(ColorToDepth, "/reconstruct/color_to_depth")
        self._pose_cli = node.create_client(GetLinkPose, "/behav3d/get_link_pose")
        self._tsdf_cropped_cli = node.create_client(TsdfCropped, "/reconstruct/tsdf_cropped")
        self._world_mesh_cli = node.create_client(UpdateWorldMesh, "/behav3d/update_world_mesh")

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

    def register(self, router) -> None:
        router.register("capture", self._handle_capture)
        router.register("reconstruct_color_to_depth", self._handle_reconstruct_color_to_depth)
        router.register("reconstruct_color_to_depth_grid_sweep", self._handle_reconstruct_color_to_depth_grid_sweep)
        router.register("get_pose", self._handle_get_pose)
        router.register("reconstruct_tsdf_cropped", self._handle_reconstruct_tsdf_cropped)
        router.register("reconstruct_tsdf_grid_sweep", self._handle_reconstruct_tsdf_grid_sweep)
        router.register("update_world_mesh", self._handle_update_world_mesh)

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

    def reconstruct_color_to_depth(
        self,
        *,
        use_latest: bool = True,
        session_path: Optional[str] = "",
        scan_folder: str = "manual_caps",
        visualize: bool = False,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "reconstruct_color_to_depth",
            {
                "use_latest": bool(use_latest),
                "session_path": (session_path or ""),
                "scan_folder": str(scan_folder),
                "visualize": bool(visualize),
            },
            cmd_kind="reconstruct_color_to_depth",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def reconstruct_color_to_depth_grid_sweep(
        self,
        *,
        use_latest: bool = True,
        session_path: Optional[str] = "",
        scan_folder: str = "grid_sweep",
        visualize: bool = False,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "reconstruct_color_to_depth_grid_sweep",
            {
                "use_latest": bool(use_latest),
                "session_path": (session_path or ""),
                "scan_folder": str(scan_folder),
                "visualize": bool(visualize),
            },
            cmd_kind="reconstruct_color_to_depth_grid_sweep",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def reconstruct_tsdf_cropped(
        self,
        *,
        use_latest: bool = True,
        session_path: Optional[str] = "",
        scan_folder: str = "manual_caps",
        visualize: bool = False,
        device: str = "CPU:0",
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "reconstruct_tsdf_cropped",
            {
                "use_latest": bool(use_latest),
                "session_path": (session_path or ""),
                "scan_folder": str(scan_folder),
                "visualize": bool(visualize),
                "device": str(device),
            },
            cmd_kind="reconstruct_tsdf_cropped",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def reconstruct_tsdf_grid_sweep(
        self,
        *,
        use_latest: bool = True,
        session_path: Optional[str] = "",
        scan_folder: str = "grid_sweep",
        visualize: bool = False,
        device: str = "CPU:0",
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "reconstruct_tsdf_grid_sweep",
            {
                "use_latest": bool(use_latest),
                "session_path": (session_path or ""),
                "scan_folder": str(scan_folder),
                "visualize": bool(visualize),
                "device": str(device),
            },
            cmd_kind="reconstruct_tsdf_grid_sweep",
            on_done=on_done,
        )
        return self._queue_or_item(item, enqueue=enqueue)

    def update_world_mesh(
        self,
        *,
        use_latest: bool = True,
        session_path: Optional[str] = "",
        mesh_path: Optional[str] = "",
        ply_path: Optional[str] = "",
        prefer: str = "mesh",
        wait_timeout_s: float = 30.0,
        on_done: OnCommandDone = None,
        enqueue: bool = True,
    ):
        item = QueueItem(
            "update_world_mesh",
            {
                "use_latest": bool(use_latest),
                "session_path": (session_path or ""),
                "mesh_path": (mesh_path or ""),
                "ply_path": (ply_path or ""),
                "prefer": str(prefer),
                "wait_timeout_s": float(wait_timeout_s),
            },
            cmd_kind="update_world_mesh",
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

    def _handle_reconstruct_color_to_depth(self, payload: Dict[str, Any], cmd: Command) -> None:
        self._handle_color_to_depth_common(
            payload,
            cmd,
            default_scan_folder="manual_caps",
            label="COLOR_TO_DEPTH",
        )

    def _handle_reconstruct_color_to_depth_grid_sweep(self, payload: Dict[str, Any], cmd: Command) -> None:
        self._handle_color_to_depth_common(
            payload,
            cmd,
            default_scan_folder="grid_sweep",
            label="COLOR_TO_DEPTH_GRID_SWEEP",
        )

    def _handle_color_to_depth_common(
        self,
        payload: Dict[str, Any],
        cmd: Command,
        *,
        default_scan_folder: str,
        label: str,
    ) -> None:
        if not self._color_to_depth_cli.wait_for_service(timeout_sec=3.0):
            cmd.finish_flag(ok=False, phase="exec", error="color_to_depth service not available")
            return

        req = ColorToDepth.Request()
        req.session_path = str(payload.get("session_path", "")).strip()
        req.use_latest = bool(payload.get("use_latest", True))
        if req.session_path:
            # Explicit session path takes precedence over "latest".
            req.use_latest = False

        scan_folder = str(payload.get("scan_folder", default_scan_folder)).strip()
        req.scan_folder = scan_folder if scan_folder else default_scan_folder
        req.visualize = bool(payload.get("visualize", False))

        self._node.get_logger().info(
            f"{label}: use_latest={req.use_latest} session_path='{req.session_path}' "
            f"scan_folder='{req.scan_folder}' visualize={req.visualize}"
        )

        fut = self._color_to_depth_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"exception: {exc}")
                return

            ok = bool(getattr(resp, "success", False))
            msg = getattr(resp, "message", "")
            output_path = str(getattr(resp, "output_path", ""))
            cmd.finish_flag(
                ok=ok,
                phase="exec",
                metrics={
                    "message": msg,
                    "output_path": output_path,
                    "scan_folder": req.scan_folder,
                },
                error=None if ok else msg,
            )

        fut.add_done_callback(_on_resp)

    def _handle_reconstruct_tsdf_cropped(self, payload: Dict[str, Any], cmd: Command) -> None:
        self._handle_tsdf_cropped_common(
            payload,
            cmd,
            default_scan_folder="manual_caps",
            label="TSDF_CROPPED",
        )

    def _handle_reconstruct_tsdf_grid_sweep(self, payload: Dict[str, Any], cmd: Command) -> None:
        self._handle_tsdf_cropped_common(
            payload,
            cmd,
            default_scan_folder="grid_sweep",
            label="TSDF_GRID_SWEEP",
        )

    def _handle_tsdf_cropped_common(
        self,
        payload: Dict[str, Any],
        cmd: Command,
        *,
        default_scan_folder: str,
        label: str,
    ) -> None:
        if not self._tsdf_cropped_cli.wait_for_service(timeout_sec=3.0):
            cmd.finish_flag(ok=False, phase="exec", error="tsdf_cropped service not available")
            return

        req = TsdfCropped.Request()
        req.session_path = str(payload.get("session_path", "")).strip()
        req.use_latest = bool(payload.get("use_latest", True))
        if req.session_path:
            # Explicit session path takes precedence over "latest".
            req.use_latest = False

        scan_folder = str(payload.get("scan_folder", default_scan_folder)).strip()
        req.scan_folder = scan_folder if scan_folder else default_scan_folder
        req.visualize = bool(payload.get("visualize", False))

        device = str(payload.get("device", "CPU:0")).strip()
        req.device = device if device else "CPU:0"

        self._node.get_logger().info(
            f"{label}: use_latest={req.use_latest} session_path='{req.session_path}' "
            f"scan_folder='{req.scan_folder}' visualize={req.visualize} device='{req.device}'"
        )

        fut = self._tsdf_cropped_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as exc:
                cmd.finish_flag(ok=False, phase="exec", error=f"exception: {exc}")
                return

            ok = bool(getattr(resp, "success", False))
            msg = getattr(resp, "message", "")
            output_path = str(getattr(resp, "output_path", ""))
            mesh_path = str(getattr(resp, "mesh_path", ""))
            rgb_ply_path = str(getattr(resp, "rgb_ply_path", ""))
            confidence_ply_path = str(getattr(resp, "confidence_ply_path", ""))
            cmd.finish_flag(
                ok=ok,
                phase="exec",
                metrics={
                    "message": msg,
                    "output_path": output_path,
                    "mesh_path": mesh_path,
                    "rgb_ply_path": rgb_ply_path,
                    "confidence_ply_path": confidence_ply_path,
                    "scan_folder": req.scan_folder,
                    "device": req.device,
                },
                error=None if ok else msg,
            )

        fut.add_done_callback(_on_resp)

    def _handle_update_world_mesh(self, payload: Dict[str, Any], cmd: Command) -> None:
        if not self._world_mesh_cli.wait_for_service(timeout_sec=3.0):
            cmd.finish_flag(ok=False, phase="exec", error="update_world_mesh service not available")
            return

        req = UpdateWorldMesh.Request()
        req.session_path = str(payload.get("session_path", "")).strip()
        req.use_latest = bool(payload.get("use_latest", True))
        if req.session_path:
            req.use_latest = False

        req.mesh_path = str(payload.get("mesh_path", "")).strip()
        req.ply_path = str(payload.get("ply_path", "")).strip()

        prefer = str(payload.get("prefer", "mesh")).strip().lower()
        req.prefer = prefer if prefer in ("mesh", "ply") else "mesh"

        try:
            req.wait_timeout_s = float(payload.get("wait_timeout_s", 30.0))
        except (TypeError, ValueError):
            req.wait_timeout_s = 30.0
        if req.wait_timeout_s < 0.0:
            req.wait_timeout_s = 0.0

        self._node.get_logger().info(
            f"UPDATE_WORLD_MESH: use_latest={req.use_latest} session_path='{req.session_path}' "
            f"mesh_path='{req.mesh_path}' ply_path='{req.ply_path}' prefer='{req.prefer}' "
            f"wait_timeout_s={req.wait_timeout_s:.2f}"
        )

        fut = self._world_mesh_cli.call_async(req)

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
                    "resolved_mesh_path": str(getattr(resp, "resolved_mesh_path", "")),
                    "resolved_ply_path": str(getattr(resp, "resolved_ply_path", "")),
                    "published_path": str(getattr(resp, "published_path", "")),
                    "published_kind": str(getattr(resp, "published_kind", "")),
                    "prefer": req.prefer,
                },
                error=None if ok else msg,
            )

        fut.add_done_callback(_on_resp)
