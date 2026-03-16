#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import math
import os
import shutil
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np
import open3d as o3d
import yaml

import rclpy
from behav3d_interfaces.srv import UpdateWorldMesh
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .reconstruct.service_utils import resolve_session_path


class WorldNode(Node):
    """
    World-state cache and mesh visualization interface.
    Source of truth: latest capture session + manifest.yaml.
    """

    def __init__(self):
        super().__init__("world_node")

        env_root = os.environ.get("BEHAV3D_CAPTURES_ROOT", "")
        self.captures_root = Path(env_root).expanduser() if env_root else (Path.home() / "behav3d_ws" / "captures")

        self.declare_parameter("poll_period_s", 1.0)
        self.declare_parameter("session_path", "")  # optional explicit session path
        self.declare_parameter("mesh_frame_id", "ur20_base_link")
        self.declare_parameter("mesh_topic", "/visualization_marker")
        self.declare_parameter("mesh_ns", "reconstructed_mesh")
        self.declare_parameter("mesh_scale", 1.0)
        self.declare_parameter("mesh_rgba", [0.8, 0.8, 0.8, 1.0])
        self.declare_parameter("mesh_wait_poll_s", 0.25)
        self.declare_parameter("mesh_wait_freshness_epsilon_s", 0.1)
        self.declare_parameter("mesh_source_settle_s", 0.35)
        self.declare_parameter("mesh_stage_dir", "/tmp/behav3d_world_mesh_cache")
        self.declare_parameter("mesh_stage_keep", 20)
        self.declare_parameter("mesh_accumulate", False)
        self.declare_parameter("mesh_accumulate_max_markers", 100)
        self.declare_parameter("ply_marker_point_size", 0.0015)
        self.declare_parameter("ply_marker_max_points", 120000)
        self.declare_parameter("ply_marker_max_triangles", 80000)
        self.declare_parameter("ply_marker_prefer_points", True)
        self.declare_parameter("ply_marker_use_colors", True)

        self._mesh_frame_id = str(self.get_parameter("mesh_frame_id").value)
        self._mesh_topic = str(self.get_parameter("mesh_topic").value)
        self._mesh_ns = str(self.get_parameter("mesh_ns").value)
        self._mesh_scale = float(self.get_parameter("mesh_scale").value)
        self._mesh_rgba = self._parse_rgba(self.get_parameter("mesh_rgba").value)
        self._mesh_wait_poll_s = max(0.01, float(self.get_parameter("mesh_wait_poll_s").value))
        self._mesh_wait_freshness_epsilon_s = max(
            0.0, float(self.get_parameter("mesh_wait_freshness_epsilon_s").value)
        )
        self._mesh_source_settle_s = max(
            0.0, float(self.get_parameter("mesh_source_settle_s").value)
        )
        self._mesh_stage_dir = Path(str(self.get_parameter("mesh_stage_dir").value)).expanduser().resolve()
        self._mesh_stage_keep = max(1, int(self.get_parameter("mesh_stage_keep").value))
        self._mesh_accumulate = bool(self.get_parameter("mesh_accumulate").value)
        self._mesh_accumulate_max_markers = max(
            1, int(self.get_parameter("mesh_accumulate_max_markers").value)
        )
        self._ply_marker_point_size = max(1e-6, float(self.get_parameter("ply_marker_point_size").value))
        self._ply_marker_max_points = max(1, int(self.get_parameter("ply_marker_max_points").value))
        self._ply_marker_max_triangles = max(1, int(self.get_parameter("ply_marker_max_triangles").value))
        self._ply_marker_prefer_points = bool(self.get_parameter("ply_marker_prefer_points").value)
        self._ply_marker_use_colors = bool(self.get_parameter("ply_marker_use_colors").value)
        self._mesh_next_marker_id = 0
        self._mesh_active_marker_ids: List[int] = []

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._state_pub = self.create_publisher(String, "/behav3d/world_state", qos)
        self._mesh_pub = self.create_publisher(Marker, self._mesh_topic, qos)
        self._state_srv = self.create_service(Trigger, "/behav3d/get_world_state", self._handle_get_world_state)
        self._mesh_srv = self.create_service(UpdateWorldMesh, "/behav3d/update_world_mesh", self._handle_update_world_mesh)

        self._last_json: Optional[str] = None
        self._state: Dict[str, Any] = {}
        self._last_mesh_path: Optional[Path] = None
        self._last_mesh_kind: str = ""

        period = float(self.get_parameter("poll_period_s").value)
        self._timer = self.create_timer(period, self._poll)

        # Initial publish
        self._poll()
        self.get_logger().info(
            "World node ready: /behav3d/world_state, /behav3d/get_world_state, /behav3d/update_world_mesh"
        )
        self.get_logger().info(f"Mesh marker topic: {self._mesh_topic} (frame={self._mesh_frame_id})")
        self.get_logger().info(f"Mesh staging dir: {self._mesh_stage_dir}")
        self.get_logger().info(f"Mesh source settle time: {self._mesh_source_settle_s:.2f}s")
        self.get_logger().info(
            "PLY marker config: "
            f"prefer_points={self._ply_marker_prefer_points} "
            f"use_colors={self._ply_marker_use_colors} "
            f"point_size={self._ply_marker_point_size:.6f} "
            f"max_points={self._ply_marker_max_points} "
            f"max_triangles={self._ply_marker_max_triangles}"
        )
        self.get_logger().info(
            "Mesh accumulate mode: "
            f"{self._mesh_accumulate} (max markers={self._mesh_accumulate_max_markers})"
        )

    @staticmethod
    def _parse_rgba(value: Any) -> Tuple[float, float, float, float]:
        default = (0.8, 0.8, 0.8, 1.0)
        if not isinstance(value, (list, tuple)) or len(value) != 4:
            return default
        parsed = []
        for idx, component in enumerate(value):
            try:
                val = float(component)
            except (TypeError, ValueError):
                return default
            if idx < 3:
                val = max(0.0, min(1.0, val))
            else:
                val = max(0.0, min(1.0, val))
            parsed.append(val)
        return tuple(parsed)  # type: ignore[return-value]

    def _handle_get_world_state(self, req: Trigger.Request, res: Trigger.Response):
        _ = req
        if not self._state:
            res.success = False
            res.message = "world_state empty (no sessions found)"
            return res
        res.success = True
        res.message = self._last_json or json.dumps(self._state, ensure_ascii=False)
        return res

    def _handle_update_world_mesh(self, req: UpdateWorldMesh.Request, res: UpdateWorldMesh.Response):
        request_started_wall = time.time()
        session_dir = resolve_session_path(req.session_path, bool(req.use_latest), self.captures_root)
        res.session_dir = str(session_dir)

        if not session_dir.exists():
            res.success = False
            res.message = f"Session directory not found: {session_dir}"
            res.resolved_mesh_path = ""
            res.resolved_ply_path = ""
            res.published_path = ""
            res.published_kind = ""
            return res

        mesh_path = self._resolve_candidate_path(req.mesh_path, session_dir)
        ply_path = self._resolve_candidate_path(req.ply_path, session_dir)
        if mesh_path is None and ply_path is None:
            mesh_path = self._default_mesh_path(session_dir)
            ply_path = self._default_ply_path(session_dir)

        res.resolved_mesh_path = str(mesh_path) if mesh_path is not None else ""
        res.resolved_ply_path = str(ply_path) if ply_path is not None else ""

        prefer = str(req.prefer).strip().lower() or "mesh"
        if prefer not in ("mesh", "ply"):
            self.get_logger().warn(f"Unknown prefer='{req.prefer}', using 'mesh'")
            prefer = "mesh"

        candidates = self._ordered_candidates(mesh_path, ply_path, prefer=prefer)
        wait_timeout_s = max(0.0, float(req.wait_timeout_s))
        has_explicit_paths = bool(str(req.mesh_path).strip()) or bool(str(req.ply_path).strip())
        require_fresh_update = (wait_timeout_s > 0.0) and (not has_explicit_paths)
        min_mtime = None
        if require_fresh_update:
            min_mtime = request_started_wall - self._mesh_wait_freshness_epsilon_s

        selected_kind, selected_path = self._wait_for_candidate(
            candidates,
            wait_timeout_s,
            min_mtime=min_mtime,
        )

        if selected_path is None:
            labels = [f"{kind}:{path}" for kind, path in candidates if path is not None]
            label_text = ", ".join(labels) if labels else "(none)"
            res.success = False
            freshness_note = " (fresh update required)" if require_fresh_update else ""
            res.message = (
                f"No mesh candidate found after {wait_timeout_s:.1f}s{freshness_note}. "
                f"Candidates={label_text}"
            )
            res.published_path = ""
            res.published_kind = ""
            return res

        self._publish_mesh(selected_path, selected_kind)
        res.success = True
        res.message = f"Published {selected_kind} mesh: {selected_path}"
        res.published_path = str(selected_path)
        res.published_kind = selected_kind
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
            self._state_pub.publish(msg)

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
            "visualized_mesh_path": str(self._last_mesh_path) if self._last_mesh_path else "",
            "visualized_mesh_kind": self._last_mesh_kind,
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

    def _resolve_candidate_path(self, raw_path: str, session_dir: Path) -> Optional[Path]:
        value = str(raw_path or "").strip()
        if not value:
            return None

        if value.startswith("file://"):
            value = value[7:] if value.startswith("file:///") else value[7:]

        if value.startswith("@session"):
            return resolve_session_path(value, False, self.captures_root).resolve()

        p = Path(value).expanduser()
        if p.is_absolute():
            return p.resolve()
        return (session_dir / p).resolve()

    @staticmethod
    def _default_mesh_path(session_dir: Path) -> Path:
        latest = WorldNode._latest_existing(
            session_dir,
            (
                "reconstruct/**/tsdf_surface_mesh.stl",
                "reconstruct/**/tsdf_surface_mesh.ply",
                "reconstruct/**/tsdf_surface_mesh.obj",
                "**/reconstruct/tsdf_surface_mesh.stl",
                "**/reconstruct/tsdf_surface_mesh.ply",
                "**/reconstruct/tsdf_surface_mesh.obj",
                "tsdf_surface_mesh.stl",
                "tsdf_surface_mesh.ply",
                "tsdf_surface_mesh.obj",
            ),
        )
        if latest is not None:
            return latest

        candidates = (
            session_dir / "tsdf_surface_mesh.stl",
            session_dir / "tsdf_surface_mesh.ply",
            session_dir / "tsdf_surface_mesh.obj",
        )
        for candidate in candidates:
            if candidate.is_file():
                return candidate
        return candidates[0]

    @staticmethod
    def _default_ply_path(session_dir: Path) -> Path:
        latest = WorldNode._latest_existing(
            session_dir,
            (
                "reconstruct/**/tsdf_surface_rgb_colored.ply",
                "reconstruct/**/tsdf_surface_confidence_colored.ply",
                "reconstruct/**/tsdf_surface_mesh.ply",
                "**/reconstruct/tsdf_surface_rgb_colored.ply",
                "**/reconstruct/tsdf_surface_confidence_colored.ply",
                "**/reconstruct/tsdf_surface_mesh.ply",
                "tsdf_surface_rgb_colored.ply",
                "tsdf_surface_confidence_colored.ply",
                "tsdf_surface_mesh.ply",
            ),
        )
        if latest is not None:
            return latest

        candidates = (
            session_dir / "tsdf_surface_rgb_colored.ply",
            session_dir / "tsdf_surface_confidence_colored.ply",
            session_dir / "tsdf_surface_mesh.ply",
        )
        for candidate in candidates:
            if candidate.is_file():
                return candidate
        return candidates[0]

    @staticmethod
    def _latest_existing(session_dir: Path, patterns: Iterable[str]) -> Optional[Path]:
        found = []
        for pattern in patterns:
            for path in session_dir.glob(pattern):
                if path.is_file():
                    found.append(path.resolve())
        if not found:
            return None
        try:
            return max(found, key=lambda p: p.stat().st_mtime)
        except OSError:
            return found[-1]

    @staticmethod
    def _ordered_candidates(
        mesh_path: Optional[Path],
        ply_path: Optional[Path],
        *,
        prefer: str,
    ) -> Iterable[Tuple[str, Optional[Path]]]:
        if prefer == "ply":
            return (("ply", ply_path), ("mesh", mesh_path))
        return (("mesh", mesh_path), ("ply", ply_path))

    def _wait_for_candidate(
        self,
        candidates: Iterable[Tuple[str, Optional[Path]]],
        timeout_s: float,
        *,
        min_mtime: Optional[float] = None,
    ) -> Tuple[str, Optional[Path]]:
        candidates_list = [(kind, path) for kind, path in candidates if path is not None]
        if not candidates_list:
            return "", None

        if timeout_s <= 0.0:
            for kind, path in candidates_list:
                if self._is_candidate_ready(path, min_mtime=min_mtime):
                    return kind, path
            return "", None

        deadline = time.monotonic() + timeout_s
        while True:
            for kind, path in candidates_list:
                if self._is_candidate_ready(path, min_mtime=min_mtime):
                    return kind, path
            if time.monotonic() >= deadline:
                return "", None
            time.sleep(self._mesh_wait_poll_s)

    def _is_candidate_ready(self, path: Path, *, min_mtime: Optional[float] = None) -> bool:
        try:
            if not path.is_file():
                return False
            stat = path.stat()
            if stat.st_size <= 0:
                return False
            if min_mtime is None:
                if self._mesh_source_settle_s <= 0.0:
                    return True
                return (time.time() - stat.st_mtime) >= self._mesh_source_settle_s
            if stat.st_mtime < float(min_mtime):
                return False
            return (time.time() - stat.st_mtime) >= self._mesh_source_settle_s
        except OSError:
            return False

    def _publish_mesh(self, path: Path, kind: str) -> None:
        suffix = path.suffix.lower()
        if suffix not in (".stl", ".obj", ".ply", ".dae", ".mesh"):
            self.get_logger().warn(f"Publishing uncommon mesh extension '{suffix}' from {path}")

        self._refresh_mesh_accumulate_settings()

        staged_path = self._stage_mesh_file(path, kind)
        marker_id = self._reserve_marker_id()

        if suffix == ".ply":
            ply_marker, render_mode = self._build_ply_marker(staged_path=staged_path, marker_id=marker_id)
            if ply_marker is not None:
                self._mesh_pub.publish(ply_marker)
                if self._mesh_accumulate:
                    self._mesh_active_marker_ids.append(int(marker_id))
                    self._trim_accumulated_markers()
                self._last_mesh_path = path.resolve()
                self._last_mesh_kind = kind
                self.get_logger().info(
                    f"Published world mesh ({kind}/{render_mode}): "
                    f"source={path.resolve()} staged={staged_path}"
                )
                return

            self.get_logger().warn(
                f"PLY marker decode failed for {staged_path}; falling back to MESH_RESOURCE."
            )

        marker = Marker()
        marker.header.frame_id = self._mesh_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self._mesh_ns
        marker.id = int(marker_id)
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = staged_path.as_uri()
        marker.pose = Pose()

        scale = max(1e-6, float(self._mesh_scale))
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = float(self._mesh_rgba[0])
        marker.color.g = float(self._mesh_rgba[1])
        marker.color.b = float(self._mesh_rgba[2])
        marker.color.a = float(self._mesh_rgba[3])

        self._mesh_pub.publish(marker)
        if self._mesh_accumulate:
            self._mesh_active_marker_ids.append(int(marker_id))
            self._trim_accumulated_markers()
        self._last_mesh_path = path.resolve()
        self._last_mesh_kind = kind
        self.get_logger().info(
            f"Published world mesh ({kind}): source={path.resolve()} staged={staged_path}"
        )

    def _reserve_marker_id(self) -> int:
        if self._mesh_accumulate:
            marker_id = int(self._mesh_next_marker_id)
            self._mesh_next_marker_id += 1
            return marker_id

        if self._mesh_active_marker_ids:
            self._clear_accumulated_markers()
        self._mesh_next_marker_id = 1
        return 0

    def _build_base_add_marker(self, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._mesh_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self._mesh_ns
        marker.id = int(marker_id)
        marker.action = Marker.ADD
        marker.pose = Pose()
        marker.color.r = float(self._mesh_rgba[0])
        marker.color.g = float(self._mesh_rgba[1])
        marker.color.b = float(self._mesh_rgba[2])
        marker.color.a = float(self._mesh_rgba[3])
        return marker

    def _build_ply_marker(self, staged_path: Path, marker_id: int) -> tuple[Optional[Marker], str]:
        if self._ply_marker_prefer_points:
            marker = self._build_ply_points_marker(staged_path, marker_id)
            if marker is not None:
                return marker, "points"
            marker = self._build_ply_triangle_marker(staged_path, marker_id)
            if marker is not None:
                return marker, "triangle_list"
            return None, ""

        marker = self._build_ply_triangle_marker(staged_path, marker_id)
        if marker is not None:
            return marker, "triangle_list"
        marker = self._build_ply_points_marker(staged_path, marker_id)
        if marker is not None:
            return marker, "points"
        return None, ""

    def _build_ply_points_marker(self, staged_path: Path, marker_id: int) -> Optional[Marker]:
        try:
            pcd = o3d.io.read_point_cloud(str(staged_path))
        except Exception as exc:
            self.get_logger().warn(f"Failed to read PLY point cloud '{staged_path}': {exc}")
            return None

        points = np.asarray(pcd.points, dtype=np.float64)
        if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] != 3:
            return None

        colors: Optional[np.ndarray] = None
        if pcd.has_colors():
            c = np.asarray(pcd.colors, dtype=np.float64)
            if c.shape == points.shape:
                colors = np.clip(c, 0.0, 1.0)

        if points.shape[0] > self._ply_marker_max_points:
            stride = int(math.ceil(float(points.shape[0]) / float(self._ply_marker_max_points)))
            idx = np.arange(0, points.shape[0], stride, dtype=np.int64)[: self._ply_marker_max_points]
            points = points[idx]
            if colors is not None:
                colors = colors[idx]

        marker = self._build_base_add_marker(marker_id)
        marker.type = Marker.POINTS
        marker.scale.x = float(self._ply_marker_point_size)
        marker.scale.y = float(self._ply_marker_point_size)
        marker.scale.z = float(self._ply_marker_point_size)
        marker.points = [
            Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
            for p in points
        ]

        if self._ply_marker_use_colors and colors is not None:
            alpha = float(self._mesh_rgba[3])
            marker.colors = [
                ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=alpha)
                for c in colors
            ]

        return marker

    def _build_ply_triangle_marker(self, staged_path: Path, marker_id: int) -> Optional[Marker]:
        try:
            mesh = o3d.io.read_triangle_mesh(str(staged_path))
        except Exception as exc:
            self.get_logger().warn(f"Failed to read PLY triangle mesh '{staged_path}': {exc}")
            return None

        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        triangles = np.asarray(mesh.triangles, dtype=np.int32)
        if (
            vertices.ndim != 2
            or triangles.ndim != 2
            or vertices.shape[0] == 0
            or triangles.shape[0] == 0
            or vertices.shape[1] != 3
            or triangles.shape[1] != 3
        ):
            return None

        vcolors: Optional[np.ndarray] = None
        if mesh.has_vertex_colors():
            c = np.asarray(mesh.vertex_colors, dtype=np.float64)
            if c.shape[0] == vertices.shape[0] and c.shape[1] == 3:
                vcolors = np.clip(c, 0.0, 1.0)

        if triangles.shape[0] > self._ply_marker_max_triangles:
            stride = int(math.ceil(float(triangles.shape[0]) / float(self._ply_marker_max_triangles)))
            triangles = triangles[::stride][: self._ply_marker_max_triangles]

        marker = self._build_base_add_marker(marker_id)
        marker.type = Marker.TRIANGLE_LIST
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        out_points: List[Point] = []
        out_colors: List[ColorRGBA] = []
        alpha = float(self._mesh_rgba[3])
        use_colors = self._ply_marker_use_colors and (vcolors is not None)

        for tri in triangles:
            for vid in tri:
                p = vertices[int(vid)]
                out_points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
                if use_colors and vcolors is not None:
                    c = vcolors[int(vid)]
                    out_colors.append(ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=alpha))

        marker.points = out_points
        if use_colors:
            marker.colors = out_colors
        return marker

    def _refresh_mesh_accumulate_settings(self) -> None:
        mesh_accumulate = bool(self.get_parameter("mesh_accumulate").value)
        max_markers = max(1, int(self.get_parameter("mesh_accumulate_max_markers").value))

        if mesh_accumulate != self._mesh_accumulate:
            self._mesh_accumulate = mesh_accumulate
            self.get_logger().info(
                "Mesh accumulate mode changed: "
                f"{self._mesh_accumulate} (max markers={max_markers})"
            )
            if self._mesh_accumulate:
                self._mesh_next_marker_id = max(1, int(self._mesh_next_marker_id))

        if max_markers != self._mesh_accumulate_max_markers:
            self._mesh_accumulate_max_markers = max_markers
            self.get_logger().info(
                f"mesh_accumulate_max_markers set to {self._mesh_accumulate_max_markers}"
            )
            if self._mesh_accumulate:
                self._trim_accumulated_markers()

    def _clear_accumulated_markers(self) -> None:
        if not self._mesh_active_marker_ids:
            return
        for marker_id in self._mesh_active_marker_ids:
            self._publish_delete_marker(marker_id)
        self._mesh_active_marker_ids.clear()

    def _trim_accumulated_markers(self) -> None:
        while len(self._mesh_active_marker_ids) > self._mesh_accumulate_max_markers:
            old_id = self._mesh_active_marker_ids.pop(0)
            self._publish_delete_marker(old_id)

    def _publish_delete_marker(self, marker_id: int) -> None:
        marker = Marker()
        marker.header.frame_id = self._mesh_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self._mesh_ns
        marker.id = int(marker_id)
        marker.action = Marker.DELETE
        self._mesh_pub.publish(marker)

    def _stage_mesh_file(self, src_path: Path, kind: str) -> Path:
        self._mesh_stage_dir.mkdir(parents=True, exist_ok=True)
        ts_ms = int(time.time() * 1000.0)
        suffix = src_path.suffix or ".mesh"
        staged_name = f"{kind}_{ts_ms}{suffix}"
        tmp_name = f".{staged_name}.tmp"
        tmp_path = self._mesh_stage_dir / tmp_name
        staged_path = self._mesh_stage_dir / staged_name

        shutil.copy2(src_path, tmp_path)
        tmp_path.replace(staged_path)

        pattern = f"{kind}_*"
        staged_files = sorted(
            self._mesh_stage_dir.glob(pattern),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        for stale in staged_files[self._mesh_stage_keep:]:
            try:
                stale.unlink()
            except OSError:
                pass
        return staged_path.resolve()


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
