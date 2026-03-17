#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import copy
from pathlib import Path
from typing import Iterable, List, Optional

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node

try:
    from behav3d_interfaces.srv import InitFieldFromScan
except ImportError:  # pragma: no cover
    from behav3d_interfaces.srv._init_field_from_scan import InitFieldFromScan

from .reconstruct.service_utils import get_captures_root, resolve_session_path

try:
    from behav3d_py.scalar_field.lib_scalar.compute_heat_field import compute_heat_field
    from behav3d_py.scalar_field.lib_scalar.geometry import load_triangle_mesh_arrays
    from behav3d_py.scalar_field.lib_scalar.loop_simulation import position_field_with_attempts
    from behav3d_py.scalar_field.lib_scalar.viz import make_point_cloud, yellow_to_red_colors

    _SCALAR_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover
    compute_heat_field = None
    load_triangle_mesh_arrays = None
    position_field_with_attempts = None
    make_point_cloud = None
    yellow_to_red_colors = None
    _SCALAR_IMPORT_ERROR = exc


def _resolve_input_path(raw_path: str, session_dir: Path, captures_root: Path) -> Optional[Path]:
    value = str(raw_path or "").strip()
    if not value:
        return None
    if value.startswith("file://"):
        value = value[7:] if value.startswith("file:///") else value[7:]
    if value.startswith("@session"):
        return resolve_session_path(value, False, captures_root).resolve()

    p = Path(value).expanduser()
    if p.is_absolute():
        return p.resolve()

    candidate_session = (session_dir / p).resolve()
    if candidate_session.exists():
        return candidate_session
    return (Path.cwd() / p).resolve()


def _resolve_output_dir(raw_path: str, session_dir: Path, captures_root: Path) -> Path:
    value = str(raw_path or "").strip()
    if not value:
        value = "@session/field_loop/init"
    if value.startswith("@session"):
        return resolve_session_path(value, False, captures_root).resolve()
    p = Path(value).expanduser()
    if p.is_absolute():
        return p.resolve()
    return (session_dir / p).resolve()


def _latest_existing(base_dir: Path, patterns: Iterable[str]) -> Optional[Path]:
    matches: List[Path] = []
    for pattern in patterns:
        try:
            for path in base_dir.glob(pattern):
                if path.is_file():
                    matches.append(path.resolve())
        except Exception:
            continue
    if not matches:
        return None
    matches.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return matches[0]


def _default_scan_mesh_path(session_dir: Path) -> Optional[Path]:
    return _latest_existing(
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


def _load_scan_mesh(paths: List[Path]) -> o3d.geometry.TriangleMesh:
    if not paths:
        raise ValueError("No scan mesh paths to load.")

    merged: Optional[o3d.geometry.TriangleMesh] = None
    for path in paths:
        mesh = o3d.io.read_triangle_mesh(str(path))
        if not mesh.has_triangles():
            raise ValueError(f"Scan mesh has no triangles: {path}")
        if merged is None:
            merged = copy.deepcopy(mesh)
        else:
            merged += mesh
    assert merged is not None
    return merged


class FieldsNode(Node):
    def __init__(self):
        super().__init__("fields_node")
        self._captures_root = get_captures_root()

        self.declare_parameter("field_scale", 0.001)
        self.declare_parameter("seed_level", 1.0)
        self.declare_parameter("t_coef", 2000.0)
        self.declare_parameter("clearance", 0.0)
        self.declare_parameter("search_step_x", 0.01)
        self.declare_parameter("search_step_y", 0.01)
        self.declare_parameter("search_max_candidates", 20000)
        self.declare_parameter("positioning_attempts", 3)
        self.declare_parameter("base_epsilon", 1e-6)
        self.declare_parameter("state_filename", "field_state_init.npz")
        self.declare_parameter("debug_field_ply_filename", "field_masked_init.ply")

        self._srv = self.create_service(
            InitFieldFromScan,
            "/behav3d/init_field_from_scan",
            self._handle_init_field_from_scan,
        )
        self.get_logger().info("Fields node ready: /behav3d/init_field_from_scan")

    def _handle_init_field_from_scan(
        self,
        req: InitFieldFromScan.Request,
        res: InitFieldFromScan.Response,
    ) -> InitFieldFromScan.Response:
        if _SCALAR_IMPORT_ERROR is not None:
            res.success = False
            res.message = f"scalar_field import failed: {_SCALAR_IMPORT_ERROR}"
            return res

        try:
            session_dir = resolve_session_path(req.session_path, bool(req.use_latest), self._captures_root).resolve()
            res.session_dir = str(session_dir)
            if not session_dir.exists():
                raise FileNotFoundError(f"Session directory not found: {session_dir}")

            scan_paths: List[Path] = []
            for raw in list(req.scan_mesh_paths):
                p = _resolve_input_path(raw, session_dir, self._captures_root)
                if p is not None and p.is_file():
                    scan_paths.append(p)
            if not scan_paths:
                default_scan = _default_scan_mesh_path(session_dir)
                if default_scan is None:
                    raise FileNotFoundError("No scan mesh path provided and no default tsdf mesh found in session.")
                scan_paths = [default_scan]

            field_mesh_path = _resolve_input_path(req.field_mesh_path, session_dir, self._captures_root)
            if field_mesh_path is None or not field_mesh_path.is_file():
                raise FileNotFoundError(f"Field mesh not found: '{req.field_mesh_path}'")

            scan_mesh = _load_scan_mesh(scan_paths)
            field_mesh = load_triangle_mesh_arrays(field_mesh_path)
            field_vertices = np.asarray(field_mesh.vertices, dtype=np.float64)
            field_faces = np.asarray(field_mesh.faces, dtype=np.int32)

            seed_level = float(self.get_parameter("seed_level").value)
            t_coef = float(self.get_parameter("t_coef").value)
            heat_field = compute_heat_field(
                vertices=field_vertices,
                faces=field_faces,
                seed=None,
                seed_level=seed_level,
                t_coef=t_coef,
            )
            heat_dist = np.asarray(heat_field.dist, dtype=np.float64)
            heat_norm = np.asarray(heat_field.norm, dtype=np.float64)

            field_scale = float(self.get_parameter("field_scale").value)
            if field_scale <= 0.0:
                raise ValueError(f"field_scale must be > 0, got {field_scale}")
            field_vertices_scaled = field_vertices * field_scale

            position = position_field_with_attempts(
                scan_mesh=scan_mesh,
                field_vertices_scaled=field_vertices_scaled,
                heat_norm=heat_norm,
                clearance=float(self.get_parameter("clearance").value),
                iso_level=0.0,
                base_epsilon=float(self.get_parameter("base_epsilon").value),
                search_step_x=float(self.get_parameter("search_step_x").value),
                search_step_y=float(self.get_parameter("search_step_y").value),
                positioning_attempts=int(self.get_parameter("positioning_attempts").value),
                search_max_candidates=int(self.get_parameter("search_max_candidates").value),
            )
            viable = np.asarray(position.viable, dtype=bool)

            out_dir = _resolve_output_dir(req.state_output_dir, session_dir, self._captures_root)
            out_dir.mkdir(parents=True, exist_ok=True)
            state_path = out_dir / str(self.get_parameter("state_filename").value)
            debug_field_path = out_dir / str(self.get_parameter("debug_field_ply_filename").value)

            np.savez_compressed(
                state_path,
                field_vertices=field_vertices,
                field_faces=field_faces,
                field_vertices_scaled=field_vertices_scaled,
                field_vertices_world=position.field_vertices_world,
                heat_dist=heat_dist,
                heat_norm=heat_norm,
                phi=position.phi,
                viable=viable.astype(np.uint8),
                offset_xyz=np.asarray(position.offset_xyz, dtype=np.float64),
                field_scale=np.asarray([field_scale], dtype=np.float64),
                seed_level=np.asarray([seed_level], dtype=np.float64),
                t_coef=np.asarray([t_coef], dtype=np.float64),
                clearance=np.asarray([float(self.get_parameter("clearance").value)], dtype=np.float64),
            )

            colors = yellow_to_red_colors(heat_norm)
            colors[~viable] = np.array([0.2, 0.2, 0.2], dtype=np.float64)
            pcd = make_point_cloud(position.field_vertices_world, colors)
            if not o3d.io.write_point_cloud(str(debug_field_path), pcd):
                raise RuntimeError(f"Failed to write debug field PLY: {debug_field_path}")

            res.success = True
            res.message = (
                f"Field initialized. scan_meshes={len(scan_paths)} "
                f"field_v={field_vertices.shape[0]} field_f={field_faces.shape[0]} "
                f"viable={int(np.count_nonzero(viable))}"
            )
            res.resolved_scan_mesh_path = str(scan_paths[0])
            res.resolved_field_mesh_path = str(field_mesh_path)
            res.field_state_path = str(state_path)
            res.debug_field_ply_path = str(debug_field_path)
            res.offset_x = float(position.offset_xyz[0])
            res.offset_y = float(position.offset_xyz[1])
            res.offset_z = float(position.offset_xyz[2])
            return res
        except Exception as exc:
            res.success = False
            res.message = str(exc)
            return res


def main(args=None):
    rclpy.init(args=args)
    node = FieldsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
