#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node

try:
    import potpourri3d as pp3d
except Exception:  # pragma: no cover
    pp3d = None

try:
    from behav3d_interfaces.srv import InitFieldFromScan
except ImportError:  # pragma: no cover
    from behav3d_interfaces.srv._init_field_from_scan import InitFieldFromScan

from .reconstruct.service_utils import get_captures_root, resolve_session_path


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


def _load_triangle_mesh_arrays(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    vertices, faces = pp3d.read_mesh(str(path))
    if vertices.ndim != 2 or vertices.shape[1] != 3:
        raise ValueError(f"Unexpected vertex array shape for field mesh: {vertices.shape}")
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(f"Field mesh must contain triangles. Faces shape: {faces.shape}")
    return vertices.astype(np.float64), faces.astype(np.int32)


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


def _make_scan_scene(scan_mesh: o3d.geometry.TriangleMesh):
    vertices = np.asarray(scan_mesh.vertices, dtype=np.float64)
    if vertices.size == 0:
        raise ValueError("Scan mesh has no vertices.")
    z_top = float(np.max(vertices[:, 2]) + 1e3)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(scan_mesh))
    return scene, z_top


def _query_scan_z_with_vertical_rays(
    scene: o3d.t.geometry.RaycastingScene,
    points_world: np.ndarray,
    z_top: float,
) -> Tuple[np.ndarray, np.ndarray]:
    n = points_world.shape[0]
    rays = np.zeros((n, 6), dtype=np.float32)
    rays[:, 0] = points_world[:, 0]
    rays[:, 1] = points_world[:, 1]
    rays[:, 2] = z_top
    rays[:, 5] = -1.0
    out = scene.cast_rays(o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32))
    t_hit = out["t_hit"].numpy()
    has_hit = np.isfinite(t_hit)
    z_scan = np.full(n, np.nan, dtype=np.float64)
    z_scan[has_hit] = z_top - t_hit[has_hit]
    return z_scan, has_hit


def _make_axis_samples(vmin: float, vmax: float, step: float) -> np.ndarray:
    if step <= 0.0:
        raise ValueError(f"Search step must be > 0, got {step}")
    lo = float(min(vmin, vmax))
    hi = float(max(vmin, vmax))
    count = int(np.floor((hi - lo) / step)) + 1
    values = lo + step * np.arange(max(count, 1), dtype=np.float64)
    if values.size == 0:
        values = np.array([lo], dtype=np.float64)
    if values[-1] < (hi - 0.5 * step):
        values = np.append(values, hi)
    return values


def _default_xy_search_bounds(
    scan_vertices: np.ndarray,
    field_vertices_scaled: np.ndarray,
) -> Tuple[float, float, float, float]:
    scan_min = np.min(scan_vertices, axis=0)
    scan_max = np.max(scan_vertices, axis=0)
    field_min = np.min(field_vertices_scaled, axis=0)
    field_max = np.max(field_vertices_scaled, axis=0)
    x_min = float(scan_min[0] - field_max[0])
    x_max = float(scan_max[0] - field_min[0])
    y_min = float(scan_min[1] - field_max[1])
    y_max = float(scan_max[1] - field_min[1])
    return x_min, x_max, y_min, y_max


def _search_position(
    scene: o3d.t.geometry.RaycastingScene,
    z_top: float,
    field_vertices_scaled: np.ndarray,
    heat_norm: np.ndarray,
    x_values: np.ndarray,
    y_values: np.ndarray,
    clearance: float,
    base_epsilon: float,
) -> dict:
    n = field_vertices_scaled.shape[0]
    base_local_z = float(np.min(field_vertices_scaled[:, 2]))
    best = None
    best_key = None
    tested = 0
    accepted = 0

    for ox in x_values:
        for oy in y_values:
            tested += 1

            probe = field_vertices_scaled.copy()
            probe[:, 0] += float(ox)
            probe[:, 1] += float(oy)
            z_scan, has_hit = _query_scan_z_with_vertical_rays(scene, probe, z_top=z_top)
            hit_count = int(np.count_nonzero(has_hit))
            if hit_count < n:
                continue

            z_offset = float(np.min(z_scan[has_hit] - base_local_z) - float(base_epsilon))
            field_world = probe.copy()
            field_world[:, 2] = field_vertices_scaled[:, 2] + z_offset

            phi = np.full(n, -np.inf, dtype=np.float64)
            phi[has_hit] = field_world[has_hit, 2] - z_scan[has_hit] - float(clearance)
            viable = (phi > 0.0) & has_hit
            viable_count = int(np.count_nonzero(viable))
            viable_heat = float(np.sum(heat_norm[viable]))

            base_world_z = float(base_local_z + z_offset)
            base_dz = np.full(n, np.inf, dtype=np.float64)
            base_dz[has_hit] = base_world_z - z_scan[has_hit]
            if not bool(np.all(base_dz[has_hit] <= 1e-9)):
                continue
            accepted += 1

            key = (viable_count, viable_heat, hit_count, z_offset)
            if best_key is None or key > best_key:
                best_key = key
                best = {
                    "offset_xyz": (float(ox), float(oy), float(z_offset)),
                    "field_vertices_world": field_world,
                    "phi": phi,
                    "viable": viable,
                    "tested": tested,
                    "accepted": accepted,
                    "viable_count": viable_count,
                }

    if best is None:
        raise RuntimeError("No feasible field position found.")
    best["tested"] = tested
    best["accepted"] = accepted
    return best


def _position_field_with_attempts(
    scan_mesh: o3d.geometry.TriangleMesh,
    field_vertices_scaled: np.ndarray,
    heat_norm: np.ndarray,
    *,
    clearance: float,
    base_epsilon: float,
    search_step_x: float,
    search_step_y: float,
    search_max_candidates: int,
    positioning_attempts: int,
) -> dict:
    attempts = max(1, int(positioning_attempts))
    scene, z_top = _make_scan_scene(scan_mesh)
    scan_vertices = np.asarray(scan_mesh.vertices, dtype=np.float64)
    x_min0, x_max0, y_min0, y_max0 = _default_xy_search_bounds(scan_vertices, field_vertices_scaled)

    best_position = None
    best_score = -np.inf
    for attempt in range(attempts):
        grow_x = float(attempt) * float(search_step_x)
        grow_y = float(attempt) * float(search_step_y)
        x_values = _make_axis_samples(x_min0 - grow_x, x_max0 + grow_x, float(search_step_x))
        y_values = _make_axis_samples(y_min0 - grow_y, y_max0 + grow_y, float(search_step_y))
        total = int(x_values.size * y_values.size)
        if total > int(search_max_candidates):
            raise ValueError(
                f"Position search grid too large on attempt {attempt + 1}: {total} "
                f"(limit={search_max_candidates})."
            )

        position = _search_position(
            scene=scene,
            z_top=z_top,
            field_vertices_scaled=field_vertices_scaled,
            heat_norm=heat_norm,
            x_values=x_values,
            y_values=y_values,
            clearance=float(clearance),
            base_epsilon=float(base_epsilon),
        )
        score = float(np.sum(heat_norm[position["viable"]]))
        if score > best_score:
            best_position = position
            best_score = score
        if position["viable_count"] > 0:
            break

    if best_position is None:
        raise RuntimeError("No position found in search attempts.")
    return best_position


def _yellow_to_red_colors(norm_scalar: np.ndarray) -> np.ndarray:
    n = np.clip(norm_scalar, 0.0, 1.0)
    colors = np.zeros((norm_scalar.shape[0], 3), dtype=np.float64)
    colors[:, 0] = 1.0
    colors[:, 1] = 1.0 - n
    colors[:, 2] = 0.0
    return colors


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
        if pp3d is None:
            res.success = False
            res.message = "potpourri3d not available in runtime environment."
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
            field_vertices, field_faces = _load_triangle_mesh_arrays(field_mesh_path)

            seed_level = float(self.get_parameter("seed_level").value)
            seed_vertices = np.flatnonzero(field_vertices[:, 2] < seed_level).astype(np.int64)
            if seed_vertices.size == 0:
                raise ValueError(
                    f"No seed vertices for z < {seed_level}. "
                    f"Field z range=[{float(np.min(field_vertices[:, 2])):.6f},"
                    f"{float(np.max(field_vertices[:, 2])):.6f}]"
                )

            t_coef = float(self.get_parameter("t_coef").value)
            solver = pp3d.MeshHeatMethodDistanceSolver(
                field_vertices,
                field_faces,
                t_coef=t_coef,
                use_robust=True,
            )
            heat_dist = solver.compute_distance_multisource(seed_vertices.tolist())
            finite = np.isfinite(heat_dist)
            if not np.any(finite):
                raise RuntimeError("Heat field has no finite values.")
            h_min = float(np.min(heat_dist[finite]))
            h_max = float(np.max(heat_dist[finite]))
            denom = max(h_max - h_min, 1e-12)
            heat_norm = np.ones_like(heat_dist, dtype=np.float64)
            heat_norm[finite] = (heat_dist[finite] - h_min) / denom

            field_scale = float(self.get_parameter("field_scale").value)
            if field_scale <= 0.0:
                raise ValueError(f"field_scale must be > 0, got {field_scale}")
            field_vertices_scaled = field_vertices * field_scale

            position = _position_field_with_attempts(
                scan_mesh=scan_mesh,
                field_vertices_scaled=field_vertices_scaled,
                heat_norm=heat_norm,
                clearance=float(self.get_parameter("clearance").value),
                base_epsilon=float(self.get_parameter("base_epsilon").value),
                search_step_x=float(self.get_parameter("search_step_x").value),
                search_step_y=float(self.get_parameter("search_step_y").value),
                search_max_candidates=int(self.get_parameter("search_max_candidates").value),
                positioning_attempts=int(self.get_parameter("positioning_attempts").value),
            )

            out_dir = _resolve_output_dir(req.state_output_dir, session_dir, self._captures_root)
            out_dir.mkdir(parents=True, exist_ok=True)
            state_path = out_dir / str(self.get_parameter("state_filename").value)
            debug_field_path = out_dir / str(self.get_parameter("debug_field_ply_filename").value)

            np.savez_compressed(
                state_path,
                field_vertices=field_vertices,
                field_faces=field_faces,
                field_vertices_scaled=field_vertices_scaled,
                field_vertices_world=position["field_vertices_world"],
                heat_dist=heat_dist,
                heat_norm=heat_norm,
                phi=position["phi"],
                viable=position["viable"].astype(np.uint8),
                offset_xyz=np.asarray(position["offset_xyz"], dtype=np.float64),
                field_scale=np.asarray([field_scale], dtype=np.float64),
                seed_level=np.asarray([seed_level], dtype=np.float64),
                t_coef=np.asarray([t_coef], dtype=np.float64),
                clearance=np.asarray([float(self.get_parameter("clearance").value)], dtype=np.float64),
            )

            colors = _yellow_to_red_colors(heat_norm)
            colors[~position["viable"]] = np.array([0.2, 0.2, 0.2], dtype=np.float64)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(position["field_vertices_world"])
            pcd.colors = o3d.utility.Vector3dVector(colors)
            if not o3d.io.write_point_cloud(str(debug_field_path), pcd):
                raise RuntimeError(f"Failed to write debug field PLY: {debug_field_path}")

            res.success = True
            res.message = (
                f"Field initialized. scan_meshes={len(scan_paths)} "
                f"field_v={field_vertices.shape[0]} field_f={field_faces.shape[0]} "
                f"viable={int(np.count_nonzero(position['viable']))}"
            )
            res.resolved_scan_mesh_path = str(scan_paths[0])
            res.resolved_field_mesh_path = str(field_mesh_path)
            res.field_state_path = str(state_path)
            res.debug_field_ply_path = str(debug_field_path)
            res.offset_x = float(position["offset_xyz"][0])
            res.offset_y = float(position["offset_xyz"][1])
            res.offset_z = float(position["offset_xyz"][2])
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
