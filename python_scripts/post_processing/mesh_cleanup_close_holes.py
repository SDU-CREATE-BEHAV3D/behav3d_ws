#!/usr/bin/env python3
"""
Clean a triangle mesh by repairing open boundaries and optionally capping the bottom.

This script is intended as a post-process after TSDF / scan-mesh generation.
It focuses on:
- loading a mesh and printing diagnostics
- removing obviously bad mesh elements
- optionally keeping only the largest connected triangle component
- detecting open boundary loops
- filling planar-ish holes directly on the existing boundary
- optionally capping low boundary loops against a flat bottom plane
- saving the repaired mesh

This is a pragmatic cleanup pass, not a full watertight remeshing pipeline.
The hole-filling step assumes boundary loops are reasonably simple polygons.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d

try:
    from scipy.ndimage import binary_dilation, binary_fill_holes
    from scipy.spatial import Delaunay, QhullError
except ImportError:  # pragma: no cover - optional dependency at runtime
    binary_dilation = None
    binary_fill_holes = None
    Delaunay = None
    QhullError = RuntimeError


# -----------------------------------------------------------------------------
# In-script parameters
# -----------------------------------------------------------------------------

default_input_mesh_path = "~/Downloads/260227_160709/print_scan_042/reconstruct/previous_plus_new_geometry.stl"
default_output_mesh_path = ""
show_debug_vis = True
keep_largest_component = True
keep_component_min_triangle_fraction = 0.10  # Preserve meaningful trimmed bead chunks instead of only the single largest one.
keep_component_min_triangles = 50
repair_target = "bead_component"  # "all_posttrim", "largest_component_only", or "bead_component" (prefers the tallest component above the table).
fill_holes_enable = True
fill_holes_max_boundary_edges = 400
fill_holes_max_planarity_mm = 4.0
table_trim_enable = True
table_ransac_thresh_mm = 2.0
table_ransac_n = 3
table_ransac_iters = 2500
table_ransac_seed = 1  # Stabilizes the table-plane fit so repair results are repeatable.
table_fit_low_band_mm = 2.0  # For the current print_scan_014 mesh. Set 0.0 to go back to fitting on all vertices.
z_min_above_plane_mm = 2.0
bottom_cap_enable = True
bottom_cap_mode = "planar_in_place"  # "auto", "planar_in_place", "snap_to_plane", "local_patch", or "extruded"
bottom_axis = "z"  # "x", "y", or "z"
bottom_loop_height_tol_mm = 4.0
bottom_cap_plane_offset_mm = 0.0
bottom_local_patch_resolution_mm = 0.8
bottom_local_patch_margin_mm = 1.6
loop_repair_order = "try_both"  # "side_first", "bottom_first", or "try_both"
repair_backend = "custom"  # "none", "custom", or "pymeshfix". Custom preserves the trimmed shape better than PyMeshFix on the current 014 case.
pymeshfix_fix_connectivity = True
pymeshfix_clean_max_iters = 20
pymeshfix_clean_inner_loops = 5
pymeshfix_post_trim_bottom = False  # Experimental mixed-method cleanup. Keep off for the normal table-trim->repair flow.
pymeshfix_post_trim_bottom_mode = "planar_in_place"  # Clean bottom cap mode used after the PyMeshFix retrim.
output_write_ascii = False

mesh_color_before = (0.72, 0.72, 0.78)
mesh_color_after = (0.90, 0.20, 0.20)


@dataclass(frozen=True)
class LoopRepairResult:
    triangles_added: list[tuple[int, int, int]]
    vertices_added: np.ndarray
    repaired_loop_count: int
    side_fill_count: int
    bottom_cap_count: int
    local_patch_fallback_count: int
    skipped_loop_count: int
    backend_used: str
    order_used: str


@dataclass(frozen=True)
class TableTrimResult:
    mesh_table_trimmed: o3d.geometry.TriangleMesh
    T_table_from_world: np.ndarray
    T_world_from_table: np.ndarray
    plane_model_world: np.ndarray
    source_vertices: int
    source_triangles: int
    fit_seed_vertices: int
    trimmed_vertices: int
    trimmed_triangles: int
    cut_vertices_added: int


def _mm_to_m(value_mm: float) -> float:
    return float(value_mm) * 1e-3


def _axis_to_index(axis_name: str) -> int:
    axis_name = str(axis_name).strip().lower()
    mapping = {"x": 0, "y": 1, "z": 2}
    if axis_name not in mapping:
        raise ValueError(f"Unsupported bottom axis: {axis_name}. Expected one of x, y, z.")
    return mapping[axis_name]


def _normalize_plane(plane: np.ndarray) -> np.ndarray:
    plane = np.asarray(plane, dtype=np.float64).reshape(4)
    normal_norm = np.linalg.norm(plane[:3])
    if normal_norm < 1e-12:
        raise RuntimeError("Degenerate table plane model.")
    return plane / normal_norm


def _rotation_matrix_from_a_to_b(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)

    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))

    if s < 1e-12:
        if c > 0.0:
            return np.eye(3, dtype=np.float64)
        axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        axis = axis - axis.dot(a) * a
        axis /= np.linalg.norm(axis)
        x, y, z = axis
        R = -np.eye(3, dtype=np.float64)
        R += 2.0 * np.array(
            [[x * x, x * y, x * z], [y * x, y * y, y * z], [z * x, z * y, z * z]],
            dtype=np.float64,
        )
        return R

    vx = np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + vx + (vx @ vx) * ((1.0 - c) / (s * s))


def _transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    return (R @ points.T).T + t


def _resolve_input_mesh_path(mesh_path_str: str | None) -> Path:
    candidate = mesh_path_str if mesh_path_str and str(mesh_path_str).strip() else default_input_mesh_path
    if not str(candidate).strip():
        raise ValueError(
            "Missing input mesh path. Set default_input_mesh_path at the top of the script or pass --input-mesh."
        )
    mesh_path = Path(candidate).expanduser().resolve()
    if not mesh_path.exists():
        raise FileNotFoundError(f"Input mesh not found: {mesh_path}")
    return mesh_path


def _resolve_output_mesh_path(input_mesh_path: Path, output_mesh_path_str: str | None) -> Path:
    if output_mesh_path_str and str(output_mesh_path_str).strip():
        return Path(output_mesh_path_str).expanduser().resolve()
    return input_mesh_path.with_name(f"{input_mesh_path.stem}_cleaned{input_mesh_path.suffix}")


def _load_mesh(mesh_path: Path) -> o3d.geometry.TriangleMesh:
    mesh = o3d.io.read_triangle_mesh(str(mesh_path), enable_post_processing=True)
    if len(mesh.vertices) == 0:
        raise RuntimeError(f"Mesh has no vertices: {mesh_path}")
    if len(mesh.triangles) == 0:
        raise RuntimeError(f"Mesh has no triangles: {mesh_path}")
    mesh.compute_vertex_normals()
    return mesh


def _copy_mesh(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    return o3d.geometry.TriangleMesh(mesh)


def _basic_cleanup(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    mesh_out = _copy_mesh(mesh)
    mesh_out.remove_duplicated_vertices()
    mesh_out.remove_duplicated_triangles()
    mesh_out.remove_degenerate_triangles()
    mesh_out.remove_unreferenced_vertices()
    if hasattr(mesh_out, "remove_non_manifold_edges"):
        mesh_out.remove_non_manifold_edges()
    mesh_out.remove_unreferenced_vertices()
    mesh_out.compute_vertex_normals()
    return mesh_out


def _pretrim_sanitize(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    # Keep only the cheapest fixes before table trim so the plane cut sees a sane mesh.
    mesh_out = _copy_mesh(mesh)
    mesh_out.remove_duplicated_vertices()
    mesh_out.remove_duplicated_triangles()
    mesh_out.remove_degenerate_triangles()
    mesh_out.remove_unreferenced_vertices()
    mesh_out.compute_vertex_normals()
    return mesh_out


def _dedupe_polygon_indices(indices: list[int]) -> list[int]:
    cleaned: list[int] = []
    for index in indices:
        if not cleaned or cleaned[-1] != int(index):
            cleaned.append(int(index))
    if len(cleaned) > 1 and cleaned[0] == cleaned[-1]:
        cleaned.pop()
    return cleaned


def _is_valid_triangle(
    triangle: tuple[int, int, int],
    vertices: list[np.ndarray],
    area_epsilon: float = 1e-18,
) -> bool:
    i0, i1, i2 = triangle
    if i0 == i1 or i1 == i2 or i2 == i0:
        return False
    p0 = np.asarray(vertices[i0], dtype=np.float64)
    p1 = np.asarray(vertices[i1], dtype=np.float64)
    p2 = np.asarray(vertices[i2], dtype=np.float64)
    cross = np.cross(p1 - p0, p2 - p0)
    return float(np.dot(cross, cross)) > area_epsilon


def _clip_mesh_above_axis(
    mesh: o3d.geometry.TriangleMesh,
    axis_index: int,
    min_axis_value_m: float,
) -> tuple[o3d.geometry.TriangleMesh, int]:
    vertices_in = np.asarray(mesh.vertices, dtype=np.float64)
    triangles_in = np.asarray(mesh.triangles, dtype=np.int64)
    if vertices_in.shape[0] == 0 or triangles_in.shape[0] == 0:
        return _copy_mesh(mesh), 0

    cut_value = float(min_axis_value_m)
    epsilon = 1e-10
    vertices_out: list[np.ndarray] = [vertex.copy() for vertex in vertices_in]
    triangles_out: list[tuple[int, int, int]] = []
    edge_intersection_cache: dict[tuple[int, int], int] = {}

    def is_inside(vertex_index: int) -> bool:
        return float(vertices_out[vertex_index][axis_index]) >= (cut_value - epsilon)

    def get_intersection_index(vertex_index_a: int, vertex_index_b: int) -> int:
        point_a = np.asarray(vertices_out[vertex_index_a], dtype=np.float64)
        point_b = np.asarray(vertices_out[vertex_index_b], dtype=np.float64)
        delta_a = float(point_a[axis_index] - cut_value)
        delta_b = float(point_b[axis_index] - cut_value)

        if abs(delta_a) <= epsilon:
            return int(vertex_index_a)
        if abs(delta_b) <= epsilon:
            return int(vertex_index_b)

        edge_key = (
            (vertex_index_a, vertex_index_b)
            if vertex_index_a < vertex_index_b
            else (vertex_index_b, vertex_index_a)
        )
        cached = edge_intersection_cache.get(edge_key)
        if cached is not None:
            return cached

        denom = float(point_b[axis_index] - point_a[axis_index])
        if abs(denom) <= epsilon:
            return int(vertex_index_a)

        t = (cut_value - float(point_a[axis_index])) / denom
        t = float(np.clip(t, 0.0, 1.0))
        intersection = point_a + t * (point_b - point_a)
        intersection[axis_index] = cut_value

        new_index = len(vertices_out)
        vertices_out.append(intersection)
        edge_intersection_cache[edge_key] = new_index
        return new_index

    for triangle in triangles_in:
        triangle_indices = [int(triangle[0]), int(triangle[1]), int(triangle[2])]
        clipped_polygon: list[int] = []
        for start_index, end_index in zip(
            triangle_indices,
            triangle_indices[1:] + triangle_indices[:1],
        ):
            start_inside = is_inside(start_index)
            end_inside = is_inside(end_index)

            if start_inside and end_inside:
                clipped_polygon.append(end_index)
                continue

            if start_inside and not end_inside:
                clipped_polygon.append(get_intersection_index(start_index, end_index))
                continue

            if (not start_inside) and end_inside:
                clipped_polygon.append(get_intersection_index(start_index, end_index))
                clipped_polygon.append(end_index)

        clipped_polygon = _dedupe_polygon_indices(clipped_polygon)
        if len(clipped_polygon) < 3:
            continue

        for local_index in range(1, len(clipped_polygon) - 1):
            clipped_triangle = (
                clipped_polygon[0],
                clipped_polygon[local_index],
                clipped_polygon[local_index + 1],
            )
            if _is_valid_triangle(clipped_triangle, vertices_out):
                triangles_out.append(clipped_triangle)

    triangles_array = np.asarray(triangles_out, dtype=np.int64).reshape((-1, 3))
    mesh_out = o3d.geometry.TriangleMesh()
    mesh_out.vertices = o3d.utility.Vector3dVector(np.asarray(vertices_out, dtype=np.float64))
    mesh_out.triangles = o3d.utility.Vector3iVector(triangles_array)
    mesh_out = _basic_cleanup(mesh_out)
    return mesh_out, max(0, len(vertices_out) - int(vertices_in.shape[0]))


def _fit_table_frame(
    points_world: np.ndarray,
    ransac_thresh_mm: float,
    ransac_n: int,
    ransac_iters: int,
    ransac_seed: int | None,
    fit_low_band_mm: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    candidate_points = np.asarray(points_world, dtype=np.float64)
    if float(fit_low_band_mm) > 0.0:
        global_min_z = float(np.min(candidate_points[:, 2]))
        z_limit = global_min_z + _mm_to_m(fit_low_band_mm)
        seeded_points = candidate_points[candidate_points[:, 2] <= z_limit]
        if seeded_points.shape[0] >= max(50, int(ransac_n) * 10):
            candidate_points = seeded_points

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(candidate_points)
    if ransac_seed is not None:
        o3d.utility.random.seed(int(ransac_seed))
    plane_model, _ = pcd.segment_plane(
        distance_threshold=_mm_to_m(ransac_thresh_mm),
        ransac_n=int(ransac_n),
        num_iterations=int(ransac_iters),
    )
    plane = _normalize_plane(plane_model)

    signed = points_world @ plane[:3] + plane[3]
    if np.median(signed) < 0.0:
        plane = -plane

    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    R = _rotation_matrix_from_a_to_b(plane[:3], up)
    t = np.array([0.0, 0.0, plane[3]], dtype=np.float64)

    T_table_from_world = np.eye(4, dtype=np.float64)
    T_table_from_world[:3, :3] = R
    T_table_from_world[:3, 3] = t

    T_world_from_table = np.eye(4, dtype=np.float64)
    T_world_from_table[:3, :3] = R.T
    T_world_from_table[:3, 3] = -R.T @ t

    points_table = _transform_points(points_world, T_table_from_world)
    return points_table, T_table_from_world, T_world_from_table, plane


def _trim_mesh_above_table(
    mesh_world: o3d.geometry.TriangleMesh,
    ransac_thresh_mm: float,
    ransac_n: int,
    ransac_iters: int,
    ransac_seed: int | None,
    fit_low_band_mm: float,
    z_min_above_plane_mm: float,
) -> TableTrimResult:
    vertices_world = np.asarray(mesh_world.vertices, dtype=np.float64)
    if vertices_world.shape[0] == 0:
        raise RuntimeError("Cannot fit table plane on an empty mesh.")

    _, T_table_from_world, T_world_from_table, plane = _fit_table_frame(
        points_world=vertices_world,
        ransac_thresh_mm=ransac_thresh_mm,
        ransac_n=ransac_n,
        ransac_iters=ransac_iters,
        ransac_seed=ransac_seed,
        fit_low_band_mm=fit_low_band_mm,
    )

    mesh_table = _copy_mesh(mesh_world)
    mesh_table.transform(T_table_from_world)

    source_vertices = int(len(mesh_table.vertices))
    source_triangles = int(len(mesh_table.triangles))
    mesh_table_trimmed, cut_vertices_added = _clip_mesh_above_axis(
        mesh=mesh_table,
        axis_index=2,
        min_axis_value_m=_mm_to_m(z_min_above_plane_mm),
    )
    if len(mesh_table_trimmed.vertices) == 0 or len(mesh_table_trimmed.triangles) == 0:
        raise RuntimeError(
            "No mesh remains above the fitted table threshold. Lower z_min_above_plane_mm."
        )

    return TableTrimResult(
        mesh_table_trimmed=mesh_table_trimmed,
        T_table_from_world=T_table_from_world,
        T_world_from_table=T_world_from_table,
        plane_model_world=plane,
        source_vertices=source_vertices,
        source_triangles=source_triangles,
        fit_seed_vertices=int(
            np.count_nonzero(
                vertices_world[:, 2]
                <= (float(np.min(vertices_world[:, 2])) + _mm_to_m(fit_low_band_mm))
            )
        )
        if float(fit_low_band_mm) > 0.0
        else source_vertices,
        trimmed_vertices=int(len(mesh_table_trimmed.vertices)),
        trimmed_triangles=int(len(mesh_table_trimmed.triangles)),
        cut_vertices_added=cut_vertices_added,
    )


def _keep_only_largest_component(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    if len(mesh.triangles) == 0:
        return _copy_mesh(mesh)

    triangle_clusters, cluster_triangle_counts, _ = mesh.cluster_connected_triangles()
    cluster_ids = np.asarray(triangle_clusters, dtype=np.int64)
    if cluster_ids.size == 0:
        return _copy_mesh(mesh)

    keep_cluster = int(np.argmax(np.asarray(cluster_triangle_counts, dtype=np.int64)))
    remove_mask = cluster_ids != keep_cluster

    mesh_out = _copy_mesh(mesh)
    mesh_out.remove_triangles_by_mask(remove_mask)
    mesh_out.remove_unreferenced_vertices()
    mesh_out.compute_vertex_normals()
    return mesh_out


def _keep_significant_components(
    mesh: o3d.geometry.TriangleMesh,
    min_triangle_fraction: float,
    min_triangles: int,
) -> o3d.geometry.TriangleMesh:
    if len(mesh.triangles) == 0:
        return _copy_mesh(mesh)

    triangle_clusters, cluster_triangle_counts, _ = mesh.cluster_connected_triangles()
    cluster_ids = np.asarray(triangle_clusters, dtype=np.int64)
    cluster_triangle_counts = np.asarray(cluster_triangle_counts, dtype=np.int64)
    if cluster_ids.size == 0 or cluster_triangle_counts.size == 0:
        return _copy_mesh(mesh)

    largest_count = int(np.max(cluster_triangle_counts))
    threshold = max(int(min_triangles), int(np.ceil(float(min_triangle_fraction) * largest_count)))
    keep_cluster_ids = np.flatnonzero(cluster_triangle_counts >= threshold)
    if keep_cluster_ids.size == 0:
        keep_cluster_ids = np.asarray([int(np.argmax(cluster_triangle_counts))], dtype=np.int64)

    remove_mask = ~np.isin(cluster_ids, keep_cluster_ids)
    mesh_out = _copy_mesh(mesh)
    mesh_out.remove_triangles_by_mask(remove_mask)
    mesh_out.remove_unreferenced_vertices()
    mesh_out.compute_vertex_normals()
    return mesh_out


def _select_bead_component(
    mesh: o3d.geometry.TriangleMesh,
    axis_index: int,
) -> o3d.geometry.TriangleMesh:
    if len(mesh.triangles) == 0:
        return _copy_mesh(mesh)

    triangle_clusters, cluster_triangle_counts, _ = mesh.cluster_connected_triangles()
    cluster_ids = np.asarray(triangle_clusters, dtype=np.int64)
    cluster_triangle_counts = np.asarray(cluster_triangle_counts, dtype=np.int64)
    if cluster_ids.size == 0 or cluster_triangle_counts.size == 0:
        return _copy_mesh(mesh)

    best_cluster_id = None
    best_key = None
    for cluster_id in np.argsort(cluster_triangle_counts)[::-1].tolist():
        component_mesh = _copy_mesh(mesh)
        component_mesh.remove_triangles_by_mask(cluster_ids != int(cluster_id))
        component_mesh.remove_unreferenced_vertices()
        if len(component_mesh.triangles) == 0:
            continue
        aabb = component_mesh.get_axis_aligned_bounding_box()
        bbox_min = np.asarray(aabb.min_bound, dtype=np.float64)
        bbox_max = np.asarray(aabb.max_bound, dtype=np.float64)
        extent = bbox_max - bbox_min
        height_extent = float(extent[int(axis_index)])
        lateral_extent = float(np.max(np.delete(extent, int(axis_index)))) if extent.size == 3 else 0.0
        slenderness = height_extent / max(lateral_extent, 1e-12)
        candidate_key = (
            height_extent,
            slenderness,
            int(len(component_mesh.triangles)),
        )
        if best_key is None or candidate_key > best_key:
            best_key = candidate_key
            best_cluster_id = int(cluster_id)

    if best_cluster_id is None:
        return _copy_mesh(mesh)

    mesh_out = _copy_mesh(mesh)
    mesh_out.remove_triangles_by_mask(cluster_ids != best_cluster_id)
    mesh_out.remove_unreferenced_vertices()
    mesh_out.compute_vertex_normals()
    return mesh_out


def _select_repair_target_mesh(
    mesh: o3d.geometry.TriangleMesh,
    target_mode: str,
    axis_index: int,
) -> o3d.geometry.TriangleMesh:
    if target_mode == "all_posttrim":
        return _copy_mesh(mesh)
    if target_mode == "largest_component_only":
        return _keep_only_largest_component(mesh)
    if target_mode == "bead_component":
        return _select_bead_component(mesh, axis_index=axis_index)
    raise ValueError(
        f"Unsupported repair_target: {target_mode}. Expected all_posttrim, largest_component_only, or bead_component."
    )


def _build_boundary_edges(mesh: o3d.geometry.TriangleMesh) -> list[tuple[int, int]]:
    triangles = np.asarray(mesh.triangles, dtype=np.int64)
    edge_counts: Counter[tuple[int, int]] = Counter()
    for tri in triangles:
        i0, i1, i2 = map(int, tri.tolist())
        for u, v in ((i0, i1), (i1, i2), (i2, i0)):
            key = (u, v) if u < v else (v, u)
            edge_counts[key] += 1
    return [edge for edge, count in edge_counts.items() if count == 1]


def _extract_boundary_loops(mesh: o3d.geometry.TriangleMesh) -> list[list[int]]:
    boundary_edges = _build_boundary_edges(mesh)
    if not boundary_edges:
        return []

    adjacency: dict[int, list[int]] = defaultdict(list)
    for u, v in boundary_edges:
        adjacency[u].append(v)
        adjacency[v].append(u)

    visited_edges: set[tuple[int, int]] = set()
    loops: list[list[int]] = []

    for start_u, start_v in boundary_edges:
        start_edge = (start_u, start_v) if start_u < start_v else (start_v, start_u)
        if start_edge in visited_edges:
            continue

        loop = [start_u, start_v]
        visited_edges.add(start_edge)
        prev_vertex = start_u
        current_vertex = start_v

        while True:
            neighbors = adjacency[current_vertex]
            next_candidates = [n for n in neighbors if n != prev_vertex]
            if not next_candidates:
                break

            if len(loop) >= 3 and loop[0] in next_candidates:
                closing_edge = (
                    (current_vertex, loop[0])
                    if current_vertex < loop[0]
                    else (loop[0], current_vertex)
                )
                visited_edges.add(closing_edge)
                loop.append(loop[0])
                break

            next_vertex = None
            for candidate in next_candidates:
                edge_key = (
                    (current_vertex, candidate)
                    if current_vertex < candidate
                    else (candidate, current_vertex)
                )
                if edge_key not in visited_edges:
                    next_vertex = candidate
                    visited_edges.add(edge_key)
                    break

            if next_vertex is None:
                break

            loop.append(next_vertex)
            prev_vertex, current_vertex = current_vertex, next_vertex

        if len(loop) >= 4 and loop[0] == loop[-1]:
            loops.append(loop[:-1])

    return loops


def _polygon_signed_area_2d(points_2d: np.ndarray) -> float:
    x = points_2d[:, 0]
    y = points_2d[:, 1]
    return 0.5 * float(np.sum((x * np.roll(y, -1)) - (np.roll(x, -1) * y)))


def _point_in_triangle_2d(point: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
    v0 = c - a
    v1 = b - a
    v2 = point - a

    dot00 = float(np.dot(v0, v0))
    dot01 = float(np.dot(v0, v1))
    dot02 = float(np.dot(v0, v2))
    dot11 = float(np.dot(v1, v1))
    dot12 = float(np.dot(v1, v2))

    denom = (dot00 * dot11) - (dot01 * dot01)
    if abs(denom) <= 1e-18:
        return False

    inv_denom = 1.0 / denom
    u = ((dot11 * dot02) - (dot01 * dot12)) * inv_denom
    v = ((dot00 * dot12) - (dot01 * dot02)) * inv_denom
    return (u >= 0.0) and (v >= 0.0) and ((u + v) <= 1.0)


def _ear_clip_polygon(points_2d: np.ndarray) -> list[tuple[int, int, int]]:
    if points_2d.shape[0] < 3:
        return []

    signed_area = _polygon_signed_area_2d(points_2d)
    if abs(signed_area) <= 1e-16:
        raise RuntimeError("Boundary loop projection is degenerate; polygon area is too small.")

    ccw = signed_area > 0.0
    active = list(range(points_2d.shape[0]))
    triangles: list[tuple[int, int, int]] = []
    guard = 0
    max_guard = max(32, points_2d.shape[0] * points_2d.shape[0])

    while len(active) > 3 and guard < max_guard:
        ear_found = False
        for local_index, vertex_index in enumerate(active):
            prev_index = active[(local_index - 1) % len(active)]
            next_index = active[(local_index + 1) % len(active)]

            a = points_2d[prev_index]
            b = points_2d[vertex_index]
            c = points_2d[next_index]

            cross_z = ((b[0] - a[0]) * (c[1] - a[1])) - ((b[1] - a[1]) * (c[0] - a[0]))
            if ccw and cross_z <= 1e-12:
                continue
            if not ccw and cross_z >= -1e-12:
                continue

            contains_other_point = False
            for candidate_index in active:
                if candidate_index in (prev_index, vertex_index, next_index):
                    continue
                if _point_in_triangle_2d(points_2d[candidate_index], a, b, c):
                    contains_other_point = True
                    break
            if contains_other_point:
                continue

            if ccw:
                triangles.append((prev_index, vertex_index, next_index))
            else:
                triangles.append((prev_index, next_index, vertex_index))
            del active[local_index]
            ear_found = True
            break

        if not ear_found:
            raise RuntimeError("Ear clipping failed; boundary loop is likely self-intersecting or non-simple.")
        guard += 1

    if len(active) == 3:
        if ccw:
            triangles.append((active[0], active[1], active[2]))
        else:
            triangles.append((active[0], active[2], active[1]))

    return triangles


def _fit_plane_projection(points_3d: np.ndarray) -> tuple[np.ndarray, float]:
    centroid = np.mean(points_3d, axis=0)
    centered = points_3d - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    basis_u = vh[0]
    basis_v = vh[1]
    normal = vh[2]
    projected_2d = np.column_stack((centered @ basis_u, centered @ basis_v))
    planarity_rms = float(np.sqrt(np.mean(np.square(centered @ normal))))
    return projected_2d, planarity_rms


def _triangulate_loop_in_place(
    loop_vertex_indices: list[int],
    vertices: np.ndarray,
    max_planarity_m: float,
) -> list[tuple[int, int, int]]:
    loop_points = vertices[np.asarray(loop_vertex_indices, dtype=np.int64)]
    projected_2d, planarity_rms = _fit_plane_projection(loop_points)
    if planarity_rms > max_planarity_m:
        raise RuntimeError(
            f"Boundary loop is too non-planar to fill directly. "
            f"planarity_rms_m={planarity_rms:.6f} > max_planarity_m={max_planarity_m:.6f}"
        )

    local_triangles = _ear_clip_polygon(projected_2d)
    return [
        (
            loop_vertex_indices[i0],
            loop_vertex_indices[i1],
            loop_vertex_indices[i2],
        )
        for i0, i1, i2 in local_triangles
    ]


def _triangulate_bottom_cap(
    loop_vertex_indices: list[int],
    vertices: np.ndarray,
    axis_index: int,
    cap_axis_value: float,
) -> tuple[np.ndarray, list[tuple[int, int, int]], list[tuple[int, int, int, int]]]:
    loop_points = np.asarray(vertices[np.asarray(loop_vertex_indices, dtype=np.int64)], dtype=np.float64)
    cap_ring = np.asarray(loop_points, dtype=np.float64)
    cap_ring[:, axis_index] = cap_axis_value
    cap_center = np.mean(cap_ring, axis=0, keepdims=True)
    cap_vertices = np.vstack([cap_ring, cap_center])
    center_index = cap_ring.shape[0]
    local_cap_triangles = [
        (center_index, (local_index + 1) % len(loop_vertex_indices), local_index)
        for local_index in range(len(loop_vertex_indices))
    ]

    side_wall_quads: list[tuple[int, int, int, int]] = []
    for local_index in range(len(loop_vertex_indices)):
        next_local_index = (local_index + 1) % len(loop_vertex_indices)
        orig_a = loop_vertex_indices[local_index]
        orig_b = loop_vertex_indices[next_local_index]
        cap_a = local_index
        cap_b = next_local_index
        side_wall_quads.append((orig_a, orig_b, cap_b, cap_a))

    return cap_vertices, local_cap_triangles, side_wall_quads


def _triangulate_bottom_cap_in_place(
    loop_vertex_indices: list[int],
    vertices: np.ndarray,
    axis_index: int,
    cap_axis_value: float,
    max_planarity_m: float,
) -> list[tuple[int, int, int]]:
    loop_points = np.asarray(vertices[np.asarray(loop_vertex_indices, dtype=np.int64)], dtype=np.float64)
    loop_axis_values = loop_points[:, axis_index]
    if np.max(np.abs(loop_axis_values - float(cap_axis_value))) > max(1e-8, max_planarity_m):
        raise RuntimeError(
            "Bottom loop is not already on the requested cap plane for planar_in_place mode. "
            "Use bottom_cap_plane_offset_mm=0 or switch to bottom_cap_mode=extruded."
        )
    return _triangulate_loop_in_place(
        loop_vertex_indices=loop_vertex_indices,
        vertices=vertices,
        max_planarity_m=max_planarity_m,
    )


def _triangle_circumradius_2d(points_2d: np.ndarray) -> float:
    a = float(np.linalg.norm(points_2d[1] - points_2d[0]))
    b = float(np.linalg.norm(points_2d[2] - points_2d[1]))
    c = float(np.linalg.norm(points_2d[0] - points_2d[2]))
    twice_area = abs(
        ((points_2d[1, 0] - points_2d[0, 0]) * (points_2d[2, 1] - points_2d[0, 1]))
        - ((points_2d[1, 1] - points_2d[0, 1]) * (points_2d[2, 0] - points_2d[0, 0]))
    )
    if twice_area <= 1e-15:
        return float("inf")
    return (a * b * c) / (2.0 * twice_area)


def _largest_triangle_component(triangles_local: np.ndarray) -> np.ndarray:
    if triangles_local.shape[0] <= 1:
        return triangles_local

    edge_to_triangles: dict[tuple[int, int], list[int]] = defaultdict(list)
    for tri_index, tri in enumerate(triangles_local):
        for u, v in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            edge_key = (int(u), int(v)) if int(u) < int(v) else (int(v), int(u))
            edge_to_triangles[edge_key].append(tri_index)

    adjacency: dict[int, set[int]] = defaultdict(set)
    for shared_triangles in edge_to_triangles.values():
        if len(shared_triangles) < 2:
            continue
        for tri_a in shared_triangles:
            for tri_b in shared_triangles:
                if tri_a != tri_b:
                    adjacency[int(tri_a)].add(int(tri_b))

    visited: set[int] = set()
    largest_component: list[int] = []
    for start_index in range(triangles_local.shape[0]):
        if start_index in visited:
            continue
        stack = [start_index]
        component: list[int] = []
        visited.add(start_index)
        while stack:
            current_index = stack.pop()
            component.append(current_index)
            for neighbor_index in adjacency.get(current_index, set()):
                if neighbor_index not in visited:
                    visited.add(neighbor_index)
                    stack.append(neighbor_index)
        if len(component) > len(largest_component):
            largest_component = component

    return triangles_local[np.asarray(largest_component, dtype=np.int64)]


def _rasterize_loop_region(
    loop_points_2d: np.ndarray,
    resolution_m: float,
    margin_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if binary_dilation is None or binary_fill_holes is None:
        raise RuntimeError(
            "scipy.ndimage is required for bottom_cap_mode=local_patch or local patch fallback."
        )

    min_xy = np.min(loop_points_2d, axis=0) - float(margin_m)
    max_xy = np.max(loop_points_2d, axis=0) + float(margin_m)
    spans = np.maximum(max_xy - min_xy, float(resolution_m))
    width = int(np.ceil(spans[0] / float(resolution_m))) + 3
    height = int(np.ceil(spans[1] / float(resolution_m))) + 3
    edge_mask = np.zeros((height, width), dtype=bool)

    grid_points = (loop_points_2d - min_xy[None, :]) / float(resolution_m)
    for start_point, end_point in zip(grid_points, np.roll(grid_points, -1, axis=0)):
        segment = end_point - start_point
        steps = max(2, int(np.ceil(np.linalg.norm(segment))) * 3)
        sample_points = start_point[None, :] + np.linspace(0.0, 1.0, steps)[:, None] * segment[None, :]
        sample_cols = np.clip(np.round(sample_points[:, 0]).astype(np.int64), 0, width - 1)
        sample_rows = np.clip(np.round(sample_points[:, 1]).astype(np.int64), 0, height - 1)
        edge_mask[sample_rows, sample_cols] = True

    edge_mask = binary_dilation(edge_mask, structure=np.ones((3, 3), dtype=bool), iterations=1)
    filled_mask = binary_fill_holes(edge_mask)
    interior_mask = filled_mask & (~edge_mask)
    if int(np.count_nonzero(interior_mask)) == 0:
        raise RuntimeError("Rasterized bottom patch produced no interior region.")
    return edge_mask, interior_mask, min_xy


def _ensure_patch_boundary_edges(
    triangles_local: list[tuple[int, int, int]],
    all_points_2d: np.ndarray,
    boundary_count: int,
    preferred_support_indices: np.ndarray,
) -> list[tuple[int, int, int]]:
    edge_counts: Counter[tuple[int, int]] = Counter()
    for tri in triangles_local:
        for u, v in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            edge_key = (u, v) if u < v else (v, u)
            edge_counts[edge_key] += 1

    if preferred_support_indices.size == 0:
        return triangles_local

    for boundary_index in range(boundary_count):
        next_boundary_index = (boundary_index + 1) % boundary_count
        edge_key = (
            (boundary_index, next_boundary_index)
            if boundary_index < next_boundary_index
            else (next_boundary_index, boundary_index)
        )
        if edge_counts.get(edge_key, 0) > 0:
            continue

        midpoint = 0.5 * (all_points_2d[boundary_index] + all_points_2d[next_boundary_index])
        candidate_points = all_points_2d[preferred_support_indices]
        order = np.argsort(np.linalg.norm(candidate_points - midpoint[None, :], axis=1))
        support_index = None
        for order_index in order.tolist():
            candidate_index = int(preferred_support_indices[order_index])
            area = abs(
                ((all_points_2d[next_boundary_index, 0] - all_points_2d[boundary_index, 0])
                 * (all_points_2d[candidate_index, 1] - all_points_2d[boundary_index, 1]))
                - ((all_points_2d[next_boundary_index, 1] - all_points_2d[boundary_index, 1])
                   * (all_points_2d[candidate_index, 0] - all_points_2d[boundary_index, 0]))
            )
            if area > 1e-12:
                support_index = candidate_index
                break
        if support_index is None:
            continue
        triangles_local.append((boundary_index, next_boundary_index, support_index))
        edge_counts[edge_key] += 1

    return triangles_local


def _triangulate_bottom_cap_local_patch(
    loop_vertex_indices: list[int],
    vertices: np.ndarray,
    axis_index: int,
    cap_axis_value: float,
    resolution_m: float,
    margin_m: float,
) -> tuple[np.ndarray, list[tuple[int, int, int]]]:
    if Delaunay is None:
        raise RuntimeError(
            "scipy.spatial is required for bottom_cap_mode=local_patch or auto local patch fallback."
        )

    loop_points = np.asarray(vertices[np.asarray(loop_vertex_indices, dtype=np.int64)], dtype=np.float64)
    plane_axes = [axis for axis in range(3) if axis != int(axis_index)]
    boundary_points_2d = np.asarray(loop_points[:, plane_axes], dtype=np.float64)
    edge_mask, interior_mask, min_xy = _rasterize_loop_region(
        loop_points_2d=boundary_points_2d,
        resolution_m=resolution_m,
        margin_m=margin_m,
    )

    interior_rows, interior_cols = np.nonzero(interior_mask)
    interior_points_2d = np.column_stack(
        (
            min_xy[0] + float(resolution_m) * interior_cols.astype(np.float64),
            min_xy[1] + float(resolution_m) * interior_rows.astype(np.float64),
        )
    )
    if interior_points_2d.shape[0] == 0:
        raise RuntimeError("Rasterized local bottom patch has no interior samples.")

    all_points_2d = np.vstack([boundary_points_2d, interior_points_2d])
    try:
        delaunay = Delaunay(all_points_2d)
    except QhullError as exc:
        raise RuntimeError(f"Raster local bottom patch Delaunay failed: {exc}") from exc

    kept_triangles_local: list[tuple[int, int, int]] = []
    for simplex in np.asarray(delaunay.simplices, dtype=np.int64):
        simplex_points = all_points_2d[np.asarray(simplex, dtype=np.int64)]
        centroid = np.mean(simplex_points, axis=0)
        col = int(np.clip(np.round((centroid[0] - min_xy[0]) / float(resolution_m)), 0, interior_mask.shape[1] - 1))
        row = int(np.clip(np.round((centroid[1] - min_xy[1]) / float(resolution_m)), 0, interior_mask.shape[0] - 1))
        if not interior_mask[row, col] and not edge_mask[row, col]:
            continue
        kept_triangles_local.append((int(simplex[0]), int(simplex[1]), int(simplex[2])))

    if not kept_triangles_local:
        raise RuntimeError("Raster local bottom patch kept no interior triangles.")

    kept_triangles_local_np = _largest_triangle_component(np.asarray(kept_triangles_local, dtype=np.int64))
    kept_triangles_local = [tuple(map(int, tri.tolist())) for tri in kept_triangles_local_np]
    preferred_support_indices = np.arange(boundary_points_2d.shape[0], all_points_2d.shape[0], dtype=np.int64)
    kept_triangles_local = _ensure_patch_boundary_edges(
        triangles_local=kept_triangles_local,
        all_points_2d=all_points_2d,
        boundary_count=boundary_points_2d.shape[0],
        preferred_support_indices=preferred_support_indices,
    )

    interior_vertices_3d = np.zeros((interior_points_2d.shape[0], 3), dtype=np.float64)
    interior_vertices_3d[:, plane_axes[0]] = interior_points_2d[:, 0]
    interior_vertices_3d[:, plane_axes[1]] = interior_points_2d[:, 1]
    interior_vertices_3d[:, axis_index] = float(cap_axis_value)
    return interior_vertices_3d, kept_triangles_local


def _apply_bottom_cap_for_loop(
    loop_indices: list[int],
    vertices: np.ndarray,
    added_vertices: list[np.ndarray],
    added_triangles: list[tuple[int, int, int]],
    bottom_cap_mode: str,
    axis_index: int,
    cap_axis_value: float,
    max_planarity_m: float,
    local_patch_resolution_m: float,
    local_patch_margin_m: float,
) -> str:
    original_mode = str(bottom_cap_mode).strip().lower()
    effective_mode = original_mode

    if effective_mode in ("auto", "planar_in_place"):
        try:
            cap_triangles = _triangulate_bottom_cap_in_place(
                loop_vertex_indices=loop_indices,
                vertices=vertices,
                axis_index=axis_index,
                cap_axis_value=cap_axis_value,
                max_planarity_m=max_planarity_m,
            )
            added_triangles.extend(cap_triangles)
            return "planar_in_place"
        except RuntimeError as exc:
            if original_mode == "planar_in_place":
                raise
            print(
                f"Bottom loop with {len(loop_indices)} vertices fell back from planar_in_place: {exc}"
            )
            effective_mode = "snap_to_plane"

    if effective_mode in ("auto", "snap_to_plane"):
        loop_vertex_indices_np = np.asarray(loop_indices, dtype=np.int64)
        original_axis_values = np.asarray(vertices[loop_vertex_indices_np, axis_index], dtype=np.float64).copy()
        try:
            vertices[loop_vertex_indices_np, axis_index] = cap_axis_value
            cap_triangles = _triangulate_loop_in_place(
                loop_vertex_indices=loop_indices,
                vertices=vertices,
                max_planarity_m=max_planarity_m,
            )
            added_triangles.extend(cap_triangles)
            return "snap_to_plane"
        except RuntimeError as exc:
            vertices[loop_vertex_indices_np, axis_index] = original_axis_values
            if original_mode == "snap_to_plane":
                raise RuntimeError(
                    f"Bottom loop could not be capped after snapping to the bottom plane: {exc}"
                ) from exc
            print(
                f"Bottom loop with {len(loop_indices)} vertices fell back from snap_to_plane: {exc}"
            )
            effective_mode = "local_patch"

    if effective_mode in ("auto", "local_patch"):
        patch_vertices, patch_triangles_local = _triangulate_bottom_cap_local_patch(
            loop_vertex_indices=loop_indices,
            vertices=vertices,
            axis_index=axis_index,
            cap_axis_value=cap_axis_value,
            resolution_m=local_patch_resolution_m,
            margin_m=local_patch_margin_m,
        )
        patch_vertex_start = vertices.shape[0] + sum(block.shape[0] for block in added_vertices)
        boundary_count = len(loop_indices)
        added_vertices.append(patch_vertices)
        for tri_local in patch_triangles_local:
            tri_global: list[int] = []
            for local_index in tri_local:
                if int(local_index) < boundary_count:
                    tri_global.append(int(loop_indices[int(local_index)]))
                else:
                    tri_global.append(patch_vertex_start + int(local_index) - boundary_count)
            tri_points = np.zeros((3, 3), dtype=np.float64)
            for point_index, global_index in enumerate(tri_global):
                if global_index < vertices.shape[0]:
                    tri_points[point_index] = vertices[global_index]
                else:
                    tri_points[point_index] = patch_vertices[global_index - patch_vertex_start]
            normal = np.cross(tri_points[1] - tri_points[0], tri_points[2] - tri_points[0])
            if abs(float(normal[axis_index])) <= 1e-15:
                continue
            if float(normal[axis_index]) > 0.0:
                tri_global[1], tri_global[2] = tri_global[2], tri_global[1]
            added_triangles.append((tri_global[0], tri_global[1], tri_global[2]))
        return "local_patch"

    if effective_mode == "extruded":
        cap_vertices, local_cap_triangles, side_wall_quads = _triangulate_bottom_cap(
            loop_vertex_indices=loop_indices,
            vertices=vertices,
            axis_index=axis_index,
            cap_axis_value=cap_axis_value,
        )
        cap_vertex_start = vertices.shape[0] + sum(block.shape[0] for block in added_vertices)
        added_vertices.append(cap_vertices)
        added_triangles.extend(
            [
                (
                    cap_vertex_start + i0,
                    cap_vertex_start + i2,
                    cap_vertex_start + i1,
                )
                for i0, i1, i2 in local_cap_triangles
            ]
        )
        for orig_a, orig_b, local_cap_b, local_cap_a in side_wall_quads:
            cap_a = cap_vertex_start + local_cap_a
            cap_b = cap_vertex_start + local_cap_b
            added_triangles.append((orig_a, orig_b, cap_b))
            added_triangles.append((orig_a, cap_b, cap_a))
        return "extruded"

    raise RuntimeError(
        f"Unsupported bottom_cap_mode: {bottom_cap_mode}. "
        "Expected auto, planar_in_place, snap_to_plane, local_patch, or extruded."
    )


def _is_bottom_like_loop(
    loop_indices: list[int],
    vertices: np.ndarray,
    bottom_axis_index: int,
    global_min_axis: float,
    bottom_loop_height_tol_m: float,
) -> bool:
    if len(loop_indices) < 3:
        return False
    loop_points = vertices[np.asarray(loop_indices, dtype=np.int64)]
    return float(np.mean(loop_points[:, bottom_axis_index])) <= (global_min_axis + bottom_loop_height_tol_m)


def _build_mesh_from_loop_repairs(
    vertices: np.ndarray,
    triangle_list: list[tuple[int, int, int]],
    added_vertices: list[np.ndarray],
    added_triangles: list[tuple[int, int, int]],
) -> o3d.geometry.TriangleMesh:
    if added_vertices:
        vertices_out = np.vstack([vertices, *added_vertices])
    else:
        vertices_out = vertices

    triangles_out = np.asarray([*triangle_list, *added_triangles], dtype=np.int64).reshape((-1, 3))
    mesh_out = o3d.geometry.TriangleMesh()
    mesh_out.vertices = o3d.utility.Vector3dVector(vertices_out)
    mesh_out.triangles = o3d.utility.Vector3iVector(triangles_out)
    return _basic_cleanup(mesh_out)


def _apply_loop_repair_pass(
    mesh: o3d.geometry.TriangleMesh,
    repair_bottom_loops: bool,
    fill_holes_enable: bool,
    fill_holes_max_boundary_edges: int,
    fill_holes_max_planarity_m: float,
    bottom_cap_enable: bool,
    bottom_cap_mode: str,
    bottom_axis_index: int,
    bottom_loop_height_tol_m: float,
    bottom_cap_plane_offset_m: float,
    bottom_local_patch_resolution_m: float,
    bottom_local_patch_margin_m: float,
    emit_debug: bool = True,
) -> tuple[o3d.geometry.TriangleMesh, LoopRepairResult]:
    vertices = np.asarray(mesh.vertices, dtype=np.float64).copy()
    triangles = np.asarray(mesh.triangles, dtype=np.int64)
    triangle_list = [tuple(map(int, tri.tolist())) for tri in triangles]
    boundary_loops = _extract_boundary_loops(mesh)
    if not boundary_loops:
        return _copy_mesh(mesh), LoopRepairResult([], np.zeros((0, 3)), 0, 0, 0, 0, 0, "custom", "")

    global_min_axis = float(np.min(vertices[:, bottom_axis_index]))
    added_vertices: list[np.ndarray] = []
    added_triangles: list[tuple[int, int, int]] = []
    repaired_loop_count = 0
    side_fill_count = 0
    bottom_cap_count = 0
    local_patch_fallback_count = 0
    skipped_loop_count = 0

    for loop_indices in boundary_loops:
        if len(loop_indices) < 3:
            skipped_loop_count += 1
            continue

        is_bottom_loop = _is_bottom_like_loop(
            loop_indices=loop_indices,
            vertices=vertices,
            bottom_axis_index=bottom_axis_index,
            global_min_axis=global_min_axis,
            bottom_loop_height_tol_m=bottom_loop_height_tol_m,
        )
        if is_bottom_loop != repair_bottom_loops:
            continue

        try:
            if repair_bottom_loops:
                if not bottom_cap_enable:
                    skipped_loop_count += 1
                    continue

                cap_axis_value = global_min_axis - bottom_cap_plane_offset_m
                used_mode = _apply_bottom_cap_for_loop(
                    loop_indices=loop_indices,
                    vertices=vertices,
                    added_vertices=added_vertices,
                    added_triangles=added_triangles,
                    bottom_cap_mode=bottom_cap_mode,
                    axis_index=bottom_axis_index,
                    cap_axis_value=cap_axis_value,
                    max_planarity_m=fill_holes_max_planarity_m,
                    local_patch_resolution_m=bottom_local_patch_resolution_m,
                    local_patch_margin_m=bottom_local_patch_margin_m,
                )
                repaired_loop_count += 1
                bottom_cap_count += 1
                if used_mode == "local_patch":
                    local_patch_fallback_count += 1
                continue

            if not fill_holes_enable:
                skipped_loop_count += 1
                continue
            if len(loop_indices) > int(fill_holes_max_boundary_edges):
                skipped_loop_count += 1
                continue

            loop_triangles = _triangulate_loop_in_place(
                loop_vertex_indices=loop_indices,
                vertices=vertices,
                max_planarity_m=fill_holes_max_planarity_m,
            )
            added_triangles.extend(loop_triangles)
            repaired_loop_count += 1
            side_fill_count += 1
        except RuntimeError as exc:
            loop_kind = "bottom" if repair_bottom_loops else "side"
            if emit_debug:
                print(f"Skipping {loop_kind} boundary loop with {len(loop_indices)} vertices: {exc}")
            skipped_loop_count += 1

    if not added_vertices and not added_triangles:
        return _copy_mesh(mesh), LoopRepairResult(
            [], np.zeros((0, 3)), 0, 0, 0, 0, skipped_loop_count, "custom", ""
        )

    mesh_out = _build_mesh_from_loop_repairs(
        vertices=vertices,
        triangle_list=triangle_list,
        added_vertices=added_vertices,
        added_triangles=added_triangles,
    )
    return mesh_out, LoopRepairResult(
        triangles_added=added_triangles,
        vertices_added=np.vstack(added_vertices) if added_vertices else np.zeros((0, 3), dtype=np.float64),
        repaired_loop_count=repaired_loop_count,
        side_fill_count=side_fill_count,
        bottom_cap_count=bottom_cap_count,
        local_patch_fallback_count=local_patch_fallback_count,
        skipped_loop_count=skipped_loop_count,
        backend_used="custom",
        order_used="",
    )


def _merge_loop_repair_results(
    first_result: LoopRepairResult,
    second_result: LoopRepairResult,
    order_used: str,
) -> LoopRepairResult:
    return LoopRepairResult(
        triangles_added=[*first_result.triangles_added, *second_result.triangles_added],
        vertices_added=np.vstack(
            [block for block in (first_result.vertices_added, second_result.vertices_added) if block.size > 0]
        )
        if first_result.vertices_added.size > 0 or second_result.vertices_added.size > 0
        else np.zeros((0, 3), dtype=np.float64),
        repaired_loop_count=first_result.repaired_loop_count + second_result.repaired_loop_count,
        side_fill_count=first_result.side_fill_count + second_result.side_fill_count,
        bottom_cap_count=first_result.bottom_cap_count + second_result.bottom_cap_count,
        local_patch_fallback_count=(
            first_result.local_patch_fallback_count + second_result.local_patch_fallback_count
        ),
        skipped_loop_count=first_result.skipped_loop_count + second_result.skipped_loop_count,
        backend_used=first_result.backend_used,
        order_used=order_used,
    )


def _combine_repair_results(
    first_result: LoopRepairResult,
    second_result: LoopRepairResult,
    backend_used: str,
    order_used: str,
) -> LoopRepairResult:
    vertex_blocks = [
        block
        for block in (first_result.vertices_added, second_result.vertices_added)
        if block.size > 0
    ]
    return LoopRepairResult(
        triangles_added=[*first_result.triangles_added, *second_result.triangles_added],
        vertices_added=(
            np.vstack(vertex_blocks)
            if vertex_blocks
            else np.zeros((0, 3), dtype=np.float64)
        ),
        repaired_loop_count=first_result.repaired_loop_count + second_result.repaired_loop_count,
        side_fill_count=first_result.side_fill_count + second_result.side_fill_count,
        bottom_cap_count=first_result.bottom_cap_count + second_result.bottom_cap_count,
        local_patch_fallback_count=(
            first_result.local_patch_fallback_count + second_result.local_patch_fallback_count
        ),
        skipped_loop_count=first_result.skipped_loop_count + second_result.skipped_loop_count,
        backend_used=backend_used,
        order_used=order_used,
    )


def _run_loop_repair_order(
    mesh: o3d.geometry.TriangleMesh,
    order_used: str,
    fill_holes_enable: bool,
    fill_holes_max_boundary_edges: int,
    fill_holes_max_planarity_m: float,
    bottom_cap_enable: bool,
    bottom_cap_mode: str,
    bottom_axis_index: int,
    bottom_loop_height_tol_m: float,
    bottom_cap_plane_offset_m: float,
    bottom_local_patch_resolution_m: float,
    bottom_local_patch_margin_m: float,
    emit_debug: bool,
) -> tuple[o3d.geometry.TriangleMesh, LoopRepairResult]:
    if order_used == "side_first":
        pass_order = (False, True)
    elif order_used == "bottom_first":
        pass_order = (True, False)
    else:
        raise ValueError(f"Unsupported loop repair order: {order_used}")

    mesh_after_first, first_result = _apply_loop_repair_pass(
        mesh=mesh,
        repair_bottom_loops=pass_order[0],
        fill_holes_enable=fill_holes_enable,
        fill_holes_max_boundary_edges=fill_holes_max_boundary_edges,
        fill_holes_max_planarity_m=fill_holes_max_planarity_m,
        bottom_cap_enable=bottom_cap_enable,
        bottom_cap_mode=bottom_cap_mode,
        bottom_axis_index=bottom_axis_index,
        bottom_loop_height_tol_m=bottom_loop_height_tol_m,
        bottom_cap_plane_offset_m=bottom_cap_plane_offset_m,
        bottom_local_patch_resolution_m=bottom_local_patch_resolution_m,
        bottom_local_patch_margin_m=bottom_local_patch_margin_m,
        emit_debug=emit_debug,
    )
    mesh_after_second, second_result = _apply_loop_repair_pass(
        mesh=mesh_after_first,
        repair_bottom_loops=pass_order[1],
        fill_holes_enable=fill_holes_enable,
        fill_holes_max_boundary_edges=fill_holes_max_boundary_edges,
        fill_holes_max_planarity_m=fill_holes_max_planarity_m,
        bottom_cap_enable=bottom_cap_enable,
        bottom_cap_mode=bottom_cap_mode,
        bottom_axis_index=bottom_axis_index,
        bottom_loop_height_tol_m=bottom_loop_height_tol_m,
        bottom_cap_plane_offset_m=bottom_cap_plane_offset_m,
        bottom_local_patch_resolution_m=bottom_local_patch_resolution_m,
        bottom_local_patch_margin_m=bottom_local_patch_margin_m,
        emit_debug=emit_debug,
    )
    return mesh_after_second, _merge_loop_repair_results(first_result, second_result, order_used)


def _loop_repair_quality_key(
    mesh: o3d.geometry.TriangleMesh,
    result: LoopRepairResult,
) -> tuple[int, int, int, int, int, int, int, int, int, int]:
    boundary_loop_count = len(_extract_boundary_loops(mesh))
    return (
        boundary_loop_count,
        int(bool(mesh.is_self_intersecting())),
        int(not bool(mesh.is_watertight())),
        int(not bool(mesh.is_edge_manifold(False))),
        int(not bool(mesh.is_vertex_manifold())),
        result.skipped_loop_count,
        result.local_patch_fallback_count,
        int(result.vertices_added.shape[0]),
        len(result.triangles_added),
        0 if result.order_used == "side_first" else 1,
    )


def _bottom_band_metrics(
    mesh: o3d.geometry.TriangleMesh,
    bottom_cut_axis_value_m: float,
    band_height_m: float = 8e-4,
    min_abs_normal_axis: float = 0.7,
) -> tuple[int, float]:
    triangles = np.asarray(mesh.triangles, dtype=np.int64)
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    if triangles.size == 0 or vertices.size == 0:
        return 0, 0.0

    tri_p0 = vertices[triangles[:, 0]]
    tri_p1 = vertices[triangles[:, 1]]
    tri_p2 = vertices[triangles[:, 2]]
    normals = np.cross(tri_p1 - tri_p0, tri_p2 - tri_p0)
    normal_norms = np.linalg.norm(normals, axis=1)
    abs_normal_z = np.zeros_like(normal_norms)
    valid_normals = normal_norms > 1e-12
    abs_normal_z[valid_normals] = np.abs(normals[valid_normals, 2]) / normal_norms[valid_normals]
    centroid_z = (tri_p0[:, 2] + tri_p1[:, 2] + tri_p2[:, 2]) / 3.0
    bottom_mask = (
        (centroid_z <= (float(bottom_cut_axis_value_m) + float(band_height_m)))
        & (abs_normal_z >= float(min_abs_normal_axis))
    )
    band_area = 0.5 * float(np.sum(np.abs(normals[bottom_mask, 2])))
    return int(np.count_nonzero(bottom_mask)), band_area


def _pymeshfix_cleanup_quality_key(
    mesh_table: o3d.geometry.TriangleMesh,
    mesh_world: o3d.geometry.TriangleMesh,
    bottom_cut_axis_value_m: float,
) -> tuple[int, int, int, int, int, int, int]:
    bottom_band_count, bottom_band_area = _bottom_band_metrics(
        mesh=mesh_table,
        bottom_cut_axis_value_m=bottom_cut_axis_value_m,
    )
    return (
        len(_extract_boundary_loops(mesh_world)),
        int(bool(mesh_world.is_self_intersecting())),
        int(not bool(mesh_world.is_watertight())),
        int(not bool(mesh_world.is_edge_manifold(False))),
        int(not bool(mesh_world.is_vertex_manifold())),
        bottom_band_count,
        int(round(bottom_band_area * 1e12)),
    )


def _repair_with_pymeshfix(
    mesh: o3d.geometry.TriangleMesh,
    fix_connectivity: bool,
    clean_max_iters: int,
    clean_inner_loops: int,
) -> tuple[o3d.geometry.TriangleMesh, LoopRepairResult]:
    try:
        from pymeshfix import PyTMesh
    except ImportError as exc:
        raise RuntimeError(
            "PyMeshFix is not installed in the current interpreter. "
            "Install pymeshfix or switch repair_backend to custom."
        ) from exc

    vertices_in = np.asarray(mesh.vertices, dtype=np.float64)
    triangles_in = np.asarray(mesh.triangles, dtype=np.int32)
    fix = PyTMesh()
    fix.set_quiet(True)
    fix.load_array(vertices_in, triangles_in)
    fix.fill_small_boundaries(0, True)
    if bool(fix_connectivity):
        fix.fix_connectivity()
    fix.clean(int(clean_max_iters), int(clean_inner_loops))
    vertices_out, triangles_out = fix.return_arrays()

    mesh_out = o3d.geometry.TriangleMesh()
    mesh_out.vertices = o3d.utility.Vector3dVector(np.asarray(vertices_out, dtype=np.float64))
    mesh_out.triangles = o3d.utility.Vector3iVector(np.asarray(triangles_out, dtype=np.int32))
    mesh_out = _basic_cleanup(mesh_out)

    added_vertex_count = max(0, int(len(mesh_out.vertices)) - int(len(mesh.vertices)))
    added_triangle_count = max(0, int(len(mesh_out.triangles)) - int(len(mesh.triangles)))
    return mesh_out, LoopRepairResult(
        triangles_added=[(-1, -1, -1)] * added_triangle_count,
        vertices_added=np.zeros((added_vertex_count, 3), dtype=np.float64),
        repaired_loop_count=int(len(_extract_boundary_loops(mesh))) - int(len(_extract_boundary_loops(mesh_out))),
        side_fill_count=0,
        bottom_cap_count=0,
        local_patch_fallback_count=0,
        skipped_loop_count=0,
        backend_used="pymeshfix",
        order_used="n/a",
    )


def _merge_meshes(meshes: list[o3d.geometry.TriangleMesh]) -> o3d.geometry.TriangleMesh:
    if not meshes:
        return o3d.geometry.TriangleMesh()

    vertices_blocks: list[np.ndarray] = []
    triangles_blocks: list[np.ndarray] = []
    vertex_offset = 0
    for mesh in meshes:
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        triangles = np.asarray(mesh.triangles, dtype=np.int64)
        if vertices.shape[0] == 0 or triangles.shape[0] == 0:
            continue
        vertices_blocks.append(vertices)
        triangles_blocks.append(triangles + vertex_offset)
        vertex_offset += vertices.shape[0]

    if not vertices_blocks or not triangles_blocks:
        return o3d.geometry.TriangleMesh()

    merged = o3d.geometry.TriangleMesh()
    merged.vertices = o3d.utility.Vector3dVector(np.vstack(vertices_blocks))
    merged.triangles = o3d.utility.Vector3iVector(np.vstack(triangles_blocks))
    return _basic_cleanup(merged)


def _repair_with_pymeshfix_per_component(
    mesh: o3d.geometry.TriangleMesh,
    fix_connectivity: bool,
    clean_max_iters: int,
    clean_inner_loops: int,
) -> tuple[o3d.geometry.TriangleMesh, LoopRepairResult]:
    triangle_clusters, cluster_triangle_counts, _ = mesh.cluster_connected_triangles()
    cluster_ids = np.asarray(triangle_clusters, dtype=np.int64)
    cluster_triangle_counts = np.asarray(cluster_triangle_counts, dtype=np.int64)
    if cluster_ids.size == 0 or cluster_triangle_counts.size == 0:
        return _repair_with_pymeshfix(
            mesh=mesh,
            fix_connectivity=fix_connectivity,
            clean_max_iters=clean_max_iters,
            clean_inner_loops=clean_inner_loops,
        )

    repaired_parts: list[o3d.geometry.TriangleMesh] = []
    total_result = LoopRepairResult(
        triangles_added=[],
        vertices_added=np.zeros((0, 3), dtype=np.float64),
        repaired_loop_count=0,
        side_fill_count=0,
        bottom_cap_count=0,
        local_patch_fallback_count=0,
        skipped_loop_count=0,
        backend_used="pymeshfix_components",
        order_used="n/a",
    )

    for cluster_id in np.argsort(cluster_triangle_counts)[::-1].tolist():
        component_mesh = _copy_mesh(mesh)
        component_mesh.remove_triangles_by_mask(cluster_ids != int(cluster_id))
        component_mesh.remove_unreferenced_vertices()
        if len(component_mesh.triangles) == 0:
            continue
        repaired_component, component_result = _repair_with_pymeshfix(
            mesh=component_mesh,
            fix_connectivity=fix_connectivity,
            clean_max_iters=clean_max_iters,
            clean_inner_loops=clean_inner_loops,
        )
        repaired_parts.append(repaired_component)
        total_result = _combine_repair_results(
            first_result=total_result,
            second_result=component_result,
            backend_used="pymeshfix_components",
            order_used="n/a",
        )

    merged_mesh = _merge_meshes(repaired_parts)
    return merged_mesh, total_result


def _skip_repair(mesh: o3d.geometry.TriangleMesh) -> tuple[o3d.geometry.TriangleMesh, LoopRepairResult]:
    return _copy_mesh(mesh), LoopRepairResult(
        triangles_added=[],
        vertices_added=np.zeros((0, 3), dtype=np.float64),
        repaired_loop_count=0,
        side_fill_count=0,
        bottom_cap_count=0,
        local_patch_fallback_count=0,
        skipped_loop_count=len(_extract_boundary_loops(mesh)),
        backend_used="none",
        order_used="n/a",
    )


def _postprocess_pymeshfix_bottom(
    mesh: o3d.geometry.TriangleMesh,
    bottom_cut_axis_value_m: float,
    bottom_cap_mode: str,
    fill_holes_max_planarity_m: float,
    bottom_loop_height_tol_m: float,
    bottom_local_patch_resolution_m: float,
    bottom_local_patch_margin_m: float,
) -> tuple[o3d.geometry.TriangleMesh, LoopRepairResult]:
    mesh_retrimmed, cut_vertices_added = _clip_mesh_above_axis(
        mesh=mesh,
        axis_index=2,
        min_axis_value_m=bottom_cut_axis_value_m,
    )

    mesh_bottom_cleaned, cap_result = _apply_loop_repair_pass(
        mesh=mesh_retrimmed,
        repair_bottom_loops=True,
        fill_holes_enable=False,
        fill_holes_max_boundary_edges=3,
        fill_holes_max_planarity_m=fill_holes_max_planarity_m,
        bottom_cap_enable=True,
        bottom_cap_mode=bottom_cap_mode,
        bottom_axis_index=2,
        bottom_loop_height_tol_m=bottom_loop_height_tol_m,
        bottom_cap_plane_offset_m=0.0,
        bottom_local_patch_resolution_m=bottom_local_patch_resolution_m,
        bottom_local_patch_margin_m=bottom_local_patch_margin_m,
        emit_debug=True,
    )

    cut_vertices_block = (
        np.zeros((int(cut_vertices_added), 3), dtype=np.float64)
        if int(cut_vertices_added) > 0
        else np.zeros((0, 3), dtype=np.float64)
    )
    post_result = LoopRepairResult(
        triangles_added=cap_result.triangles_added,
        vertices_added=(
            np.vstack([cut_vertices_block, cap_result.vertices_added])
            if cut_vertices_block.size > 0 or cap_result.vertices_added.size > 0
            else np.zeros((0, 3), dtype=np.float64)
        ),
        repaired_loop_count=cap_result.repaired_loop_count,
        side_fill_count=cap_result.side_fill_count,
        bottom_cap_count=cap_result.bottom_cap_count,
        local_patch_fallback_count=cap_result.local_patch_fallback_count,
        skipped_loop_count=cap_result.skipped_loop_count,
        backend_used="pymeshfix_bottom_cleanup",
        order_used="bottom_only",
    )
    return mesh_bottom_cleaned, post_result


def _repair_boundary_loops(
    mesh: o3d.geometry.TriangleMesh,
    fill_holes_enable: bool,
    fill_holes_max_boundary_edges: int,
    fill_holes_max_planarity_m: float,
    bottom_cap_enable: bool,
    bottom_cap_mode: str,
    bottom_axis_index: int,
    bottom_loop_height_tol_m: float,
    bottom_cap_plane_offset_m: float,
    bottom_local_patch_resolution_m: float,
    bottom_local_patch_margin_m: float,
    repair_order: str,
) -> tuple[o3d.geometry.TriangleMesh, LoopRepairResult]:
    if repair_order in ("side_first", "bottom_first"):
        return _run_loop_repair_order(
            mesh=mesh,
            order_used=repair_order,
            fill_holes_enable=fill_holes_enable,
            fill_holes_max_boundary_edges=fill_holes_max_boundary_edges,
            fill_holes_max_planarity_m=fill_holes_max_planarity_m,
            bottom_cap_enable=bottom_cap_enable,
            bottom_cap_mode=bottom_cap_mode,
            bottom_axis_index=bottom_axis_index,
            bottom_loop_height_tol_m=bottom_loop_height_tol_m,
            bottom_cap_plane_offset_m=bottom_cap_plane_offset_m,
            bottom_local_patch_resolution_m=bottom_local_patch_resolution_m,
            bottom_local_patch_margin_m=bottom_local_patch_margin_m,
            emit_debug=True,
        )

    if repair_order != "try_both":
        raise ValueError(
            f"Unsupported repair_order: {repair_order}. Expected side_first, bottom_first, or try_both."
        )

    side_mesh, side_result = _run_loop_repair_order(
        mesh=mesh,
        order_used="side_first",
        fill_holes_enable=fill_holes_enable,
        fill_holes_max_boundary_edges=fill_holes_max_boundary_edges,
        fill_holes_max_planarity_m=fill_holes_max_planarity_m,
        bottom_cap_enable=bottom_cap_enable,
        bottom_cap_mode=bottom_cap_mode,
        bottom_axis_index=bottom_axis_index,
        bottom_loop_height_tol_m=bottom_loop_height_tol_m,
        bottom_cap_plane_offset_m=bottom_cap_plane_offset_m,
        bottom_local_patch_resolution_m=bottom_local_patch_resolution_m,
        bottom_local_patch_margin_m=bottom_local_patch_margin_m,
        emit_debug=False,
    )
    bottom_mesh, bottom_result = _run_loop_repair_order(
        mesh=mesh,
        order_used="bottom_first",
        fill_holes_enable=fill_holes_enable,
        fill_holes_max_boundary_edges=fill_holes_max_boundary_edges,
        fill_holes_max_planarity_m=fill_holes_max_planarity_m,
        bottom_cap_enable=bottom_cap_enable,
        bottom_cap_mode=bottom_cap_mode,
        bottom_axis_index=bottom_axis_index,
        bottom_loop_height_tol_m=bottom_loop_height_tol_m,
        bottom_cap_plane_offset_m=bottom_cap_plane_offset_m,
        bottom_local_patch_resolution_m=bottom_local_patch_resolution_m,
        bottom_local_patch_margin_m=bottom_local_patch_margin_m,
        emit_debug=False,
    )

    side_key = _loop_repair_quality_key(side_mesh, side_result)
    bottom_key = _loop_repair_quality_key(bottom_mesh, bottom_result)
    if side_key[:-1] == bottom_key[:-1]:
        print(f"Repair order selection: side_first and bottom_first tied on quality {side_key[:-1]}; keeping side_first.")
        return side_mesh, side_result
    if side_key <= bottom_key:
        print(f"Repair order selection: side_first won over bottom_first ({side_key} <= {bottom_key})")
        return side_mesh, side_result
    print(f"Repair order selection: bottom_first won over side_first ({bottom_key} < {side_key})")
    return bottom_mesh, bottom_result


def _print_mesh_info(label: str, mesh: o3d.geometry.TriangleMesh) -> None:
    aabb = mesh.get_axis_aligned_bounding_box()
    bbox_min = np.asarray(aabb.min_bound, dtype=np.float64)
    bbox_max = np.asarray(aabb.max_bound, dtype=np.float64)
    boundary_loop_count = len(_extract_boundary_loops(mesh))

    print(f"{label}:")
    print(f"  vertices: {len(mesh.vertices)}")
    print(f"  triangles: {len(mesh.triangles)}")
    print(f"  boundary_loops: {boundary_loop_count}")
    print(f"  is_watertight: {bool(mesh.is_watertight())}")
    print(f"  is_edge_manifold_allow_boundary: {bool(mesh.is_edge_manifold(True))}")
    print(f"  is_edge_manifold_closed: {bool(mesh.is_edge_manifold(False))}")
    print(f"  is_vertex_manifold: {bool(mesh.is_vertex_manifold())}")
    print(f"  is_self_intersecting: {bool(mesh.is_self_intersecting())}")
    print(f"  aabb_min_m: [{bbox_min[0]:.6f}, {bbox_min[1]:.6f}, {bbox_min[2]:.6f}]")
    print(f"  aabb_max_m: [{bbox_max[0]:.6f}, {bbox_max[1]:.6f}, {bbox_max[2]:.6f}]")


def _print_table_trim_info(table_trim: TableTrimResult) -> None:
    plane = np.asarray(table_trim.plane_model_world, dtype=np.float64)
    print("Table trim:")
    print(
        f"  plane_model_world: "
        f"[{plane[0]:.8f}, {plane[1]:.8f}, {plane[2]:.8f}, {plane[3]:.8f}]"
    )
    print(f"  source_vertices: {table_trim.source_vertices}")
    print(f"  source_triangles: {table_trim.source_triangles}")
    print(f"  fit_seed_vertices: {table_trim.fit_seed_vertices}")
    print(f"  trimmed_vertices: {table_trim.trimmed_vertices}")
    print(f"  trimmed_triangles: {table_trim.trimmed_triangles}")
    print(f"  cut_vertices_added: {table_trim.cut_vertices_added}")


def _show_before_after_meshes(
    mesh_before: o3d.geometry.TriangleMesh,
    mesh_after: o3d.geometry.TriangleMesh,
) -> None:
    mesh_before_viz = _copy_mesh(mesh_before)
    mesh_after_viz = _copy_mesh(mesh_after)
    mesh_before_viz.paint_uniform_color(list(mesh_color_before))
    mesh_after_viz.paint_uniform_color(list(mesh_color_after))

    o3d.visualization.draw_geometries(
        [mesh_before_viz, mesh_after_viz],
        window_name="Mesh cleanup: before (grey) and after (red)",
        mesh_show_back_face=True,
    )


def _save_mesh(mesh: o3d.geometry.TriangleMesh, output_path: Path, write_ascii: bool) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    success = o3d.io.write_triangle_mesh(
        str(output_path),
        mesh,
        write_ascii=bool(write_ascii),
        write_triangle_uvs=False,
        write_vertex_colors=False,
    )
    if not success:
        raise RuntimeError(f"Failed to save cleaned mesh: {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Clean a triangle mesh by filling holes and optionally capping the bottom.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--input-mesh", type=str, default=default_input_mesh_path)
    parser.add_argument("--output-mesh", type=str, default=default_output_mesh_path)
    parser.add_argument("--show-visualization", action="store_true", default=show_debug_vis)
    parser.add_argument("--no-visualization", action="store_false", dest="show_visualization")
    parser.add_argument("--keep-largest-component", action="store_true", default=keep_largest_component)
    parser.add_argument("--keep-all-components", action="store_false", dest="keep_largest_component")
    parser.add_argument(
        "--keep-component-min-triangle-fraction",
        type=float,
        default=keep_component_min_triangle_fraction,
    )
    parser.add_argument(
        "--keep-component-min-triangles",
        type=int,
        default=keep_component_min_triangles,
    )
    parser.add_argument(
        "--repair-target",
        choices=("all_posttrim", "largest_component_only", "bead_component"),
        default=repair_target,
    )
    parser.add_argument(
        "--repair-backend",
        choices=("none", "custom", "pymeshfix"),
        default=repair_backend,
    )
    parser.add_argument(
        "--pymeshfix-fix-connectivity",
        action="store_true",
        default=pymeshfix_fix_connectivity,
    )
    parser.add_argument(
        "--skip-pymeshfix-fix-connectivity",
        action="store_false",
        dest="pymeshfix_fix_connectivity",
    )
    parser.add_argument(
        "--pymeshfix-clean-max-iters",
        type=int,
        default=pymeshfix_clean_max_iters,
    )
    parser.add_argument(
        "--pymeshfix-clean-inner-loops",
        type=int,
        default=pymeshfix_clean_inner_loops,
    )
    parser.add_argument(
        "--pymeshfix-post-trim-bottom",
        action="store_true",
        default=pymeshfix_post_trim_bottom,
    )
    parser.add_argument(
        "--skip-pymeshfix-post-trim-bottom",
        action="store_false",
        dest="pymeshfix_post_trim_bottom",
    )
    parser.add_argument(
        "--pymeshfix-post-trim-bottom-mode",
        choices=("auto", "planar_in_place", "snap_to_plane", "local_patch", "extruded"),
        default=pymeshfix_post_trim_bottom_mode,
    )
    parser.add_argument("--fill-holes", action="store_true", default=fill_holes_enable)
    parser.add_argument("--skip-hole-fill", action="store_false", dest="fill_holes")
    parser.add_argument(
        "--fill-holes-max-boundary-edges",
        type=int,
        default=fill_holes_max_boundary_edges,
    )
    parser.add_argument(
        "--fill-holes-max-planarity-mm",
        type=float,
        default=fill_holes_max_planarity_mm,
    )
    parser.add_argument("--table-trim", action="store_true", default=table_trim_enable)
    parser.add_argument("--skip-table-trim", action="store_false", dest="table_trim")
    parser.add_argument(
        "--table-ransac-thresh-mm",
        type=float,
        default=table_ransac_thresh_mm,
    )
    parser.add_argument("--table-ransac-n", type=int, default=table_ransac_n)
    parser.add_argument("--table-ransac-iters", type=int, default=table_ransac_iters)
    parser.add_argument("--table-ransac-seed", type=int, default=table_ransac_seed)
    parser.add_argument("--table-fit-low-band-mm", type=float, default=table_fit_low_band_mm)
    parser.add_argument(
        "--z-min-above-plane-mm",
        type=float,
        default=z_min_above_plane_mm,
    )
    parser.add_argument("--bottom-cap", action="store_true", default=bottom_cap_enable)
    parser.add_argument("--skip-bottom-cap", action="store_false", dest="bottom_cap")
    parser.add_argument(
        "--bottom-cap-mode",
        choices=("auto", "planar_in_place", "snap_to_plane", "local_patch", "extruded"),
        default=bottom_cap_mode,
    )
    parser.add_argument(
        "--bottom-local-patch-resolution-mm",
        type=float,
        default=bottom_local_patch_resolution_mm,
    )
    parser.add_argument(
        "--bottom-local-patch-margin-mm",
        type=float,
        default=bottom_local_patch_margin_mm,
    )
    parser.add_argument(
        "--repair-order",
        choices=("side_first", "bottom_first", "try_both"),
        default=loop_repair_order,
    )
    parser.add_argument("--bottom-axis", choices=("x", "y", "z"), default=bottom_axis)
    parser.add_argument(
        "--bottom-loop-height-tol-mm",
        type=float,
        default=bottom_loop_height_tol_mm,
    )
    parser.add_argument(
        "--bottom-cap-plane-offset-mm",
        type=float,
        default=bottom_cap_plane_offset_mm,
    )
    parser.add_argument("--write-ascii", action="store_true", default=output_write_ascii)
    parser.add_argument("--write-binary", action="store_false", dest="write_ascii")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.fill_holes_max_boundary_edges < 3:
        raise ValueError("--fill-holes-max-boundary-edges must be at least 3.")
    if args.keep_component_min_triangle_fraction < 0.0:
        raise ValueError("--keep-component-min-triangle-fraction must be non-negative.")
    if args.keep_component_min_triangles < 1:
        raise ValueError("--keep-component-min-triangles must be at least 1.")
    if args.fill_holes_max_planarity_mm < 0.0:
        raise ValueError("--fill-holes-max-planarity-mm must be non-negative.")
    if args.table_ransac_thresh_mm <= 0.0:
        raise ValueError("--table-ransac-thresh-mm must be positive.")
    if args.table_ransac_n < 3:
        raise ValueError("--table-ransac-n must be at least 3.")
    if args.table_ransac_iters <= 0:
        raise ValueError("--table-ransac-iters must be positive.")
    if args.table_ransac_seed < 0:
        raise ValueError("--table-ransac-seed must be non-negative.")
    if args.table_fit_low_band_mm < 0.0:
        raise ValueError("--table-fit-low-band-mm must be non-negative.")
    if args.z_min_above_plane_mm < 0.0:
        raise ValueError("--z-min-above-plane-mm must be non-negative.")
    if args.bottom_loop_height_tol_mm < 0.0:
        raise ValueError("--bottom-loop-height-tol-mm must be non-negative.")
    if args.bottom_cap_plane_offset_mm < 0.0:
        raise ValueError("--bottom-cap-plane-offset-mm must be non-negative.")
    if args.pymeshfix_clean_max_iters <= 0:
        raise ValueError("--pymeshfix-clean-max-iters must be positive.")
    if args.pymeshfix_clean_inner_loops <= 0:
        raise ValueError("--pymeshfix-clean-inner-loops must be positive.")
    if args.bottom_local_patch_resolution_mm <= 0.0:
        raise ValueError("--bottom-local-patch-resolution-mm must be positive.")
    if args.bottom_local_patch_margin_mm < 0.0:
        raise ValueError("--bottom-local-patch-margin-mm must be non-negative.")
    if args.bottom_cap_mode == "planar_in_place" and args.bottom_cap_plane_offset_mm > 0.0:
        raise ValueError(
            "--bottom-cap-plane-offset-mm must be 0 when --bottom-cap-mode=planar_in_place."
        )

    input_mesh_path = _resolve_input_mesh_path(args.input_mesh)
    output_mesh_path = _resolve_output_mesh_path(input_mesh_path, args.output_mesh)
    bottom_axis_index = _axis_to_index(args.bottom_axis)
    if args.table_trim and bottom_axis_index != 2:
        raise ValueError("--bottom-axis must be z when --table-trim is enabled.")

    mesh_input = _load_mesh(input_mesh_path)
    mesh_pretrim = _pretrim_sanitize(mesh_input)
    mesh_working = _copy_mesh(mesh_pretrim)
    T_world_from_table: np.ndarray | None = None

    print()
    _print_mesh_info("Input mesh", mesh_input)
    print()
    _print_mesh_info("After pre-trim sanitize", mesh_pretrim)

    if args.table_trim:
        table_trim = _trim_mesh_above_table(
            mesh_world=mesh_pretrim,
            ransac_thresh_mm=args.table_ransac_thresh_mm,
            ransac_n=args.table_ransac_n,
            ransac_iters=args.table_ransac_iters,
            ransac_seed=args.table_ransac_seed,
            fit_low_band_mm=args.table_fit_low_band_mm,
            z_min_above_plane_mm=args.z_min_above_plane_mm,
        )
        mesh_working = table_trim.mesh_table_trimmed
        T_world_from_table = table_trim.T_world_from_table

        print()
        _print_table_trim_info(table_trim)
        print()
        _print_mesh_info("After table trim", mesh_working)

    if args.keep_largest_component:
        mesh_working = _keep_significant_components(
            mesh=mesh_working,
            min_triangle_fraction=args.keep_component_min_triangle_fraction,
            min_triangles=args.keep_component_min_triangles,
        )

    print()
    _print_mesh_info("After post-trim filtering", mesh_working)

    mesh_repair_input = _copy_mesh(mesh_working)
    if args.repair_backend != "none":
        mesh_repair_input = _select_repair_target_mesh(
            mesh=mesh_working,
            target_mode=args.repair_target,
            axis_index=bottom_axis_index,
        )
        if len(mesh_repair_input.triangles) != len(mesh_working.triangles):
            print()
            print(
                "Repair target selection: "
                f"{args.repair_target} kept "
                f"{len(mesh_repair_input.triangles)} / {len(mesh_working.triangles)} triangles for closure."
            )
            _print_mesh_info("Repair target mesh", mesh_repair_input)

    if args.repair_backend == "none":
        mesh_repaired, repair_result = _skip_repair(mesh_working)
    elif args.repair_backend == "pymeshfix":
        mesh_repaired, repair_result = _repair_with_pymeshfix_per_component(
            mesh=mesh_repair_input,
            fix_connectivity=args.pymeshfix_fix_connectivity,
            clean_max_iters=args.pymeshfix_clean_max_iters,
            clean_inner_loops=args.pymeshfix_clean_inner_loops,
        )
        if args.table_trim and args.bottom_cap and args.pymeshfix_post_trim_bottom:
            mesh_bottom_candidate, bottom_cleanup_result = _postprocess_pymeshfix_bottom(
                mesh=mesh_repaired,
                bottom_cut_axis_value_m=_mm_to_m(args.z_min_above_plane_mm),
                bottom_cap_mode=args.pymeshfix_post_trim_bottom_mode,
                fill_holes_max_planarity_m=_mm_to_m(args.fill_holes_max_planarity_mm),
                bottom_loop_height_tol_m=_mm_to_m(args.bottom_loop_height_tol_mm),
                bottom_local_patch_resolution_m=_mm_to_m(args.bottom_local_patch_resolution_mm),
                bottom_local_patch_margin_m=_mm_to_m(args.bottom_local_patch_margin_mm),
            )
            mesh_world_base = _copy_mesh(mesh_repaired)
            mesh_world_base.transform(T_world_from_table)
            mesh_world_base.compute_vertex_normals()
            mesh_world_candidate = _copy_mesh(mesh_bottom_candidate)
            mesh_world_candidate.transform(T_world_from_table)
            mesh_world_candidate.compute_vertex_normals()

            base_quality = _pymeshfix_cleanup_quality_key(
                mesh_table=mesh_repaired,
                mesh_world=mesh_world_base,
                bottom_cut_axis_value_m=_mm_to_m(args.z_min_above_plane_mm),
            )
            candidate_quality = _pymeshfix_cleanup_quality_key(
                mesh_table=mesh_bottom_candidate,
                mesh_world=mesh_world_candidate,
                bottom_cut_axis_value_m=_mm_to_m(args.z_min_above_plane_mm),
            )
            if candidate_quality < base_quality:
                mesh_repaired = mesh_bottom_candidate
                repair_result = _combine_repair_results(
                    first_result=repair_result,
                    second_result=bottom_cleanup_result,
                    backend_used=f"pymeshfix+{args.pymeshfix_post_trim_bottom_mode}",
                    order_used="n/a",
                )
                print(
                    "PyMeshFix bottom cleanup kept: "
                    f"{candidate_quality} < {base_quality}"
                )
                print()
                _print_mesh_info("After PyMeshFix bottom cleanup", mesh_repaired)
            else:
                print(
                    "PyMeshFix bottom cleanup discarded: "
                    f"{candidate_quality} >= {base_quality}"
                )
                print("Keeping the raw PyMeshFix output because it is topologically better.")
    else:
        mesh_repaired, repair_result = _repair_boundary_loops(
            mesh=mesh_repair_input,
            fill_holes_enable=args.fill_holes,
            fill_holes_max_boundary_edges=args.fill_holes_max_boundary_edges,
            fill_holes_max_planarity_m=_mm_to_m(args.fill_holes_max_planarity_mm),
            bottom_cap_enable=args.bottom_cap,
            bottom_cap_mode=args.bottom_cap_mode,
            bottom_axis_index=bottom_axis_index,
            bottom_loop_height_tol_m=_mm_to_m(args.bottom_loop_height_tol_mm),
            bottom_cap_plane_offset_m=_mm_to_m(args.bottom_cap_plane_offset_mm),
            bottom_local_patch_resolution_m=_mm_to_m(args.bottom_local_patch_resolution_mm),
            bottom_local_patch_margin_m=_mm_to_m(args.bottom_local_patch_margin_mm),
            repair_order=args.repair_order,
        )

    print()
    print("Repair summary:")
    print(f"  backend_used: {repair_result.backend_used}")
    print(f"  order_used: {repair_result.order_used}")
    print(f"  repaired_loops: {repair_result.repaired_loop_count}")
    print(f"  side_fills: {repair_result.side_fill_count}")
    print(f"  bottom_caps: {repair_result.bottom_cap_count}")
    print(f"  local_patch_fallbacks: {repair_result.local_patch_fallback_count}")
    print(f"  skipped_loops: {repair_result.skipped_loop_count}")
    print(f"  added_vertices: {repair_result.vertices_added.shape[0]}")
    print(f"  added_triangles: {len(repair_result.triangles_added)}")

    mesh_output = _copy_mesh(mesh_repaired)
    if T_world_from_table is not None:
        mesh_output.transform(T_world_from_table)
        mesh_output.compute_vertex_normals()

    print()
    _print_mesh_info("After loop repair", mesh_repaired)
    if T_world_from_table is not None:
        print()
        _print_mesh_info("Final output mesh (world frame)", mesh_output)

    if args.show_visualization:
        _show_before_after_meshes(mesh_input, mesh_output)

    _save_mesh(mesh_output, output_mesh_path, write_ascii=args.write_ascii)
    print()
    print(f"Saved cleaned mesh: {output_mesh_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
