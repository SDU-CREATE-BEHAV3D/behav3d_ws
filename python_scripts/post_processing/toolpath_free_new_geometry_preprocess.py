#!/usr/bin/env python3
"""
Preprocess consecutive TSDF meshes to isolate newly added geometry.

Steps implemented here:
- Load previous TSDF mesh
- Load current TSDF mesh
- Print basic mesh diagnostics
- Optionally visualize both meshes together
- Sample both meshes into point clouds
- Print sampled point counts
- Optionally visualize both sampled point clouds together
- Compute nearest-neighbor distance from current sampled points to previous sampled points
- Print distance diagnostics in millimeters
- Optionally visualize current sampled points colored by NN distance
- Threshold distances to isolate new geometry
- Print retained-point diagnostics
- Optionally visualize retained new points in red
- Keep the matching part of the current mesh using the retained new points as a guide
- Print kept-mesh diagnostics
- Optionally visualize the kept mesh
- Visualize the final kept mesh by itself
- Save the final kept mesh
- Try to merge the extracted new mesh back into the previous cumulative mesh
- Repair boundary holes on the merged cumulative mesh when possible
- Save the merged cumulative mesh under a different name
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import open3d as o3d


# -----------------------------------------------------------------------------
# In-script parameters
# -----------------------------------------------------------------------------

default_prev_mesh_path = "~/Downloads/260227_160709/print_scan_070/reconstruct/previous_plus_new_geometry.stl"
default_curr_mesh_path = "~/Downloads/260227_160709/print_scan_077/reconstruct/tsdf_surface_mesh.stl"
show_debug_vis = True
sample_point_count_prev = 100000
sample_point_count_curr = 100000
new_geom_dist_thresh_mm = 1.5
mesh_keep_dist_mm = 1.0
default_output_mesh_path = ""
default_output_combined_mesh_path = ""
merge_seam_weld_dist_mm = 0.60
merge_fill_holes_enable = True
merge_fill_holes_max_boundary_edges = 800
merge_fill_holes_max_planarity_mm = 4.0

# Visualization colors for inspection.
prev_mesh_color = (0.65, 0.65, 0.65)
curr_mesh_color = (0.15, 0.35, 0.95)
final_output_color = (0.95, 0.15, 0.15)
merged_output_color = (0.20, 0.72, 0.28)

_mesh_cleanup_module = None
_mesh_cleanup_module_load_attempted = False


def _mm_to_m(v_mm: float) -> float:
    return float(v_mm) * 1e-3


def _resolve_mesh_path(mesh_path_str: str | None, default_path: str, arg_name: str) -> Path:
    candidate = mesh_path_str if mesh_path_str and str(mesh_path_str).strip() else default_path
    if not str(candidate).strip():
        raise ValueError(f"Missing required mesh path for {arg_name}.")

    mesh_path = Path(candidate).expanduser().resolve()
    if not mesh_path.exists():
        raise FileNotFoundError(f"{arg_name} file not found: {mesh_path}")
    return mesh_path


def _resolve_output_mesh_path(curr_mesh_path: Path, output_mesh_path_str: str | None) -> Path:
    if output_mesh_path_str is not None and str(output_mesh_path_str).strip():
        return Path(output_mesh_path_str).expanduser().resolve()
    return curr_mesh_path.parent / "new_geometry_mesh_kept.stl"


def _resolve_combined_output_mesh_path(curr_mesh_path: Path, output_mesh_path_str: str | None) -> Path:
    if output_mesh_path_str is not None and str(output_mesh_path_str).strip():
        return Path(output_mesh_path_str).expanduser().resolve()
    return curr_mesh_path.parent / "previous_plus_new_geometry.stl"


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
    if len(mesh_out.vertices) > 0 and len(mesh_out.triangles) > 0:
        mesh_out.compute_vertex_normals()
    return mesh_out


def _build_boundary_edges(mesh: o3d.geometry.TriangleMesh) -> list[tuple[int, int]]:
    triangles = np.asarray(mesh.triangles, dtype=np.int32)
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
                    (current_vertex, loop[0]) if current_vertex < loop[0] else (loop[0], current_vertex)
                )
                visited_edges.add(closing_edge)
                loop.append(loop[0])
                break

            next_vertex = None
            for candidate in next_candidates:
                edge_key = (
                    (current_vertex, candidate) if current_vertex < candidate else (candidate, current_vertex)
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


def _mesh_summary(mesh: o3d.geometry.TriangleMesh) -> dict[str, object]:
    return {
        "vertices": int(len(mesh.vertices)),
        "triangles": int(len(mesh.triangles)),
        "boundary_loops": int(len(_extract_boundary_loops(mesh))),
        "is_watertight": bool(mesh.is_watertight()) if len(mesh.triangles) > 0 else False,
        "is_edge_manifold_closed": bool(mesh.is_edge_manifold(False)) if len(mesh.triangles) > 0 else False,
        "is_vertex_manifold": bool(mesh.is_vertex_manifold()) if len(mesh.triangles) > 0 else False,
    }


def _mesh_quality_key(mesh: o3d.geometry.TriangleMesh) -> tuple[int, int, int, int, int]:
    summary = _mesh_summary(mesh)
    return (
        int(not bool(summary["is_watertight"])),
        int(summary["boundary_loops"]),
        int(not bool(summary["is_edge_manifold_closed"])),
        int(not bool(summary["is_vertex_manifold"])),
        -int(summary["triangles"]),
    )


def _print_mesh_info(name: str, mesh: o3d.geometry.TriangleMesh) -> None:
    aabb = mesh.get_axis_aligned_bounding_box()
    bbox_min = np.asarray(aabb.min_bound, dtype=np.float64)
    bbox_max = np.asarray(aabb.max_bound, dtype=np.float64)
    summary = _mesh_summary(mesh)

    print(f"{name}:")
    print(f"  vertices: {summary['vertices']}")
    print(f"  triangles: {summary['triangles']}")
    print(f"  boundary_loops: {summary['boundary_loops']}")
    print(f"  is_watertight: {summary['is_watertight']}")
    print(f"  is_edge_manifold_closed: {summary['is_edge_manifold_closed']}")
    print(f"  is_vertex_manifold: {summary['is_vertex_manifold']}")
    print(f"  aabb_min_m: [{bbox_min[0]:.6f}, {bbox_min[1]:.6f}, {bbox_min[2]:.6f}]")
    print(f"  aabb_max_m: [{bbox_max[0]:.6f}, {bbox_max[1]:.6f}, {bbox_max[2]:.6f}]")


def _show_two_meshes(
    prev_mesh: o3d.geometry.TriangleMesh,
    curr_mesh: o3d.geometry.TriangleMesh,
) -> None:
    prev_mesh_viz = o3d.geometry.TriangleMesh(prev_mesh)
    curr_mesh_viz = o3d.geometry.TriangleMesh(curr_mesh)

    prev_mesh_viz.paint_uniform_color(list(prev_mesh_color))
    curr_mesh_viz.paint_uniform_color(list(curr_mesh_color))

    o3d.visualization.draw_geometries(
        [prev_mesh_viz, curr_mesh_viz],
        window_name="Step 1: Previous mesh (grey) and current mesh (blue)",
        mesh_show_back_face=True,
    )


def _sample_mesh_to_points(mesh: o3d.geometry.TriangleMesh, num_points: int) -> o3d.geometry.PointCloud:
    num_points = max(1, int(num_points))
    sampled = mesh.sample_points_uniformly(number_of_points=num_points, use_triangle_normal=True)
    if len(sampled.points) == 0:
        raise RuntimeError(f"Sampling produced an empty point cloud for requested count={num_points}.")
    return sampled


def _print_sample_info(name: str, pcd: o3d.geometry.PointCloud) -> None:
    print(f"{name}:")
    print(f"  sampled_points: {len(pcd.points)}")


def _show_two_point_clouds(
    prev_points: o3d.geometry.PointCloud,
    curr_points: o3d.geometry.PointCloud,
) -> None:
    prev_points_viz = o3d.geometry.PointCloud(prev_points)
    curr_points_viz = o3d.geometry.PointCloud(curr_points)

    prev_points_viz.paint_uniform_color(list(prev_mesh_color))
    curr_points_viz.paint_uniform_color(list(curr_mesh_color))

    o3d.visualization.draw_geometries(
        [prev_points_viz, curr_points_viz],
        window_name="Step 2: Previous sampled cloud (grey) and current sampled cloud (blue)",
    )


def _compute_curr_to_prev_nn_distance(
    curr_points: o3d.geometry.PointCloud,
    prev_points: o3d.geometry.PointCloud,
) -> np.ndarray:
    dist_m = np.asarray(curr_points.compute_point_cloud_distance(prev_points), dtype=np.float64)
    if dist_m.shape[0] != len(curr_points.points):
        raise RuntimeError("Nearest-neighbor distance count does not match current sampled point count.")
    return dist_m


def _print_distance_stats(dist_m: np.ndarray) -> None:
    dist_mm = 1e3 * np.asarray(dist_m, dtype=np.float64)
    print("Current-to-previous NN distance stats:")
    print(f"  min_mm: {np.min(dist_mm):.6f}")
    print(f"  max_mm: {np.max(dist_mm):.6f}")
    print(f"  mean_mm: {np.mean(dist_mm):.6f}")
    print(f"  median_mm: {np.median(dist_mm):.6f}")
    print(f"  p90_mm: {np.percentile(dist_mm, 90):.6f}")
    print(f"  p95_mm: {np.percentile(dist_mm, 95):.6f}")
    print(f"  p99_mm: {np.percentile(dist_mm, 99):.6f}")


def _distance_to_colors(dist_m: np.ndarray) -> np.ndarray:
    dist_m = np.asarray(dist_m, dtype=np.float64)
    d_min = float(np.min(dist_m))
    d_max = float(np.max(dist_m))
    if d_max <= d_min + 1e-12:
        return np.tile(np.array([[0.05, 0.10, 0.35]], dtype=np.float64), (dist_m.shape[0], 1))

    t = (dist_m - d_min) / (d_max - d_min)
    colors = np.zeros((dist_m.shape[0], 3), dtype=np.float64)

    cool_dark = np.array([0.05, 0.10, 0.35], dtype=np.float64)
    cool_bright = np.array([0.10, 0.80, 0.90], dtype=np.float64)
    warm_mid = np.array([0.95, 0.85, 0.20], dtype=np.float64)
    warm_hot = np.array([0.95, 0.15, 0.10], dtype=np.float64)

    mask_lo = t <= (1.0 / 3.0)
    if np.any(mask_lo):
        alpha = (t[mask_lo] / (1.0 / 3.0))[:, None]
        colors[mask_lo] = (1.0 - alpha) * cool_dark + alpha * cool_bright

    mask_mid = (t > (1.0 / 3.0)) & (t <= (2.0 / 3.0))
    if np.any(mask_mid):
        alpha = ((t[mask_mid] - (1.0 / 3.0)) / (1.0 / 3.0))[:, None]
        colors[mask_mid] = (1.0 - alpha) * cool_bright + alpha * warm_mid

    mask_hi = t > (2.0 / 3.0)
    if np.any(mask_hi):
        alpha = ((t[mask_hi] - (2.0 / 3.0)) / (1.0 / 3.0))[:, None]
        colors[mask_hi] = (1.0 - alpha) * warm_mid + alpha * warm_hot

    return colors


def _show_distance_debug(
    prev_points: o3d.geometry.PointCloud,
    curr_points: o3d.geometry.PointCloud,
    dist_m: np.ndarray,
) -> None:
    prev_points_viz = o3d.geometry.PointCloud(prev_points)
    curr_points_viz = o3d.geometry.PointCloud(curr_points)

    prev_points_viz.paint_uniform_color([0.82, 0.82, 0.82])
    curr_points_viz.colors = o3d.utility.Vector3dVector(_distance_to_colors(dist_m))

    o3d.visualization.draw_geometries(
        [prev_points_viz, curr_points_viz],
        window_name="Step 3: Current sampled cloud colored by NN distance to previous cloud",
    )


def _filter_new_points(
    curr_points: o3d.geometry.PointCloud,
    dist_m: np.ndarray,
    thresh_mm: float,
) -> tuple[o3d.geometry.PointCloud, np.ndarray]:
    thresh_m = _mm_to_m(float(thresh_mm))
    is_new = np.asarray(dist_m, dtype=np.float64) > thresh_m
    keep_idx = np.flatnonzero(is_new).tolist()
    points_new = curr_points.select_by_index(keep_idx)
    return points_new, is_new


def _print_new_point_stats(
    curr_points: o3d.geometry.PointCloud,
    points_new: o3d.geometry.PointCloud,
    thresh_mm: float,
) -> None:
    total = len(curr_points.points)
    kept = len(points_new.points)
    kept_pct = 100.0 * float(kept) / max(1, total)
    print("Step 4: Threshold distances to isolate new geometry")
    print(f"  threshold_mm: {float(thresh_mm):.6f}")
    print(f"  total_curr_sampled_points: {total}")
    print(f"  retained_new_points: {kept}")
    print(f"  retained_percent: {kept_pct:.3f}")


def _show_threshold_debug(
    prev_points: o3d.geometry.PointCloud,
    curr_points: o3d.geometry.PointCloud,
    points_new: o3d.geometry.PointCloud,
) -> None:
    prev_points_viz = o3d.geometry.PointCloud(prev_points)
    curr_points_viz = o3d.geometry.PointCloud(curr_points)
    points_new_viz = o3d.geometry.PointCloud(points_new)

    prev_points_viz.paint_uniform_color([0.72, 0.72, 0.72])
    curr_points_viz.paint_uniform_color([0.55, 0.72, 0.95])
    points_new_viz.paint_uniform_color(list(final_output_color))

    o3d.visualization.draw_geometries(
        [prev_points_viz, curr_points_viz, points_new_viz],
        window_name="Step 4: Previous cloud (grey), current cloud (blue), retained new points (red)",
    )


def _extract_mesh_near_points(
    curr_mesh: o3d.geometry.TriangleMesh,
    guide_points: o3d.geometry.PointCloud,
    keep_dist_mm: float,
) -> tuple[o3d.geometry.TriangleMesh, np.ndarray]:
    if len(guide_points.points) == 0:
        return o3d.geometry.TriangleMesh(), np.zeros((len(curr_mesh.triangles),), dtype=bool)

    tris = np.asarray(curr_mesh.triangles, dtype=np.int32)
    verts = np.asarray(curr_mesh.vertices, dtype=np.float64)
    if tris.shape[0] == 0 or verts.shape[0] == 0:
        return o3d.geometry.TriangleMesh(), np.zeros((0,), dtype=bool)

    tri_centroids = verts[tris].mean(axis=1)
    tri_centroid_pcd = o3d.geometry.PointCloud()
    tri_centroid_pcd.points = o3d.utility.Vector3dVector(tri_centroids)

    dist_m = np.asarray(tri_centroid_pcd.compute_point_cloud_distance(guide_points), dtype=np.float64)
    keep_tri_mask = dist_m <= _mm_to_m(float(keep_dist_mm))

    mesh_kept = o3d.geometry.TriangleMesh(curr_mesh)
    mesh_kept.remove_triangles_by_mask(~keep_tri_mask)
    mesh_kept.remove_unreferenced_vertices()
    if len(mesh_kept.vertices) > 0 and len(mesh_kept.triangles) > 0:
        mesh_kept.compute_vertex_normals()
    return mesh_kept, keep_tri_mask


def _print_kept_mesh_stats(
    curr_mesh: o3d.geometry.TriangleMesh,
    kept_mesh: o3d.geometry.TriangleMesh,
    keep_tri_mask: np.ndarray,
    keep_dist_mm: float,
) -> None:
    total_triangles = len(curr_mesh.triangles)
    kept_triangles = int(np.count_nonzero(keep_tri_mask))
    kept_vertices = len(kept_mesh.vertices)
    kept_percent = 100.0 * float(kept_triangles) / max(1, total_triangles)
    print("Step 5: Keep current-mesh region guided by retained new points")
    print(f"  mesh_keep_dist_mm: {float(keep_dist_mm):.6f}")
    print(f"  total_curr_mesh_triangles: {total_triangles}")
    print(f"  kept_mesh_triangles: {kept_triangles}")
    print(f"  kept_mesh_vertices: {kept_vertices}")
    print(f"  kept_triangle_percent: {kept_percent:.3f}")


def _show_kept_mesh_debug(
    prev_mesh: o3d.geometry.TriangleMesh,
    curr_mesh: o3d.geometry.TriangleMesh,
    kept_mesh: o3d.geometry.TriangleMesh,
    guide_points: o3d.geometry.PointCloud,
) -> None:
    prev_mesh_viz = o3d.geometry.TriangleMesh(prev_mesh)
    curr_mesh_viz = o3d.geometry.TriangleMesh(curr_mesh)
    kept_mesh_viz = o3d.geometry.TriangleMesh(kept_mesh)
    guide_points_viz = o3d.geometry.PointCloud(guide_points)

    prev_mesh_viz.paint_uniform_color(list(prev_mesh_color))
    curr_mesh_viz.paint_uniform_color([0.70, 0.82, 0.98])
    kept_mesh_viz.paint_uniform_color(list(final_output_color))
    guide_points_viz.paint_uniform_color([0.15, 0.85, 0.20])

    o3d.visualization.draw_geometries(
        [prev_mesh_viz, curr_mesh_viz, kept_mesh_viz, guide_points_viz],
        window_name="Step 5: Previous mesh (grey), current mesh (blue), kept mesh (red), guide points (green)",
        mesh_show_back_face=True,
    )


def _show_final_mesh(kept_mesh: o3d.geometry.TriangleMesh) -> None:
    kept_mesh_viz = o3d.geometry.TriangleMesh(kept_mesh)
    kept_mesh_viz.paint_uniform_color(list(final_output_color))
    o3d.visualization.draw_geometries(
        [kept_mesh_viz],
        window_name="Step 6: Final kept mesh",
        mesh_show_back_face=True,
    )


def _combine_meshes(meshes: list[o3d.geometry.TriangleMesh]) -> o3d.geometry.TriangleMesh:
    verts_all: list[np.ndarray] = []
    tris_all: list[np.ndarray] = []
    vert_offset = 0

    for mesh in meshes:
        verts = np.asarray(mesh.vertices, dtype=np.float64)
        tris = np.asarray(mesh.triangles, dtype=np.int32)
        if verts.shape[0] == 0 or tris.shape[0] == 0:
            continue

        verts_all.append(verts)
        tris_all.append(tris + vert_offset)
        vert_offset += verts.shape[0]

    if len(verts_all) == 0:
        return o3d.geometry.TriangleMesh()

    mesh_combined = o3d.geometry.TriangleMesh()
    mesh_combined.vertices = o3d.utility.Vector3dVector(np.vstack(verts_all))
    mesh_combined.triangles = o3d.utility.Vector3iVector(np.vstack(tris_all))
    return _basic_cleanup(mesh_combined)


def _load_mesh_cleanup_module():
    global _mesh_cleanup_module, _mesh_cleanup_module_load_attempted
    if _mesh_cleanup_module_load_attempted:
        return _mesh_cleanup_module

    _mesh_cleanup_module_load_attempted = True
    module_path = Path(__file__).with_name("mesh_cleanup_close_holes.py")
    if not module_path.exists():
        return None

    try:
        spec = importlib.util.spec_from_file_location("mesh_cleanup_close_holes_runtime", module_path)
        if spec is None or spec.loader is None:
            return None

        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        _mesh_cleanup_module = module
    except Exception as exc:
        print(f"Warning: failed to load mesh cleanup helper module: {exc}")
        _mesh_cleanup_module = None

    return _mesh_cleanup_module


def _merge_close_vertices(
    mesh: o3d.geometry.TriangleMesh,
    weld_dist_mm: float,
) -> tuple[o3d.geometry.TriangleMesh, int]:
    mesh_out = _copy_mesh(mesh)
    if weld_dist_mm <= 0.0 or len(mesh_out.vertices) == 0 or not hasattr(mesh_out, "merge_close_vertices"):
        return _basic_cleanup(mesh_out), 0

    vertices_before = len(mesh_out.vertices)
    mesh_out.merge_close_vertices(_mm_to_m(float(weld_dist_mm)))
    mesh_out = _basic_cleanup(mesh_out)
    collapsed = max(0, int(vertices_before) - int(len(mesh_out.vertices)))
    return mesh_out, collapsed


def _repair_boundary_holes(
    mesh: o3d.geometry.TriangleMesh,
) -> tuple[o3d.geometry.TriangleMesh, object | None, str | None]:
    if not merge_fill_holes_enable or len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        return _basic_cleanup(mesh), None, None

    cleanup_module = _load_mesh_cleanup_module()
    if cleanup_module is None:
        return _basic_cleanup(mesh), None, "mesh_cleanup_close_holes.py could not be loaded"

    try:
        repaired_mesh, repair_result = cleanup_module._repair_boundary_loops(
            mesh=_copy_mesh(mesh),
            fill_holes_enable=True,
            fill_holes_max_boundary_edges=int(merge_fill_holes_max_boundary_edges),
            fill_holes_max_planarity_m=_mm_to_m(merge_fill_holes_max_planarity_mm),
            bottom_cap_enable=False,
            bottom_cap_mode="planar_in_place",
            bottom_axis_index=2,
            bottom_loop_height_tol_m=0.0,
            bottom_cap_plane_offset_m=0.0,
            bottom_local_patch_resolution_m=0.0,
            bottom_local_patch_margin_m=0.0,
            repair_order="side_first",
        )
        return _basic_cleanup(repaired_mesh), repair_result, None
    except Exception as exc:
        return _basic_cleanup(mesh), None, str(exc)


def _merge_extracted_mesh_into_previous(
    prev_mesh: o3d.geometry.TriangleMesh,
    kept_mesh: o3d.geometry.TriangleMesh,
) -> tuple[o3d.geometry.TriangleMesh, dict[str, object]]:
    candidate_info: list[dict[str, object]] = []

    def add_candidate(name: str, mesh: o3d.geometry.TriangleMesh) -> None:
        mesh_clean = _basic_cleanup(mesh)
        summary = _mesh_summary(mesh_clean)
        candidate_info.append(
            {
                "name": name,
                "mesh": mesh_clean,
                "summary": summary,
                "quality_key": _mesh_quality_key(mesh_clean),
            }
        )

    if len(kept_mesh.vertices) == 0 or len(kept_mesh.triangles) == 0:
        prev_only = _basic_cleanup(prev_mesh)
        add_candidate("previous_only", prev_only)
        return prev_only, {
            "selected_candidate": "previous_only",
            "candidate_info": candidate_info,
            "collapsed_vertices": 0,
            "repair_result": None,
            "repair_warning": "No extracted new mesh was available, so the previous mesh was kept as-is.",
        }

    merged_raw = _combine_meshes([prev_mesh, kept_mesh])
    add_candidate("combined_raw", merged_raw)

    merged_welded, collapsed_vertices = _merge_close_vertices(merged_raw, merge_seam_weld_dist_mm)
    add_candidate("seam_welded", merged_welded)

    repaired_mesh, repair_result, repair_warning = _repair_boundary_holes(merged_welded)
    add_candidate("hole_repaired", repaired_mesh)

    best_candidate = min(candidate_info, key=lambda item: item["quality_key"])
    return best_candidate["mesh"], {
        "selected_candidate": best_candidate["name"],
        "candidate_info": candidate_info,
        "collapsed_vertices": int(collapsed_vertices),
        "repair_result": repair_result,
        "repair_warning": repair_warning,
    }


def _print_merged_mesh_stats(
    merge_debug: dict[str, object],
    merged_mesh: o3d.geometry.TriangleMesh,
) -> None:
    print("Step 7: Merge extracted new mesh into previous cumulative mesh")
    print(f"  seam_weld_dist_mm: {float(merge_seam_weld_dist_mm):.6f}")
    print(f"  seam_weld_collapsed_vertices: {int(merge_debug['collapsed_vertices'])}")
    print(f"  selected_candidate: {merge_debug['selected_candidate']}")

    repair_result = merge_debug.get("repair_result")
    if repair_result is not None:
        print(f"  repaired_loop_count: {int(repair_result.repaired_loop_count)}")
        print(f"  skipped_loop_count: {int(repair_result.skipped_loop_count)}")
        print(f"  repair_backend: {repair_result.backend_used}")
        print(f"  repair_order: {repair_result.order_used}")

    repair_warning = merge_debug.get("repair_warning")
    if repair_warning:
        print(f"  repair_warning: {repair_warning}")

    for candidate in merge_debug["candidate_info"]:
        summary = candidate["summary"]
        print(
            "  candidate_{}: vertices={}, triangles={}, boundary_loops={}, is_watertight={}, "
            "is_edge_manifold_closed={}, is_vertex_manifold={}".format(
                candidate["name"],
                int(summary["vertices"]),
                int(summary["triangles"]),
                int(summary["boundary_loops"]),
                bool(summary["is_watertight"]),
                bool(summary["is_edge_manifold_closed"]),
                bool(summary["is_vertex_manifold"]),
            )
        )

    final_summary = _mesh_summary(merged_mesh)
    print(f"  merged_vertices: {int(final_summary['vertices'])}")
    print(f"  merged_triangles: {int(final_summary['triangles'])}")
    print(f"  merged_boundary_loops: {int(final_summary['boundary_loops'])}")
    print(f"  merged_is_watertight: {bool(final_summary['is_watertight'])}")


def _show_merged_mesh(merged_mesh: o3d.geometry.TriangleMesh) -> None:
    combined_mesh_viz = o3d.geometry.TriangleMesh(merged_mesh)
    combined_mesh_viz.paint_uniform_color(list(merged_output_color))
    o3d.visualization.draw_geometries(
        [combined_mesh_viz],
        window_name="Step 7: Merged cumulative mesh",
        mesh_show_back_face=True,
    )


def _save_mesh(mesh: o3d.geometry.TriangleMesh, output_path: Path, label: str) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    success = o3d.io.write_triangle_mesh(str(output_path), mesh)
    if not success:
        raise RuntimeError(f"Failed to save mesh: {output_path}")
    print(label)
    print(f"  output_mesh_path: {output_path}")
    print(f"  saved_vertices: {len(mesh.vertices)}")
    print(f"  saved_triangles: {len(mesh.triangles)}")
    if output_path.suffix.lower() == ".stl" and len(mesh.vertex_colors) == len(mesh.vertices):
        print("  note: STL export does not preserve mesh colors; any visualization colors are not saved.")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Load consecutive TSDF meshes, extract newly added geometry, and merge it back into the cumulative mesh."
    )
    parser.add_argument(
        "--prev_mesh",
        type=str,
        default=default_prev_mesh_path,
        help="Path to the previous TSDF mesh.",
    )
    parser.add_argument(
        "--curr_mesh",
        type=str,
        default=default_curr_mesh_path,
        help="Path to the current TSDF mesh.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        default=show_debug_vis,
        help="Show mesh and sampled-point debug viewers.",
    )
    parser.add_argument(
        "--output_mesh",
        type=str,
        default=default_output_mesh_path,
        help="Output mesh path for the final kept mesh. Defaults next to the current mesh.",
    )
    parser.add_argument(
        "--output_combined_mesh",
        type=str,
        default=default_output_combined_mesh_path,
        help="Output mesh path for the merged cumulative mesh. Defaults next to the current mesh as STL.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    prev_mesh_path = _resolve_mesh_path(args.prev_mesh, default_prev_mesh_path, "--prev_mesh")
    curr_mesh_path = _resolve_mesh_path(args.curr_mesh, default_curr_mesh_path, "--curr_mesh")
    output_mesh_path = _resolve_output_mesh_path(curr_mesh_path, args.output_mesh)
    output_combined_mesh_path = _resolve_combined_output_mesh_path(curr_mesh_path, args.output_combined_mesh)

    print("Step 1: Load and verify previous/current TSDF meshes")
    print(f"  prev_mesh_path: {prev_mesh_path}")
    print(f"  curr_mesh_path: {curr_mesh_path}")

    prev_mesh = _load_mesh(prev_mesh_path)
    curr_mesh = _load_mesh(curr_mesh_path)

    _print_mesh_info("Previous mesh", prev_mesh)
    _print_mesh_info("Current mesh", curr_mesh)

    print("Step 2: Uniformly sample both meshes into point clouds")
    print(f"  sample_point_count_prev: {int(sample_point_count_prev)}")
    print(f"  sample_point_count_curr: {int(sample_point_count_curr)}")

    prev_points = _sample_mesh_to_points(prev_mesh, sample_point_count_prev)
    curr_points = _sample_mesh_to_points(curr_mesh, sample_point_count_curr)

    _print_sample_info("Previous sampled cloud", prev_points)
    _print_sample_info("Current sampled cloud", curr_points)

    print("Step 3: Compute current-to-previous nearest-neighbor distances")
    dist_m = _compute_curr_to_prev_nn_distance(curr_points, prev_points)
    _print_distance_stats(dist_m)

    points_new, _ = _filter_new_points(curr_points, dist_m, new_geom_dist_thresh_mm)
    _print_new_point_stats(curr_points, points_new, new_geom_dist_thresh_mm)

    kept_mesh, keep_tri_mask = _extract_mesh_near_points(curr_mesh, points_new, mesh_keep_dist_mm)
    _print_kept_mesh_stats(curr_mesh, kept_mesh, keep_tri_mask, mesh_keep_dist_mm)
    merged_mesh, merge_debug = _merge_extracted_mesh_into_previous(prev_mesh, kept_mesh)
    _print_merged_mesh_stats(merge_debug, merged_mesh)

    if args.show:
        print("Opening mesh viewer...")
        _show_two_meshes(prev_mesh, curr_mesh)
        print("Opening sampled point cloud viewer...")
        _show_two_point_clouds(prev_points, curr_points)
        print("Opening NN-distance viewer...")
        _show_distance_debug(prev_points, curr_points, dist_m)
        print("Opening thresholded new-geometry viewer...")
        _show_threshold_debug(prev_points, curr_points, points_new)
        print("Opening kept-mesh viewer...")
        _show_kept_mesh_debug(prev_mesh, curr_mesh, kept_mesh, points_new)
        print("Opening final kept-mesh viewer...")
        _show_final_mesh(kept_mesh)
        print("Opening merged cumulative mesh viewer...")
        _show_merged_mesh(merged_mesh)
    else:
        print("Visualization skipped. Use --show to inspect mesh, sampled-cloud, NN-distance, threshold, kept-mesh, and merged-mesh alignment.")

    _save_mesh(kept_mesh, output_mesh_path, "Step 6: Saved final kept mesh")
    _save_mesh(merged_mesh, output_combined_mesh_path, "Step 7: Saved merged cumulative mesh")


if __name__ == "__main__":
    main()
