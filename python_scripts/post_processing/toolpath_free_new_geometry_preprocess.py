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
- Build a combined colored mesh from the previous mesh plus the extracted new mesh
- Save the combined mesh under a different name
"""

from __future__ import annotations

import argparse
import colorsys
from pathlib import Path

import numpy as np
import open3d as o3d


# -----------------------------------------------------------------------------
# In-script parameters
# -----------------------------------------------------------------------------

default_prev_mesh_path = "~/Downloads/260227_160709/print_scan_098/reconstruct/previous_plus_new_geometry.stl"
default_curr_mesh_path = "~/Downloads/260227_160709/print_scan_105/reconstruct/tsdf_surface_mesh.stl"
show_debug_vis = True
sample_point_count_prev = 100000
sample_point_count_curr = 100000
new_geom_dist_thresh_mm = 1.5
mesh_keep_dist_mm = 1.0
default_output_mesh_path = ""
default_output_combined_mesh_path = ""
combined_mesh_color_seed = 17

# Visualization colors for inspection.
prev_mesh_color = (0.65, 0.65, 0.65)
curr_mesh_color = (0.15, 0.35, 0.95)
final_output_color = (0.95, 0.15, 0.15)


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


def _print_mesh_info(name: str, mesh: o3d.geometry.TriangleMesh) -> None:
    aabb = mesh.get_axis_aligned_bounding_box()
    bbox_min = np.asarray(aabb.min_bound, dtype=np.float64)
    bbox_max = np.asarray(aabb.max_bound, dtype=np.float64)

    print(f"{name}:")
    print(f"  vertices: {len(mesh.vertices)}")
    print(f"  triangles: {len(mesh.triangles)}")
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


def _seeded_mesh_colors(num_colors: int, seed: int) -> list[list[float]]:
    rng = np.random.default_rng(int(seed))
    hue0 = float(rng.uniform(0.0, 1.0))
    colors: list[list[float]] = []
    for idx in range(max(0, int(num_colors))):
        hue = (hue0 + (idx / max(1, num_colors))) % 1.0
        sat = float(rng.uniform(0.55, 0.80))
        val = float(rng.uniform(0.80, 0.95))
        rgb = colorsys.hsv_to_rgb(hue, sat, val)
        colors.append([float(rgb[0]), float(rgb[1]), float(rgb[2])])
    return colors


def _paint_mesh_copy(mesh: o3d.geometry.TriangleMesh, color: list[float]) -> o3d.geometry.TriangleMesh:
    mesh_out = o3d.geometry.TriangleMesh(mesh)
    mesh_out.paint_uniform_color(color)
    if len(mesh_out.vertices) > 0 and len(mesh_out.triangles) > 0:
        mesh_out.compute_vertex_normals()
    return mesh_out


def _combine_meshes(meshes: list[o3d.geometry.TriangleMesh]) -> o3d.geometry.TriangleMesh:
    verts_all: list[np.ndarray] = []
    tris_all: list[np.ndarray] = []
    colors_all: list[np.ndarray] = []
    vert_offset = 0

    for mesh in meshes:
        verts = np.asarray(mesh.vertices, dtype=np.float64)
        tris = np.asarray(mesh.triangles, dtype=np.int32)
        if verts.shape[0] == 0 or tris.shape[0] == 0:
            continue

        if len(mesh.vertex_colors) == len(mesh.vertices):
            vcols = np.asarray(mesh.vertex_colors, dtype=np.float64)
        else:
            vcols = np.full((verts.shape[0], 3), 0.7, dtype=np.float64)

        verts_all.append(verts)
        tris_all.append(tris + vert_offset)
        colors_all.append(vcols)
        vert_offset += verts.shape[0]

    if len(verts_all) == 0:
        return o3d.geometry.TriangleMesh()

    mesh_combined = o3d.geometry.TriangleMesh()
    mesh_combined.vertices = o3d.utility.Vector3dVector(np.vstack(verts_all))
    mesh_combined.triangles = o3d.utility.Vector3iVector(np.vstack(tris_all))
    mesh_combined.vertex_colors = o3d.utility.Vector3dVector(np.vstack(colors_all))
    mesh_combined.compute_vertex_normals()
    return mesh_combined


def _build_combined_colored_mesh(
    prev_mesh: o3d.geometry.TriangleMesh,
    kept_mesh: o3d.geometry.TriangleMesh,
    seed: int,
) -> tuple[o3d.geometry.TriangleMesh, list[list[float]]]:
    colors = _seeded_mesh_colors(2, seed)
    prev_colored = _paint_mesh_copy(prev_mesh, colors[0])
    kept_colored = _paint_mesh_copy(kept_mesh, colors[1])
    return _combine_meshes([prev_colored, kept_colored]), colors


def _print_combined_mesh_stats(
    combined_mesh: o3d.geometry.TriangleMesh,
    colors: list[list[float]],
    seed: int,
) -> None:
    print("Step 7: Build combined colored mesh from previous + new extracted mesh")
    print(f"  combined_mesh_color_seed: {int(seed)}")
    if len(colors) >= 2:
        print(f"  prev_component_color_rgb: [{colors[0][0]:.3f}, {colors[0][1]:.3f}, {colors[0][2]:.3f}]")
        print(f"  new_component_color_rgb: [{colors[1][0]:.3f}, {colors[1][1]:.3f}, {colors[1][2]:.3f}]")
    print(f"  combined_vertices: {len(combined_mesh.vertices)}")
    print(f"  combined_triangles: {len(combined_mesh.triangles)}")


def _show_combined_colored_mesh(combined_mesh: o3d.geometry.TriangleMesh) -> None:
    combined_mesh_viz = o3d.geometry.TriangleMesh(combined_mesh)
    o3d.visualization.draw_geometries(
        [combined_mesh_viz],
        window_name="Step 7: Combined previous + new colored mesh",
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
        print("  note: STL export does not preserve mesh colors; seeded colors are visualization-only.")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 7 loader, sampled-point, NN-distance, threshold, mesh-guided extraction, final mesh viewer, and combined mesh export for consecutive TSDF meshes."
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
        help="Output mesh path for the combined mesh. Defaults next to the current mesh as STL.",
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
    combined_mesh, combined_colors = _build_combined_colored_mesh(prev_mesh, kept_mesh, combined_mesh_color_seed)
    _print_combined_mesh_stats(combined_mesh, combined_colors, combined_mesh_color_seed)

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
        print("Opening combined colored mesh viewer...")
        _show_combined_colored_mesh(combined_mesh)
    else:
        print("Visualization skipped. Use --show to inspect mesh, sampled-cloud, NN-distance, threshold, kept-mesh, and combined-mesh alignment.")

    _save_mesh(kept_mesh, output_mesh_path, "Step 6: Saved final kept mesh")
    _save_mesh(combined_mesh, output_combined_mesh_path, "Step 7: Saved combined mesh")


if __name__ == "__main__":
    main()
