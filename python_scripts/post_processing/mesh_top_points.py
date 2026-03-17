#!/usr/bin/env python3
"""
Detect and visualize top points on a mesh using a 2.5D height-map peak detector.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np
import open3d as o3d

try:
    from scipy import ndimage as ndi
except Exception:
    ndi = None

try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


# -----------------------------------------------------------------------------
# Important parameters
# -----------------------------------------------------------------------------

mesh_input_path = "~/Downloads/260227_160709/print_scan_049/reconstruct/tsdf_surface_mesh.stl"
show_3d = True
show_2d = False
viz_frame = "table"  # "table" or "world"
pause_ms = 0  # 0 waits for a key in the 2D OpenCV windows
open3d_random_seed = 7

downsample_voxel_mm = 0.0
table_ransac_thresh_mm = 1.0
table_ransac_n = 3
table_ransac_iters = 2500
z_min_above_plane_mm = 1.9

grid_mm = 0.05
height_percentile = 95.0
gaussian_sigma_px = 0.0
mask_percentile = 50.0
morph_kernel_px = 1
peak_min_height_mm = 3.0
peak_min_prominence_mm = 0.1
peak_prominence_relax_fraction_of_best = 0.5
peak_prominence_radius_mm = 2.0
min_peak_distance_mm = 4.5
max_num_peaks = 1700
peak_max_snap_z_error_mm = 1.5

peak_marker_radius_mm = 3
mesh_color = (0.72, 0.72, 0.72)
show_point_cloud_background = False
preview_peak_count = 22
show_bounding_box = True
box_bottom_z_mm = 0.0
box_padding_xy_mm = 0.0
box_padding_top_mm = 0.0
box_color = (0.10, 0.95, 0.95)


def mm_to_m(v_mm: float) -> float:
    return float(v_mm) * 1e-3


def ensure_odd(k: int) -> int:
    k = int(k)
    return 1 if k < 1 else (k if k % 2 == 1 else k + 1)


def normalize_plane(plane: np.ndarray) -> np.ndarray:
    plane = np.asarray(plane, dtype=np.float64).reshape(4)
    n = np.linalg.norm(plane[:3])
    if n < 1e-12:
        raise RuntimeError("Degenerate table plane model.")
    return plane / n


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    return (T[:3, :3] @ points.T).T + T[:3, 3]


def rotation_matrix_from_a_to_b(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)

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
        return -np.eye(3, dtype=np.float64) + 2.0 * np.outer(axis, axis)

    vx = np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + vx + (vx @ vx) * ((1.0 - c) / (s * s))


def load_mesh_vertices(mesh_path_str: str) -> Tuple[o3d.geometry.TriangleMesh, np.ndarray, Path]:
    if not str(mesh_path_str).strip():
        raise ValueError("Set mesh_input_path at the top of the script.")

    mesh_path = Path(mesh_path_str).expanduser().resolve()
    if not mesh_path.exists():
        raise FileNotFoundError(f"Mesh input file not found: {mesh_path}")

    mesh = o3d.io.read_triangle_mesh(str(mesh_path), enable_post_processing=True)
    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        raise RuntimeError(f"Input mesh is empty or invalid: {mesh_path}")
    mesh.compute_vertex_normals()

    points = np.asarray(mesh.vertices, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if points.shape[0] == 0:
        raise RuntimeError("Input mesh has no finite vertices.")

    if downsample_voxel_mm > 0.0:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.voxel_down_sample(voxel_size=mm_to_m(downsample_voxel_mm))
        points = np.asarray(pcd.points, dtype=np.float64)

    if points.shape[0] == 0:
        raise RuntimeError("No mesh vertices remain after downsampling.")
    return mesh, points, mesh_path


def fit_table_frame(points_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if hasattr(o3d.utility, "random") and hasattr(o3d.utility.random, "seed"):
        o3d.utility.random.seed(int(open3d_random_seed))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_world)
    plane_model, _ = pcd.segment_plane(
        distance_threshold=mm_to_m(table_ransac_thresh_mm),
        ransac_n=int(table_ransac_n),
        num_iterations=int(table_ransac_iters),
    )

    plane = normalize_plane(plane_model)
    if np.median(points_world @ plane[:3] + plane[3]) < 0.0:
        plane = -plane

    R = rotation_matrix_from_a_to_b(plane[:3], np.array([0.0, 0.0, 1.0], dtype=np.float64))
    t = np.array([0.0, 0.0, plane[3]], dtype=np.float64)

    T_table_from_world = np.eye(4, dtype=np.float64)
    T_table_from_world[:3, :3] = R
    T_table_from_world[:3, 3] = t

    T_world_from_table = np.eye(4, dtype=np.float64)
    T_world_from_table[:3, :3] = R.T
    T_world_from_table[:3, 3] = -R.T @ t

    return transform_points(points_world, T_table_from_world), T_table_from_world, T_world_from_table, plane


def nearest_fill_height(H: np.ndarray, valid: np.ndarray) -> np.ndarray:
    if np.all(valid):
        return H.copy()
    if not np.any(valid):
        raise RuntimeError("Height map has no valid cells.")

    if ndi is not None:
        nearest_idx = ndi.distance_transform_edt(~valid, return_distances=False, return_indices=True)
        return H[tuple(nearest_idx)]

    filled = H.copy()
    known = valid.copy()
    fallback = float(np.nanmean(H[valid]))
    kernel = np.ones((3, 3), dtype=np.uint8)
    for _ in range(max(H.shape) * 2):
        if np.all(known):
            break
        dilated = cv2.dilate(np.where(known, filled, fallback).astype(np.float32), kernel)
        filled[~known] = dilated[~known]
        known[:] = True
    return filled


def weighted_gaussian(H: np.ndarray, valid: np.ndarray) -> np.ndarray:
    if gaussian_sigma_px <= 0.0:
        return H.copy()

    Hf = H.astype(np.float32)
    W = valid.astype(np.float32)
    if ndi is not None:
        Hw = ndi.gaussian_filter(Hf * W, sigma=gaussian_sigma_px, mode="nearest")
        Wb = ndi.gaussian_filter(W, sigma=gaussian_sigma_px, mode="nearest")
    else:
        k = ensure_odd(max(3, int(round(6.0 * gaussian_sigma_px))))
        Hw = cv2.GaussianBlur(Hf * W, (k, k), gaussian_sigma_px, borderType=cv2.BORDER_REPLICATE)
        Wb = cv2.GaussianBlur(W, (k, k), gaussian_sigma_px, borderType=cv2.BORDER_REPLICATE)

    out = Hf.copy()
    good = Wb > 1e-6
    out[good] = Hw[good] / Wb[good]
    return out


def build_height_map(points_table: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    xy = points_table[:, :2]
    z = points_table[:, 2]
    grid_m = mm_to_m(grid_mm)
    min_xy = xy.min(axis=0)
    max_xy = xy.max(axis=0)

    nx = max(1, int(np.floor((max_xy[0] - min_xy[0]) / grid_m)) + 1)
    ny = max(1, int(np.floor((max_xy[1] - min_xy[1]) / grid_m)) + 1)
    ix = np.clip(np.floor((xy[:, 0] - min_xy[0]) / grid_m).astype(np.int32), 0, nx - 1)
    iy = np.clip(np.floor((xy[:, 1] - min_xy[1]) / grid_m).astype(np.int32), 0, ny - 1)
    linear = iy * nx + ix

    H = np.full((ny, nx), np.nan, dtype=np.float32)
    valid = np.zeros((ny, nx), dtype=bool)
    order = np.argsort(linear, kind="mergesort")
    linear_sorted = linear[order]
    splits = np.flatnonzero(np.diff(linear_sorted)) + 1
    for grp in np.split(order, splits):
        cell = int(linear[grp[0]])
        y = int(cell // nx)
        x = int(cell % nx)
        H[y, x] = float(np.percentile(z[grp], height_percentile))
        valid[y, x] = True

    H = nearest_fill_height(H, valid)
    H = weighted_gaussian(H, valid).astype(np.float32)
    return H, valid, min_xy.astype(np.float64), grid_m


def build_mask_and_dt(H: np.ndarray, valid: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    vals = H[valid]
    if vals.size == 0:
        raise RuntimeError("No valid height cells for mask creation.")

    threshold = float(np.percentile(vals, mask_percentile))
    mask = (H > threshold) & valid
    if not np.any(mask):
        mask = (H >= float(np.percentile(vals, 90.0))) & valid

    k = ensure_odd(max(1, int(morph_kernel_px)))
    if k > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel).astype(bool)
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, kernel).astype(bool)

    mask &= valid
    if not np.any(mask):
        mask = valid.copy()

    if ndi is not None:
        dt_mm = ndi.distance_transform_edt(mask).astype(np.float32) * float(grid_mm)
    else:
        dt_mm = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5).astype(np.float32) * float(grid_mm)
    return mask, dt_mm


def local_maxima(arr: np.ndarray, support: np.ndarray, radius_px: int) -> np.ndarray:
    size = 2 * max(1, int(radius_px)) + 1
    if ndi is not None:
        mx = ndi.maximum_filter(arr, size=size, mode="nearest")
    else:
        mx = cv2.dilate(arr.astype(np.float32), np.ones((size, size), dtype=np.uint8))
    return (arr >= (mx - 1e-6)) & support


def local_minimum(arr: np.ndarray, radius_px: int) -> np.ndarray:
    size = 2 * max(1, int(radius_px)) + 1
    if ndi is not None:
        return ndi.minimum_filter(arr, size=size, mode="nearest")
    return -cv2.dilate((-arr).astype(np.float32), np.ones((size, size), dtype=np.uint8))


def nms(coords: np.ndarray, scores: np.ndarray, min_dist_px: int) -> np.ndarray:
    if coords.shape[0] <= 1:
        return coords.astype(np.int32)

    kept = []
    d2_min = float(min_dist_px * min_dist_px)
    for i in np.argsort(-scores, kind="mergesort"):
        c = coords[i]
        if not kept:
            kept.append(c)
            continue
        if np.all(np.sum((np.asarray(kept) - c) ** 2, axis=1) >= d2_min):
            kept.append(c)
    return np.asarray(kept, dtype=np.int32)


def detect_peaks(H: np.ndarray, DT_mm: np.ndarray, mask: np.ndarray, valid: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
    source_mm = H.astype(np.float32) * 1e3
    support = (mask & valid).copy()

    info = {
        "num_support_cells": float(np.count_nonzero(support)),
        "max_dt_mm": float(np.max(DT_mm[support])) if np.any(support) else 0.0,
        "max_height_mm": float(np.max(source_mm[support])) if np.any(support) else 0.0,
    }
    if not np.any(support):
        info.update(
            {
                "num_local_maxima": 0.0,
                "num_after_filter": 0.0,
                "num_after_nms": 0.0,
                "min_height_thr": float(peak_min_height_mm),
                "min_prominence_thr": float(peak_min_prominence_mm),
                "effective_prominence_thr": float(peak_min_prominence_mm),
                "max_prominence_mm": 0.0,
            }
        )
        return np.empty((0, 2), dtype=np.int32), info

    min_dist_px = max(1, int(round(min_peak_distance_mm / grid_mm)))
    prominence_radius_px = max(1, int(round(peak_prominence_radius_mm / grid_mm)))
    prominence_mm = source_mm - local_minimum(source_mm, prominence_radius_px)
    prominence_mm[~valid] = 0.0

    coords = np.column_stack(np.nonzero(local_maxima(source_mm, support, min_dist_px)))
    info["num_local_maxima"] = float(coords.shape[0])
    info["max_prominence_mm"] = float(np.max(prominence_mm[support])) if np.any(support) else 0.0
    if coords.shape[0] == 0:
        peak = np.unravel_index(int(np.argmax(prominence_mm * support.astype(np.float32))), source_mm.shape)
        coords = np.asarray([[int(peak[0]), int(peak[1])]], dtype=np.int32)

    vals = source_mm[coords[:, 0], coords[:, 1]]
    prom_vals = prominence_mm[coords[:, 0], coords[:, 1]]
    effective_prominence_thr = float(peak_min_prominence_mm)
    if prom_vals.size > 0 and float(peak_prominence_relax_fraction_of_best) > 0.0:
        best_prominence = float(np.max(prom_vals))
        effective_prominence_thr = min(
            effective_prominence_thr,
            float(peak_prominence_relax_fraction_of_best) * best_prominence,
        )

    keep = (vals >= float(peak_min_height_mm)) & (prom_vals >= effective_prominence_thr)
    coords = coords[keep]
    vals = vals[keep]
    prom_vals = prom_vals[keep]
    info["min_height_thr"] = float(peak_min_height_mm)
    info["min_prominence_thr"] = float(peak_min_prominence_mm)
    info["effective_prominence_thr"] = float(effective_prominence_thr)
    info["num_after_filter"] = float(coords.shape[0])
    if coords.shape[0] == 0:
        peak = np.unravel_index(int(np.argmax(prominence_mm * support.astype(np.float32))), source_mm.shape)
        coords = np.asarray([[int(peak[0]), int(peak[1])]], dtype=np.int32)
        vals = source_mm[coords[:, 0], coords[:, 1]]
        prom_vals = prominence_mm[coords[:, 0], coords[:, 1]]

    scores = prom_vals + 1e-3 * vals
    coords = nms(coords, scores, min_dist_px)
    info["num_after_nms"] = float(coords.shape[0])
    if coords.shape[0] > int(max_num_peaks):
        scores = prominence_mm[coords[:, 0], coords[:, 1]] + 1e-3 * source_mm[coords[:, 0], coords[:, 1]]
        coords = coords[np.argsort(-scores, kind="mergesort")[: int(max_num_peaks)]]

    coords = coords[np.lexsort((coords[:, 1], coords[:, 0]))]
    return coords.astype(np.int32), info


def peaks_to_xyz(peaks_yx: np.ndarray, H: np.ndarray, min_xy: np.ndarray, grid_m: float) -> np.ndarray:
    if peaks_yx.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)
    x = min_xy[0] + (peaks_yx[:, 1].astype(np.float64) + 0.5) * grid_m
    y = min_xy[1] + (peaks_yx[:, 0].astype(np.float64) + 0.5) * grid_m
    z = H[peaks_yx[:, 0], peaks_yx[:, 1]].astype(np.float64)
    return np.column_stack([x, y, z])


def anchor_peaks(peaks_xyz_grid: np.ndarray, points_xyz: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
    if peaks_xyz_grid.shape[0] == 0:
        return peaks_xyz_grid.copy(), {"num_snapped": 0.0, "mean_z_error_mm": 0.0, "max_z_error_mm": 0.0}

    if cKDTree is not None:
        tree = cKDTree(points_xyz)
        _, nn = tree.query(peaks_xyz_grid, k=1)
        snapped = points_xyz[np.asarray(nn, dtype=np.int32)]
    else:
        snapped = np.empty_like(peaks_xyz_grid)
        for i, p in enumerate(peaks_xyz_grid):
            snapped[i] = points_xyz[int(np.argmin(np.sum((points_xyz - p[None, :]) ** 2, axis=1)))]

    z_err_mm = np.abs(snapped[:, 2] - peaks_xyz_grid[:, 2]) * 1e3
    ok = z_err_mm <= peak_max_snap_z_error_mm
    anchored = snapped.copy()
    anchored[~ok] = peaks_xyz_grid[~ok]
    return anchored, {
        "num_snapped": float(np.count_nonzero(ok)),
        "mean_z_error_mm": float(np.mean(z_err_mm)),
        "max_z_error_mm": float(np.max(z_err_mm)),
    }


def crop_mesh_above_table(mesh: o3d.geometry.TriangleMesh, z_min_m: float) -> o3d.geometry.TriangleMesh:
    mesh_out = o3d.geometry.TriangleMesh(mesh)
    verts = np.asarray(mesh_out.vertices, dtype=np.float64)
    if verts.shape[0] == 0:
        return mesh_out
    mesh_out.remove_vertices_by_mask(verts[:, 2] <= float(z_min_m))
    mesh_out.remove_unreferenced_vertices()
    if len(mesh_out.vertices) > 0 and len(mesh_out.triangles) > 0:
        mesh_out.compute_vertex_normals()
    return mesh_out


def build_bounding_box_info(points_table: np.ndarray) -> Dict[str, object]:
    if points_table.shape[0] == 0:
        raise RuntimeError("Cannot build a bounding box without object points.")

    pad_xy_m = mm_to_m(box_padding_xy_mm)
    pad_top_m = mm_to_m(box_padding_top_mm)
    min_xy = np.min(points_table[:, :2], axis=0)
    max_xy = np.max(points_table[:, :2], axis=0)
    top_z_m = float(np.max(points_table[:, 2])) + pad_top_m
    bottom_z_m = mm_to_m(box_bottom_z_mm)

    min_corner = np.array([min_xy[0] - pad_xy_m, min_xy[1] - pad_xy_m, bottom_z_m], dtype=np.float64)
    max_corner = np.array([max_xy[0] + pad_xy_m, max_xy[1] + pad_xy_m, top_z_m], dtype=np.float64)
    size_xy_mm = (max_corner[:2] - min_corner[:2]) * 1e3
    return {
        "min_corner_table": min_corner,
        "max_corner_table": max_corner,
        "size_x_mm": float(size_xy_mm[0]),
        "size_y_mm": float(size_xy_mm[1]),
        "top_z_mm": float(top_z_m * 1e3),
    }


def build_box_lineset(min_corner: np.ndarray, max_corner: np.ndarray) -> o3d.geometry.LineSet:
    x0, y0, z0 = min_corner.tolist()
    x1, y1, z1 = max_corner.tolist()
    corners = np.array(
        [
            [x0, y0, z0],
            [x1, y0, z0],
            [x1, y1, z0],
            [x0, y1, z0],
            [x0, y0, z1],
            [x1, y0, z1],
            [x1, y1, z1],
            [x0, y1, z1],
        ],
        dtype=np.float64,
    )
    lines = np.array(
        [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7],
        ],
        dtype=np.int32,
    )
    colors = np.tile(np.asarray(box_color, dtype=np.float64)[None, :], (lines.shape[0], 1))
    box = o3d.geometry.LineSet()
    box.points = o3d.utility.Vector3dVector(corners)
    box.lines = o3d.utility.Vector2iVector(lines)
    box.colors = o3d.utility.Vector3dVector(colors)
    return box


def prepare_box_for_viz(box_info: Dict[str, object], T_world_from_table: np.ndarray, viz_frame_local: str) -> o3d.geometry.LineSet:
    min_corner = np.asarray(box_info["min_corner_table"], dtype=np.float64)
    max_corner = np.asarray(box_info["max_corner_table"], dtype=np.float64)
    box = build_box_lineset(min_corner, max_corner)
    if viz_frame_local == "world":
        box.transform(T_world_from_table)
    return box


def prepare_mesh_for_viz(
    mesh_world: o3d.geometry.TriangleMesh,
    T_table_from_world: np.ndarray,
    T_world_from_table: np.ndarray,
    viz_frame_local: str,
) -> o3d.geometry.TriangleMesh:
    mesh_viz = o3d.geometry.TriangleMesh(mesh_world)
    mesh_viz.transform(T_table_from_world)
    mesh_viz = crop_mesh_above_table(mesh_viz, mm_to_m(z_min_above_plane_mm))
    if viz_frame_local == "world":
        mesh_viz.transform(T_world_from_table)
    mesh_viz.paint_uniform_color(list(mesh_color))
    return mesh_viz


def normalize_for_vis(img: np.ndarray, mask: np.ndarray | None = None) -> np.ndarray:
    vals = img[mask] if mask is not None and np.any(mask) else img.ravel()
    if vals.size == 0:
        return np.zeros_like(img, dtype=np.uint8)
    vmin = float(np.min(vals))
    vmax = float(np.max(vals))
    if vmax - vmin < 1e-9:
        return np.zeros_like(img, dtype=np.uint8)
    return ((img - vmin) / (vmax - vmin) * 255.0).clip(0, 255).astype(np.uint8)


def show_2d_debug(H: np.ndarray, mask: np.ndarray, DT_mm: np.ndarray, peaks_yx: np.ndarray) -> None:
    height_vis = cv2.applyColorMap(normalize_for_vis(H), cv2.COLORMAP_TURBO)
    mask_vis = np.repeat((mask.astype(np.uint8) * 255)[:, :, None], 3, axis=2)
    peaks_vis = cv2.applyColorMap(normalize_for_vis(DT_mm, mask), cv2.COLORMAP_INFERNO)
    for y, x in peaks_yx:
        cv2.circle(peaks_vis, (int(x), int(y)), 3, (0, 255, 255), -1, lineType=cv2.LINE_AA)
        cv2.circle(peaks_vis, (int(x), int(y)), 5, (0, 0, 0), 1, lineType=cv2.LINE_AA)

    for name, img in {
        "top_points_height_map": height_vis,
        "top_points_mask": mask_vis,
        "top_points_distance_transform": cv2.applyColorMap(normalize_for_vis(DT_mm, mask), cv2.COLORMAP_INFERNO),
        "top_points_detected_peaks": peaks_vis,
    }.items():
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.imshow(name, img)

    cv2.waitKey(int(pause_ms) if pause_ms >= 0 else 0)


def show_3d_debug(
    points_xyz: np.ndarray,
    peaks_xyz: np.ndarray,
    mesh_viz: o3d.geometry.TriangleMesh,
    box_viz: o3d.geometry.LineSet | None = None,
) -> None:
    geoms = [mesh_viz, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02)]
    if box_viz is not None:
        geoms.append(box_viz)

    if show_point_cloud_background:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_xyz.astype(np.float64))
        pcd.paint_uniform_color([0.75, 0.75, 0.75])
        geoms.insert(0, pcd)

    if peaks_xyz.shape[0] > 0:
        r_m = mm_to_m(peak_marker_radius_mm)
        peak_cloud = o3d.geometry.PointCloud()
        peak_cloud.points = o3d.utility.Vector3dVector(peaks_xyz.astype(np.float64))
        peak_cloud.paint_uniform_color([1.0, 0.0, 0.0])
        geoms.append(peak_cloud)
        for p in peaks_xyz:
            outer = o3d.geometry.TriangleMesh.create_sphere(radius=r_m)
            outer.compute_vertex_normals()
            outer.paint_uniform_color([1.0, 0.95, 0.15])
            outer.translate(p)
            geoms.append(outer)

            inner = o3d.geometry.TriangleMesh.create_sphere(radius=0.55 * r_m)
            inner.compute_vertex_normals()
            inner.paint_uniform_color([1.0, 0.05, 0.05])
            inner.translate(p)
            geoms.append(inner)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Mesh Top Points", width=1600, height=960)
    for g in geoms:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    if opt is not None:
        opt.background_color = np.array([0.05, 0.05, 0.05], dtype=np.float64)
        opt.point_size = 4.0
        opt.light_on = True
    vis.run()
    vis.destroy_window()


def run() -> Dict[str, object]:
    if viz_frame not in {"table", "world"}:
        raise ValueError("viz_frame must be 'table' or 'world'.")

    mesh_world, points_world, mesh_path = load_mesh_vertices(mesh_input_path)
    points_table, T_table_from_world, T_world_from_table, plane = fit_table_frame(points_world)

    points_obj_table = points_table[points_table[:, 2] > mm_to_m(z_min_above_plane_mm)]
    if points_obj_table.shape[0] == 0:
        raise RuntimeError("No mesh vertices remain above table threshold; lower z_min_above_plane_mm.")

    H, valid, min_xy, grid_m = build_height_map(points_obj_table)
    mask, DT_mm = build_mask_and_dt(H, valid)
    peaks_yx, peak_info = detect_peaks(H, DT_mm, mask, valid)
    peaks_xyz_grid = peaks_to_xyz(peaks_yx, H, min_xy, grid_m)
    peaks_xyz_table, anchor_info = anchor_peaks(peaks_xyz_grid, points_obj_table)
    peaks_xyz_world = transform_points(peaks_xyz_table, T_world_from_table) if peaks_xyz_table.shape[0] else np.empty((0, 3))
    box_info = build_bounding_box_info(points_obj_table)

    print(f"Mesh: {mesh_path}")
    print(f"Vertices loaded: {points_world.shape[0]}")
    print(f"Vertices above table: {points_obj_table.shape[0]}")
    print(f"Top points detected: {peaks_xyz_table.shape[0]}")
    print(
        "Bounding box (table frame): "
        f"size_x={box_info['size_x_mm']:.3f} mm, "
        f"size_y={box_info['size_y_mm']:.3f} mm, "
        f"top_z={box_info['top_z_mm']:.3f} mm"
    )
    print(
        "Peak filter: "
        f"support={int(peak_info['num_support_cells'])}, "
        f"local={int(peak_info['num_local_maxima'])}, "
        f"after_filter={int(peak_info['num_after_filter'])}, "
        f"after_nms={int(peak_info['num_after_nms'])}, "
        f"min_h={peak_info['min_height_thr']:.3f} mm, "
        f"min_prom={peak_info['min_prominence_thr']:.3f} mm, "
        f"eff_prom={peak_info['effective_prominence_thr']:.3f} mm, "
        f"max_prom={peak_info['max_prominence_mm']:.3f} mm"
    )
    print(
        "Anchor: "
        f"snapped={int(anchor_info['num_snapped'])}, "
        f"mean_z_err={anchor_info['mean_z_error_mm']:.3f} mm, "
        f"max_z_err={anchor_info['max_z_error_mm']:.3f} mm"
    )
    for i, p in enumerate(peaks_xyz_table[: min(preview_peak_count, peaks_xyz_table.shape[0])], start=1):
        print(f"  {i:02d}: x={p[0]:.6f} m, y={p[1]:.6f} m, z={p[2]:.6f} m")

    points_viz = points_obj_table
    peaks_viz = peaks_xyz_table
    if viz_frame == "world":
        points_viz = transform_points(points_obj_table, T_world_from_table)
        peaks_viz = peaks_xyz_world

    if show_2d:
        show_2d_debug(H, mask, DT_mm, peaks_yx)
    if show_3d:
        box_viz = prepare_box_for_viz(box_info, T_world_from_table, viz_frame) if show_bounding_box else None
        show_3d_debug(
            points_viz,
            peaks_viz,
            prepare_mesh_for_viz(mesh_world, T_table_from_world, T_world_from_table, viz_frame),
            box_viz=box_viz,
        )

    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass

    return {
        "mesh_input_path": str(mesh_path),
        "num_vertices_loaded": int(points_world.shape[0]),
        "num_vertices_above_table": int(points_obj_table.shape[0]),
        "num_top_points": int(peaks_xyz_table.shape[0]),
        "plane_model_world": plane.tolist(),
        "bounding_box_table": {
            "min_corner": np.asarray(box_info["min_corner_table"], dtype=np.float64).tolist(),
            "max_corner": np.asarray(box_info["max_corner_table"], dtype=np.float64).tolist(),
            "size_x_mm": float(box_info["size_x_mm"]),
            "size_y_mm": float(box_info["size_y_mm"]),
            "top_z_mm": float(box_info["top_z_mm"]),
        },
        "top_points_xyz_table": peaks_xyz_table.tolist(),
        "top_points_xyz_world": peaks_xyz_world.tolist(),
    }


if __name__ == "__main__":
    run()
