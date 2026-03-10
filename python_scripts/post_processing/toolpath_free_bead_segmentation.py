#!/usr/bin/env python3
"""
Toolpath-free bead debug pipeline:
- Load point cloud
- Fit table plane and move to table frame
- Build 2.5D height map
- Build mask + distance transform
- Detect bead peaks from the height map
- Label points with heat-method geodesic ownership
- Transfer labels to the TSDF mesh for visualization
"""

from __future__ import annotations

import os
from pathlib import Path
import sys
from typing import Dict, List, Sequence, Tuple

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

try:
    import potpourri3d as pp3d
except Exception:
    pp3d = None

# -----------------------------------------------------------------------------
# In-script parameters
# -----------------------------------------------------------------------------

default_input_path = "~/Downloads/260227_160709/print_scan_014/reconstruct/tsdf_surface_rgb_colored.ply"
default_mesh_input_path = ""
default_show_3d = True
default_show_2d = False
default_viz_frame = "table"  # "table" or "world"
default_pause_ms = 0  # 0 -> wait key, >0 -> auto-advance
open3d_random_seed = 7


# -----------------------------------------------------------------------------
# Stage-1 parameters (all distances in millimeters)
# -----------------------------------------------------------------------------
downsample_voxel_mm = 0  # <= 0 disables downsampling

# Table alignment / slicing
table_ransac_thresh_mm = 1.0
table_ransac_n = 3
table_ransac_iters = 2500
z_min_above_plane_mm = 1.9

# Height map / mask / peak detection
grid_mm = 0.10
height_percentile = 99.0
gaussian_sigma_px = 0.2
mask_percentile = 50.0
morph_kernel_px = 1
peak_height_support_percentile = 45.0
peak_min_height_mm = 1.0
min_peak_distance_mm = 4
peak_strength_fraction = 0.10
peak_keep_percentile = 0.0  # keep only top DT peaks by value
max_num_peaks = 1500

# 3D display
peak_marker_radius_mm = 1.6
label_background_color = (0.60, 0.60, 0.60)

# Peak-to-point anchoring
peak_anchor_mode = "nearest3d"  # "grid" or "nearest3d"
peak_max_snap_z_error_mm = 1.5

# Heat/geodesic labeling
geodesic_heat_t_coef = 1.2
geodesic_max_seed_count = 0  # 0 -> use all detected peak seeds

# Mesh label transfer onto TSDF mesh
mesh_enable = True
mesh_auto_filename = "new_geometry_mesh_kept.stl"
mesh_vertex_label_max_dist_mm = 1.5
mesh_min_vertices_per_label = 20
mesh_min_triangles_per_label = 100
mesh_show_point_cloud_background = False


# -----------------------------------------------------------------------------
# Utilities
# -----------------------------------------------------------------------------
def _mm_to_m(v_mm: float) -> float:
    return float(v_mm) * 1e-3


def _ensure_odd(k: int) -> int:
    k = int(k)
    if k < 1:
        return 1
    return k if (k % 2 == 1) else (k + 1)


def _normalize_plane(plane: Sequence[float]) -> np.ndarray:
    p = np.asarray(plane, dtype=np.float64).reshape(4)
    n = np.linalg.norm(p[:3])
    if n < 1e-12:
        raise RuntimeError("Degenerate table plane model.")
    return p / n


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


def _load_points(input_path: Path) -> np.ndarray:
    pcd = o3d.io.read_point_cloud(str(input_path))
    if len(pcd.points) == 0:
        raise RuntimeError(f"Input point cloud is empty: {input_path}")

    pts = np.asarray(pcd.points, dtype=np.float64)
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if pts.shape[0] == 0:
        raise RuntimeError("Input point cloud has no finite XYZ points.")

    if downsample_voxel_mm > 0.0:
        tmp = o3d.geometry.PointCloud()
        tmp.points = o3d.utility.Vector3dVector(pts)
        tmp = tmp.voxel_down_sample(voxel_size=_mm_to_m(downsample_voxel_mm))
        pts = np.asarray(tmp.points, dtype=np.float64)

    if pts.shape[0] == 0:
        raise RuntimeError("No points remain after optional downsampling.")
    return pts


def _resolve_mesh_input_path(input_path: Path, mesh_input_path: str | os.PathLike[str] | None) -> Path | None:
    if mesh_input_path is not None and str(mesh_input_path).strip():
        mesh_path = Path(mesh_input_path).expanduser().resolve()
        if not mesh_path.exists():
            raise FileNotFoundError(f"Mesh input file not found: {mesh_path}")
        return mesh_path

    auto_path = (input_path.parent / str(mesh_auto_filename)).expanduser().resolve()
    if auto_path.exists():
        return auto_path
    return None


def _load_triangle_mesh(mesh_path: Path) -> o3d.geometry.TriangleMesh:
    mesh = o3d.io.read_triangle_mesh(str(mesh_path), enable_post_processing=True)
    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        raise RuntimeError(f"Input mesh is empty or invalid: {mesh_path}")
    return mesh


def _fit_table_frame(points_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if hasattr(o3d.utility, "random") and hasattr(o3d.utility.random, "seed"):
        o3d.utility.random.seed(int(open3d_random_seed))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_world)

    plane_model, _ = pcd.segment_plane(
        distance_threshold=_mm_to_m(table_ransac_thresh_mm),
        ransac_n=int(table_ransac_n),
        num_iterations=int(table_ransac_iters),
    )
    plane = _normalize_plane(plane_model)

    # Prefer orientation with most points above plane.
    signed = points_world @ plane[:3] + plane[3]
    if np.median(signed) < 0.0:
        plane = -plane

    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    R = _rotation_matrix_from_a_to_b(plane[:3], up)

    # Rotate + translate so table becomes z=0.
    t = np.array([0.0, 0.0, plane[3]], dtype=np.float64)

    T_table_from_world = np.eye(4, dtype=np.float64)
    T_table_from_world[:3, :3] = R
    T_table_from_world[:3, 3] = t

    T_world_from_table = np.eye(4, dtype=np.float64)
    T_world_from_table[:3, :3] = R.T
    T_world_from_table[:3, 3] = -R.T @ t

    points_table = _transform_points(points_world, T_table_from_world)
    return points_table, T_table_from_world, T_world_from_table, plane


def _nearest_fill_height(H: np.ndarray, valid: np.ndarray) -> np.ndarray:
    if np.all(valid):
        return H.copy()
    if not np.any(valid):
        raise RuntimeError("Height map has no valid cells.")

    if ndi is not None:
        nearest_idx = ndi.distance_transform_edt(~valid, return_distances=False, return_indices=True)
        return H[tuple(nearest_idx)]

    filled = H.copy()
    cur = valid.copy()
    kernel = np.ones((3, 3), dtype=np.uint8)
    fallback = float(np.nanmean(H[valid]))
    for _ in range(max(H.shape) * 2):
        if np.all(cur):
            break
        candidate = cv2.dilate(np.where(cur, filled, fallback).astype(np.float32), kernel)
        new_cells = ~cur
        filled[new_cells] = candidate[new_cells]
        cur[new_cells] = True
    return filled


def _weighted_gaussian(H: np.ndarray, valid: np.ndarray, sigma_px: float) -> np.ndarray:
    if sigma_px <= 0.0:
        return H.copy()

    Hf = H.astype(np.float32)
    W = valid.astype(np.float32)

    if ndi is not None:
        Hw = ndi.gaussian_filter(Hf * W, sigma=sigma_px, mode="nearest")
        Wb = ndi.gaussian_filter(W, sigma=sigma_px, mode="nearest")
        out = Hf.copy()
        good = Wb > 1e-6
        out[good] = Hw[good] / Wb[good]
        return out

    k = _ensure_odd(max(3, int(round(6.0 * sigma_px))))
    Hw = cv2.GaussianBlur(Hf * W, (k, k), sigmaX=sigma_px, sigmaY=sigma_px, borderType=cv2.BORDER_REPLICATE)
    Wb = cv2.GaussianBlur(W, (k, k), sigmaX=sigma_px, sigmaY=sigma_px, borderType=cv2.BORDER_REPLICATE)
    out = Hf.copy()
    good = Wb > 1e-6
    out[good] = Hw[good] / Wb[good]
    return out


def _build_height_map(
    points_table: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float, Dict[Tuple[int, int], np.ndarray]]:
    xy = points_table[:, :2]
    z = points_table[:, 2]

    g = _mm_to_m(grid_mm)
    min_xy = xy.min(axis=0)
    max_xy = xy.max(axis=0)

    nx = max(1, int(np.floor((max_xy[0] - min_xy[0]) / g)) + 1)
    ny = max(1, int(np.floor((max_xy[1] - min_xy[1]) / g)) + 1)

    ix = np.clip(np.floor((xy[:, 0] - min_xy[0]) / g).astype(np.int32), 0, nx - 1)
    iy = np.clip(np.floor((xy[:, 1] - min_xy[1]) / g).astype(np.int32), 0, ny - 1)
    linear = iy * nx + ix

    order = np.argsort(linear, kind="mergesort")
    linear_sorted = linear[order]
    splits = np.flatnonzero(np.diff(linear_sorted)) + 1
    groups = np.split(order, splits)

    H = np.full((ny, nx), np.nan, dtype=np.float32)
    valid = np.zeros((ny, nx), dtype=bool)
    cell_point_indices: Dict[Tuple[int, int], np.ndarray] = {}

    for grp in groups:
        lin = int(linear[grp[0]])
        y = lin // nx
        x = lin % nx
        H[y, x] = float(np.percentile(z[grp], float(height_percentile)))
        valid[y, x] = True
        # Indices are aligned to points_table (points_obj_table in the caller).
        cell_point_indices[(int(y), int(x))] = np.asarray(grp, dtype=np.int32)

    H = _nearest_fill_height(H, valid)
    H = _weighted_gaussian(H, valid, sigma_px=float(gaussian_sigma_px)).astype(np.float32)
    num_valid_cells = int(np.count_nonzero(valid))
    num_mapped_cells = int(len(cell_point_indices))
    total_mapped_points = int(sum(v.size for v in cell_point_indices.values()))
    avg_points_per_valid = (
        float(total_mapped_points) / float(num_valid_cells) if num_valid_cells > 0 else 0.0
    )
    print(
        "  Height-map cell mapping: "
        f"valid_cells={num_valid_cells}, "
        f"mapped_cells={num_mapped_cells}, "
        f"avg_points_per_valid_cell={avg_points_per_valid:.2f}"
    )
    return H, valid, min_xy.astype(np.float64), g, cell_point_indices


def _build_mask_and_dt(H: np.ndarray, valid: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    vals = H[valid]
    if vals.size == 0:
        raise RuntimeError("No valid height cells for mask creation.")

    th = float(np.percentile(vals, float(mask_percentile)))
    M = (H > th) & valid

    if not np.any(M):
        th = float(np.percentile(vals, 90.0))
        M = (H >= th) & valid

    k = _ensure_odd(max(1, int(morph_kernel_px)))
    if k > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        Mu8 = M.astype(np.uint8)
        Mu8 = cv2.morphologyEx(Mu8, cv2.MORPH_CLOSE, kernel, iterations=1)
        Mu8 = cv2.morphologyEx(Mu8, cv2.MORPH_OPEN, kernel, iterations=1)
        M = Mu8.astype(bool)

    # Keep mask constrained to cells that have real support in the original grid.
    M = M & valid

    if not np.any(M):
        M = valid.copy()

    if ndi is not None:
        dt_px = ndi.distance_transform_edt(M).astype(np.float32)
    else:
        dt_px = cv2.distanceTransform(M.astype(np.uint8), cv2.DIST_L2, 5)

    DT_mm = dt_px * float(grid_mm)
    return M, DT_mm.astype(np.float32), th


def _local_maxima(arr: np.ndarray, support_mask: np.ndarray, radius_px: int) -> np.ndarray:
    r = max(1, int(radius_px))
    size = 2 * r + 1
    if ndi is not None:
        mx = ndi.maximum_filter(arr, size=size, mode="nearest")
    else:
        mx = cv2.dilate(arr.astype(np.float32), np.ones((size, size), dtype=np.uint8))
    return (arr >= (mx - 1e-6)) & support_mask


def _nms(coords: np.ndarray, scores: np.ndarray, min_dist_px: int) -> np.ndarray:
    if coords.size == 0:
        return coords
    if coords.shape[0] == 1:
        return coords

    order = np.argsort(-scores, kind="mergesort")
    keep: List[np.ndarray] = []
    d2_min = float(min_dist_px * min_dist_px)

    for i in order:
        c = coords[i]
        if not keep:
            keep.append(c)
            continue
        d2 = np.sum((np.asarray(keep) - c) ** 2, axis=1)
        if np.all(d2 >= d2_min):
            keep.append(c)
    return np.asarray(keep, dtype=np.int32)


def _detect_peaks(H: np.ndarray, DT_mm: np.ndarray, mask: np.ndarray, valid: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
    source_mm = H.astype(np.float32) * 1e3
    support = (mask & valid).copy()
    if np.any(support):
        hot_th = float(np.percentile(source_mm[support], float(peak_height_support_percentile)))
        support &= source_mm >= hot_th

    if not np.any(support):
        return np.empty((0, 2), dtype=np.int32), {
            "mode": "height",
            "num_local_maxima": 0.0,
            "num_after_strength": 0.0,
            "num_after_nms": 0.0,
            "num_after_cap": 0.0,
            "num_support_cells": 0.0,
            "min_dist_px": 0.0,
            "strong_thr": 0.0,
            "max_dt_mm": 0.0,
            "max_height_mm": 0.0,
        }

    min_dist_px = max(1, int(round(float(min_peak_distance_mm) / float(grid_mm))))
    lm = _local_maxima(source_mm, support, min_dist_px)
    coords = np.column_stack(np.nonzero(lm))
    n_local = int(coords.shape[0])
    n_support = int(np.count_nonzero(support))
    if coords.size == 0:
        peak = np.unravel_index(int(np.argmax(source_mm * support.astype(np.float32))), source_mm.shape)
        p = np.asarray([[int(peak[0]), int(peak[1])]], dtype=np.int32)
        return p, {
            "mode": "height",
            "num_local_maxima": 0.0,
            "num_after_strength": 1.0,
            "num_after_nms": 1.0,
            "num_after_cap": 1.0,
            "num_support_cells": float(n_support),
            "min_dist_px": float(min_dist_px),
            "strong_thr": float(source_mm[peak]),
            "max_dt_mm": float(np.max(DT_mm[support])) if np.any(support) else 0.0,
            "max_height_mm": float(np.max((H * 1e3)[support])) if np.any(support) else 0.0,
        }

    vals = source_mm[coords[:, 0], coords[:, 1]]
    strong_abs = float(peak_strength_fraction) * float(vals.max())
    strong_pct = float(np.percentile(vals, float(peak_keep_percentile)))
    strong_thr = max(strong_abs, strong_pct, float(peak_min_height_mm))
    strong = vals >= strong_thr
    coords = coords[strong]
    vals = vals[strong]
    n_strong = int(coords.shape[0])

    if coords.size == 0:
        peak = np.unravel_index(int(np.argmax(source_mm * support.astype(np.float32))), source_mm.shape)
        p = np.asarray([[int(peak[0]), int(peak[1])]], dtype=np.int32)
        return p, {
            "mode": "height",
            "num_local_maxima": float(n_local),
            "num_after_strength": 1.0,
            "num_after_nms": 1.0,
            "num_after_cap": 1.0,
            "num_support_cells": float(n_support),
            "min_dist_px": float(min_dist_px),
            "strong_thr": float(strong_thr),
            "max_dt_mm": float(np.max(DT_mm[support])) if np.any(support) else 0.0,
            "max_height_mm": float(np.max((H * 1e3)[support])) if np.any(support) else 0.0,
        }

    coords = _nms(coords, vals, min_dist_px)
    n_nms = int(coords.shape[0])
    if coords.shape[0] > int(max_num_peaks):
        vals_nms = source_mm[coords[:, 0], coords[:, 1]]
        keep_order = np.argsort(-vals_nms, kind="mergesort")[: int(max_num_peaks)]
        coords = coords[keep_order]

    n_cap = int(coords.shape[0])
    coords = coords[np.lexsort((coords[:, 1], coords[:, 0]))]
    return coords.astype(np.int32), {
        "mode": "height",
        "num_local_maxima": float(n_local),
        "num_after_strength": float(n_strong),
        "num_after_nms": float(n_nms),
        "num_after_cap": float(n_cap),
        "num_support_cells": float(n_support),
        "min_dist_px": float(min_dist_px),
        "strong_thr": float(strong_thr),
        "max_dt_mm": float(np.max(DT_mm[support])) if np.any(support) else 0.0,
        "max_height_mm": float(np.max((H * 1e3)[support])) if np.any(support) else 0.0,
    }


def _peaks_to_xyz(peaks_yx: np.ndarray, H: np.ndarray, min_xy: np.ndarray, grid_m: float) -> np.ndarray:
    if peaks_yx.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    y = peaks_yx[:, 0].astype(np.float64)
    x = peaks_yx[:, 1].astype(np.float64)

    px = min_xy[0] + (x + 0.5) * grid_m
    py = min_xy[1] + (y + 0.5) * grid_m
    pz = H[peaks_yx[:, 0], peaks_yx[:, 1]].astype(np.float64)
    return np.column_stack([px, py, pz])


def _build_peak_markers(peaks_yx: np.ndarray, shape_hw: Tuple[int, int]) -> Tuple[np.ndarray, Dict[str, int]]:
    h, w = int(shape_hw[0]), int(shape_hw[1])
    markers = np.zeros((h, w), dtype=np.int32)

    if peaks_yx.size == 0:
        return markers, {
            "num_detected_peaks": 0,
            "num_markers_placed": 0,
            "num_duplicates_ignored": 0,
            "num_out_of_bounds": 0,
        }

    peaks_int = np.asarray(peaks_yx, dtype=np.int32)
    duplicates = 0
    out_of_bounds = 0
    placed = 0
    next_label = 1
    seen_cells = set()

    for yx in peaks_int:
        y = int(yx[0])
        x = int(yx[1])
        if y < 0 or y >= h or x < 0 or x >= w:
            out_of_bounds += 1
            continue
        key = (y, x)
        if key in seen_cells:
            duplicates += 1
            continue
        seen_cells.add(key)
        markers[y, x] = next_label
        next_label += 1
        placed += 1

    return markers, {
        "num_detected_peaks": int(peaks_int.shape[0]),
        "num_markers_placed": int(placed),
        "num_duplicates_ignored": int(duplicates),
        "num_out_of_bounds": int(out_of_bounds),
    }


def _seed_vertex_indices(points_xyz: np.ndarray, seed_xyz: np.ndarray) -> np.ndarray:
    if points_xyz.shape[0] == 0 or seed_xyz.shape[0] == 0:
        return np.empty((0,), dtype=np.int32)
    if cKDTree is not None:
        tree = cKDTree(points_xyz)
        _, seed_idx = tree.query(seed_xyz, k=1)
        return np.unique(np.asarray(seed_idx, dtype=np.int32))

    seed_idx: List[int] = []
    for s in seed_xyz:
        d2 = np.sum((points_xyz - s[None, :]) ** 2, axis=1)
        seed_idx.append(int(np.argmin(d2)))
    return np.unique(np.asarray(seed_idx, dtype=np.int32))


def _compute_seeded_heat_fields(points_xyz: np.ndarray, seed_xyz: np.ndarray) -> Dict[str, object]:
    n = int(points_xyz.shape[0])
    info: Dict[str, object] = {
        "backend_used": "heat",
        "num_points": float(n),
        "num_seeds": float(seed_xyz.shape[0]),
        "num_seed_vertices": 0.0,
        "num_seed_solves": 0.0,
        "num_reachable": 0.0,
        "num_unreachable": float(n),
        "mean_mm": 0.0,
        "max_mm": 0.0,
    }
    fields: Dict[str, object] = {
        "dist_min_mm": np.full((n,), np.inf, dtype=np.float64),
        "dist_by_seed_mm": np.empty((0, n), dtype=np.float64),
        "owner_seed": np.full((n,), -1, dtype=np.int32),
        "seed_vertex_idx": np.empty((0,), dtype=np.int32),
        "info": info,
    }
    if n == 0:
        fields["dist_min_mm"] = np.empty((0,), dtype=np.float64)
        fields["owner_seed"] = np.empty((0,), dtype=np.int32)
        return fields
    if seed_xyz.shape[0] == 0:
        return fields
    if pp3d is None:
        info["error"] = "potpourri3d_not_available"
        return fields

    seed_idx = _seed_vertex_indices(points_xyz, seed_xyz)
    if seed_idx.size == 0:
        return fields

    max_seeds = int(geodesic_max_seed_count)
    if max_seeds > 0 and seed_idx.size > max_seeds:
        z_seed = points_xyz[seed_idx, 2]
        keep = np.argsort(-z_seed, kind="mergesort")[:max_seeds]
        seed_idx = seed_idx[keep]
    info["num_seed_vertices"] = float(seed_idx.size)
    fields["seed_vertex_idx"] = seed_idx.astype(np.int32)

    try:
        solver = pp3d.PointCloudHeatSolver(points_xyz.astype(np.float64), float(geodesic_heat_t_coef))
        per_seed = np.empty((seed_idx.size, n), dtype=np.float64)
        for i, s in enumerate(seed_idx.tolist()):
            dist_i = np.asarray(solver.compute_distance(int(s)), dtype=np.float64).reshape(-1)
            if dist_i.shape[0] != n:
                raise RuntimeError(f"Unexpected heat distance shape for seed {s}: {dist_i.shape}")
            dist_i *= 1e3
            dist_i = np.maximum(dist_i, 0.0)
            dist_i[int(s)] = 0.0
            per_seed[i] = dist_i

        dist_min = np.min(per_seed, axis=0)
        owner = np.argmin(per_seed, axis=0).astype(np.int32)
        finite = np.isfinite(dist_min)
        owner[~finite] = -1

        fields["dist_by_seed_mm"] = per_seed
        fields["dist_min_mm"] = dist_min
        fields["owner_seed"] = owner

        info["num_seed_solves"] = float(seed_idx.size)
        info["num_reachable"] = float(np.count_nonzero(finite))
        info["num_unreachable"] = float(np.count_nonzero(~finite))
        info["mean_mm"] = float(np.mean(dist_min[finite])) if np.any(finite) else 0.0
        info["max_mm"] = float(np.max(dist_min[finite])) if np.any(finite) else 0.0
        return fields
    except Exception as exc:
        info["error"] = str(exc)
        return fields


def _anchor_peaks_to_points(peaks_xyz_grid: np.ndarray, points_xyz: np.ndarray) -> Tuple[np.ndarray, Dict[str, float]]:
    if peaks_xyz_grid.shape[0] == 0:
        return peaks_xyz_grid.copy(), {
            "num_peaks": 0.0,
            "num_snapped": 0.0,
            "num_rejected_z_guard": 0.0,
            "mean_dist3d_mm": 0.0,
            "max_dist3d_mm": 0.0,
            "mean_z_error_mm": 0.0,
            "max_z_error_mm": 0.0,
        }
    if points_xyz.shape[0] == 0 or str(peak_anchor_mode).lower() == "grid":
        return peaks_xyz_grid.copy(), {
            "num_peaks": float(peaks_xyz_grid.shape[0]),
            "num_snapped": 0.0,
            "num_rejected_z_guard": 0.0,
            "mean_dist3d_mm": 0.0,
            "max_dist3d_mm": 0.0,
            "mean_z_error_mm": 0.0,
            "max_z_error_mm": 0.0,
        }

    if cKDTree is not None:
        tree = cKDTree(points_xyz)
        dist3d, nn = tree.query(peaks_xyz_grid, k=1)
        nn = np.asarray(nn, dtype=np.int32)
        dist3d = np.asarray(dist3d, dtype=np.float64)
        snapped = points_xyz[nn]
    else:
        snapped = np.empty_like(peaks_xyz_grid, dtype=np.float64)
        dist3d = np.empty((peaks_xyz_grid.shape[0],), dtype=np.float64)
        for i, p in enumerate(peaks_xyz_grid):
            d2 = np.sum((points_xyz - p[None, :]) ** 2, axis=1)
            j = int(np.argmin(d2))
            snapped[i] = points_xyz[j]
            dist3d[i] = float(np.sqrt(d2[j]))

    z_err_mm = np.abs(snapped[:, 2] - peaks_xyz_grid[:, 2]) * 1e3
    z_guard = z_err_mm <= float(peak_max_snap_z_error_mm)

    # If nearest 3D point still has too large Z error, keep grid peak directly.
    anchored = snapped.copy()
    anchored[~z_guard] = peaks_xyz_grid[~z_guard]

    return anchored, {
        "num_peaks": float(peaks_xyz_grid.shape[0]),
        "num_snapped": float(np.count_nonzero(z_guard)),
        "num_rejected_z_guard": float(np.count_nonzero(~z_guard)),
        "mean_dist3d_mm": float(np.mean(dist3d) * 1e3),
        "max_dist3d_mm": float(np.max(dist3d) * 1e3),
        "mean_z_error_mm": float(np.mean(z_err_mm)),
        "max_z_error_mm": float(np.max(z_err_mm)),
    }


def _normalize_for_vis(img: np.ndarray, mask: np.ndarray | None = None) -> np.ndarray:
    vals = img[mask] if (mask is not None and np.any(mask)) else img.ravel()
    if vals.size == 0:
        return np.zeros_like(img, dtype=np.uint8)
    vmin = float(np.min(vals))
    vmax = float(np.max(vals))
    if vmax - vmin < 1e-9:
        return np.zeros_like(img, dtype=np.uint8)
    return ((img - vmin) / (vmax - vmin) * 255.0).clip(0, 255).astype(np.uint8)


def _labels_to_colors(labels: np.ndarray) -> np.ndarray:
    n = int(labels.shape[0])
    colors = np.tile(np.asarray(label_background_color, dtype=np.float64)[None, :], (n, 1))
    if n == 0:
        return colors

    labels_i = np.asarray(labels, dtype=np.int32)
    unique_labels = np.unique(labels_i[labels_i > 0])
    for lab in unique_labels:
        hue = int((47 * int(lab) + 23) % 180)
        color_hsv = np.array([[[hue, 240, 255]]], dtype=np.uint8)
        color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0, 0].astype(np.float64) / 255.0
        colors[labels_i == int(lab)] = color_bgr[::-1]
    return colors


def _summarize_point_labels(
    points_xyz: np.ndarray,
    point_labels: np.ndarray,
    cell_point_indices: Dict[Tuple[int, int], np.ndarray],
) -> List[Dict[str, float]]:
    labels = np.asarray(point_labels, dtype=np.int32)
    points = np.asarray(points_xyz, dtype=np.float64)
    unique_labels = np.unique(labels[labels > 0])
    if unique_labels.size == 0:
        return []

    cell_counts: Dict[int, int] = {}
    for _, point_idx in cell_point_indices.items():
        if point_idx.size == 0:
            continue
        cell_labels = np.unique(labels[np.asarray(point_idx, dtype=np.int32)])
        cell_labels = cell_labels[cell_labels > 0]
        for lab in cell_labels:
            lab_i = int(lab)
            cell_counts[lab_i] = cell_counts.get(lab_i, 0) + 1

    summary: List[Dict[str, float]] = []
    for lab in unique_labels.tolist():
        lab_i = int(lab)
        mask = labels == lab_i
        if not np.any(mask):
            continue
        z_mm = points[mask, 2] * 1e3
        summary.append(
            {
                "label": float(lab_i),
                "point_count": float(np.count_nonzero(mask)),
                "grid_cells": float(cell_counts.get(lab_i, 0)),
                "max_height_mm": float(np.max(z_mm)),
                "mean_height_mm": float(np.mean(z_mm)),
            }
        )

    summary.sort(key=lambda row: int(row["label"]))
    return summary


def _crop_mesh_above_table(
    mesh: o3d.geometry.TriangleMesh,
    z_min_m: float,
) -> o3d.geometry.TriangleMesh:
    mesh_out = o3d.geometry.TriangleMesh(mesh)
    verts = np.asarray(mesh_out.vertices, dtype=np.float64)
    if verts.shape[0] == 0:
        return mesh_out
    remove_mask = verts[:, 2] <= float(z_min_m)
    if np.any(remove_mask):
        mesh_out.remove_vertices_by_mask(remove_mask)
        mesh_out.remove_unreferenced_vertices()
    return mesh_out


def _label_mesh_vertices_from_points(
    mesh_vertices: np.ndarray,
    point_vertices: np.ndarray,
    point_labels: np.ndarray,
) -> Tuple[np.ndarray, Dict[str, float]]:
    vertex_labels = np.zeros((mesh_vertices.shape[0],), dtype=np.int32)
    if mesh_vertices.shape[0] == 0:
        return vertex_labels, {
            "num_mesh_vertices": 0.0,
            "num_labeled_vertices": 0.0,
            "num_unlabeled_vertices": 0.0,
            "mean_snap_dist_mm": 0.0,
            "max_snap_dist_mm": 0.0,
        }

    point_labels_i = np.asarray(point_labels, dtype=np.int32)
    valid = point_labels_i > 0
    if not np.any(valid):
        return vertex_labels, {
            "num_mesh_vertices": float(mesh_vertices.shape[0]),
            "num_labeled_vertices": 0.0,
            "num_unlabeled_vertices": float(mesh_vertices.shape[0]),
            "mean_snap_dist_mm": 0.0,
            "max_snap_dist_mm": 0.0,
        }

    src_points = np.asarray(point_vertices[valid], dtype=np.float64)
    src_labels = point_labels_i[valid]
    if cKDTree is not None:
        tree = cKDTree(src_points)
        dist_m, nn = tree.query(mesh_vertices.astype(np.float64), k=1)
        dist_m = np.asarray(dist_m, dtype=np.float64).reshape(-1)
        nn = np.asarray(nn, dtype=np.int32).reshape(-1)
    else:
        dist_m = np.empty((mesh_vertices.shape[0],), dtype=np.float64)
        nn = np.empty((mesh_vertices.shape[0],), dtype=np.int32)
        for i, q in enumerate(mesh_vertices.astype(np.float64)):
            d2 = np.sum((src_points - q[None, :]) ** 2, axis=1)
            j = int(np.argmin(d2))
            nn[i] = j
            dist_m[i] = float(np.sqrt(d2[j]))

    keep = dist_m <= _mm_to_m(float(mesh_vertex_label_max_dist_mm))
    vertex_labels[keep] = src_labels[nn[keep]]
    labeled = int(np.count_nonzero(vertex_labels > 0))
    unlabeled = int(vertex_labels.shape[0] - labeled)
    return vertex_labels, {
        "num_mesh_vertices": float(mesh_vertices.shape[0]),
        "num_labeled_vertices": float(labeled),
        "num_unlabeled_vertices": float(unlabeled),
        "mean_snap_dist_mm": float(np.mean(dist_m) * 1e3) if dist_m.size > 0 else 0.0,
        "max_snap_dist_mm": float(np.max(dist_m) * 1e3) if dist_m.size > 0 else 0.0,
    }


def _build_submeshes_from_vertex_labels(
    mesh: o3d.geometry.TriangleMesh,
    vertex_labels: np.ndarray,
) -> Tuple[List[o3d.geometry.TriangleMesh], List[Dict[str, float]]]:
    verts = np.asarray(mesh.vertices, dtype=np.float64)
    tris = np.asarray(mesh.triangles, dtype=np.int32)
    if verts.shape[0] == 0 or tris.shape[0] == 0:
        return [], []

    tri_labels = np.zeros((tris.shape[0],), dtype=np.int32)
    tri_vertex_labels = vertex_labels[tris]
    for i in range(tri_vertex_labels.shape[0]):
        labs = tri_vertex_labels[i]
        labs = labs[labs > 0]
        if labs.size < 2:
            continue
        uniq, counts = np.unique(labs, return_counts=True)
        best = int(np.argmax(counts))
        if int(counts[best]) < 2:
            continue
        tri_labels[i] = int(uniq[best])

    meshes: List[o3d.geometry.TriangleMesh] = []
    summary: List[Dict[str, float]] = []
    unique_labels = np.unique(tri_labels[tri_labels > 0])
    for lab in unique_labels.tolist():
        lab_i = int(lab)
        tri_mask = tri_labels == lab_i
        tri_count = int(np.count_nonzero(tri_mask))
        info: Dict[str, float] = {
            "label": float(lab_i),
            "vertex_count": 0.0,
            "triangle_count": float(tri_count),
            "status": 0.0,
        }
        if tri_count < int(mesh_min_triangles_per_label):
            summary.append(info)
            continue

        mesh_lab = o3d.geometry.TriangleMesh(mesh)
        mesh_lab.remove_triangles_by_mask(~tri_mask)
        mesh_lab.remove_unreferenced_vertices()
        if len(mesh_lab.vertices) < int(mesh_min_vertices_per_label) or len(mesh_lab.triangles) == 0:
            summary.append(info)
            continue

        mesh_lab.compute_vertex_normals()
        color = _labels_to_colors(np.array([lab_i], dtype=np.int32))[0]
        mesh_lab.paint_uniform_color(color.tolist())
        info["vertex_count"] = float(len(mesh_lab.vertices))
        info["triangle_count"] = float(len(mesh_lab.triangles))
        info["status"] = 1.0
        meshes.append(mesh_lab)
        summary.append(info)

    summary.sort(key=lambda row: int(row["label"]))
    return meshes, summary


def _build_meshes_from_tsdf_mesh(
    mesh_world: o3d.geometry.TriangleMesh,
    T_table_from_world: np.ndarray,
    points_obj_table: np.ndarray,
    point_labels_table: np.ndarray,
) -> Tuple[List[o3d.geometry.TriangleMesh], List[Dict[str, float]], Dict[str, float], o3d.geometry.TriangleMesh]:
    mesh_table = o3d.geometry.TriangleMesh(mesh_world)
    mesh_table.transform(T_table_from_world)

    source_vertices = int(len(mesh_table.vertices))
    source_triangles = int(len(mesh_table.triangles))
    mesh_table = _crop_mesh_above_table(mesh_table, _mm_to_m(z_min_above_plane_mm))
    cropped_vertices = int(len(mesh_table.vertices))
    cropped_triangles = int(len(mesh_table.triangles))

    vertex_labels, label_info = _label_mesh_vertices_from_points(
        np.asarray(mesh_table.vertices, dtype=np.float64),
        points_obj_table,
        point_labels_table,
    )
    submeshes, summary = _build_submeshes_from_vertex_labels(mesh_table, vertex_labels)
    debug = {
        "source_vertices": float(source_vertices),
        "source_triangles": float(source_triangles),
        "cropped_vertices": float(cropped_vertices),
        "cropped_triangles": float(cropped_triangles),
        "num_mesh_vertices": float(label_info["num_mesh_vertices"]),
        "num_labeled_vertices": float(label_info["num_labeled_vertices"]),
        "num_unlabeled_vertices": float(label_info["num_unlabeled_vertices"]),
        "mean_snap_dist_mm": float(label_info["mean_snap_dist_mm"]),
        "max_snap_dist_mm": float(label_info["max_snap_dist_mm"]),
    }
    return submeshes, summary, debug, mesh_table


def _show_2d_debug(
    H: np.ndarray,
    M: np.ndarray,
    DT_mm: np.ndarray,
    peaks_yx: np.ndarray,
    pause_ms: int,
    markers: np.ndarray | None = None,
) -> None:
    h_vis = cv2.applyColorMap(_normalize_for_vis(H), cv2.COLORMAP_TURBO)
    m_vis = np.repeat((M.astype(np.uint8) * 255)[:, :, None], 3, axis=2)
    dt_vis = cv2.applyColorMap(_normalize_for_vis(DT_mm, M), cv2.COLORMAP_INFERNO)

    peak_overlay = dt_vis.copy()
    for y, x in peaks_yx:
        cv2.circle(peak_overlay, (int(x), int(y)), 3, (0, 255, 255), -1, lineType=cv2.LINE_AA)
        cv2.circle(peak_overlay, (int(x), int(y)), 5, (0, 0, 0), 1, lineType=cv2.LINE_AA)

    marker_overlay = None
    if markers is not None and markers.shape[:2] == DT_mm.shape[:2]:
        marker_overlay = dt_vis.copy()
        ys, xs = np.nonzero(markers > 0)
        labs = markers[ys, xs].astype(np.int32)
        for y, x, lab in zip(ys, xs, labs):
            # Deterministic per-label color in HSV, rendered in BGR for OpenCV.
            hue = int((47 * int(lab) + 23) % 180)
            color_hsv = np.array([[[hue, 255, 255]]], dtype=np.uint8)
            color_bgr = tuple(int(v) for v in cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0, 0].tolist())
            cv2.circle(marker_overlay, (int(x), int(y)), 4, color_bgr, -1, lineType=cv2.LINE_AA)
            cv2.circle(marker_overlay, (int(x), int(y)), 6, (0, 0, 0), 1, lineType=cv2.LINE_AA)
            if int(lab) <= 200:
                cv2.putText(
                    marker_overlay,
                    str(int(lab)),
                    (int(x) + 3, int(y) - 3),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.30,
                    color_bgr,
                    1,
                    cv2.LINE_AA,
                )

    views: Dict[str, np.ndarray] = {
        "stage1_height_map": h_vis,
        "stage1_mask": m_vis,
        "stage1_distance_transform": dt_vis,
        "stage1_detected_peaks": peak_overlay,
    }
    if marker_overlay is not None:
        views["stage1_peak_markers"] = marker_overlay

    for name, img in views.items():
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.imshow(name, img)

    cv2.waitKey(int(pause_ms) if pause_ms >= 0 else 0)


def _show_3d_debug(
    points_xyz: np.ndarray,
    peaks_xyz: np.ndarray,
    point_labels: np.ndarray | None = None,
    meshes: List[o3d.geometry.TriangleMesh] | None = None,
) -> None:
    pcd_main = o3d.geometry.PointCloud()
    pcd_main.points = o3d.utility.Vector3dVector(points_xyz.astype(np.float64))

    if point_labels is not None and point_labels.shape[0] == points_xyz.shape[0]:
        pcd_main.colors = o3d.utility.Vector3dVector(_labels_to_colors(point_labels))
    else:
        base_color = np.tile(np.array([[0.75, 0.75, 0.75]], dtype=np.float64), (points_xyz.shape[0], 1))
        pcd_main.colors = o3d.utility.Vector3dVector(base_color)

    geoms: List[o3d.geometry.Geometry] = [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02)]
    if bool(mesh_show_point_cloud_background) or not meshes:
        geoms.insert(0, pcd_main)
    if meshes:
        geoms.extend(meshes)

    if peaks_xyz.shape[0] > 0:
        r_m = _mm_to_m(float(peak_marker_radius_mm))
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

        peak_cloud = o3d.geometry.PointCloud()
        peak_cloud.points = o3d.utility.Vector3dVector(peaks_xyz.astype(np.float64))
        peak_cloud.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([[1.0, 0.0, 0.0]], dtype=np.float64), (peaks_xyz.shape[0], 1))
        )
        geoms.append(peak_cloud)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Bead Segmentation Debug", width=1600, height=960)
    for g in geoms:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    if opt is not None:
        opt.background_color = np.array([0.05, 0.05, 0.05], dtype=np.float64)
        opt.point_size = 4.0
        opt.light_on = True
        try:
            opt.line_width = 4.0
        except Exception:
            pass
    vis.run()
    vis.destroy_window()


def run_first_stage_peak_debug(
    input_path,
    mesh_input_path=None,
    show_3d=True,
    show_2d=True,
    viz_frame="table",
    pause_ms=0,
) -> Dict[str, object]:
    input_path = Path(input_path).expanduser().resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")
    mesh_input_resolved = _resolve_mesh_input_path(input_path, mesh_input_path)

    viz_frame = str(viz_frame).strip().lower()
    if viz_frame not in {"table", "world"}:
        raise ValueError("viz_frame must be 'table' or 'world'.")

    print(f"[1] Load points: {input_path}")
    points_world = _load_points(input_path)
    print(f"  Points loaded: {points_world.shape[0]}")

    print("[2] Fit table plane + transform to table frame")
    points_table, T_table_from_world, T_world_from_table, plane = _fit_table_frame(points_world)

    keep = points_table[:, 2] > _mm_to_m(z_min_above_plane_mm)
    points_obj_table = points_table[keep]
    if points_obj_table.shape[0] == 0:
        raise RuntimeError("No points remain above table threshold; lower z_min_above_plane_mm.")
    print(f"  Points above table: {points_obj_table.shape[0]}")

    print("[3] Build height map")
    H, valid, min_xy, grid_m, cell_point_indices = _build_height_map(points_obj_table)
    num_valid_cells = int(np.count_nonzero(valid))
    num_mapped_cells = int(len(cell_point_indices))
    print(
        "  Step-1 mapping check: "
        f"mapping_exists={cell_point_indices is not None}, "
        f"mapped_vs_valid={num_mapped_cells}/{num_valid_cells}"
    )

    print("[4] Build mask + distance transform")
    M, DT_mm, mask_th = _build_mask_and_dt(H, valid)
    print(f"  Mask cells: {int(np.count_nonzero(M))}, threshold(z): {mask_th:.6f} m")

    print("[5] Detect peaks")
    print(
        "  Peak params: "
        f"min_dist={min_peak_distance_mm:.3f}mm, "
        f"strength_frac={peak_strength_fraction:.3f}, "
        f"min_h={peak_min_height_mm:.3f}mm, "
        f"h_support_pct={peak_height_support_percentile:.1f}, "
        f"keep_pct={peak_keep_percentile:.1f}, "
        f"max_peaks={int(max_num_peaks)}, "
        f"anchor={peak_anchor_mode}, "
        f"max_anchor_z_err={peak_max_snap_z_error_mm:.2f}mm"
    )
    peaks_yx, peak_info = _detect_peaks(H, DT_mm, M, valid)
    peak_markers, marker_info = _build_peak_markers(peaks_yx, H.shape)
    print(
        "  Peak markers: "
        f"detected={int(marker_info['num_detected_peaks'])}, "
        f"placed={int(marker_info['num_markers_placed'])}, "
        f"duplicates_ignored={int(marker_info['num_duplicates_ignored'])}, "
        f"out_of_bounds={int(marker_info['num_out_of_bounds'])}"
    )
    if int(marker_info["num_duplicates_ignored"]) > 0:
        print(
            "  Peak marker warning: "
            f"ignored {int(marker_info['num_duplicates_ignored'])} duplicate peaks mapping to same cell."
        )
    peaks_xyz_grid = _peaks_to_xyz(peaks_yx, H, min_xy, grid_m)
    peaks_xyz_table, anchor_info = _anchor_peaks_to_points(peaks_xyz_grid, points_obj_table)

    thr_str = f"{peak_info['strong_thr']:.3f}mm(height)"
    print(
        "  Peak filtering: "
        f"support={int(peak_info['num_support_cells'])}, "
        f"local={int(peak_info['num_local_maxima'])}, "
        f"after_strength={int(peak_info['num_after_strength'])}, "
        f"after_nms={int(peak_info['num_after_nms'])}, "
        f"after_cap={int(peak_info['num_after_cap'])}, "
        f"strong_thr={thr_str}, "
        f"max_dt={peak_info['max_dt_mm']:.3f}mm, "
        f"max_h={peak_info['max_height_mm']:.3f}mm"
    )
    print(f"  Peaks detected: {peaks_xyz_table.shape[0]}")

    z_pts = points_obj_table[:, 2]
    zp = peaks_xyz_table[:, 2] if peaks_xyz_table.shape[0] > 0 else np.empty((0,), dtype=np.float64)
    if zp.size > 0:
        print(
            "  Z diagnostics (table frame, mm): "
            f"points[min/max]=({z_pts.min() * 1e3:.3f}, {z_pts.max() * 1e3:.3f}), "
            f"peaks[min/max]=({zp.min() * 1e3:.3f}, {zp.max() * 1e3:.3f})"
        )
        print(
            "  Peak anchoring (mm): "
            f"3d_dist_mean={anchor_info['mean_dist3d_mm']:.3f}, "
            f"3d_dist_max={anchor_info['max_dist3d_mm']:.3f}, "
            f"z_err_mean={anchor_info['mean_z_error_mm']:.3f}, "
            f"z_err_max={anchor_info['max_z_error_mm']:.3f}, "
            f"snapped={int(anchor_info['num_snapped'])}, "
            f"z_guard_reject={int(anchor_info['num_rejected_z_guard'])}"
        )

    print("[5.5] Heat distance from detected peaks")
    heat_fields = _compute_seeded_heat_fields(points_obj_table, peaks_xyz_table)
    geodesic_info = dict(heat_fields["info"])
    geodesic_owner_table = np.asarray(heat_fields["owner_seed"], dtype=np.int32)
    print(
        "  Heat: "
        f"backend={geodesic_info.get('backend_used', 'unknown')}, "
        f"seed_vertices={int(geodesic_info['num_seed_vertices'])}, "
        f"seed_solves={int(geodesic_info.get('num_seed_solves', 0.0))}, "
        f"reachable={int(geodesic_info['num_reachable'])}/{int(geodesic_info['num_points'])}, "
        f"mean={geodesic_info['mean_mm']:.3f}mm, "
        f"max={geodesic_info['max_mm']:.3f}mm"
    )
    if "error" in geodesic_info:
        print(f"  Heat error: {geodesic_info['error']}")

    print("[5.6] Heat-geodesic labels on 3D points")
    point_labels_table = np.zeros((points_obj_table.shape[0],), dtype=np.int32)
    if geodesic_owner_table.shape[0] == points_obj_table.shape[0]:
        good_owner = geodesic_owner_table >= 0
        point_labels_table[good_owner] = geodesic_owner_table[good_owner] + 1
    labeled_points = int(np.count_nonzero(point_labels_table > 0))
    unlabeled_points = int(point_labels_table.shape[0] - labeled_points)
    unique_point_labels = np.unique(point_labels_table[point_labels_table > 0])
    print(
        "  Heat labels: "
        f"array_len={point_labels_table.shape[0]}, "
        f"labeled_points={labeled_points}, "
        f"unlabeled_points={unlabeled_points}, "
        f"unique_labels={unique_point_labels.tolist() if unique_point_labels.size > 0 else []}"
    )
    if unique_point_labels.size > 0:
        for lab in unique_point_labels:
            n_lab = int(np.count_nonzero(point_labels_table == int(lab)))
            print(f"    label {int(lab)}: {n_lab} points")

    print("[7] Segmentation quality summary")
    label_summary = _summarize_point_labels(points_obj_table, point_labels_table, cell_point_indices)
    if len(label_summary) == 0:
        print("  No labeled beads available for summary.")
    else:
        print("  label | points | cells | max_z_mm | mean_z_mm")
        for row in label_summary:
            print(
                f"  {int(row['label']):5d} | "
                f"{int(row['point_count']):6d} | "
                f"{int(row['grid_cells']):5d} | "
                f"{row['max_height_mm']:8.3f} | "
                f"{row['mean_height_mm']:9.3f}"
            )

    print("[8] TSDF mesh label transfer")
    mesh_geoms_table: List[o3d.geometry.TriangleMesh] = []
    mesh_summary: List[Dict[str, float]] = []
    mesh_debug: Dict[str, float] = {}
    if bool(mesh_enable):
        if mesh_input_resolved is None:
            print(
                "  Mesh input not found; "
                f"set default_mesh_input_path or place '{mesh_auto_filename}' next to the point cloud."
            )
        else:
            print(f"  Mesh input: {mesh_input_resolved}")
            mesh_world = _load_triangle_mesh(mesh_input_resolved)
            mesh_geoms_table, mesh_summary, mesh_debug, _ = _build_meshes_from_tsdf_mesh(
                mesh_world,
                T_table_from_world,
                points_obj_table,
                point_labels_table,
            )
    if len(mesh_summary) == 0:
        if mesh_input_resolved is not None and bool(mesh_enable):
            print("  No labeled TSDF submeshes available.")
    else:
        print(
            "  Mesh transfer: "
            f"source_verts={int(mesh_debug.get('source_vertices', 0))}, "
            f"source_tris={int(mesh_debug.get('source_triangles', 0))}, "
            f"cropped_verts={int(mesh_debug.get('cropped_vertices', 0))}, "
            f"cropped_tris={int(mesh_debug.get('cropped_triangles', 0))}, "
            f"labeled_verts={int(mesh_debug.get('num_labeled_vertices', 0))}, "
            f"unlabeled_verts={int(mesh_debug.get('num_unlabeled_vertices', 0))}, "
            f"mean_snap={float(mesh_debug.get('mean_snap_dist_mm', 0.0)):.3f}mm, "
            f"max_snap={float(mesh_debug.get('max_snap_dist_mm', 0.0)):.3f}mm"
        )
        print("  label | verts | tris | status")
        for row in mesh_summary:
            status = "ok" if int(row["status"]) == 1 else "skip"
            print(
                f"  {int(row['label']):5d} | "
                f"{int(row['vertex_count']):5d} | "
                f"{int(row['triangle_count']):4d} | "
                f"{status}"
            )

    if viz_frame == "world":
        points_viz = _transform_points(points_obj_table, T_world_from_table)
        peaks_viz = _transform_points(peaks_xyz_table, T_world_from_table) if peaks_xyz_table.size else peaks_xyz_table
    else:
        points_viz = points_obj_table
        peaks_viz = peaks_xyz_table
    point_labels_viz = point_labels_table
    mesh_geoms_viz = mesh_geoms_table
    if viz_frame == "world" and len(mesh_geoms_viz) > 0:
        mesh_geoms_world: List[o3d.geometry.TriangleMesh] = []
        for mesh in mesh_geoms_viz:
            mesh_w = o3d.geometry.TriangleMesh(mesh)
            mesh_w.transform(T_world_from_table)
            mesh_geoms_world.append(mesh_w)
        mesh_geoms_viz = mesh_geoms_world

    print("[5.4] Heat-geodesic 3D label visualization")
    unique_labels_viz = np.unique(point_labels_viz[point_labels_viz > 0])
    print(
        "  3D colors: "
        "label_source=heat, "
        f"visualized_labels={int(unique_labels_viz.size)}, "
        f"background_label_color={tuple(float(v) for v in label_background_color)}, "
        "deterministic_mapping=True"
    )
    print(f"  3D meshes: count={len(mesh_geoms_viz)}, point_background={bool(mesh_show_point_cloud_background)}")

    if show_2d:
        _show_2d_debug(
            H,
            M,
            DT_mm,
            peaks_yx,
            pause_ms=int(pause_ms),
            markers=peak_markers,
        )

    if show_3d:
        _show_3d_debug(
            points_viz,
            peaks_viz,
            point_labels=point_labels_viz,
            meshes=mesh_geoms_viz,
        )

    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass

    return {
        "input_path": str(input_path),
        "mesh_input_path": str(mesh_input_resolved) if mesh_input_resolved is not None else None,
        "num_points_loaded": int(points_world.shape[0]),
        "num_points_above_table": int(points_obj_table.shape[0]),
        "num_peaks": int(peaks_xyz_table.shape[0]),
        "peak_filtering": peak_info,
        "peak_anchoring": anchor_info,
        "label_source": "heat",
        "point_labels_table": point_labels_table.tolist(),
        "label_summary": label_summary,
        "mesh_summary": mesh_summary,
        "mesh_debug": mesh_debug,
        "geodesic": geodesic_info,
        "viz_frame": viz_frame,
        "plane_model_world": plane.tolist(),
        "table_transform": T_table_from_world.tolist(),
        "peaks_xyz_table": peaks_xyz_table.tolist(),
    }


def _print_runtime_context() -> None:
    conda_env = os.environ.get("CONDA_DEFAULT_ENV", "")
    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    seed_str = str(int(open3d_random_seed))
    print(
        "Runtime: "
        f"python={sys.executable}, "
        f"conda_env={conda_env or '<unset>'}, "
        f"conda_prefix={conda_prefix or '<unset>'}, "
        f"open3d_seed={seed_str}"
    )

    if hasattr(o3d.utility, "random") and hasattr(o3d.utility.random, "seed"):
        o3d.utility.random.seed(int(open3d_random_seed))


def main() -> None:
    _print_runtime_context()
    input_path = default_input_path
    mesh_input_path = default_mesh_input_path
    show_3d = bool(default_show_3d)
    show_2d = bool(default_show_2d)
    viz_frame = default_viz_frame
    pause_ms = int(default_pause_ms)
    print("Using in-script parameters.")

    result = run_first_stage_peak_debug(
        input_path=input_path,
        mesh_input_path=mesh_input_path,
        show_3d=show_3d,
        show_2d=show_2d,
        viz_frame=viz_frame,
        pause_ms=pause_ms,
    )
    print("\nDone.")
    print(f"Peaks detected: {result['num_peaks']}")


if __name__ == "__main__":
    main()
