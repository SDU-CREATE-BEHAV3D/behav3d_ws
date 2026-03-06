#!/usr/bin/env python3
"""
Stage-1 bead debug pipeline:
- Load point cloud
- (Optional) light voxel downsample
- Fit table plane and move to table frame
- Build 2.5D height map
- Build mask + distance transform
- Detect DT peaks (seed candidates)
- Visualize cloud with detected peaks

This is intentionally minimal to tune parameters before full segmentation.
"""

from __future__ import annotations

import argparse
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

try:
    from skimage.segmentation import watershed as skimage_watershed
except Exception:
    skimage_watershed = None


# -----------------------------------------------------------------------------
# Runtime defaults
# -----------------------------------------------------------------------------
# VS Code "Run Python File" mode: keep this False to use only values in this file.
use_cli_args = False

default_input_path = "~/Downloads/260227_160709/print_scan_007/reconstruct/tsdf_surface_rgb_colored.ply"
default_show_3d = True
default_show_2d = False
default_viz_frame = "table"  # "table" or "world"
default_pause_ms = 0  # 0 -> wait key, >0 -> auto-advance
debug_visual_focus = "mesh3d"  # "all", "watershed", "segmentation3d", or "mesh3d"
open3d_random_seed = 7
default_output_dir = "bead_segmentation_output"
default_export_colored = False
default_export_world_frame = False


# -----------------------------------------------------------------------------
# Stage-1 parameters (all distances in millimeters)
# -----------------------------------------------------------------------------
downsample_voxel_mm = 0  # <= 0 disables downsampling

# Table alignment / slicing
table_ransac_thresh_mm = 1.0
table_ransac_n = 3
table_ransac_iters = 2500
z_min_above_plane_mm = 1.55

# Height map / mask / peak detection
grid_mm = 0.10
height_percentile = 99.0
gaussian_sigma_px = 0.2
mask_percentile = 50.0
morph_kernel_px = 1
watershed_mask_kernel_px = 5
peak_detection_mode = "height"  # "height" or "dt"
peak_height_support_percentile = 45.0
peak_min_height_mm = 1.0
min_peak_distance_mm = 4
peak_strength_fraction = 0.10
peak_min_dt_mm = 0.10
peak_keep_percentile = 0.0  # keep only top DT peaks by value
max_num_peaks = 1400

# 3D peak marker display
peak_marker_radius_mm = 1.6
peak_marker_outer_color = (1.0, 0.95, 0.15)
peak_marker_inner_color = (1.0, 0.05, 0.05)
colorize_cloud_by_height = True
label_background_color = (0.60, 0.60, 0.60)

# Peak-to-point anchoring
peak_anchor_mode = "nearest3d"  # "grid" or "nearest3d"
peak_max_snap_z_error_mm = 1.5

# Side visualization of top-N height-map points
top_height_points_k = 0

# Geodesic distance from detected peaks
geodesic_enable = True
geodesic_heat_t_coef = 1.2
geodesic_clip_percentile = 98.0
geodesic_colormap = cv2.COLORMAP_JET
geodesic_invert_colormap = True  # near peaks -> warmer color
geodesic_clamp_negative_mm = True
geodesic_seeded_rings = True
geodesic_max_seed_count = 0  # 0 -> use all detected peak seeds
geodesic_show_heat_panel = True
geodesic_panel_offset_mm = 45.0
geodesic_show_rings = True
geodesic_show_ring_points = False
geodesic_ring_method = "triangulated"  # "logmap" or "triangulated"
geodesic_logmap_bins = 96
geodesic_logmap_min_bins_filled = 10
geodesic_logmap_close_gap_bins = 1
geodesic_ring_step_mm = 0.3
geodesic_ring_band_mm = 0.50
geodesic_ring_color = (0.20, 1.00, 0.25)
geodesic_ring_connect_radius_mm = 0.5
geodesic_ring_min_component_points = 4
geodesic_ring_min_vertex_spacing_mm = 0.10
geodesic_ring_smooth_window = 4
geodesic_curve_min_points = 2
geodesic_curve_min_length_mm = 0.3

# Mesh reconstruction from heat-labeled grid support
mesh_enable = True
mesh_min_points = 10
mesh_min_cells = 8
mesh_label_close_px = 1
mesh_allow_boundary_triangles = True
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


def _connected_components(mask: np.ndarray) -> Tuple[int, np.ndarray]:
    num_labels, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    return int(num_labels), labels.astype(np.int32)


def _build_watershed_mask_and_dt(
    H: np.ndarray,
    valid: np.ndarray,
    mask_threshold_m: float,
    supported_mask: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    candidate = H > float(mask_threshold_m)
    if not np.any(candidate):
        candidate = (H >= float(mask_threshold_m)) | supported_mask.astype(bool)

    k = _ensure_odd(max(3, int(watershed_mask_kernel_px)))
    if k > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        Mu8 = candidate.astype(np.uint8)
        Mu8 = cv2.morphologyEx(Mu8, cv2.MORPH_CLOSE, kernel, iterations=1)
        Mu8 = cv2.morphologyEx(Mu8, cv2.MORPH_OPEN, kernel, iterations=1)
        candidate = Mu8.astype(bool)

    # Keep only dense components that are supported by the original sparse mask.
    supported = supported_mask.astype(bool)
    if np.any(candidate) and np.any(supported):
        num_labels, comp = _connected_components(candidate)
        keep = np.zeros_like(candidate, dtype=bool)
        for lab in range(1, num_labels):
            comp_mask = comp == lab
            if np.any(comp_mask & supported):
                keep |= comp_mask
        if np.any(keep):
            candidate = keep

    # Always preserve original sparse support cells to avoid dropping seed locations.
    candidate |= supported

    if not np.any(candidate):
        candidate = valid.astype(bool).copy()

    if ndi is not None:
        dt_px = ndi.distance_transform_edt(candidate).astype(np.float32)
    else:
        dt_px = cv2.distanceTransform(candidate.astype(np.uint8), cv2.DIST_L2, 5)

    DT_mm = dt_px * float(grid_mm)
    return candidate.astype(bool), DT_mm.astype(np.float32)


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
    mode = str(peak_detection_mode).strip().lower()

    if mode == "height":
        source_mm = H.astype(np.float32) * 1e3
        support = (mask & valid).copy()
        if np.any(support):
            hot_th = float(np.percentile(source_mm[support], float(peak_height_support_percentile)))
            support &= source_mm >= hot_th
    elif mode == "dt":
        source_mm = DT_mm.astype(np.float32)
        support = (mask & valid).copy()
    else:
        raise ValueError(f"Unsupported peak_detection_mode: {peak_detection_mode}")

    if not np.any(support):
        return np.empty((0, 2), dtype=np.int32), {
            "mode": mode,
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
            "mode": mode,
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
    if mode == "height":
        strong_floor = float(peak_min_height_mm)
    else:
        strong_floor = float(peak_min_dt_mm)
    strong_thr = max(strong_abs, strong_pct, strong_floor)
    strong = vals >= strong_thr
    coords = coords[strong]
    vals = vals[strong]
    n_strong = int(coords.shape[0])

    if coords.size == 0:
        peak = np.unravel_index(int(np.argmax(source_mm * support.astype(np.float32))), source_mm.shape)
        p = np.asarray([[int(peak[0]), int(peak[1])]], dtype=np.int32)
        return p, {
            "mode": mode,
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
        "mode": mode,
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


def _watershed_labels(energy_img: np.ndarray, M: np.ndarray, markers: np.ndarray) -> np.ndarray:
    if skimage_watershed is None:
        raise RuntimeError(
            "scikit-image is not installed; cannot run watershed. "
            "Install with: pip install scikit-image"
        )

    if energy_img.shape != M.shape or markers.shape != M.shape:
        raise ValueError("energy_img, M, and markers must have the same 2D shape.")

    mask = M.astype(bool)
    marks = markers.astype(np.int32).copy()
    marks[~mask] = 0
    if not np.any(marks > 0):
        raise RuntimeError("No peak markers available inside mask; cannot run watershed.")

    energy = energy_img.astype(np.float32)
    labels = skimage_watershed(energy, markers=marks, mask=mask).astype(np.int32)
    labels[~mask] = 0
    return labels


def _grid_labels_to_point_labels(
    point_count: int,
    cell_point_indices: Dict[Tuple[int, int], np.ndarray],
    grid_labels: np.ndarray,
) -> np.ndarray:
    point_labels = np.zeros((int(point_count),), dtype=np.int32)
    for (y, x), point_idx in cell_point_indices.items():
        if y < 0 or x < 0 or y >= grid_labels.shape[0] or x >= grid_labels.shape[1]:
            continue
        if point_idx.size == 0:
            continue
        point_labels[np.asarray(point_idx, dtype=np.int32)] = int(grid_labels[int(y), int(x)])
    return point_labels


def _top_height_points_xyz(
    H: np.ndarray,
    valid: np.ndarray,
    min_xy: np.ndarray,
    grid_m: float,
    top_k: int,
) -> Tuple[np.ndarray, np.ndarray]:
    ys, xs = np.nonzero(valid)
    if ys.size == 0 or top_k <= 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0,), dtype=np.float64)

    z = H[ys, xs].astype(np.float64)
    top_k = min(int(top_k), int(z.size))
    order = np.argsort(-z, kind="mergesort")[:top_k]

    ys = ys[order].astype(np.float64)
    xs = xs[order].astype(np.float64)
    z = z[order]

    px = min_xy[0] + (xs + 0.5) * grid_m
    py = min_xy[1] + (ys + 0.5) * grid_m
    pts = np.column_stack([px, py, z])
    return pts, z * 1e3


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
        "enabled": 1.0 if bool(geodesic_enable) else 0.0,
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
    if not bool(geodesic_enable):
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
            if bool(geodesic_clamp_negative_mm):
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


def _compute_geodesic_mm(points_xyz: np.ndarray, seed_xyz: np.ndarray) -> Tuple[np.ndarray, Dict[str, object]]:
    fields = _compute_seeded_heat_fields(points_xyz, seed_xyz)
    return np.asarray(fields["dist_min_mm"], dtype=np.float64), dict(fields["info"])


def _geodesic_colors(dist_mm: np.ndarray) -> np.ndarray:
    n = int(dist_mm.size)
    colors = np.tile(np.array([[0.55, 0.55, 0.55]], dtype=np.float64), (n, 1))
    if n == 0:
        return colors
    finite = np.isfinite(dist_mm)
    if not np.any(finite):
        return colors

    vals = dist_mm[finite]
    vmax = float(np.percentile(vals, float(geodesic_clip_percentile)))
    if not np.isfinite(vmax) or vmax <= 1e-6:
        vmax = float(np.max(vals))
    vmax = max(vmax, 1e-6)
    norm = np.clip(vals / vmax, 0.0, 1.0)
    if bool(geodesic_invert_colormap):
        norm = 1.0 - norm

    u8 = np.clip(norm * 255.0, 0, 255).astype(np.uint8)
    bgr = cv2.applyColorMap(u8.reshape(-1, 1), int(geodesic_colormap)).reshape(-1, 3)
    colors[finite] = bgr[:, ::-1].astype(np.float64) / 255.0
    return colors


def _geodesic_ring_cloud(points_xyz: np.ndarray, dist_mm: np.ndarray) -> o3d.geometry.PointCloud | None:
    if not bool(geodesic_show_rings) or not bool(geodesic_show_ring_points):
        return None
    if points_xyz.shape[0] == 0 or dist_mm.size == 0:
        return None
    step = float(geodesic_ring_step_mm)
    band = float(geodesic_ring_band_mm)
    if step <= 0.0 or band <= 0.0:
        return None

    finite = np.isfinite(dist_mm)
    if not np.any(finite):
        return None
    r = np.mod(dist_mm[finite], step)
    near = np.minimum(r, step - r) <= band
    mask = np.zeros_like(finite, dtype=bool)
    idx = np.flatnonzero(finite)
    mask[idx[near]] = True
    if not np.any(mask):
        return None

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz[mask].astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(
        np.tile(np.asarray(geodesic_ring_color, dtype=np.float64).reshape(1, 3), (int(np.count_nonzero(mask)), 1))
    )
    return pcd


def _pointcloud_triangles(points_xyz: np.ndarray) -> np.ndarray:
    if pp3d is None or points_xyz.shape[0] == 0:
        return np.empty((0, 3), dtype=np.int32)
    try:
        tri_local = pp3d.PointCloudLocalTriangulation(points_xyz.astype(np.float64)).get_local_triangulation()
    except Exception:
        return np.empty((0, 3), dtype=np.int32)
    if tri_local.size == 0:
        return np.empty((0, 3), dtype=np.int32)
    tri_valid = tri_local[np.all(tri_local >= 0, axis=2)]
    if tri_valid.size == 0:
        return np.empty((0, 3), dtype=np.int32)
    tri = np.sort(tri_valid.astype(np.int32), axis=1)
    tri = np.unique(tri, axis=0)
    tri = tri[(tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])]
    return tri


def _geodesic_ring_linesets(
    points_xyz: np.ndarray,
    dist_mm: np.ndarray,
    point_mask: np.ndarray | None = None,
    triangles: np.ndarray | None = None,
) -> List[o3d.geometry.LineSet]:
    if not bool(geodesic_show_rings):
        return []
    if points_xyz.shape[0] == 0 or dist_mm.size == 0:
        return []
    if triangles is not None and triangles.size > 0:
        tri = np.asarray(triangles, dtype=np.int32)
    else:
        tri = _pointcloud_triangles(points_xyz)
    if tri.shape[0] == 0:
        return []

    finite = np.isfinite(dist_mm)
    if point_mask is not None:
        finite = finite & np.asarray(point_mask, dtype=bool)
    if not np.any(finite):
        return []

    step = float(geodesic_ring_step_mm)
    band = float(geodesic_ring_band_mm)
    if step <= 0.0 or band <= 0.0:
        return []

    min_comp = max(4, int(geodesic_ring_min_component_points))
    min_spacing_m = _mm_to_m(float(geodesic_ring_min_vertex_spacing_mm))
    smooth_k = _ensure_odd(max(1, int(geodesic_ring_smooth_window)))
    merge_eps_m = max(_mm_to_m(0.05), 0.35 * _mm_to_m(float(geodesic_ring_connect_radius_mm)))
    curve_min_pts = max(2, int(geodesic_curve_min_points))
    curve_min_len_m = _mm_to_m(float(geodesic_curve_min_length_mm))

    vals = dist_mm[finite]
    dmax = float(np.max(vals))
    if dmax <= step:
        return []

    def _smooth_polyline(points: np.ndarray, window: int, closed: bool) -> np.ndarray:
        if points.shape[0] < 4 or window <= 1:
            return points
        k = _ensure_odd(window)
        if points.shape[0] < k:
            return points
        r = k // 2
        if closed:
            pad = np.vstack([points[-r:], points, points[:r]])
        else:
            pad = np.vstack([np.repeat(points[:1], r, axis=0), points, np.repeat(points[-1:], r, axis=0)])
        out = np.empty_like(points)
        for i in range(points.shape[0]):
            out[i] = np.mean(pad[i : i + k], axis=0)
        return out

    # Keep triangles with finite distances only.
    tri_finite = finite[tri].all(axis=1)
    tri = tri[tri_finite]
    if tri.shape[0] == 0:
        return []

    def _dedup_pts(pts: List[np.ndarray], eps_m: float) -> List[np.ndarray]:
        out: List[np.ndarray] = []
        for p in pts:
            keep = True
            for q in out:
                if np.linalg.norm(p - q) <= eps_m:
                    keep = False
                    break
            if keep:
                out.append(p)
        return out

    def _add_node(node_map: Dict[Tuple[int, int, int], int], nodes: List[np.ndarray], p: np.ndarray, eps_m: float) -> int:
        key = tuple(np.round(p / eps_m).astype(np.int64).tolist())
        if key in node_map:
            return int(node_map[key])
        idx = len(nodes)
        node_map[key] = idx
        nodes.append(p.astype(np.float64))
        return idx

    def _component_nodes(adj: Dict[int, List[int]]) -> List[np.ndarray]:
        comps: List[np.ndarray] = []
        seen: set[int] = set()
        for s in adj.keys():
            if s in seen:
                continue
            stack = [int(s)]
            seen.add(int(s))
            comp: List[int] = []
            while stack:
                u = int(stack.pop())
                comp.append(u)
                for v in adj.get(u, []):
                    if int(v) not in seen:
                        seen.add(int(v))
                        stack.append(int(v))
            comps.append(np.asarray(comp, dtype=np.int32))
        return comps

    def _trace_component_path(adj_comp: Dict[int, List[int]], comp_nodes: List[int]) -> Tuple[List[int], bool]:
        deg = {u: len(adj_comp.get(u, [])) for u in comp_nodes}
        endpoints = [u for u in comp_nodes if deg[u] == 1]
        if any(d > 2 for d in deg.values()):
            return [], False
        if len(endpoints) == 2:
            start = endpoints[0]
            is_closed = False
        elif len(endpoints) == 0 and all(d == 2 for d in deg.values()):
            start = comp_nodes[0]
            is_closed = True
        else:
            return [], False

        path = [int(start)]
        prev = -1
        cur = int(start)
        visited_edges: set[Tuple[int, int]] = set()
        max_steps = max(2, len(comp_nodes) * 3)

        for _ in range(max_steps):
            nbrs = [int(v) for v in adj_comp.get(cur, []) if int(v) != prev]
            nxt = None
            for v in nbrs:
                e = (cur, v) if cur < v else (v, cur)
                if e not in visited_edges:
                    nxt = v
                    visited_edges.add(e)
                    break
            if nxt is None:
                break

            if is_closed and nxt == start:
                break

            path.append(int(nxt))
            prev, cur = cur, int(nxt)

            if not is_closed and len(adj_comp.get(cur, [])) == 1:
                break

        return path, is_closed

    def _polyline_length(points: np.ndarray, closed: bool) -> float:
        if points.shape[0] < 2:
            return 0.0
        seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
        total = float(np.sum(seg))
        if closed and points.shape[0] > 2:
            total += float(np.linalg.norm(points[0] - points[-1]))
        return total

    linesets: List[o3d.geometry.LineSet] = []
    levels = np.arange(step, dmax, step, dtype=np.float64)
    eps_val = max(1e-6, 0.02 * step)  # in mm
    edges = ((0, 1), (1, 2), (2, 0))

    for lvl in levels:
        nodes: List[np.ndarray] = []
        node_map: Dict[Tuple[int, int, int], int] = {}
        edge_set: set[Tuple[int, int]] = set()

        # Extract level-set segments from triangle edges.
        for ia, ib, ic in tri.tolist():
            ids = [int(ia), int(ib), int(ic)]
            s = dist_mm[ids]
            smin = float(np.min(s))
            smax = float(np.max(s))
            if lvl < smin or lvl > smax:
                continue

            p = points_xyz[ids].astype(np.float64)
            inter: List[np.ndarray] = []
            for a, b in edges:
                sa = float(s[a])
                sb = float(s[b])
                da = sa - float(lvl)
                db = sb - float(lvl)
                pa = p[a]
                pb = p[b]

                if da * db < 0.0:
                    t = (float(lvl) - sa) / (sb - sa)
                    inter.append(pa + t * (pb - pa))
                elif abs(da) <= eps_val and abs(db) <= eps_val:
                    # Entire edge sits on contour level; skip to avoid ambiguous branching.
                    continue
                elif abs(da) <= eps_val:
                    inter.append(pa.copy())
                elif abs(db) <= eps_val:
                    inter.append(pb.copy())

            inter = _dedup_pts(inter, merge_eps_m)
            if len(inter) < 2:
                continue
            if len(inter) > 2:
                # Degenerate case: pick the farthest two points.
                best = None
                best_d = -1.0
                for i in range(len(inter)):
                    for j in range(i + 1, len(inter)):
                        d = float(np.linalg.norm(inter[i] - inter[j]))
                        if d > best_d:
                            best_d = d
                            best = (inter[i], inter[j])
                if best is None:
                    continue
                p0, p1 = best
            else:
                p0, p1 = inter[0], inter[1]

            u = _add_node(node_map, nodes, p0, merge_eps_m)
            v = _add_node(node_map, nodes, p1, merge_eps_m)
            if u == v:
                continue
            e = (u, v) if u < v else (v, u)
            edge_set.add(e)

        if not edge_set or len(nodes) < min_comp:
            continue

        # Build connectivity graph for this iso-level.
        adj: Dict[int, List[int]] = {}
        for u, v in edge_set:
            adj.setdefault(int(u), []).append(int(v))
            adj.setdefault(int(v), []).append(int(u))

        node_arr = np.asarray(nodes, dtype=np.float64)
        for comp in _component_nodes(adj):
            if comp.size < min_comp:
                continue

            comp_list = [int(u) for u in comp.tolist()]
            comp_set = set(comp_list)
            adj_comp: Dict[int, List[int]] = {}
            for u in comp_list:
                adj_comp[u] = [int(v) for v in adj.get(u, []) if int(v) in comp_set]

            path_nodes, is_closed = _trace_component_path(adj_comp, comp_list)
            if len(path_nodes) < min_comp:
                continue

            ring = node_arr[np.asarray(path_nodes, dtype=np.int32)]

            # Remove overly dense consecutive points for cleaner curves.
            if min_spacing_m > 0.0 and ring.shape[0] > 4:
                keep = [0]
                for i in range(1, ring.shape[0]):
                    if np.linalg.norm(ring[i, :2] - ring[keep[-1], :2]) >= min_spacing_m:
                        keep.append(i)
                ring = ring[np.asarray(keep, dtype=np.int32)]
                if is_closed and ring.shape[0] > 3 and np.linalg.norm(ring[0, :2] - ring[-1, :2]) < min_spacing_m:
                    ring = ring[:-1]

            if ring.shape[0] < min_comp:
                continue

            ring = _smooth_polyline(ring, smooth_k, closed=is_closed)
            if ring.shape[0] < curve_min_pts:
                continue
            if _polyline_length(ring, closed=is_closed) < curve_min_len_m:
                continue

            n_ring = int(ring.shape[0])
            if is_closed:
                lines = np.column_stack(
                    [np.arange(n_ring, dtype=np.int32), np.roll(np.arange(n_ring, dtype=np.int32), -1)]
                )
            else:
                if n_ring < 2:
                    continue
                lines = np.column_stack(
                    [np.arange(n_ring - 1, dtype=np.int32), np.arange(1, n_ring, dtype=np.int32)]
                )

            ls = o3d.geometry.LineSet()
            ls.points = o3d.utility.Vector3dVector(ring.astype(np.float64))
            ls.lines = o3d.utility.Vector2iVector(lines.astype(np.int32))
            ls.paint_uniform_color(list(geodesic_ring_color))
            linesets.append(ls)

    return linesets


def _seeded_geodesic_ring_linesets(
    points_xyz: np.ndarray,
    dist_by_seed_mm: np.ndarray,
    owner_seed: np.ndarray,
    seed_vertex_idx: np.ndarray | None = None,
) -> List[o3d.geometry.LineSet]:
    if not bool(geodesic_show_rings):
        return []
    if points_xyz.shape[0] == 0:
        return []
    if dist_by_seed_mm.ndim != 2 or dist_by_seed_mm.shape[1] != points_xyz.shape[0]:
        return []
    if owner_seed.shape[0] != points_xyz.shape[0]:
        return []
    if not bool(geodesic_seeded_rings):
        dist_min = np.min(dist_by_seed_mm, axis=0)
        return _geodesic_ring_linesets(points_xyz, dist_min)

    method = str(geodesic_ring_method).strip().lower()
    if method not in {"logmap", "triangulated"}:
        method = "logmap"

    if seed_vertex_idx is None or seed_vertex_idx.size != dist_by_seed_mm.shape[0]:
        seed_vertex_idx = np.argmin(dist_by_seed_mm, axis=1).astype(np.int32)
    else:
        seed_vertex_idx = np.asarray(seed_vertex_idx, dtype=np.int32).reshape(-1)

    def _extract_triangulated() -> List[o3d.geometry.LineSet]:
        tri = _pointcloud_triangles(points_xyz)
        if tri.shape[0] == 0:
            return []
        all_lines: List[o3d.geometry.LineSet] = []
        for s in range(dist_by_seed_mm.shape[0]):
            mask = owner_seed == int(s)
            if np.count_nonzero(mask) < max(4, int(geodesic_ring_min_component_points)):
                continue
            d = dist_by_seed_mm[s]
            all_lines.extend(_geodesic_ring_linesets(points_xyz, d, point_mask=mask, triangles=tri))
        return all_lines

    if method == "triangulated" or pp3d is None:
        return _extract_triangulated()

    # Log-map based ring extraction (closer to vector heat usage).
    min_comp_base = max(4, int(geodesic_ring_min_component_points))
    min_spacing_base_m = _mm_to_m(float(geodesic_ring_min_vertex_spacing_mm))
    smooth_k = _ensure_odd(max(1, int(geodesic_ring_smooth_window)))
    bins_base = max(24, int(geodesic_logmap_bins))
    bins_min_base = max(8, int(geodesic_logmap_min_bins_filled))
    close_gap_base = max(0, int(geodesic_logmap_close_gap_bins))
    step = float(geodesic_ring_step_mm)
    band_base = float(geodesic_ring_band_mm)
    if step <= 0.0 or band_base <= 0.0:
        return []

    def _smooth_polyline(points: np.ndarray, window: int, closed: bool) -> np.ndarray:
        if points.shape[0] < 4 or window <= 1:
            return points
        k = _ensure_odd(window)
        if points.shape[0] < k:
            return points
        r = k // 2
        if closed:
            pad = np.vstack([points[-r:], points, points[:r]])
        else:
            pad = np.vstack([np.repeat(points[:1], r, axis=0), points, np.repeat(points[-1:], r, axis=0)])
        out = np.empty_like(points)
        for i in range(points.shape[0]):
            out[i] = np.mean(pad[i : i + k], axis=0)
        return out

    try:
        solver = pp3d.PointCloudHeatSolver(points_xyz.astype(np.float64), float(geodesic_heat_t_coef))
    except Exception:
        return _extract_triangulated()

    def _extract_logmap_lines(
        bins_target: int,
        bins_min_filled: int,
        min_comp: int,
        band: float,
        close_gap_bins: int,
        min_spacing_m: float,
    ) -> List[o3d.geometry.LineSet]:
        all_lines: List[o3d.geometry.LineSet] = []
        for s in range(dist_by_seed_mm.shape[0]):
            owner_mask = owner_seed == int(s)
            if np.count_nonzero(owner_mask) < min_comp:
                continue

            d = dist_by_seed_mm[s]
            finite = owner_mask & np.isfinite(d)
            if np.count_nonzero(finite) < min_comp:
                continue

            dmax = float(np.max(d[finite]))
            if not np.isfinite(dmax) or dmax <= step:
                continue

            try:
                uv = np.asarray(solver.compute_log_map(int(seed_vertex_idx[s])), dtype=np.float64)
            except Exception:
                continue
            if uv.ndim != 2 or uv.shape[0] != points_xyz.shape[0] or uv.shape[1] != 2:
                continue
            finite = finite & np.isfinite(uv[:, 0]) & np.isfinite(uv[:, 1])
            if np.count_nonzero(finite) < min_comp:
                continue

            theta = np.arctan2(uv[:, 1], uv[:, 0])
            levels = np.arange(step, dmax, step, dtype=np.float64)
            if levels.size == 0:
                continue

            for lvl in levels:
                shell = finite & (np.abs(d - lvl) <= band)
                idx = np.flatnonzero(shell)
                if idx.size < min_comp:
                    continue

                bins = bins_target
                if min_spacing_m > 1e-9:
                    circum = 2.0 * np.pi * max(float(lvl) * 1e-3, min_spacing_m)
                    bins = max(bins, int(np.ceil(circum / min_spacing_m)))
                bins = int(np.clip(bins, 24, 720))
                edges = np.linspace(-np.pi, np.pi, bins + 1)

                bin_hits: List[Tuple[int, int]] = []
                for b in range(bins):
                    in_bin = idx[(theta[idx] >= edges[b]) & (theta[idx] < edges[b + 1])]
                    if in_bin.size == 0:
                        continue
                    best = int(in_bin[np.argmin(np.abs(d[in_bin] - lvl))])
                    if bin_hits and bin_hits[-1][1] == best:
                        continue
                    bin_hits.append((b, best))

                if len(bin_hits) < min_comp:
                    continue

                segments: List[List[Tuple[int, int]]] = [[bin_hits[0]]]
                for (b0, _i0), (b1, i1) in zip(bin_hits[:-1], bin_hits[1:]):
                    if (b1 - b0) <= (1 + close_gap_bins):
                        segments[-1].append((b1, i1))
                    else:
                        segments.append([(b1, i1)])

                seam_contiguous = ((bin_hits[0][0] + bins - bin_hits[-1][0]) <= (1 + close_gap_bins))
                if seam_contiguous and len(segments) > 1:
                    merged = segments[-1] + segments[0]
                    segments = [merged] + segments[1:-1]

                for seg in segments:
                    is_closed = seam_contiguous and (len(segments) == 1)
                    min_bins_needed = max(min_comp, bins_min_filled) if is_closed else min_comp
                    if len(seg) < min_bins_needed:
                        continue

                    seen: set[int] = set()
                    ordered_ids: List[int] = []
                    for _, rid in seg:
                        if rid not in seen:
                            seen.add(rid)
                            ordered_ids.append(rid)
                    if len(ordered_ids) < min_bins_needed:
                        continue

                    curve = points_xyz[np.asarray(ordered_ids, dtype=np.int32)].astype(np.float64)
                    if min_spacing_m > 0.0 and curve.shape[0] > 4:
                        keep = [0]
                        for i in range(1, curve.shape[0]):
                            if np.linalg.norm(curve[i, :2] - curve[keep[-1], :2]) >= min_spacing_m:
                                keep.append(i)
                        curve = curve[np.asarray(keep, dtype=np.int32)]
                        if is_closed and curve.shape[0] > 3 and np.linalg.norm(curve[0, :2] - curve[-1, :2]) < min_spacing_m:
                            curve = curve[:-1]

                    if curve.shape[0] < min_comp:
                        continue

                    curve = _smooth_polyline(curve, smooth_k, closed=is_closed)
                    n_curve = int(curve.shape[0])
                    if is_closed:
                        lines = np.column_stack(
                            [np.arange(n_curve, dtype=np.int32), np.roll(np.arange(n_curve, dtype=np.int32), -1)]
                        )
                    else:
                        if n_curve < 2:
                            continue
                        lines = np.column_stack(
                            [np.arange(n_curve - 1, dtype=np.int32), np.arange(1, n_curve, dtype=np.int32)]
                        )

                    ls = o3d.geometry.LineSet()
                    ls.points = o3d.utility.Vector3dVector(curve.astype(np.float64))
                    ls.lines = o3d.utility.Vector2iVector(lines.astype(np.int32))
                    ls.paint_uniform_color(list(geodesic_ring_color))
                    all_lines.append(ls)
        return all_lines

    attempts = [
        (bins_base, bins_min_base, min_comp_base, band_base, close_gap_base, min_spacing_base_m),
        (
            max(24, bins_base // 2),
            max(4, bins_min_base // 2),
            max(3, min_comp_base - 2),
            1.7 * band_base,
            close_gap_base + 2,
            max(_mm_to_m(0.05), 0.7 * min_spacing_base_m),
        ),
    ]
    for params in attempts:
        lines = _extract_logmap_lines(*params)
        if len(lines) > 0:
            return lines

    return _extract_triangulated()


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


def _colorize_grid_labels(grid_labels: np.ndarray, mask: np.ndarray) -> np.ndarray:
    h, w = grid_labels.shape
    vis = np.zeros((h, w, 3), dtype=np.uint8)
    mask_bool = mask.astype(bool)
    vis[mask_bool] = np.array([20, 20, 20], dtype=np.uint8)

    labels = np.unique(grid_labels[mask_bool]) if np.any(mask_bool) else np.empty((0,), dtype=np.int32)
    labels = labels[labels > 0]
    for lab in labels:
        hue = int((47 * int(lab) + 23) % 180)
        color_hsv = np.array([[[hue, 240, 255]]], dtype=np.uint8)
        color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0, 0]
        vis[grid_labels == int(lab)] = color_bgr

    lbl = grid_labels.astype(np.int32)
    boundaries = np.zeros_like(mask_bool, dtype=bool)
    vdiff = (lbl[1:, :] != lbl[:-1, :]) & ((lbl[1:, :] > 0) | (lbl[:-1, :] > 0))
    hdiff = (lbl[:, 1:] != lbl[:, :-1]) & ((lbl[:, 1:] > 0) | (lbl[:, :-1] > 0))
    boundaries[1:, :] |= vdiff
    boundaries[:-1, :] |= vdiff
    boundaries[:, 1:] |= hdiff
    boundaries[:, :-1] |= hdiff
    vis[boundaries & mask_bool] = np.array([255, 255, 255], dtype=np.uint8)
    return vis


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


def _export_colored_point_cloud(
    points_xyz: np.ndarray,
    point_labels: np.ndarray,
    output_path: Path,
) -> Dict[str, object]:
    output_path = Path(output_path).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(_labels_to_colors(point_labels))

    ok = bool(
        o3d.io.write_point_cloud(
            str(output_path),
            pcd,
            write_ascii=False,
            compressed=False,
            print_progress=False,
        )
    )
    if not ok:
        raise RuntimeError(f"Failed to export point cloud: {output_path}")

    labels = np.asarray(point_labels, dtype=np.int32)
    unique_labels = np.unique(labels[labels > 0])
    return {
        "output_path": str(output_path),
        "num_points": int(points_xyz.shape[0]),
        "num_labels": int(unique_labels.size),
    }


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


def _close_mask(mask: np.ndarray, kernel_px: int) -> np.ndarray:
    out = np.asarray(mask, dtype=bool).copy()
    if not np.any(out):
        return out
    k = _ensure_odd(max(1, int(kernel_px)))
    if k <= 1:
        return out
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    return cv2.morphologyEx(out.astype(np.uint8), cv2.MORPH_CLOSE, kernel, iterations=1).astype(bool)


def _grid_majority_labels(
    point_labels: np.ndarray,
    cell_point_indices: Dict[Tuple[int, int], np.ndarray],
    shape_hw: Tuple[int, int],
) -> Tuple[np.ndarray, Dict[int, int]]:
    grid_labels = np.zeros(shape_hw, dtype=np.int32)
    ambiguity = {"mixed_cells": 0, "empty_cells": 0}
    labels = np.asarray(point_labels, dtype=np.int32)

    for (y, x), idx in cell_point_indices.items():
        if idx.size == 0:
            ambiguity["empty_cells"] += 1
            continue
        cell_labels = labels[np.asarray(idx, dtype=np.int32)]
        cell_labels = cell_labels[cell_labels > 0]
        if cell_labels.size == 0:
            continue
        uniq, counts = np.unique(cell_labels, return_counts=True)
        if uniq.size > 1:
            ambiguity["mixed_cells"] += 1
        best = np.flatnonzero(counts == np.max(counts))
        chosen = int(uniq[int(best[0])])
        grid_labels[int(y), int(x)] = chosen

    return grid_labels, ambiguity


def _densify_grid_labels(
    grid_labels_sparse: np.ndarray,
    support_mask: np.ndarray,
) -> Tuple[np.ndarray, Dict[str, int]]:
    sparse = np.asarray(grid_labels_sparse, dtype=np.int32)
    support = np.asarray(support_mask, dtype=bool)
    dense = np.zeros_like(sparse, dtype=np.int32)

    seed_mask = (sparse > 0) & support
    num_seeded = int(np.count_nonzero(seed_mask))
    if num_seeded == 0:
        return dense, {"seeded_cells": 0, "propagated_cells": 0}

    dense[seed_mask] = sparse[seed_mask]
    target_y, target_x = np.nonzero(support & ~seed_mask)
    if target_y.size == 0:
        return dense, {"seeded_cells": num_seeded, "propagated_cells": 0}

    seed_y, seed_x = np.nonzero(seed_mask)
    seed_labels = sparse[seed_y, seed_x].astype(np.int32)
    seed_coords = np.column_stack([seed_y, seed_x]).astype(np.float64)
    query_coords = np.column_stack([target_y, target_x]).astype(np.float64)

    if cKDTree is not None:
        tree = cKDTree(seed_coords)
        _, nn = tree.query(query_coords, k=1)
        nn = np.asarray(nn, dtype=np.int32).reshape(-1)
    else:
        nn = np.empty((query_coords.shape[0],), dtype=np.int32)
        for i, q in enumerate(query_coords):
            d2 = np.sum((seed_coords - q[None, :]) ** 2, axis=1)
            nn[i] = int(np.argmin(d2))

    dense[target_y, target_x] = seed_labels[nn]
    return dense, {
        "seeded_cells": num_seeded,
        "propagated_cells": int(target_y.size),
    }


def _grid_mask_to_heightfield_mesh(
    H: np.ndarray,
    cell_mask: np.ndarray,
    min_xy: np.ndarray,
    grid_m: float,
) -> o3d.geometry.TriangleMesh | None:
    mask = np.asarray(cell_mask, dtype=bool)
    if not np.any(mask):
        return None

    ys, xs = np.nonzero(mask)
    if ys.size < 3:
        return None

    y0 = int(np.min(ys))
    y1 = int(np.max(ys))
    x0 = int(np.min(xs))
    x1 = int(np.max(xs))

    vertex_map = -np.ones(mask.shape, dtype=np.int32)
    px = min_xy[0] + (xs.astype(np.float64) + 0.5) * float(grid_m)
    py = min_xy[1] + (ys.astype(np.float64) + 0.5) * float(grid_m)
    pz = H[ys, xs].astype(np.float64)
    vertices = np.column_stack([px, py, pz]).astype(np.float64)
    vertex_map[ys, xs] = np.arange(vertices.shape[0], dtype=np.int32)

    tris: List[List[int]] = []
    for y in range(y0, y1):
        for x in range(x0, x1):
            v00 = int(vertex_map[y, x])
            v10 = int(vertex_map[y, x + 1])
            v01 = int(vertex_map[y + 1, x])
            v11 = int(vertex_map[y + 1, x + 1])
            present = [v00 >= 0, v10 >= 0, v01 >= 0, v11 >= 0]
            n_present = int(sum(present))
            if n_present < 3:
                continue
            if n_present == 4:
                tris.append([v00, v10, v11])
                tris.append([v00, v11, v01])
                continue
            if not bool(mesh_allow_boundary_triangles):
                continue
            if v00 < 0:
                tris.append([v10, v11, v01])
            elif v10 < 0:
                tris.append([v00, v11, v01])
            elif v01 < 0:
                tris.append([v00, v10, v11])
            else:
                tris.append([v00, v10, v01])

    if len(tris) == 0:
        return None

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(tris, dtype=np.int32))
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_unreferenced_vertices()
    if len(mesh.triangles) == 0 or len(mesh.vertices) == 0:
        return None
    mesh.compute_vertex_normals()
    return mesh


def _build_meshes_from_point_labels(
    point_labels: np.ndarray,
    H: np.ndarray,
    min_xy: np.ndarray,
    grid_m: float,
    cell_point_indices: Dict[Tuple[int, int], np.ndarray],
    support_mask: np.ndarray | None = None,
) -> Tuple[List[o3d.geometry.TriangleMesh], List[Dict[str, float]], Dict[str, int]]:
    labels = np.asarray(point_labels, dtype=np.int32)
    unique_labels = np.unique(labels[labels > 0])

    grid_labels_sparse, ambiguity = _grid_majority_labels(labels, cell_point_indices, H.shape)
    support = np.asarray(support_mask, dtype=bool) if support_mask is not None else np.ones(H.shape, dtype=bool)
    grid_labels_dense, dense_info = _densify_grid_labels(grid_labels_sparse, support)

    meshes: List[o3d.geometry.TriangleMesh] = []
    summary: List[Dict[str, float]] = []

    for lab in unique_labels.tolist():
        lab_i = int(lab)
        point_count = int(np.count_nonzero(labels == lab_i))
        sparse_mask = grid_labels_sparse == lab_i
        sparse_cells = int(np.count_nonzero(sparse_mask))
        info: Dict[str, float] = {
            "label": float(lab_i),
            "point_count": float(point_count),
            "sparse_cells": float(sparse_cells),
            "mesh_cells": 0.0,
            "triangle_count": 0.0,
            "vertex_count": 0.0,
            "status": 0.0,
        }
        if point_count < int(mesh_min_points) or sparse_cells < int(mesh_min_cells):
            summary.append(info)
            continue

        dense_mask = grid_labels_dense == lab_i
        dense_mask = _close_mask(dense_mask, int(mesh_label_close_px))
        dense_mask &= support
        mesh_cells = int(np.count_nonzero(dense_mask))
        info["mesh_cells"] = float(mesh_cells)
        if mesh_cells < int(mesh_min_cells):
            summary.append(info)
            continue

        mesh = _grid_mask_to_heightfield_mesh(H, dense_mask, min_xy, grid_m)
        if mesh is None:
            summary.append(info)
            continue

        color = _labels_to_colors(np.array([lab_i], dtype=np.int32))[0]
        mesh.paint_uniform_color(color.tolist())
        info["triangle_count"] = float(len(mesh.triangles))
        info["vertex_count"] = float(len(mesh.vertices))
        info["status"] = 1.0
        meshes.append(mesh)
        summary.append(info)

    summary.sort(key=lambda row: int(row["label"]))
    debug = {
        "mixed_cells": int(ambiguity.get("mixed_cells", 0)),
        "empty_cells": int(ambiguity.get("empty_cells", 0)),
        "seeded_cells": int(dense_info.get("seeded_cells", 0)),
        "propagated_cells": int(dense_info.get("propagated_cells", 0)),
    }
    return meshes, summary, debug


def _show_2d_debug(
    H: np.ndarray,
    M: np.ndarray,
    DT_mm: np.ndarray,
    peaks_yx: np.ndarray,
    pause_ms: int,
    markers: np.ndarray | None = None,
    watershed_mask: np.ndarray | None = None,
    watershed_dt_mm: np.ndarray | None = None,
    grid_labels: np.ndarray | None = None,
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

    focus = str(debug_visual_focus).strip().lower()
    views: Dict[str, np.ndarray] = {}
    if focus == "watershed":
        if marker_overlay is not None:
            views["stage1_peak_markers"] = marker_overlay
        if watershed_mask is not None and watershed_mask.shape[:2] == DT_mm.shape[:2]:
            views["stage1_watershed_mask"] = np.repeat(
                (watershed_mask.astype(np.uint8) * 255)[:, :, None], 3, axis=2
            )
        if (
            watershed_dt_mm is not None
            and watershed_mask is not None
            and watershed_dt_mm.shape[:2] == DT_mm.shape[:2]
            and watershed_mask.shape[:2] == DT_mm.shape[:2]
        ):
            views["stage1_watershed_distance_transform"] = cv2.applyColorMap(
                _normalize_for_vis(watershed_dt_mm, watershed_mask),
                cv2.COLORMAP_INFERNO,
            )
        if (
            grid_labels is not None
            and watershed_mask is not None
            and grid_labels.shape[:2] == DT_mm.shape[:2]
            and watershed_mask.shape[:2] == DT_mm.shape[:2]
        ):
            views["stage1_watershed_labels"] = _colorize_grid_labels(
                grid_labels.astype(np.int32),
                watershed_mask,
            )
    else:
        views = {
            "stage1_height_map": h_vis,
            "stage1_mask": m_vis,
            "stage1_distance_transform": dt_vis,
            "stage1_detected_peaks": peak_overlay,
        }
        if marker_overlay is not None:
            views["stage1_peak_markers"] = marker_overlay
        if watershed_mask is not None and watershed_mask.shape[:2] == DT_mm.shape[:2]:
            views["stage1_watershed_mask"] = np.repeat(
                (watershed_mask.astype(np.uint8) * 255)[:, :, None], 3, axis=2
            )
        if (
            watershed_dt_mm is not None
            and watershed_mask is not None
            and watershed_dt_mm.shape[:2] == DT_mm.shape[:2]
            and watershed_mask.shape[:2] == DT_mm.shape[:2]
        ):
            views["stage1_watershed_distance_transform"] = cv2.applyColorMap(
                _normalize_for_vis(watershed_dt_mm, watershed_mask),
                cv2.COLORMAP_INFERNO,
            )
        if (
            grid_labels is not None
            and watershed_mask is not None
            and grid_labels.shape[:2] == DT_mm.shape[:2]
            and watershed_mask.shape[:2] == DT_mm.shape[:2]
        ):
            views["stage1_watershed_labels"] = _colorize_grid_labels(
                grid_labels.astype(np.int32),
                watershed_mask,
            )

    for name, img in views.items():
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.imshow(name, img)

    cv2.waitKey(int(pause_ms) if pause_ms >= 0 else 0)


def _show_3d_debug(
    points_xyz: np.ndarray,
    peaks_xyz: np.ndarray,
    geodesic_mm: np.ndarray,
    point_labels: np.ndarray | None = None,
    meshes: List[o3d.geometry.TriangleMesh] | None = None,
    ring_linesets: List[o3d.geometry.LineSet] | None = None,
) -> None:
    focus = str(debug_visual_focus).strip().lower()
    pcd_main = o3d.geometry.PointCloud()
    pcd_main.points = o3d.utility.Vector3dVector(points_xyz.astype(np.float64))

    if point_labels is not None and point_labels.shape[0] == points_xyz.shape[0]:
        pcd_main.colors = o3d.utility.Vector3dVector(_labels_to_colors(point_labels))
    elif colorize_cloud_by_height and points_xyz.shape[0] > 0:
        z = points_xyz[:, 2].astype(np.float32)
        zmin = float(np.min(z))
        zmax = float(np.max(z))
        if zmax - zmin > 1e-9:
            z_u8 = np.clip((z - zmin) / (zmax - zmin) * 255.0, 0, 255).astype(np.uint8)
        else:
            z_u8 = np.zeros_like(z, dtype=np.uint8)
        bgr = cv2.applyColorMap(z_u8.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
        rgb = bgr[:, ::-1].astype(np.float64) / 255.0
        pcd_main.colors = o3d.utility.Vector3dVector(rgb)
    else:
        base_color = np.tile(np.array([[0.75, 0.75, 0.75]], dtype=np.float64), (points_xyz.shape[0], 1))
        pcd_main.colors = o3d.utility.Vector3dVector(base_color)

    geoms: List[o3d.geometry.Geometry] = [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02)]
    if focus != "mesh3d" or bool(mesh_show_point_cloud_background) or not meshes:
        geoms.insert(0, pcd_main)
    if meshes:
        geoms.extend(meshes)

    if peaks_xyz.shape[0] > 0:
        r_m = _mm_to_m(float(peak_marker_radius_mm))
        for p in peaks_xyz:
            outer = o3d.geometry.TriangleMesh.create_sphere(radius=r_m)
            outer.compute_vertex_normals()
            outer.paint_uniform_color(list(peak_marker_outer_color))
            outer.translate(p)
            geoms.append(outer)

            inner = o3d.geometry.TriangleMesh.create_sphere(radius=0.55 * r_m)
            inner.compute_vertex_normals()
            inner.paint_uniform_color(list(peak_marker_inner_color))
            inner.translate(p)
            geoms.append(inner)

        peak_cloud = o3d.geometry.PointCloud()
        peak_cloud.points = o3d.utility.Vector3dVector(peaks_xyz.astype(np.float64))
        peak_cloud.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([[1.0, 0.0, 0.0]], dtype=np.float64), (peaks_xyz.shape[0], 1))
        )
        geoms.append(peak_cloud)

    # Adjacent panel: heat-method geodesic distances and contour-like ring curves.
    if (
        focus != "mesh3d"
        and bool(geodesic_enable)
        and bool(geodesic_show_heat_panel)
        and geodesic_mm.size == points_xyz.shape[0]
    ):
        if points_xyz.shape[0] > 0:
            ext = np.ptp(points_xyz, axis=0)
            side_offset_m = max(float(ext[0]), float(ext[1]), 0.05) + _mm_to_m(geodesic_panel_offset_mm)
        else:
            side_offset_m = 0.08
        shift = np.array([side_offset_m, 0.0, 0.0], dtype=np.float64)

        heat_cloud = o3d.geometry.PointCloud()
        heat_cloud.points = o3d.utility.Vector3dVector((points_xyz + shift[None, :]).astype(np.float64))
        heat_cloud.colors = o3d.utility.Vector3dVector(_geodesic_colors(geodesic_mm))
        geoms.append(heat_cloud)

        heat_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02)
        heat_frame.translate(shift)
        geoms.append(heat_frame)

        ring_cloud = _geodesic_ring_cloud(points_xyz, geodesic_mm)
        if ring_cloud is not None:
            ring_pts = np.asarray(ring_cloud.points, dtype=np.float64)
            ring_cloud.points = o3d.utility.Vector3dVector((ring_pts + shift[None, :]).astype(np.float64))
            geoms.append(ring_cloud)

        lines_to_draw = ring_linesets if ring_linesets is not None else _geodesic_ring_linesets(points_xyz, geodesic_mm)
        for ls in lines_to_draw:
            ls_shift = o3d.geometry.LineSet()
            ls_shift.points = o3d.utility.Vector3dVector(
                (np.asarray(ls.points, dtype=np.float64) + shift[None, :]).astype(np.float64)
            )
            ls_shift.lines = o3d.utility.Vector2iVector(np.asarray(ls.lines, dtype=np.int32))
            ls_shift.paint_uniform_color(list(geodesic_ring_color))
            geoms.append(ls_shift)

        if peaks_xyz.shape[0] > 0:
            r_h = _mm_to_m(float(peak_marker_radius_mm) * 0.85)
            for p in peaks_xyz:
                m = o3d.geometry.TriangleMesh.create_sphere(radius=r_h)
                m.compute_vertex_normals()
                m.paint_uniform_color([1.0, 0.92, 0.10])
                m.translate((p + shift).astype(np.float64))
                geoms.append(m)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Stage-1 Peaks + Heat-Geodesic Panel", width=1600, height=960)
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
    output_dir,
    show_3d=True,
    show_2d=True,
    viz_frame="table",
    pause_ms=0,
    export_colored=True,
    export_world_frame=False,
) -> Dict[str, object]:
    input_path = Path(input_path).expanduser().resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")
    output_dir = Path(output_dir).expanduser().resolve()

    viz_frame = str(viz_frame).strip().lower()
    if viz_frame not in {"table", "world"}:
        raise ValueError("viz_frame must be 'table' or 'world'.")

    focus = str(debug_visual_focus).strip().lower()
    if focus == "watershed" and show_3d:
        print("Visual debug focus is 'watershed'; suppressing 3D window for this run.")
        show_3d = False

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
    top_height_xyz_table, top_height_mm = _top_height_points_xyz(
        H=H,
        valid=valid,
        min_xy=min_xy,
        grid_m=grid_m,
        top_k=int(top_height_points_k),
    )

    print("[4] Build mask + distance transform")
    M, DT_mm, mask_th = _build_mask_and_dt(H, valid)
    print(f"  Mask cells: {int(np.count_nonzero(M))}, threshold(z): {mask_th:.6f} m")
    M_watershed, DT_watershed_mm = _build_watershed_mask_and_dt(H, valid, mask_th, M)
    print(
        "  Watershed mask: "
        f"cells={int(np.count_nonzero(M_watershed))}, "
        f"max_dt={float(np.max(DT_watershed_mm[M_watershed])) if np.any(M_watershed) else 0.0:.3f}mm"
    )

    print("[5] Detect peaks")
    print(
        "  Peak params: "
        f"mode={peak_detection_mode}, "
        f"min_dist={min_peak_distance_mm:.3f}mm, "
        f"strength_frac={peak_strength_fraction:.3f}, "
        f"min_dt={peak_min_dt_mm:.3f}mm, "
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
    watershed_labels: np.ndarray | None = None
    watershed_info: Dict[str, float | str] = {
        "status": "not_used",
        "energy_mode": "height",
        "num_labels_excluding_zero": 0.0,
        "num_labeled_cells": 0.0,
        "num_unlabeled_inside_mask": float(np.count_nonzero(M_watershed)),
    }
    peaks_xyz_grid = _peaks_to_xyz(peaks_yx, H, min_xy, grid_m)
    peaks_xyz_table, anchor_info = _anchor_peaks_to_points(peaks_xyz_grid, points_obj_table)

    mode = str(peak_info.get("mode", "unknown"))
    if mode == "height":
        thr_str = f"{peak_info['strong_thr']:.3f}mm(height)"
    else:
        thr_str = f"{peak_info['strong_thr']:.3f}mm(dt)"
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
    geodesic_mm_table = np.asarray(heat_fields["dist_min_mm"], dtype=np.float64)
    geodesic_info = dict(heat_fields["info"])
    geodesic_by_seed_table = np.asarray(heat_fields["dist_by_seed_mm"], dtype=np.float64)
    geodesic_owner_table = np.asarray(heat_fields["owner_seed"], dtype=np.int32)
    geodesic_seed_idx_table = np.asarray(heat_fields["seed_vertex_idx"], dtype=np.int32)
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

    print("[8] Height-field mesh reconstruction from heat-labeled support")
    mesh_geoms_table: List[o3d.geometry.TriangleMesh] = []
    mesh_summary: List[Dict[str, float]] = []
    mesh_debug: Dict[str, int] = {}
    if bool(mesh_enable):
        mesh_geoms_table, mesh_summary, mesh_debug = _build_meshes_from_point_labels(
            point_labels_table,
            H,
            min_xy,
            grid_m,
            cell_point_indices,
            support_mask=M_watershed,
        )
    if len(mesh_summary) == 0:
        print("  No mesh candidates available.")
    else:
        print(
            "  Height-field grid ownership: "
            f"mixed_cells={int(mesh_debug.get('mixed_cells', 0))}, "
            f"empty_cells={int(mesh_debug.get('empty_cells', 0))}, "
            f"seeded_cells={int(mesh_debug.get('seeded_cells', 0))}, "
            f"propagated_cells={int(mesh_debug.get('propagated_cells', 0))}"
        )
        print("  label | points | sparse_cells | mesh_cells | verts | tris | status")
        for row in mesh_summary:
            status = "ok" if int(row["status"]) == 1 else "skip"
            print(
                f"  {int(row['label']):5d} | "
                f"{int(row['point_count']):6d} | "
                f"{int(row['sparse_cells']):12d} | "
                f"{int(row['mesh_cells']):10d} | "
                f"{int(row['vertex_count']):5d} | "
                f"{int(row['triangle_count']):4d} | "
                f"{status}"
            )

    if viz_frame == "world":
        points_viz = _transform_points(points_obj_table, T_world_from_table)
        peaks_viz = _transform_points(peaks_xyz_table, T_world_from_table) if peaks_xyz_table.size else peaks_xyz_table
        geodesic_mm_viz = geodesic_mm_table
    else:
        points_viz = points_obj_table
        peaks_viz = peaks_xyz_table
        geodesic_mm_viz = geodesic_mm_table
    point_labels_viz = point_labels_table
    mesh_geoms_viz = mesh_geoms_table
    if viz_frame == "world" and len(mesh_geoms_viz) > 0:
        mesh_geoms_world: List[o3d.geometry.TriangleMesh] = []
        for mesh in mesh_geoms_viz:
            mesh_w = o3d.geometry.TriangleMesh(mesh)
            mesh_w.transform(T_world_from_table)
            mesh_geoms_world.append(mesh_w)
        mesh_geoms_viz = mesh_geoms_world

    ring_lines_viz: List[o3d.geometry.LineSet] | None = None
    if show_3d and bool(geodesic_enable) and bool(geodesic_show_rings) and geodesic_by_seed_table.size > 0:
        ring_lines_viz = _seeded_geodesic_ring_linesets(
            points_viz,
            geodesic_by_seed_table,
            geodesic_owner_table,
            seed_vertex_idx=geodesic_seed_idx_table,
        )
        print(f"  Heat ring loops: {len(ring_lines_viz)}")

    if focus == "segmentation3d":
        geodesic_mm_viz = np.empty((0,), dtype=np.float64)
        ring_lines_viz = None
    if focus == "mesh3d":
        geodesic_mm_viz = np.empty((0,), dtype=np.float64)
        ring_lines_viz = None

    print("[5.4] Heat-geodesic 3D label visualization")
    unique_labels_viz = np.unique(point_labels_viz[point_labels_viz > 0])
    print(
        "  3D colors: "
        "label_source=heat, "
        f"visualized_labels={int(unique_labels_viz.size)}, "
        f"background_label_color={tuple(float(v) for v in label_background_color)}, "
        "deterministic_mapping=True"
    )
    if focus == "mesh3d":
        print(f"  3D meshes: count={len(mesh_geoms_viz)}, point_background={bool(mesh_show_point_cloud_background)}")

    if top_height_xyz_table.shape[0] > 0:
        print(f"[6] Top {top_height_xyz_table.shape[0]} highest height-map points (table frame)")
        for i, (p, z_mm) in enumerate(zip(top_height_xyz_table, top_height_mm), start=1):
            print(
                f"  {i:02d}: x={p[0]*1e3:.3f} mm, y={p[1]*1e3:.3f} mm, z={z_mm:.3f} mm"
            )

    if show_2d:
        _show_2d_debug(
            H,
            M,
            DT_mm,
            peaks_yx,
            pause_ms=int(pause_ms),
            markers=peak_markers,
            watershed_mask=M_watershed,
            watershed_dt_mm=DT_watershed_mm,
            grid_labels=watershed_labels,
        )

    if show_3d:
        _show_3d_debug(
            points_viz,
            peaks_viz,
            geodesic_mm_viz,
            point_labels=point_labels_viz,
            meshes=mesh_geoms_viz,
            ring_linesets=ring_lines_viz,
        )

    export_info_table: Dict[str, object] | None = None
    export_info_world: Dict[str, object] | None = None
    if bool(export_colored):
        print("[6] Export colored segmented point cloud")
        table_path = output_dir / "segmented_beads_colored_table.ply"
        export_info_table = _export_colored_point_cloud(points_obj_table, point_labels_table, table_path)
        print(
            "  Exported table frame: "
            f"path={export_info_table['output_path']}, "
            f"points={int(export_info_table['num_points'])}, "
            f"labels={int(export_info_table['num_labels'])}"
        )
        if bool(export_world_frame):
            world_points = _transform_points(points_obj_table, T_world_from_table)
            world_path = output_dir / "segmented_beads_colored_world.ply"
            export_info_world = _export_colored_point_cloud(world_points, point_labels_table, world_path)
            print(
                "  Exported world frame: "
                f"path={export_info_world['output_path']}, "
                f"points={int(export_info_world['num_points'])}, "
                f"labels={int(export_info_world['num_labels'])}"
            )

    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass

    return {
        "input_path": str(input_path),
        "num_points_loaded": int(points_world.shape[0]),
        "num_points_above_table": int(points_obj_table.shape[0]),
        "num_peaks": int(peaks_xyz_table.shape[0]),
        "peak_filtering": peak_info,
        "peak_anchoring": anchor_info,
        "label_source": "heat",
        "watershed": watershed_info,
        "point_labels_table": point_labels_table.tolist(),
        "label_summary": label_summary,
        "mesh_summary": mesh_summary,
        "mesh_debug": mesh_debug,
        "geodesic": geodesic_info,
        "viz_frame": viz_frame,
        "plane_model_world": plane.tolist(),
        "table_transform": T_table_from_world.tolist(),
        "peaks_xyz_table": peaks_xyz_table.tolist(),
        "top_height_xyz_table": top_height_xyz_table.tolist(),
        "export_table": export_info_table,
        "export_world": export_info_world,
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stage-1 debug: visualize point cloud with detected peaks.")
    parser.add_argument("--input", default=default_input_path, help="Input point cloud path (PLY/PCD)")
    parser.add_argument("--output_dir", default=default_output_dir, help="Output directory for exports")

    parser.add_argument("--show_3d", dest="show_3d", action="store_true", default=bool(default_show_3d))
    parser.add_argument("--no_show_3d", dest="show_3d", action="store_false")

    parser.add_argument("--show_2d", dest="show_2d", action="store_true", default=bool(default_show_2d))
    parser.add_argument("--no_show_2d", dest="show_2d", action="store_false")

    parser.add_argument("--export_colored", dest="export_colored", action="store_true", default=bool(default_export_colored))
    parser.add_argument("--no_export_colored", dest="export_colored", action="store_false")
    parser.add_argument(
        "--export_world_frame",
        dest="export_world_frame",
        action="store_true",
        default=bool(default_export_world_frame),
    )
    parser.add_argument("--no_export_world_frame", dest="export_world_frame", action="store_false")

    parser.add_argument("--viz_frame", choices=["table", "world"], default=default_viz_frame)
    parser.add_argument("--pause_ms", type=int, default=int(default_pause_ms))

    parser.add_argument("--min_peak_distance_mm", type=float, default=float(min_peak_distance_mm))
    parser.add_argument("--peak_strength_fraction", type=float, default=float(peak_strength_fraction))
    parser.add_argument("--peak_min_dt_mm", type=float, default=float(peak_min_dt_mm))
    parser.add_argument("--peak_keep_percentile", type=float, default=float(peak_keep_percentile))
    parser.add_argument("--max_num_peaks", type=int, default=int(max_num_peaks))
    parser.add_argument("--peak_marker_radius_mm", type=float, default=float(peak_marker_radius_mm))
    parser.add_argument("--geodesic_heat_t_coef", type=float, default=float(geodesic_heat_t_coef))

    args = parser.parse_args()
    if not str(args.input).strip():
        parser.error("No input file provided. Set default_input_path at top or pass --input.")
    return args


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

    if bool(use_cli_args):
        args = _parse_args()

        # Apply CLI tuning overrides to stage-1 globals.
        global min_peak_distance_mm, peak_strength_fraction
        global peak_min_dt_mm, peak_keep_percentile, max_num_peaks
        global peak_marker_radius_mm, geodesic_heat_t_coef
        min_peak_distance_mm = float(args.min_peak_distance_mm)
        peak_strength_fraction = float(args.peak_strength_fraction)
        peak_min_dt_mm = float(args.peak_min_dt_mm)
        peak_keep_percentile = float(args.peak_keep_percentile)
        max_num_peaks = max(1, int(args.max_num_peaks))
        peak_marker_radius_mm = max(0.1, float(args.peak_marker_radius_mm))
        geodesic_heat_t_coef = max(1e-3, float(args.geodesic_heat_t_coef))

        input_path = args.input
        output_dir = args.output_dir
        show_3d = bool(args.show_3d)
        show_2d = bool(args.show_2d)
        viz_frame = args.viz_frame
        pause_ms = int(args.pause_ms)
        export_colored = bool(args.export_colored)
        export_world_frame = bool(args.export_world_frame)
    else:
        input_path = default_input_path
        output_dir = default_output_dir
        show_3d = bool(default_show_3d)
        show_2d = bool(default_show_2d)
        viz_frame = default_viz_frame
        pause_ms = int(default_pause_ms)
        export_colored = bool(default_export_colored)
        export_world_frame = bool(default_export_world_frame)
        print("Using in-script parameters (use_cli_args=False).")

    result = run_first_stage_peak_debug(
        input_path=input_path,
        output_dir=output_dir,
        show_3d=show_3d,
        show_2d=show_2d,
        viz_frame=viz_frame,
        pause_ms=pause_ms,
        export_colored=export_colored,
        export_world_frame=export_world_frame,
    )
    print("\nDone.")
    print(f"Peaks detected: {result['num_peaks']}")


if __name__ == "__main__":
    main()
