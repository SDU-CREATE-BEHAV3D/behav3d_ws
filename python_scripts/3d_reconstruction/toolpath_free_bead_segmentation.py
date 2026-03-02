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
from pathlib import Path
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


# -----------------------------------------------------------------------------
# Stage-1 parameters (all distances in millimeters)
# -----------------------------------------------------------------------------
downsample_voxel_mm = 0.70  # <= 0 disables downsampling

# Table alignment / slicing
table_ransac_thresh_mm = 1.2
table_ransac_n = 3
table_ransac_iters = 2500
z_min_above_plane_mm = 0.35

# Height map / mask / peak detection
grid_mm = 0.30
height_percentile = 97.0
gaussian_sigma_px = 1.0
mask_percentile = 60.0
morph_kernel_px = 3
peak_detection_mode = "height"  # "height" or "dt"
peak_height_support_percentile = 70.0
peak_min_height_mm = -2.0
min_peak_distance_mm = 4
peak_strength_fraction = 0.20
peak_min_dt_mm = 0.10
peak_keep_percentile = 20.0  # keep only top DT peaks by value
max_num_peaks = 500

# 3D peak marker display
peak_marker_radius_mm = 1.6
peak_marker_outer_color = (1.0, 0.95, 0.15)
peak_marker_inner_color = (1.0, 0.05, 0.05)
colorize_cloud_by_height = True

# Peak-to-point anchoring
peak_anchor_mode = "nearest3d"  # "grid" or "nearest3d"
peak_max_snap_z_error_mm = 1.5

# Side visualization of top-N height-map points
top_height_points_k = 100
top_height_marker_radius_mm = 1.8
top_height_marker_color = (0.10, 1.00, 1.00)
top_height_side_offset_mm = 30.0


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


def _build_height_map(points_table: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
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

    for grp in groups:
        lin = int(linear[grp[0]])
        y = lin // nx
        x = lin % nx
        H[y, x] = float(np.percentile(z[grp], float(height_percentile)))
        valid[y, x] = True

    H = _nearest_fill_height(H, valid)
    H = _weighted_gaussian(H, valid, sigma_px=float(gaussian_sigma_px)).astype(np.float32)
    return H, valid, min_xy.astype(np.float64), g


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


def _show_2d_debug(H: np.ndarray, M: np.ndarray, DT_mm: np.ndarray, peaks_yx: np.ndarray, pause_ms: int) -> None:
    h_vis = cv2.applyColorMap(_normalize_for_vis(H), cv2.COLORMAP_TURBO)
    m_vis = np.repeat((M.astype(np.uint8) * 255)[:, :, None], 3, axis=2)
    dt_vis = cv2.applyColorMap(_normalize_for_vis(DT_mm, M), cv2.COLORMAP_INFERNO)

    peak_overlay = dt_vis.copy()
    for y, x in peaks_yx:
        cv2.circle(peak_overlay, (int(x), int(y)), 3, (0, 255, 255), -1, lineType=cv2.LINE_AA)
        cv2.circle(peak_overlay, (int(x), int(y)), 5, (0, 0, 0), 1, lineType=cv2.LINE_AA)

    views = {
        "stage1_height_map": h_vis,
        "stage1_mask": m_vis,
        "stage1_distance_transform": dt_vis,
        "stage1_detected_peaks": peak_overlay,
    }

    for name, img in views.items():
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.imshow(name, img)

    cv2.waitKey(int(pause_ms) if pause_ms >= 0 else 0)


def _show_3d_debug(points_xyz: np.ndarray, peaks_xyz: np.ndarray, top_height_xyz: np.ndarray) -> None:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz.astype(np.float64))

    if colorize_cloud_by_height and points_xyz.shape[0] > 0:
        z = points_xyz[:, 2].astype(np.float32)
        zmin = float(np.min(z))
        zmax = float(np.max(z))
        if zmax - zmin > 1e-9:
            z_u8 = np.clip((z - zmin) / (zmax - zmin) * 255.0, 0, 255).astype(np.uint8)
        else:
            z_u8 = np.zeros_like(z, dtype=np.uint8)
        bgr = cv2.applyColorMap(z_u8.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
        rgb = bgr[:, ::-1].astype(np.float64) / 255.0
        pcd.colors = o3d.utility.Vector3dVector(rgb)
    else:
        base_color = np.tile(np.array([[0.75, 0.75, 0.75]], dtype=np.float64), (points_xyz.shape[0], 1))
        pcd.colors = o3d.utility.Vector3dVector(base_color)

    geoms = [pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02)]

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

    # Side panel: duplicate cloud + top-N highest height-map points.
    if top_height_xyz.shape[0] > 0:
        if points_xyz.shape[0] > 0:
            ext = np.ptp(points_xyz, axis=0)
            side_offset_m = max(float(ext[0]), float(ext[1]), 0.05) + _mm_to_m(top_height_side_offset_mm)
        else:
            side_offset_m = 0.08
        shift = np.array([side_offset_m, 0.0, 0.0], dtype=np.float64)

        side_cloud = o3d.geometry.PointCloud()
        side_cloud.points = o3d.utility.Vector3dVector((points_xyz + shift[None, :]).astype(np.float64))
        side_cloud.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([[0.28, 0.28, 0.28]], dtype=np.float64), (points_xyz.shape[0], 1))
        )
        geoms.append(side_cloud)

        side_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.02)
        side_frame.translate(shift)
        geoms.append(side_frame)

        r_side = _mm_to_m(float(top_height_marker_radius_mm))
        for p in top_height_xyz:
            m = o3d.geometry.TriangleMesh.create_sphere(radius=r_side)
            m.compute_vertex_normals()
            m.paint_uniform_color(list(top_height_marker_color))
            m.translate((p + shift).astype(np.float64))
            geoms.append(m)

        top_side_cloud = o3d.geometry.PointCloud()
        top_side_cloud.points = o3d.utility.Vector3dVector((top_height_xyz + shift[None, :]).astype(np.float64))
        top_side_cloud.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([top_height_marker_color], dtype=np.float64), (top_height_xyz.shape[0], 1))
        )
        geoms.append(top_side_cloud)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Stage-1 Peaks on Point Cloud", width=1400, height=920)
    for g in geoms:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    if opt is not None:
        opt.background_color = np.array([0.05, 0.05, 0.05], dtype=np.float64)
        opt.point_size = 2.5
        opt.light_on = True
    vis.run()
    vis.destroy_window()


def run_first_stage_peak_debug(
    input_path,
    show_3d=True,
    show_2d=True,
    viz_frame="table",
    pause_ms=0,
) -> Dict[str, object]:
    input_path = Path(input_path).expanduser().resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

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
    H, valid, min_xy, grid_m = _build_height_map(points_obj_table)
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

    if viz_frame == "world":
        points_viz = _transform_points(points_obj_table, T_world_from_table)
        peaks_viz = _transform_points(peaks_xyz_table, T_world_from_table) if peaks_xyz_table.size else peaks_xyz_table
        top_height_viz = (
            _transform_points(top_height_xyz_table, T_world_from_table)
            if top_height_xyz_table.size
            else top_height_xyz_table
        )
    else:
        points_viz = points_obj_table
        peaks_viz = peaks_xyz_table
        top_height_viz = top_height_xyz_table

    if top_height_xyz_table.shape[0] > 0:
        print(f"[6] Top {top_height_xyz_table.shape[0]} highest height-map points (table frame)")
        for i, (p, z_mm) in enumerate(zip(top_height_xyz_table, top_height_mm), start=1):
            print(
                f"  {i:02d}: x={p[0]*1e3:.3f} mm, y={p[1]*1e3:.3f} mm, z={z_mm:.3f} mm"
            )

    if show_2d:
        _show_2d_debug(H, M, DT_mm, peaks_yx, pause_ms=int(pause_ms))

    if show_3d:
        _show_3d_debug(points_viz, peaks_viz, top_height_viz)

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
        "viz_frame": viz_frame,
        "plane_model_world": plane.tolist(),
        "table_transform": T_table_from_world.tolist(),
        "peaks_xyz_table": peaks_xyz_table.tolist(),
        "top_height_xyz_table": top_height_xyz_table.tolist(),
    }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stage-1 debug: visualize point cloud with detected peaks.")
    parser.add_argument("--input", default=default_input_path, help="Input point cloud path (PLY/PCD)")

    parser.add_argument("--show_3d", dest="show_3d", action="store_true", default=bool(default_show_3d))
    parser.add_argument("--no_show_3d", dest="show_3d", action="store_false")

    parser.add_argument("--show_2d", dest="show_2d", action="store_true", default=bool(default_show_2d))
    parser.add_argument("--no_show_2d", dest="show_2d", action="store_false")

    parser.add_argument("--viz_frame", choices=["table", "world"], default=default_viz_frame)
    parser.add_argument("--pause_ms", type=int, default=int(default_pause_ms))

    parser.add_argument("--min_peak_distance_mm", type=float, default=float(min_peak_distance_mm))
    parser.add_argument("--peak_strength_fraction", type=float, default=float(peak_strength_fraction))
    parser.add_argument("--peak_min_dt_mm", type=float, default=float(peak_min_dt_mm))
    parser.add_argument("--peak_keep_percentile", type=float, default=float(peak_keep_percentile))
    parser.add_argument("--max_num_peaks", type=int, default=int(max_num_peaks))
    parser.add_argument("--peak_marker_radius_mm", type=float, default=float(peak_marker_radius_mm))

    args = parser.parse_args()
    if not str(args.input).strip():
        parser.error("No input file provided. Set default_input_path at top or pass --input.")
    return args


def main() -> None:
    if bool(use_cli_args):
        args = _parse_args()

        # Apply CLI tuning overrides to stage-1 globals.
        global min_peak_distance_mm, peak_strength_fraction
        global peak_min_dt_mm, peak_keep_percentile, max_num_peaks
        global peak_marker_radius_mm
        min_peak_distance_mm = float(args.min_peak_distance_mm)
        peak_strength_fraction = float(args.peak_strength_fraction)
        peak_min_dt_mm = float(args.peak_min_dt_mm)
        peak_keep_percentile = float(args.peak_keep_percentile)
        max_num_peaks = max(1, int(args.max_num_peaks))
        peak_marker_radius_mm = max(0.1, float(args.peak_marker_radius_mm))

        input_path = args.input
        show_3d = bool(args.show_3d)
        show_2d = bool(args.show_2d)
        viz_frame = args.viz_frame
        pause_ms = int(args.pause_ms)
    else:
        input_path = default_input_path
        show_3d = bool(default_show_3d)
        show_2d = bool(default_show_2d)
        viz_frame = default_viz_frame
        pause_ms = int(default_pause_ms)
        print("Using in-script parameters (use_cli_args=False).")

    result = run_first_stage_peak_debug(
        input_path=input_path,
        show_3d=show_3d,
        show_2d=show_2d,
        viz_frame=viz_frame,
        pause_ms=pause_ms,
    )
    print("\nDone.")
    print(f"Peaks detected: {result['num_peaks']}")


if __name__ == "__main__":
    main()
