#!/usr/bin/env python3
"""Visualization helpers for inspecting intermediate scalar-field results.

This module turns numeric outputs into quick debug artifacts.
"""

from __future__ import annotations

import numpy as np
import open3d as o3d


def yellow_to_red_colors(norm_scalar: np.ndarray) -> np.ndarray:
    """Map normalized scalar in [0,1] to yellow->red color ramp."""
    norm = np.clip(norm_scalar, 0.0, 1.0)
    colors = np.zeros((norm.shape[0], 3), dtype=np.float64)
    colors[:, 0] = 1.0
    colors[:, 1] = 1.0 - norm
    colors[:, 2] = 0.0
    return colors


def make_point_cloud(points: np.ndarray, colors: np.ndarray) -> o3d.geometry.PointCloud:
    """Build colored Open3D point cloud from arrays."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def make_line_set(
    points: np.ndarray,
    lines: np.ndarray,
    color: tuple[float, float, float],
) -> o3d.geometry.LineSet:
    """Build colored Open3D line set from points + segment indices."""
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines = o3d.utility.Vector2iVector(lines.astype(np.int32))
    if lines.shape[0] > 0:
        colors = np.tile(np.array(color, dtype=np.float64), (lines.shape[0], 1))
        ls.colors = o3d.utility.Vector3dVector(colors)
    return ls


def compute_scene_bounds(*point_sets: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Compute global AABB bounds over one or more point sets."""
    valid_sets = [p for p in point_sets if p is not None and p.size > 0]
    if not valid_sets:
        raise ValueError("No points available to compute scene bounds.")
    mins = [np.min(p, axis=0) for p in valid_sets]
    maxs = [np.max(p, axis=0) for p in valid_sets]
    bb_min = np.min(np.vstack(mins), axis=0)
    bb_max = np.max(np.vstack(maxs), axis=0)
    return bb_min, bb_max
