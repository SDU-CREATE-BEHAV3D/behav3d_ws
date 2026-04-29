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


def make_segment_line_set(
    start_points: np.ndarray,
    end_points: np.ndarray,
    color: tuple[float, float, float],
) -> o3d.geometry.LineSet:
    """Build one segment per paired start/end point."""
    starts = np.asarray(start_points, dtype=np.float64)
    ends = np.asarray(end_points, dtype=np.float64)
    if starts.shape != ends.shape or starts.ndim != 2 or starts.shape[1] != 3:
        raise ValueError("start_points and end_points must both have shape (N, 3)")

    if starts.shape[0] == 0:
        points = np.zeros((0, 3), dtype=np.float64)
        lines = np.zeros((0, 2), dtype=np.int32)
        return make_line_set(points, lines, color=color)

    n = starts.shape[0]
    points = np.vstack([starts, ends])
    lines = np.column_stack(
        [np.arange(n, dtype=np.int32), np.arange(n, dtype=np.int32) + n]
    )
    return make_line_set(points, lines, color=color)


def _rotation_from_z_axis(direction: np.ndarray) -> np.ndarray:
    """Rotation matrix mapping local +Z to `direction`."""
    z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    d = np.asarray(direction, dtype=np.float64)
    n = float(np.linalg.norm(d))
    if n <= 1e-12:
        return np.eye(3, dtype=np.float64)
    d = d / n

    c = float(np.dot(z_axis, d))
    if c > 1.0 - 1e-12:
        return np.eye(3, dtype=np.float64)
    if c < -1.0 + 1e-12:
        return np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
            ],
            dtype=np.float64,
        )

    axis = np.cross(z_axis, d)
    s = max(float(np.linalg.norm(axis)), 1e-12)
    axis = axis / s
    ax = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + ax * s + (ax @ ax) * (1.0 - c)


def make_target_orientation_sticks(
    points: np.ndarray,
    z_dirs: np.ndarray,
) -> o3d.geometry.TriangleMesh:
    """Build fixed target-orientation sticks: 8 mm long, 1 mm diameter."""
    pts = np.asarray(points, dtype=np.float64)
    dirs = np.asarray(z_dirs, dtype=np.float64)
    if pts.shape != dirs.shape or pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points and z_dirs must both have shape (N, 3)")

    length = 0.008
    radius = 0.0005
    color = (0.10, 0.95, 0.10)
    mesh = o3d.geometry.TriangleMesh()

    for i in range(pts.shape[0]):
        p = pts[i]
        d = dirs[i]
        n = float(np.linalg.norm(d))
        if n <= 1e-12 or not np.all(np.isfinite(p)) or not np.all(np.isfinite(d)):
            continue
        u = d / n
        stick = o3d.geometry.TriangleMesh.create_cylinder(
            radius=radius,
            height=length,
            resolution=12,
            split=1,
        )
        stick.paint_uniform_color(color)
        stick.rotate(_rotation_from_z_axis(u), center=np.zeros(3, dtype=np.float64))
        stick.translate(p + 0.5 * length * u)
        mesh += stick

    if len(mesh.triangles) > 0:
        mesh.compute_vertex_normals()
    return mesh


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
