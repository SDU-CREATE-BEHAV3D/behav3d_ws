from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from .common import (
    TOOL_PLUS_Z_POINTS_OUTWARD,
    order_targets_by_nearest_neighbor,
    pose_from_position_rotation,
    rotation_from_z_axis,
    target_origin_rotation,
)


def build_targets(
    *,
    target: PoseStamped,
    distance: float,
    cap_rad: float,
    samples: int,
    z_jitter: float = 0.0,
    order_start_xyz: Optional[Sequence[float]] = None,
    orientation_mode: str = "look_at",
    orientation_pose: Optional[PoseStamped] = None,
) -> list[PoseStamped]:
    if not isinstance(target, PoseStamped):
        raise TypeError("target must be geometry_msgs.msg.PoseStamped")
    if int(samples) <= 0:
        return []

    p_t, R_t = target_origin_rotation(target)
    dirs_local = fibonacci_cap_dirs_np(float(cap_rad), int(samples))
    poses: list[PoseStamped] = []
    rz_adjust = R.from_rotvec([0.0, 0.0, -math.pi / 2.0])

    mode = str(orientation_mode or "look_at").strip().lower()
    fixed_rotation = None
    reference_rotation = None
    if mode in ("fixed", "current", "current_eef", "target"):
        fixed_source = orientation_pose if orientation_pose is not None else target
        _, fixed_rotation = target_origin_rotation(fixed_source)
    elif mode in ("look_at_current_roll", "look_at_keep_roll", "look_at_min_roll"):
        reference_source = orientation_pose if orientation_pose is not None else target
        _, reference_rotation = target_origin_rotation(reference_source)

    for d in dirs_local:
        p_loc = float(distance) * d
        p_w = p_t + R_t.apply(p_loc)

        if fixed_rotation is not None:
            R_w = fixed_rotation
        elif reference_rotation is not None:
            z_axis = d if TOOL_PLUS_Z_POINTS_OUTWARD else -d
            z_axis_w = R_t.apply(z_axis)
            x_guess_w = reference_rotation.as_matrix()[:, 0]
            R_w = rotation_from_z_axis(z_axis_w, x_guess=x_guess_w)
        else:
            z_axis = d if TOOL_PLUS_Z_POINTS_OUTWARD else -d
            R_loc = rotation_from_z_axis(z_axis, x_guess=np.array([1.0, 0.0, 0.0])) * rz_adjust
            R_w = R_t * R_loc

        if float(z_jitter) > 0.0:
            ray_dir = p_t - p_w
            n_ray = np.linalg.norm(ray_dir)
            if n_ray > 1e-9:
                p_w = p_w + np.random.uniform(-float(z_jitter), float(z_jitter)) * (ray_dir / n_ray)

        poses.append(pose_from_position_rotation(p_w, R_w, target.header.frame_id or "world"))

    return order_targets_by_nearest_neighbor(poses, order_start_xyz=order_start_xyz)


def fibonacci_cap_dirs_np(cap_rad: float, n: int) -> np.ndarray:
    if n <= 0:
        return np.zeros((0, 3), dtype=float)
    cos_cap = float(np.cos(cap_rad))
    i = np.arange(n, dtype=float)
    z = cos_cap + (1.0 - cos_cap) * ((i + 0.5) / n)
    r_xy = np.sqrt(np.maximum(0.0, 1.0 - z * z))
    golden = (1.0 + np.sqrt(5.0)) * 0.5
    lon = 2.0 * np.pi * (i + 0.5) / golden
    dirs = np.stack([r_xy * np.cos(lon), r_xy * np.sin(lon), z], axis=1)
    norms = np.linalg.norm(dirs, axis=1, keepdims=True)
    norms = np.where(norms < 1e-12, 1.0, norms)
    return dirs / norms
