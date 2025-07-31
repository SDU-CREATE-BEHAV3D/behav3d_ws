#!/usr/bin/env python3
#
# =============================================================================
#   ____  _____ _   _    ___     _______ ____  
#  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
#  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
#  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
#  |____/|_____|_| |_/_/   \_\_/  |____/|____/ 
#                                               
# Institute: University of Southern Denmark — SDU Robotics
# =============================================================================

from __future__ import annotations

import math
import numpy as np
from typing import List, Tuple, Sequence, overload
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from .target_helpers import world_xy, look_at, pose_stamped, matrix_from_pose, pose_from_matrix

__all__ = [
    "grid_xy",
    "fibonacci_outside_in",
    "fibonacci_inside_out",
    "sweep_zigzag",
    "add_jitter",
]

# -----------------------------------------------------------------------------
# Helper ‑ rotation utilities
# -----------------------------------------------------------------------------

def _axes_from_pose(p: PoseStamped) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:

    rot = R.from_quat([
        p.pose.orientation.x,
        p.pose.orientation.y,
        p.pose.orientation.z,
        p.pose.orientation.w,
    ])
    x = rot.apply([1.0, 0.0, 0.0])
    y = rot.apply([0.0, 1.0, 0.0])
    z = rot.apply([0.0, 0.0, 1.0])
    return x, y, z

# -----------------------------------------------------------------------------
# 1. Planar grid
# -----------------------------------------------------------------------------

def grid_xy(
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    z: float,
    *,
    spacing: float = 0.05,
    frame: str = "world",
    flipped: bool = True,
) -> List[PoseStamped]:

    xs = np.arange(x_range[0], x_range[1] + 1e-9, spacing)
    ys = np.arange(y_range[0], y_range[1] + 1e-9, spacing)
    poses: List[PoseStamped] = []
    for y in ys:
        for x in xs:
            poses.append(world_xy(x, y, z, frame_id=frame, flipped=flipped))
    return poses

# -----------------------------------------------------------------------------
# 2. Fibonacci spiral on a spherical cone
# -----------------------------------------------------------------------------

_GA = math.pi * (3.0 - math.sqrt(5.0))  # golden angle ~ 2.39996 rad

def _fibonacci_directions(
    n: int,
    cone_half_angle: float | None = None,
) -> np.ndarray:

    dirs = np.zeros((n, 3))
    if cone_half_angle is None:
        z_vals = 1 - (2 * np.arange(n) + 1) / n      # full sphere (−1…1)
    else:
        z_max = math.cos(cone_half_angle)
        z_vals = z_max + (1 - z_max) * (np.arange(n) + 0.5) / n

    for k, z in enumerate(z_vals):
        radius_xy = math.sqrt(max(0.0, 1 - z * z))
        theta     = _GA * k
        dirs[k]   = [
            radius_xy * math.cos(theta),
            radius_xy * math.sin(theta),
            z
        ]
    return dirs

def _orient_and_place(
    target: PoseStamped,
    directions: np.ndarray,
    radius: float,
    *,
    inward: bool,
) -> List[PoseStamped]:

    rot_target = R.from_quat([
        target.pose.orientation.x,
        target.pose.orientation.y,
        target.pose.orientation.z,
        target.pose.orientation.w,
    ])
    center     = np.array([
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
    ])

    poses: List[PoseStamped] = []
    for dir_local in directions:
        dir_world  = rot_target.apply(dir_local)
        cam_pos    = center + radius * dir_world
        cam_pose   = pose_stamped(*cam_pos, frame_id=target.header.frame_id)

        if inward:
            cam_pose = look_at(cam_pose, tuple(center))
        else:  # outward
            cam_pose = look_at(target, tuple(cam_pos))  # invert => target looks at cam ⇒ cam looks outward
        poses.append(cam_pose)
    return poses

def fibonacci_outside_in(
    target: PoseStamped,
    *,
    radius: float,
    cone_half_angle: float,
    n: int,
) -> List[PoseStamped]:

    dirs = _fibonacci_directions(n, cone_half_angle)
    return _orient_and_place(target, dirs, radius, inward=True)

def fibonacci_inside_out(
    target: PoseStamped,
    *,
    radius: float,
    cone_half_angle: float,
    n: int,
) -> List[PoseStamped]:

    dirs = _fibonacci_directions(n, cone_half_angle)
    return _orient_and_place(target, dirs, radius, inward=False)

# -----------------------------------------------------------------------------
# 3. Sweep (raster) trajectory in local frame
# -----------------------------------------------------------------------------

def sweep_zigzag(
    target: PoseStamped,
    *,
    width: float,
    height: float,
    z_dist: float,
    n_x: int,
    n_y: int,
    row_major: bool = True,
) -> List[PoseStamped]:
    """
    Raster‑scan rectangle **in the XY plane of *target***.

    Parameters
    ----------
    width, height
        Size of rectangle in metres.
    z_dist
        Distance above the plane along +Z of *target*.
    n_x, n_y
        Number of sample points along X and Y (≥ 2 each).
    row_major
        If ``True`` (default) sweep row‑by‑row (Y outer loop, X inner, serpentine).
        If ``False`` sweep column‑by‑column (X outer loop, Y inner, serpentine).
    """
    if n_x < 2 or n_y < 2:
        raise ValueError("n_x and n_y must be ≥ 2.")

    xs = np.linspace(-width / 2,  width / 2,  n_x)
    ys = np.linspace(-height / 2, height / 2, n_y)

    x_axis, y_axis, z_axis = _axes_from_pose(target)
    center = np.array([
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
    ])

    def _pose_at(dx: float, dy: float) -> PoseStamped:
        offset   = dx * x_axis + dy * y_axis + z_dist * z_axis
        cam_pos  = center + offset
        cam_pose = pose_stamped(*cam_pos, frame_id=target.header.frame_id)
        return look_at(cam_pose, tuple(center))

    poses: List[PoseStamped] = []

    if row_major:
        for j, dy in enumerate(ys):
            xs_row = xs if (j % 2 == 0) else xs[::-1]
            for dx in xs_row:
                poses.append(_pose_at(dx, dy))
    else:  # column‑major
        for i, dx in enumerate(xs):
            ys_col = ys if (i % 2 == 0) else ys[::-1]
            for dy in ys_col:
                poses.append(_pose_at(dx, dy))

    return poses

# -----------------------------------------------------------------------------
# 4. Jitter for calibration robustness
# -----------------------------------------------------------------------------

def add_jitter(
    poses: Sequence[PoseStamped],
    *,
    trans_std: float = 0.001,      # metres (1 mm)
    rot_std_deg: float = 1.0,      # degrees (about random axis)
    seed: int | None = None,
) -> List[PoseStamped]:

    rng = np.random.default_rng(seed)
    jittered: List[PoseStamped] = []

    for p in poses:
        T = matrix_from_pose(p)

        # Translation jitter
        delta_t = rng.normal(0.0, trans_std, 3)
        T[:3, 3] += delta_t

        # Rotation jitter as axis‑angle
        axis = rng.normal(0.0, 1.0, 3)
        axis /= np.linalg.norm(axis) + 1e-12
        angle = math.radians(rng.normal(0.0, rot_std_deg))
        dR = R.from_rotvec(axis * angle).as_matrix()
        T[:3, :3] = T[:3, :3] @ dR

        jittered.append(pose_from_matrix(T, p.header.frame_id))
    return jittered