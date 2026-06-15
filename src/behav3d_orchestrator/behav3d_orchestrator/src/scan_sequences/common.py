from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


TOOL_PLUS_Z_POINTS_OUTWARD = False


def target_origin_rotation(target: PoseStamped) -> tuple[np.ndarray, R]:
    p_t = np.array(
        [target.pose.position.x, target.pose.position.y, target.pose.position.z],
        dtype=float,
    )
    q_t = np.array(
        [
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ],
        dtype=float,
    )
    qn = np.linalg.norm(q_t)
    return p_t, (R.identity() if qn < 1e-12 else R.from_quat(q_t / qn))


def rotation_from_z_axis(z_axis: np.ndarray, *, x_guess: np.ndarray) -> R:
    z = np.asarray(z_axis, dtype=float)
    nz = np.linalg.norm(z)
    if nz < 1e-12:
        raise ValueError("z_axis must be non-zero")
    z /= nz

    x = np.asarray(x_guess, dtype=float)
    x = x - np.dot(x, z) * z
    nx = np.linalg.norm(x)
    if nx < 1e-9:
        x = any_orthonormal(z)
    else:
        x /= nx
    y = np.cross(z, x)
    y /= max(np.linalg.norm(y), 1e-12)
    return R.from_matrix(np.column_stack((x, y, z)))


def pose_from_position_rotation(position: np.ndarray, rotation: R, frame_id: str) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = str(frame_id or "world")
    ps.pose.position.x = float(position[0])
    ps.pose.position.y = float(position[1])
    ps.pose.position.z = float(position[2])
    q = rotation.as_quat()
    ps.pose.orientation.x = float(q[0])
    ps.pose.orientation.y = float(q[1])
    ps.pose.orientation.z = float(q[2])
    ps.pose.orientation.w = float(q[3])
    return ps


def order_targets_by_nearest_neighbor(
    targets: Sequence[PoseStamped],
    *,
    order_start_xyz: Optional[Sequence[float]] = None,
) -> list[PoseStamped]:
    target_list = list(targets)
    if len(target_list) <= 2:
        return target_list

    points = np.array(
        [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z] for ps in target_list],
        dtype=float,
    )
    remaining = set(range(len(target_list)))
    ordered_indices: list[int] = []

    if order_start_xyz is not None and len(order_start_xyz) == 3:
        start = np.asarray(order_start_xyz, dtype=float).reshape(3)
        current_idx = int(np.argmin(np.sum((points - start) ** 2, axis=1)))
    else:
        current_idx = 0

    ordered_indices.append(current_idx)
    remaining.remove(current_idx)
    current = points[current_idx]

    while remaining:
        rem_idx = np.array(sorted(remaining), dtype=int)
        next_idx = int(rem_idx[int(np.argmin(np.sum((points[rem_idx] - current) ** 2, axis=1)))])
        ordered_indices.append(next_idx)
        remaining.remove(next_idx)
        current = points[next_idx]

    return [target_list[i] for i in ordered_indices]


def any_orthonormal(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    idx = int(np.argmin(np.abs(v)))
    basis = np.zeros(3, dtype=float)
    basis[idx] = 1.0
    u = basis - np.dot(basis, v) * v
    n = np.linalg.norm(u)
    return u / (n if n > 1e-12 else 1.0)
