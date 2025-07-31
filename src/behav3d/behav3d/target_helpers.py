#!/usr/bin/env python3
          
# =============================================================================
#   ____  _____ _   _    ___     _______ ____  
#  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
#  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
#  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
#  |____/|_____|_| |_/_/   \_\_/  |____/|____/ 
#                                               
#                                               
# Author: Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
# Maintainers:
#   - Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
#   - Lucas José Helle <luh@iti.sdu.dk>
# Institute: University of Southern Denmark (Syddansk Universitet)
# Date: 2025-07
# =============================================================================


import numpy as np
from typing import Tuple, List
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

__all__ = [
    "pose_stamped",
    "world_xy",
    "look_at",
    "translate",
    "rotate_euler",
    "transform_relative",
    "align_target",
    "adjust_target",
    "axes_from_pose",
    "x_axis",
    "y_axis",
    "z_axis",
    "matrix_from_pose",
    "pose_from_matrix",
]

# ---------------------------------------------------------------------------
# Core builders
# ---------------------------------------------------------------------------

def pose_stamped(
    x: float,
    y: float,
    z: float,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
    *,
    frame_id: str = "world",
) -> PoseStamped:
    """
    Create a ``PoseStamped`` from raw xyz + quaternion parameters.
    """
    ps = PoseStamped()
    ps.header.frame_id         = frame_id
    ps.pose.position.x         = float(x)
    ps.pose.position.y         = float(y)
    ps.pose.position.z         = float(z)
    ps.pose.orientation.x      = float(qx)
    ps.pose.orientation.y      = float(qy)
    ps.pose.orientation.z      = float(qz)
    ps.pose.orientation.w      = float(qw)
    return ps


def world_xy(
    x: float,
    y: float,
    z: float,
    *,
    frame_id: str = "world",
    flipped: bool = True,
) -> PoseStamped:
    """
    A common *camera‑style* target: origin at (x, y, z) with +Z pointing **down**
    (i.e. 180° rotation about X) when *flipped* is ``True``.  If *flipped* is
    ``False`` the pose has identity orientation.

    This is equivalent to many “tool‑flange down” use‑cases in UR robots.
    """
    if flipped:
        # 180° about X → quaternion (1, 0, 0, 0)
        return pose_stamped(x, y, z, 1.0, 0.0, 0.0, 0.0, frame_id=frame_id)
    else:
        return pose_stamped(x, y, z, 0.0, 0.0, 0.0, 1.0, frame_id=frame_id)

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def look_at(
    camera_pose: PoseStamped,
    target: Tuple[float, float, float],
    *,
    up: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    frame_id: str | None = None,
) -> PoseStamped:
    """
    Return a new pose where **+Z** of the camera frame points towards *target*.

    Parameters
    ----------
    camera_pose
        Current camera/tool pose.
    target
        3‑tuple (x, y, z) of the point to look at.
    up
        Preferred world “up” vector to keep the frame roll well‑defined.
    frame_id
        Optional different frame_id for the returned pose.

    Notes
    -----
    Implementation follows the classical "look‑at" matrix construction used in
    computer graphics.
    """
    cam_pos = np.array([camera_pose.pose.position.x,
                        camera_pose.pose.position.y,
                        camera_pose.pose.position.z], dtype=float)
    tgt     = np.asarray(target, dtype=float)
    forward = tgt - cam_pos
    norm    = np.linalg.norm(forward)
    if norm < 1e-9:
        raise ValueError("look_at: camera position equals target position.")
    forward /= norm   # normalise

    up_vec  = np.asarray(up, dtype=float)
    if np.linalg.norm(np.cross(forward, up_vec)) < 1e-6:
        # forward nearly collinear with up → pick a fallback
        up_vec = np.array([0.0, 1.0, 0.0])
    up_vec   = up_vec / np.linalg.norm(up_vec)

    right   = np.cross(up_vec, forward)
    right  /= np.linalg.norm(right)
    up_vec  = np.cross(forward, right)  # recompute for orthogonality

    # Build rotation matrix with columns [right, up, forward] in ROS (x,y,z)
    R_cam   = np.column_stack((right, up_vec, forward))
    rot     = R.from_matrix(R_cam)

    # Compose PoseStamped
    out = PoseStamped()
    out.header.frame_id = frame_id or camera_pose.header.frame_id
    out.pose.position   = camera_pose.pose.position  # copy xyz
    qx, qy, qz, qw = rot.as_quat()
    out.pose.orientation.x = qx
    out.pose.orientation.y = qy
    out.pose.orientation.z = qz
    out.pose.orientation.w = qw
    return out


def translate(
    pose: PoseStamped,
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
    *,
    frame_id: str | None = None,
) -> PoseStamped:
    """
    Return *pose* translated by (dx, dy, dz) **in its own frame**.
    """
    T = matrix_from_pose(pose)
    T[:3, 3] += np.array([dx, dy, dz])
    return pose_from_matrix(T, frame_id or pose.header.frame_id)


def rotate_euler(
    pose: PoseStamped,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    *,
    degrees: bool = False,
    frame_id: str | None = None,
) -> PoseStamped:
    """
    Apply relative rotation to *pose* given Euler angles (rpy).

    If *degrees* is ``True`` the inputs are interpreted as degrees.
    """
    base_R = R.from_quat([
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
    ])
    delta_R = R.from_euler("xyz", [roll, pitch, yaw], degrees=degrees)
    new_R   = base_R * delta_R
    T       = matrix_from_pose(pose)
    T[:3, :3] = new_R.as_matrix()
    return pose_from_matrix(T, frame_id or pose.header.frame_id)


def transform_relative(
    pose: PoseStamped,
    *,
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation_quat: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    frame_id: str | None = None,
) -> PoseStamped:
    """
    Apply a rigid transform (translation + quaternion) **in pose frame**.
    """
    T = matrix_from_pose(pose)

    # Build delta transform
    dR = R.from_quat(rotation_quat).as_matrix()
    dT = np.eye(4)
    dT[:3, :3] = dR
    dT[:3, 3]  = np.array(translation)

    new_T = T @ dT
    return pose_from_matrix(new_T, frame_id or pose.header.frame_id)


# ---------------------------------------------------------------------------
# Orientation helpers – “Align Plane” & “Adjust Plane”
# ---------------------------------------------------------------------------

def _minimal_rotation(v_from: np.ndarray, v_to: np.ndarray) -> R:
    """
    Return the *minimal* SO(3) rotation that takes **v_from → v_to**.
    """
    v_from = v_from / (np.linalg.norm(v_from) + 1e-12)
    v_to   = v_to   / (np.linalg.norm(v_to)   + 1e-12)
    dot    = np.clip(np.dot(v_from, v_to), -1.0, 1.0)
    if dot > 0.999999:
        return R.identity()        # already aligned
    if dot < -0.999999:
        # 180° rotation about any axis orthogonal to v_from
        axis = np.cross(v_from, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(v_from, [0.0, 1.0, 0.0])
        axis /= np.linalg.norm(axis)
        return R.from_rotvec(axis * np.pi)
    axis = np.cross(v_from, v_to)
    axis /= np.linalg.norm(axis)
    angle = np.arccos(dot)
    return R.from_rotvec(axis * angle)


def align_target(
    pose: PoseStamped,
    new_x: Tuple[float, float, float],
    *,
    frame_id: str | None = None,
) -> PoseStamped:

    # Current orientation
    base_R = R.from_quat([
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ])

    x_world = base_R.apply([1.0, 0.0, 0.0])   # current +X in world
    z_world = base_R.apply([0.0, 0.0, 1.0])   # current +Z (rotation axis)

    # Desired direction (unit)
    desired = np.asarray(new_x, float)
    desired /= np.linalg.norm(desired) + 1e-12

    # Project both vectors onto the plane perpendicular to +Z
    x_proj = x_world - np.dot(x_world, z_world) * z_world
    d_proj = desired  - np.dot(desired,  z_world) * z_world

    if np.linalg.norm(d_proj) < 1e-6:
        raise ValueError("align_target: desired direction nearly parallel to +Z; yaw-only rotation impossible.")

    x_proj /= np.linalg.norm(x_proj)
    d_proj /= np.linalg.norm(d_proj)

    # Signed yaw angle
    sin_a = np.dot(np.cross(x_proj, d_proj), z_world)
    cos_a = np.clip(np.dot(x_proj, d_proj), -1.0, 1.0)
    angle = np.arctan2(sin_a, cos_a)

    delta_R = R.from_rotvec(z_world * angle)  # rotate about +Z
    new_R   = delta_R * base_R

    T         = matrix_from_pose(pose)
    T[:3, :3] = new_R.as_matrix()
    return pose_from_matrix(T, frame_id or pose.header.frame_id)

def adjust_target(
    pose: PoseStamped,
    new_normal: Tuple[float, float, float],
    *,
    frame_id: str | None = None,
) -> PoseStamped:

    base_R = R.from_quat([
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ])
    current_z = base_R.apply([0.0, 0.0, 1.0])
    delta_R   = _minimal_rotation(current_z, np.asarray(new_normal, dtype=float))
    new_R     = delta_R * base_R
    T         = matrix_from_pose(pose)
    T[:3, :3] = new_R.as_matrix()
    return pose_from_matrix(T, frame_id or pose.header.frame_id)



# ---------------------------------------------------------------------------
# Axis‑extraction helpers
# ---------------------------------------------------------------------------

def axes_from_pose(
    pose: PoseStamped,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:

    rot = R.from_quat([
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ])
    x_vec = rot.apply([1.0, 0.0, 0.0])
    y_vec = rot.apply([0.0, 1.0, 0.0])
    z_vec = rot.apply([0.0, 0.0, 1.0])
    return x_vec, y_vec, z_vec

# ---------------------------------------------------------------------------
# Convenience / conversion utilities
# ---------------------------------------------------------------------------

def matrix_from_pose(pose: PoseStamped) -> np.ndarray:
    """Convert ``PoseStamped`` → 4×4 homogeneous matrix."""
    Rm = R.from_quat([
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w
    ]).as_matrix()
    T  = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3]  = np.array([
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z
    ])
    return T


def pose_from_matrix(T: np.ndarray, frame_id: str = "world") -> PoseStamped:
    """Convert 4×4 homogeneous matrix → ``PoseStamped``."""
    out = PoseStamped()
    out.header.frame_id = frame_id
    out.pose.position.x, out.pose.position.y, out.pose.position.z = T[:3, 3]
    qx, qy, qz, qw = R.from_matrix(T[:3, :3]).as_quat()
    out.pose.orientation.x = qx
    out.pose.orientation.y = qy
    out.pose.orientation.z = qz
    out.pose.orientation.w = qw
    return out
