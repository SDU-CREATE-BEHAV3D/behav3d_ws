#!/usr/bin/env python3
from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock


def _now_msg() -> Time:
    try:
        return Clock().now().to_msg()
    except Exception:
        return Time()


def pose_stamped(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    frame: str,
) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = _now_msg()
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = float(z)
    ps.pose.orientation.x = float(qx)
    ps.pose.orientation.y = float(qy)
    ps.pose.orientation.z = float(qz)
    ps.pose.orientation.w = float(qw)
    return ps


def world_xy(x: float, y: float, z: float, frame: str) -> PoseStamped:
    return pose_stamped(x, y, z, 0.0, 0.0, 0.0, 1.0, frame)


def world_xz(x: float, z: float, y: float, frame: str) -> PoseStamped:
    return pose_stamped(
        x,
        y,
        z,
        0.7071067811865475,
        0.0,
        0.0,
        0.7071067811865475,
        frame,
    )


def world_yz(y: float, z: float, x: float, frame: str) -> PoseStamped:
    return pose_stamped(x, y, z, 0.5, 0.5, 0.5, 0.5, frame)


def to_iso(pose: PoseStamped) -> np.ndarray:
    mat = np.eye(4, dtype=float)
    mat[:3, :3] = Rotation.from_quat(
        [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
    ).as_matrix()
    mat[:3, 3] = [
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
    ]
    return mat


def from_iso(mat: np.ndarray, frame: str) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = _now_msg()
    ps.pose.position.x = float(mat[0, 3])
    ps.pose.position.y = float(mat[1, 3])
    ps.pose.position.z = float(mat[2, 3])
    q = Rotation.from_matrix(mat[:3, :3]).as_quat()
    ps.pose.orientation.x = float(q[0])
    ps.pose.orientation.y = float(q[1])
    ps.pose.orientation.z = float(q[2])
    ps.pose.orientation.w = float(q[3])
    return ps


def translate(pose: PoseStamped, d: Sequence[float]) -> PoseStamped:
    mat = to_iso(pose)
    vec = np.asarray(d, dtype=float).reshape(3)
    mat[:3, 3] = mat[:3, 3] + mat[:3, :3].dot(vec)
    return from_iso(mat, pose.header.frame_id)


def rotate_euler(pose: PoseStamped, rpy: Sequence[float], degrees: bool) -> PoseStamped:
    mat = to_iso(pose)
    rot = Rotation.from_matrix(mat[:3, :3])
    delta = Rotation.from_euler("xyz", rpy, degrees=degrees)
    mat[:3, :3] = (rot * delta).as_matrix()
    return from_iso(mat, pose.header.frame_id)


def transform_rel(
    pose: PoseStamped,
    t: Sequence[float],
    q: Sequence[float],
) -> PoseStamped:
    mat = to_iso(pose)
    vec = np.asarray(t, dtype=float).reshape(3)
    mat[:3, 3] = mat[:3, 3] + mat[:3, :3].dot(vec)
    if isinstance(q, Rotation):
        delta = q
    else:
        delta = Rotation.from_quat([q[0], q[1], q[2], q[3]])
    mat[:3, :3] = (Rotation.from_matrix(mat[:3, :3]) * delta).as_matrix()
    return from_iso(mat, pose.header.frame_id)


def change_basis(basis: PoseStamped, local: PoseStamped) -> PoseStamped:
    iso_basis = to_iso(basis)
    iso_local = to_iso(local)
    iso_world = iso_basis @ iso_local
    return from_iso(iso_world, basis.header.frame_id)


def rebase(pose: PoseStamped, src: PoseStamped, dst: PoseStamped) -> PoseStamped:
    iso_src = to_iso(src)
    iso_pose = to_iso(pose)
    iso_world = iso_src @ iso_pose
    iso_dst = to_iso(dst)
    iso_new = np.linalg.inv(iso_dst) @ iso_world
    return from_iso(iso_new, dst.header.frame_id)


def _axis(pose: PoseStamped, idx: int) -> np.ndarray:
    mat = Rotation.from_quat(
        [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
    ).as_matrix()
    return mat[:, idx]


def x_axis(pose: PoseStamped) -> np.ndarray:
    return _axis(pose, 0)


def y_axis(pose: PoseStamped) -> np.ndarray:
    return _axis(pose, 1)


def z_axis(pose: PoseStamped) -> np.ndarray:
    return _axis(pose, 2)


def align_target(pose: PoseStamped, desired_x: Sequence[float]) -> PoseStamped:
    mat = to_iso(pose)
    rot = mat[:3, :3]
    z = rot[:, 2]
    d = np.asarray(desired_x, dtype=float).reshape(3)

    x_proj = d - d.dot(z) * z
    if float(np.dot(x_proj, x_proj)) < 1e-10:
        fallback = rot[:, 1]
        x_proj = fallback - fallback.dot(z) * z

    x = x_proj / max(np.linalg.norm(x_proj), 1e-12)
    y = np.cross(z, x)
    y /= max(np.linalg.norm(y), 1e-12)

    rot[:, 0] = x
    rot[:, 1] = y
    rot[:, 2] = z
    mat[:3, :3] = rot
    return from_iso(mat, pose.header.frame_id)


def adjust_target(pose: PoseStamped, new_normal: Sequence[float]) -> PoseStamped:
    mat = to_iso(pose)
    rot = mat[:3, :3]
    z = np.asarray(new_normal, dtype=float).reshape(3)
    z /= max(np.linalg.norm(z), 1e-12)
    x = rot[:, 0]

    if abs(float(x.dot(z))) > 0.95:
        x = np.array([1.0, 0.0, 0.0]) if abs(float(z.dot([1.0, 0.0, 0.0]))) < 0.8 else np.array([0.0, 1.0, 0.0])

    y = np.cross(z, x)
    y /= max(np.linalg.norm(y), 1e-12)
    x = np.cross(y, z)

    rot[:, 0] = x
    rot[:, 1] = y
    rot[:, 2] = z
    mat[:3, :3] = rot
    return from_iso(mat, pose.header.frame_id)


def axis_angle(pose: PoseStamped) -> Tuple[np.ndarray, float]:
    rotvec = Rotation.from_quat(
        [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
    ).as_rotvec()
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0]), 0.0
    return rotvec / angle, angle


def pose_between(a: PoseStamped, b: PoseStamped, t: float) -> PoseStamped:
    mat_a = to_iso(a)
    mat_b = to_iso(b)
    p = mat_a[:3, 3] * (1.0 - t) + mat_b[:3, 3] * t

    rot_a = Rotation.from_matrix(mat_a[:3, :3])
    rot_b = Rotation.from_matrix(mat_b[:3, :3])
    slerp = Slerp(
        [0.0, 1.0],
        Rotation.from_quat(np.stack([rot_a.as_quat(), rot_b.as_quat()], axis=0)),
    )
    rot = slerp([t])[0]

    mat = np.eye(4, dtype=float)
    mat[:3, :3] = rot.as_matrix()
    mat[:3, 3] = p
    return from_iso(mat, a.header.frame_id)


def mirror_across_plane(
    pose: PoseStamped,
    n: Sequence[float],
    p0: Sequence[float],
) -> PoseStamped:
    n_n = np.asarray(n, dtype=float).reshape(3)
    n_n /= max(np.linalg.norm(n_n), 1e-12)
    pos = np.array(
        [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
        dtype=float,
    )
    p0_v = np.asarray(p0, dtype=float).reshape(3)
    mirrored_pos = pos - 2.0 * (pos - p0_v).dot(n_n) * n_n

    q = Rotation.from_quat(
        [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
    )
    reflect = Rotation.from_rotvec(np.pi * n_n)
    mirrored_q = reflect * q

    return pose_stamped(
        mirrored_pos[0],
        mirrored_pos[1],
        mirrored_pos[2],
        *mirrored_q.as_quat(),
        pose.header.frame_id,
    )


def average(poses: Sequence[PoseStamped]) -> PoseStamped:
    if not poses:
        return PoseStamped()

    c = np.zeros(3, dtype=float)
    accum = np.zeros(4, dtype=float)

    for p in poses:
        c += np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z], dtype=float)
        q = np.array(
            [
                p.pose.orientation.x,
                p.pose.orientation.y,
                p.pose.orientation.z,
                p.pose.orientation.w,
            ],
            dtype=float,
        )
        if q[3] < 0:
            q = -q
        accum += q

    c /= float(len(poses))
    norm = np.linalg.norm(accum)
    if norm < 1e-12:
        accum = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    else:
        accum /= norm

    return pose_stamped(c[0], c[1], c[2], accum[0], accum[1], accum[2], accum[3], poses[0].header.frame_id)


def offset_target(pose: PoseStamped, offset_dist: float) -> PoseStamped:
    return translate(pose, [0.0, 0.0, float(offset_dist)])


def set_target_origin(pose: PoseStamped, x: float, y: float, z: float) -> PoseStamped:
    out = PoseStamped()
    out.header = pose.header
    out.header.stamp = _now_msg()
    out.pose = pose.pose
    out.pose.position.x = float(x)
    out.pose.position.y = float(y)
    out.pose.position.z = float(z)
    return out


def flip_target_axes(pose: PoseStamped, flip_x: bool, flip_y: bool) -> PoseStamped:
    mat = to_iso(pose)
    rot = mat[:3, :3]

    if flip_x:
        rot[:, 0] *= -1.0
    if flip_y:
        rot[:, 1] *= -1.0

    x = rot[:, 0] / max(np.linalg.norm(rot[:, 0]), 1e-12)
    y = rot[:, 1] / max(np.linalg.norm(rot[:, 1]), 1e-12)
    z = np.cross(x, y)
    z /= max(np.linalg.norm(z), 1e-12)
    if float(np.dot(np.cross(x, y), z)) < 0.0:
        z *= -1.0

    rot[:, 0] = x
    rot[:, 1] = y
    rot[:, 2] = z
    mat[:3, :3] = rot
    return from_iso(mat, pose.header.frame_id)


def swap_target_axes(pose: PoseStamped) -> PoseStamped:
    mat = to_iso(pose)
    rot = mat[:3, :3]
    rot[:, [0, 1]] = rot[:, [1, 0]]

    x = rot[:, 0] / max(np.linalg.norm(rot[:, 0]), 1e-12)
    y = rot[:, 1] / max(np.linalg.norm(rot[:, 1]), 1e-12)
    z = np.cross(x, y)
    z /= max(np.linalg.norm(z), 1e-12)
    if float(np.dot(np.cross(x, y), z)) < 0.0:
        z *= -1.0

    rot[:, 0] = x
    rot[:, 1] = y
    rot[:, 2] = z
    mat[:3, :3] = rot
    return from_iso(mat, pose.header.frame_id)


# Compatibility aliases for camelCase usage (C++ parity)
poseStamped = pose_stamped
worldXY = world_xy
worldXZ = world_xz
worldYZ = world_yz
toIso = to_iso
fromIso = from_iso
rotateEuler = rotate_euler
transformRel = transform_rel
changeBasis = change_basis
rebase = rebase
xAxis = x_axis
yAxis = y_axis
zAxis = z_axis
alignTarget = align_target
adjustTarget = adjust_target
axisAngle = axis_angle
poseBetween = pose_between
mirrorAcrossPlane = mirror_across_plane
offsetTarget = offset_target
setTargetOrigin = set_target_origin
flipTargetAxes = flip_target_axes
swapTargetAxes = swap_target_axes
