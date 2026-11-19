"""Minimal-twist free-Z-yaw regression tests."""

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from behav3d_orchestrator.src.free_z_yaw import transport_free_z_yaw


def pose_with_z(z):
    pose = PoseStamped()
    normal = np.asarray(z, dtype=float)
    normal /= np.linalg.norm(normal)
    axis = np.cross([0.0, 0.0, 1.0], normal)
    norm = np.linalg.norm(axis)
    if norm > 1e-8:
        q = Rotation.from_rotvec(axis / norm * np.arccos(np.dot([0, 0, 1], normal))).as_quat()
    elif normal[2] < 0:
        q = Rotation.from_euler("x", 180, degrees=True).as_quat()
    else:
        q = [0, 0, 0, 1]
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
    return pose


def z_of(pose):
    q = pose.pose.orientation
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0, 0, 1])


def test_preserves_normals_and_does_not_mutate_input():
    targets = [pose_with_z([0, 0, 1]), pose_with_z([0.2, 0, 1]), pose_with_z([0.4, 0.2, 1])]
    original_q = targets[1].pose.orientation.w
    output = transport_free_z_yaw(targets, pose_with_z([0, 0, 1]))
    for target, transformed in zip(targets, output):
        assert np.allclose(z_of(target), z_of(transformed), atol=1e-8)
    assert targets[1].pose.orientation.w == original_q


def test_parallel_normals_keep_reference_yaw():
    reference = pose_with_z([0, 0, 1])
    q = Rotation.from_euler("z", 75, degrees=True).as_quat()
    reference.pose.orientation.x, reference.pose.orientation.y, reference.pose.orientation.z, reference.pose.orientation.w = map(float, q)
    output = transport_free_z_yaw([pose_with_z([0, 0, 1])] * 4, reference)
    for item in output:
        actual = item.pose.orientation
        assert np.allclose([actual.x, actual.y, actual.z, actual.w], q, atol=1e-8)


def test_opposite_normals_remain_valid():
    output = transport_free_z_yaw(
        [pose_with_z([0, 0, 1]), pose_with_z([0, 0, -1])],
        pose_with_z([0, 0, 1]),
    )
    assert np.allclose(z_of(output[1]), [0, 0, -1], atol=1e-8)
