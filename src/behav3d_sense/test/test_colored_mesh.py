"""Measured mesh colors and RViz triangle publication without ROS transport."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import open3d as o3d
import pytest

from behav3d_sense.reconstruct.TSDF_cpu_cropped import TSDF_Integration


def test_surface_rgb_uses_visible_observations_without_removing_uncolored_vertices():
    integration = TSDF_Integration.__new__(TSDF_Integration)
    integration.K = np.eye(3)
    integration.depth_scale = 1000.
    integration.depth_max = 2.
    integration.images = [np.full((4, 4), 1000, np.uint16) for _ in range(3)]
    integration.T_base_ir = [np.eye(4) for _ in range(3)]
    color = np.zeros((4, 4, 3), np.uint8)
    color[0, 0], color[0, 1], color[1, 0] = [0, 0, 255], [0, 255, 0], [255, 0, 0]
    integration.color_in_depth = [color.copy() for _ in range(3)]
    # Last point is occluded by the measured depth and must remain neutral.
    xyz = np.array([[0., 0., 1.], [1., 0., 1.], [0., 1., 1.], [1.5, 1.5, 1.5]])
    surface = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    filtered, observations = integration.colorize_tsdf_surface_points(surface)
    np.testing.assert_array_equal(surface.points, xyz)
    np.testing.assert_allclose(surface.colors, [[1, 0, 0], [0, 1, 0], [0, 0, 1], [.8, .8, .8]])
    np.testing.assert_array_equal(filtered.points, xyz[:3])
    np.testing.assert_array_equal(observations, [3, 3, 3])


@pytest.fixture
def world(monkeypatch, tmp_path):
    class Pose:
        def __init__(self):
            self.position = NS(x=0., y=0., z=0.)
            self.orientation = NS(x=0., y=0., z=0., w=0.)

    class Marker:
        ADD = 0
        DELETE = 2
        POINTS = 8
        MESH_RESOURCE = 10
        TRIANGLE_LIST = 11

        def __init__(self):
            self.header = NS()
            self.pose = Pose()
            self.scale = NS()
            self.color = NS()
            self.colors = []

    for name, module in {
        'rclpy': NS(), 'rclpy.node': NS(Node=object),
        'rclpy.qos': NS(QoSProfile=object, QoSDurabilityPolicy=object, QoSReliabilityPolicy=object),
        'behav3d_interfaces.srv': NS(UpdateWorldMesh=object),
        'geometry_msgs.msg': NS(Point=NS, Pose=Pose),
        'std_msgs.msg': NS(ColorRGBA=NS, String=NS),
        'std_srvs.srv': NS(Trigger=object),
        'visualization_msgs.msg': NS(Marker=Marker),
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[1] / 'behav3d_sense/world_node.py'
    spec = importlib.util.spec_from_file_location('behav3d_sense._color_world_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.WorldNode.__new__(module.WorldNode)
    node._mesh_frame_id = 'ur20_base_link'
    node._mesh_ns = 'reconstructed_mesh'
    node._mesh_rgba = [.8, .8, .8, .4]
    node._mesh_scale = 1.
    node._mesh_accumulate = False
    node._mesh_active_marker_ids = []
    node._mesh_staged_paths_by_kind = {}
    node._mesh_stage_keep = 20
    node._mesh_stage_dir = tmp_path / 'staged'
    node._mesh_pub = Mock()
    node._refresh_mesh_accumulate_settings = Mock()
    node._ply_marker_max_triangles = 0
    node._ply_marker_use_colors = True
    node._ply_marker_prefer_points = True
    node._ply_marker_max_points = 120000
    node._ply_marker_point_size = .0015
    node.get_clock = Mock(return_value=NS(now=lambda: NS(to_msg=lambda: NS(sec=0, nanosec=0))))
    node.get_logger = Mock(return_value=Mock())
    return node


@pytest.fixture
def colored_ply(tmp_path):
    mesh = o3d.geometry.TriangleMesh.create_box(.05, .06, .07)
    mesh.translate([.1, -.8, .02])
    colors = np.tile([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [.8, .8, .8]], (2, 1))
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    path = tmp_path / 'surface.ply'
    assert o3d.io.write_triangle_mesh(str(path), mesh)
    return path, mesh


def test_mesh_request_publishes_every_colored_triangle_in_base_frame(world, colored_ply):
    path, mesh = colored_ply
    world._publish_mesh(path, 'mesh')
    marker = world._mesh_pub.publish.call_args.args[0]
    triangles = np.asarray(mesh.triangles)
    assert marker.type == 11  # TRIANGLE_LIST, even with prefer_points enabled.
    np.testing.assert_array_equal(
        [[p.x, p.y, p.z] for p in marker.points], np.asarray(mesh.vertices)[triangles].reshape(-1, 3))
    np.testing.assert_allclose(
        [[c.r, c.g, c.b] for c in marker.colors], np.asarray(mesh.vertex_colors)[triangles].reshape(-1, 3))
    assert marker.header.frame_id == 'ur20_base_link'
    assert vars(marker.pose.orientation) == dict(x=0., y=0., z=0., w=1.)
    assert vars(marker.scale) == dict(x=1., y=1., z=1.)
    assert all(c.a == .4 for c in marker.colors) and marker.color.a == 1.
    assert next(world._mesh_stage_dir.iterdir()).read_bytes() == path.read_bytes()


def test_gray_toggle_and_point_cloud_display_remain_available(world, colored_ply):
    path, mesh = colored_ply
    world._ply_marker_use_colors = False
    marker, mode = world._build_ply_marker(path, 0, prefer_mesh=True)
    assert mode == 'triangle_list' and marker.colors == []
    assert len(marker.points) == 3 * len(mesh.triangles)
    world._ply_marker_use_colors = True
    marker, mode = world._build_ply_marker(path, 0)
    assert mode == 'points' and len(marker.colors) == len(mesh.vertices)


def test_optional_display_simplification_keeps_saved_mesh_intact(world, colored_ply):
    path, mesh = colored_ply
    before = path.read_bytes()
    world._ply_marker_max_triangles = 6
    marker = world._build_ply_triangle_marker(path, 0)
    assert 0 < len(marker.points) <= 18
    assert len(marker.colors) == len(marker.points)
    assert path.read_bytes() == before
    assert len(mesh.triangles) == 12


def test_old_uncolored_ply_is_still_a_triangle_mesh(world, colored_ply):
    path, mesh = colored_ply
    mesh.vertex_colors = o3d.utility.Vector3dVector()
    assert o3d.io.write_triangle_mesh(str(path), mesh)
    marker, mode = world._build_ply_marker(path, 0, prefer_mesh=True)
    assert mode == 'triangle_list' and marker.colors == []
    assert len(marker.points) == 3 * len(mesh.triangles)
