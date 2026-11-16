"""Mesh island rejection must preserve supported edges, objects and RGB data."""
import copy

import numpy as np
import open3d as o3d
import pytest

from behav3d_sense.reconstruct.TSDF_cpu_cropped import _clean_mesh_for_export


def scene_with_island():
    main = o3d.geometry.TriangleMesh.create_box()
    vertices = np.asarray(main.vertices).copy()
    faces = np.asarray(main.triangles).copy()
    a, b = faces[0, :2]
    # A small triangle sharing an edge with the object is a retained detail.
    detail = (vertices[a] + vertices[b]) / 2 + np.cross(vertices[b] - vertices[a], [.1, .3, .7]) * .001
    main.vertices = o3d.utility.Vector3dVector(np.vstack((vertices, detail)))
    main.triangles = o3d.utility.Vector3iVector(np.vstack((faces, [a, b, len(vertices)])))
    main.paint_uniform_color([1., 0., 0.])
    second = o3d.geometry.TriangleMesh.create_box().translate([3., 0., 0.])
    second.paint_uniform_color([0., 1., 0.])
    island = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector([[0, 3, 0], [.01, 3, 0], [.01, 3.01, 0], [0, 3.01, 0]]),
        o3d.utility.Vector3iVector([[0, 1, 2], [0, 2, 3]]),
    )
    island.paint_uniform_color([0., 0., 1.])
    return main + second + island


def test_cleanup_removes_only_small_island_and_keeps_attached_detail_and_other_object():
    mesh = scene_with_island()
    old_faces = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].copy()
    old_colors = np.asarray(mesh.vertex_colors)[np.asarray(mesh.triangles)].copy()
    cleaned = _clean_mesh_for_export(mesh, min_component_triangles=3)
    assert len(cleaned.triangles) == 25
    np.testing.assert_array_equal(np.asarray(cleaned.vertices)[cleaned.triangles], old_faces[:-2])
    np.testing.assert_array_equal(np.asarray(cleaned.vertex_colors)[cleaned.triangles], old_colors[:-2])
    _, counts, _ = cleaned.cluster_connected_triangles()
    assert sorted(counts) == [12, 13]


@pytest.mark.parametrize('minimum', [0, 2])
def test_zero_and_boundary_threshold_keep_the_small_component(minimum):
    mesh = scene_with_island()
    before = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].copy()
    cleaned = _clean_mesh_for_export(mesh, min_component_triangles=minimum)
    np.testing.assert_array_equal(np.asarray(cleaned.vertices)[cleaned.triangles], before)


def test_invalid_threshold_fails_before_modifying_geometry():
    mesh = scene_with_island()
    before = copy.deepcopy(mesh)
    with pytest.raises(ValueError, match='nonnegative'):
        _clean_mesh_for_export(mesh, min_component_triangles=-1)
    np.testing.assert_array_equal(mesh.vertices, before.vertices)
    np.testing.assert_array_equal(mesh.triangles, before.triangles)


def test_vertex_merging_does_not_leave_degenerate_or_duplicate_faces():
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 0]]),
        o3d.utility.Vector3iVector([[0, 1, 2], [3, 1, 2], [0, 3, 1]]),
    )
    cleaned = _clean_mesh_for_export(mesh, min_component_triangles=0)
    assert len(cleaned.triangles) == 1 and len(cleaned.vertices) == 3
    before = np.asarray(cleaned.vertices)[np.asarray(cleaned.triangles)].copy()
    cleaned = _clean_mesh_for_export(cleaned, min_component_triangles=0)
    np.testing.assert_array_equal(np.asarray(cleaned.vertices)[np.asarray(cleaned.triangles)], before)
