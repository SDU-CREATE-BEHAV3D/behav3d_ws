import math

import numpy as np
import pytest

from behav3d_py.scalar_field.lib_scalar.bead_profile import (
    load_normalized_width_map,
    minimum_center_distance,
    normalized_width_to_mm,
    rounded_cylinder_volume_mm3,
    scale_requested_volume_mm3,
)
from behav3d_py.scalar_field.lib_scalar.generate_print_points import (
    generate_print_points,
)


def test_width_map_and_linear_mapping(tmp_path):
    path = tmp_path / "width_field.npz"
    np.savez(path, width_norm=np.array([0.0, 0.5, 1.0]))

    normalized = load_normalized_width_map(path, expected_count=3)
    widths = normalized_width_to_mm(
        normalized,
        width_min_mm=20.0,
        width_max_mm=40.0,
    )

    assert widths.tolist() == [20.0, 30.0, 40.0]


def test_width_map_rejects_wrong_vertex_count(tmp_path):
    path = tmp_path / "width_field.npz"
    np.savez(path, width_norm=np.array([0.0, 1.0]))

    with pytest.raises(ValueError, match="does not match field vertices"):
        load_normalized_width_map(path, expected_count=3)


def test_pair_spacing_uses_both_radii_and_overlap():
    distance = minimum_center_distance(20.0, 40.0, overlap=4.0)
    assert float(distance) == pytest.approx(26.0)


def test_equal_width_and_height_is_a_sphere():
    diameter = 20.0
    expected = (4.0 / 3.0) * math.pi * (diameter / 2.0) ** 3
    actual = rounded_cylinder_volume_mm3(diameter, diameter)
    assert float(actual) == pytest.approx(expected)


def test_requested_volume_factor_scales_analytic_volume():
    scaled = scale_requested_volume_mm3(
        np.array([1000.0, 2000.0]),
        factor=0.85,
    )
    assert scaled.tolist() == pytest.approx([850.0, 1700.0])


def test_requested_volume_factor_must_be_positive():
    with pytest.raises(ValueError, match="Volume factor"):
        scale_requested_volume_mm3(1000.0, factor=0.0)


def test_candidate_selection_uses_variable_width_spacing():
    points = np.column_stack(
        (
            np.arange(5, dtype=np.float64) * 0.010,
            np.zeros(5),
            np.zeros(5),
        )
    )
    lines = np.column_stack((np.arange(4), np.arange(1, 5))).astype(np.int32)

    selected = generate_print_points(
        polyline_points=points,
        polyline_lines=lines,
        field_vertices_world=points,
        field_scalar=np.arange(5, dtype=np.float64),
        count=3,
        min_spacing=1.0,
        field_bead_widths=np.full(5, 0.020),
        bead_overlap=0.004,
    )

    assert selected.points[:, 0].tolist() == pytest.approx([0.0, 0.02, 0.04])
    assert selected.bead_widths.tolist() == pytest.approx([0.02, 0.02, 0.02])
