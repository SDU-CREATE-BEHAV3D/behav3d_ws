from pathlib import Path

import numpy as np
import pytest

from behav3d_py.scalar_field.lib_scalar.print_targets import (
    OrientedLineTargets,
    build_oriented_line_targets,
    offset_line_target_starts,
    resolve_target_output_mode,
    write_line_targets_yaml,
)


def _line_targets() -> OrientedLineTargets:
    starts = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0]])
    ends = starts + np.array([0.0, 0.0, 0.01])
    z_dirs = np.tile([0.0, 0.0, 1.0], (2, 1))
    return OrientedLineTargets(starts, ends, z_dirs, z_dirs.copy())


def test_line_target_yaml_writes_optional_volume(tmp_path: Path):
    output = tmp_path / "segments.yaml"

    write_line_targets_yaml(
        output,
        _line_targets(),
        position_scale=1000.0,
        volumes_mm3=np.array([100.0, 200.0]),
    )

    text = output.read_text(encoding="utf-8")
    assert "volume_mm3: 100.000000" in text
    assert "volume_mm3: 200.000000" in text


def test_line_target_yaml_rejects_mismatched_volumes(tmp_path: Path):
    with pytest.raises(ValueError, match="length must match line targets"):
        write_line_targets_yaml(
            tmp_path / "segments.yaml",
            _line_targets(),
            position_scale=1000.0,
            volumes_mm3=np.array([100.0]),
        )


@pytest.mark.parametrize(
    ("requested", "candidate_mode", "expected"),
    [
        ("dots", "gradient_lift", "dots"),
        ("segments", "gradient_lift", "segments"),
        ("auto", "gradient_lift", "dots"),
        ("auto", "gradient_walk", "segments"),
    ],
)
def test_target_output_mode_is_independent_from_candidate_generation(
    requested: str,
    candidate_mode: str,
    expected: str,
):
    assert resolve_target_output_mode(requested, candidate_mode) == expected


def test_line_start_offset_moves_start_toward_end():
    offset_targets = offset_line_target_starts(_line_targets(), 0.002)

    np.testing.assert_allclose(offset_targets.start_points[:, 2], 0.002)
    np.testing.assert_allclose(offset_targets.end_points[:, 2], 0.01)


def test_line_targets_copy_start_orientation_to_end(monkeypatch):
    starts = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0]])
    ends = starts + np.array([0.0, 0.0, 0.01])
    sampled_z = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    sampled_points: list[np.ndarray] = []

    def _fake_orient(points, *_args, **_kwargs):
        sampled_points.append(np.asarray(points).copy())
        return np.asarray(points).copy(), sampled_z.copy()

    monkeypatch.setattr(
        "behav3d_py.scalar_field.lib_scalar.print_targets.orient_points_with_tangent",
        _fake_orient,
    )

    targets = build_oriented_line_targets(
        starts,
        ends,
        np.empty((0, 3)),
        np.empty((0, 3), dtype=np.int32),
        np.empty((0,)),
    )

    assert len(sampled_points) == 1
    np.testing.assert_allclose(sampled_points[0], starts)
    np.testing.assert_allclose(targets.start_points, starts)
    np.testing.assert_allclose(targets.end_points, ends)
    np.testing.assert_allclose(targets.start_z_dirs, sampled_z)
    np.testing.assert_allclose(targets.end_z_dirs, sampled_z)
