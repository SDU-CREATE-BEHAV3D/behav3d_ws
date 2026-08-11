import numpy as np
import pytest

from behav3d_orchestrator.src.contour_target_offset_estimator import (
    ContourTargetOffsetEstimator,
)


def _pose(position):
    return {
        "position_m": list(position),
        "z_axis": [0.0, 0.0, 1.0],
    }


def test_segment_offset_uses_full_gradient_lift_height_before_start_offset(monkeypatch):
    pre = np.asarray(
        [[x, 0.0, 0.0] for x in np.linspace(-0.004, 0.004, 17)],
        dtype=np.float64,
    )
    post = np.asarray(
        [[x, 0.0, 0.008] for x in np.linspace(-0.004, 0.004, 17)],
        dtype=np.float64,
    )
    estimator = ContourTargetOffsetEstimator()
    monkeypatch.setattr(
        estimator,
        "_load_contour_samples",
        lambda path: pre if path == "pre.ply" else post,
    )
    target = {
        "index": 0,
        "kind": "segment",
        # The original source is z=0. The printed start was offset by 4 mm.
        "start": _pose([0.0, 0.0, 0.004]),
        "end": _pose([0.0, 0.0, 0.010]),
    }

    result = estimator.evaluate(
        pre_contour_path="pre.ply",
        post_contour_path="post.ply",
        targets=[target],
        candidate_settings={
            "candidate_mode": "gradient_lift",
            "bead_height_mm": 10.0,
            "bead_width_mm": 12.0,
            "width_mode": "fixed",
            "target_output_yaw_deg": 0.0,
        },
    )

    observation = result["observations"][0]
    assert result["status"] == "complete"
    assert observation["status"] == "valid"
    assert observation["pre_contour_match_distance_mm"] == pytest.approx(0.0)
    assert observation["expected_gain_mm"] == pytest.approx(10.0)
    assert observation["observed_gain_mm"] == pytest.approx(8.0)
    assert observation["scan_offset_mm"] == pytest.approx(-2.0)
    assert result["suggested_target_correction_mm"] == pytest.approx(2.0)


def test_dot_offset_uses_nearest_pre_contour_source(monkeypatch):
    pre = np.asarray(
        [[x, 0.0, 0.0] for x in np.linspace(-0.004, 0.004, 17)],
        dtype=np.float64,
    )
    post = np.asarray(
        [[x, 0.0, 0.007] for x in np.linspace(-0.004, 0.004, 17)],
        dtype=np.float64,
    )
    estimator = ContourTargetOffsetEstimator()
    monkeypatch.setattr(
        estimator,
        "_load_contour_samples",
        lambda path: pre if path == "pre.ply" else post,
    )

    result = estimator.evaluate(
        pre_contour_path="pre.ply",
        post_contour_path="post.ply",
        targets=[{"index": 0, "kind": "dot", "pose": _pose([0.0, 0.0, 0.010])}],
        candidate_settings={
            "candidate_mode": "gradient_lift",
            "bead_height_mm": 10.0,
            "bead_width_mm": 12.0,
            "width_mode": "fixed",
            "target_output_yaw_deg": 0.0,
        },
    )

    observation = result["observations"][0]
    assert observation["status"] == "valid"
    assert observation["expected_gain_mm"] == pytest.approx(10.0)
    assert observation["observed_gain_mm"] == pytest.approx(7.0)
    assert observation["scan_offset_mm"] == pytest.approx(-3.0)
