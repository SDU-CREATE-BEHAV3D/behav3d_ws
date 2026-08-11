import csv
import json
from types import SimpleNamespace

from behav3d_orchestrator.src.contour_offset_logger import ContourOffsetLogger
from behav3d_orchestrator.src.contour_target_offset_estimator import (
    ContourTargetOffsetEstimator,
)


def _pose(x, y, z):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=float(x), y=float(y), z=float(z)),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )


def test_logger_contains_only_contour_offset_inputs_and_results(tmp_path, monkeypatch):
    pre_contour = tmp_path / "pre_contour.ply"
    post_contour = tmp_path / "post_contour.ply"
    pre_contour.write_text("fixture", encoding="utf-8")
    post_contour.write_text("fixture", encoding="utf-8")
    segment = SimpleNamespace(
        index=4,
        start=_pose(0.1, 0.2, 0.3),
        end=_pose(0.1, 0.2, 0.32),
    )
    logger = ContourOffsetLogger()
    record_path = logger.begin_cycle(
        cycle_root=tmp_path / "cycle_0004",
        cycle_number=4,
        phi_contour_path=str(pre_contour),
        candidate_settings={
            "candidate_mode": "gradient_lift",
            "bead_height_mm": 20.0,
            "target_output_yaw_deg": 180.0,
        },
        segments=[segment],
    )

    opened = json.loads(record_path.read_text(encoding="utf-8"))
    assert record_path.parent.name == "contour_offset"
    assert opened["status"] == "awaiting_post_contour"
    assert "artifacts" not in opened
    assert "print" not in opened
    assert "candidate_generation" not in opened
    assert opened["pre_phi_contour_path"] == str(pre_contour.resolve())

    monkeypatch.setattr(
        ContourTargetOffsetEstimator,
        "evaluate",
        lambda self, **kwargs: {
            "status": "complete",
            "target_count": 1,
            "valid_target_count": 1,
            "scan_offset_median_mm": -2.0,
            "scan_offset_mad_mm": 0.0,
            "confidence": 0.9,
            "observations": [
                {
                    "status": "valid",
                    "expected_gain_mm": 20.0,
                    "observed_gain_mm": 18.0,
                    "scan_offset_mm": -2.0,
                    "pre_contour_match_distance_mm": 0.1,
                    "post_axis_lateral_distance_mm": 0.2,
                    "residual_dispersion_mm": 0.3,
                    "confidence": 0.9,
                }
            ],
        },
    )
    logger.evaluate_cycle(record_path, post_phi_contour_path=str(post_contour))

    evaluated = json.loads(record_path.read_text(encoding="utf-8"))
    assert evaluated["status"] == "evaluated"
    assert evaluated["evaluation"]["scan_offset_median_mm"] == -2.0
    assert evaluated["post_phi_contour_path"] == str(post_contour.resolve())
    with (record_path.parent / "targets.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert rows[0]["index"] == "4"
    assert rows[0]["scan_offset_mm"] == "-2.0"


def test_logger_closes_when_following_contour_is_unavailable(tmp_path):
    logger = ContourOffsetLogger()
    record_path = logger.begin_cycle(
        cycle_root=tmp_path / "cycle_0001",
        cycle_number=1,
        phi_contour_path="",
        candidate_settings={},
        dot_targets=[_pose(0.0, 0.0, 0.1)],
    )

    logger.close_without_post_contour(record_path, reason="test")

    record = json.loads(record_path.read_text(encoding="utf-8"))
    assert record["status"] == "post_contour_unavailable"
    assert record["evaluation"]["reason"] == "test"
