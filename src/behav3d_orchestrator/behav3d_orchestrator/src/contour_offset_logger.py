#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

from .contour_target_offset_estimator import ContourTargetOffsetEstimator


class ContourOffsetLogger:
    """Persist target/contour inputs and their computed offset report."""

    OUTPUT_DIRNAME = "contour_offset"
    JSON_FILENAME = "contour_offset.json"
    CSV_FILENAME = "targets.csv"

    def begin_cycle(
        self,
        *,
        cycle_root: str | Path,
        cycle_number: int,
        phi_contour_path: str,
        candidate_settings: Mapping[str, Any],
        dot_targets: Sequence[Any] = (),
        segments: Sequence[Any] = (),
    ) -> Path:
        output_dir = Path(cycle_root).expanduser() / self.OUTPUT_DIRNAME
        output_dir.mkdir(parents=True, exist_ok=True)
        record_path = output_dir / self.JSON_FILENAME
        targets = (
            self._segment_records(segments)
            if segments
            else self._dot_records(dot_targets)
        )
        now = self._utc_now()
        record = {
            "schema_version": 1,
            "status": "awaiting_post_contour",
            "cycle": {
                "number": int(cycle_number),
                "tag": f"cycle_{int(cycle_number):04d}",
                "opened_at_utc": now,
                "updated_at_utc": now,
            },
            "pre_phi_contour_path": self._resolved_path(phi_contour_path),
            "post_phi_contour_path": "",
            "candidate_settings": self._json_safe(candidate_settings),
            "evaluation": {"status": "awaiting_post_contour"},
            "targets": targets,
        }
        self._write_record(record_path, record)
        return record_path

    def evaluate_cycle(
        self,
        record_path: str | Path,
        *,
        post_phi_contour_path: str,
    ) -> dict[str, Any]:
        path = Path(record_path).expanduser()
        record = json.loads(path.read_text(encoding="utf-8"))
        result = ContourTargetOffsetEstimator().evaluate(
            pre_contour_path=str(record.get("pre_phi_contour_path", "")),
            post_contour_path=post_phi_contour_path,
            targets=record.get("targets", []),
            candidate_settings=record.get("candidate_settings", {}),
        )
        observations = result.pop("observations")
        for target, observation in zip(record.get("targets", []), observations):
            target["observation"] = observation

        record["post_phi_contour_path"] = self._resolved_path(post_phi_contour_path)
        record["evaluation"] = result
        record["status"] = (
            "evaluated"
            if result.get("status") == "complete"
            else "insufficient_contour_matches"
        )
        record["cycle"]["updated_at_utc"] = self._utc_now()
        self._write_record(path, record)
        return result

    def close_without_post_contour(
        self,
        record_path: str | Path,
        *,
        reason: str,
    ) -> None:
        path = Path(record_path).expanduser()
        record = json.loads(path.read_text(encoding="utf-8"))
        record["status"] = "post_contour_unavailable"
        record["evaluation"] = {
            "status": "post_contour_unavailable",
            "reason": str(reason),
        }
        record["cycle"]["updated_at_utc"] = self._utc_now()
        for target in record.get("targets", []):
            target["observation"]["status"] = "post_contour_unavailable"
        self._write_record(path, record)

    @classmethod
    def _dot_records(cls, targets: Sequence[Any]) -> list[dict[str, Any]]:
        return [
            {
                "index": int(position),
                "kind": "dot",
                "pose": cls._pose_record(pose),
                "observation": cls._empty_observation(),
            }
            for position, pose in enumerate(targets)
        ]

    @classmethod
    def _segment_records(cls, segments: Sequence[Any]) -> list[dict[str, Any]]:
        return [
            {
                "index": int(getattr(segment, "index", position)),
                "kind": "segment",
                "start": cls._pose_record(segment.start),
                "end": cls._pose_record(segment.end),
                "observation": cls._empty_observation(),
            }
            for position, segment in enumerate(segments)
        ]

    @staticmethod
    def _empty_observation() -> dict[str, Any]:
        return {
            "status": "awaiting_post_contour",
            "expected_gain_mm": None,
            "observed_gain_mm": None,
            "scan_offset_mm": None,
            "pre_contour_match_distance_mm": None,
            "post_axis_lateral_distance_mm": None,
            "residual_dispersion_mm": None,
            "confidence": None,
        }

    @staticmethod
    def _pose_record(pose_stamped: Any) -> dict[str, Any]:
        pose = pose_stamped.pose
        quaternion = [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ]
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1e-12:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            qx, qy, qz, qw = (value / norm for value in quaternion)
        return {
            "position_m": [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ],
            "z_axis": [
                2.0 * (qx * qz + qy * qw),
                2.0 * (qy * qz - qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        }

    @classmethod
    def _write_record(cls, path: Path, record: Mapping[str, Any]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(cls._json_safe(record), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
        cls._write_csv(path.parent / cls.CSV_FILENAME, record.get("targets", []))

    @staticmethod
    def _write_csv(path: Path, targets: Sequence[Mapping[str, Any]]) -> None:
        columns = [
            "index",
            "kind",
            "expected_gain_mm",
            "observed_gain_mm",
            "scan_offset_mm",
            "pre_contour_match_distance_mm",
            "post_axis_lateral_distance_mm",
            "residual_dispersion_mm",
            "confidence",
            "status",
        ]
        temporary = path.with_suffix(path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            for target in targets:
                observation = target.get("observation", {})
                writer.writerow(
                    {
                        "index": target.get("index"),
                        "kind": target.get("kind"),
                        **{
                            column: observation.get(column)
                            for column in columns
                            if column not in ("index", "kind", "status")
                        },
                        "status": observation.get("status"),
                    }
                )
        temporary.replace(path)

    @classmethod
    def _json_safe(cls, value: Any) -> Any:
        if value is None or isinstance(value, (str, bool, int)):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else None
        if isinstance(value, Mapping):
            return {str(key): cls._json_safe(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._json_safe(item) for item in value]
        if hasattr(value, "item"):
            return cls._json_safe(value.item())
        return str(value)

    @staticmethod
    def _resolved_path(raw_path: str) -> str:
        text = str(raw_path or "").strip()
        return str(Path(text).expanduser().resolve(strict=False)) if text else ""

    @staticmethod
    def _utc_now() -> str:
        return datetime.now(timezone.utc).isoformat()
