#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


class ContourTargetOffsetEstimator:
    """Estimate target-normal scan offset from consecutive phi contours."""

    def __init__(
        self,
        *,
        contour_sample_spacing_mm: float = 0.5,
        axis_band_mm: float = 0.75,
    ) -> None:
        self.sample_spacing_m = 1e-3 * float(contour_sample_spacing_mm)
        self.axis_band_m = 1e-3 * float(axis_band_mm)
        if self.sample_spacing_m <= 0.0 or self.axis_band_m <= 0.0:
            raise ValueError("Contour estimator spacings must be > 0")

    def evaluate(
        self,
        *,
        pre_contour_path: str,
        post_contour_path: str,
        targets: Sequence[Mapping[str, Any]],
        candidate_settings: Mapping[str, Any],
    ) -> dict[str, Any]:
        pre_samples = self._load_contour_samples(pre_contour_path)
        post_samples = self._load_contour_samples(post_contour_path)
        bead_height_m = 1e-3 * float(candidate_settings.get("bead_height_mm", 0.0))
        output_yaw_deg = float(candidate_settings.get("target_output_yaw_deg", 0.0))
        candidate_mode = str(candidate_settings.get("candidate_mode", "")).strip().lower()

        observations = []
        for target in targets:
            observations.append(
                self._estimate_target(
                    target=target,
                    pre_samples=pre_samples,
                    post_samples=post_samples,
                    candidate_mode=candidate_mode,
                    bead_height_m=bead_height_m,
                    output_yaw_deg=output_yaw_deg,
                )
            )

        valid_offsets = np.asarray(
            [
                item["scan_offset_mm"]
                for item in observations
                if item.get("status") == "valid"
            ],
            dtype=np.float64,
        )
        valid_confidences = np.asarray(
            [
                item["confidence"]
                for item in observations
                if item.get("status") == "valid"
            ],
            dtype=np.float64,
        )
        valid_count = int(valid_offsets.size)
        total_count = int(len(observations))
        if valid_count:
            median = float(np.median(valid_offsets))
            mad = float(np.median(np.abs(valid_offsets - median)))
            match_confidence = float(np.median(valid_confidences))
            consistency_scale_mm = max(2.0, 0.25 * 1e3 * bead_height_m)
            spatial_consistency = math.exp(-((mad / consistency_scale_mm) ** 2))
            confidence = float(match_confidence * spatial_consistency)
            offset_p10 = float(np.percentile(valid_offsets, 10.0))
            offset_p90 = float(np.percentile(valid_offsets, 90.0))
        else:
            median = None
            mad = None
            match_confidence = 0.0
            spatial_consistency = 0.0
            confidence = 0.0
            offset_p10 = None
            offset_p90 = None

        return {
            "status": "complete" if valid_count else "insufficient_matches",
            "method": "phi_contour_to_target_axis",
            "pre_contour_sample_count": int(pre_samples.shape[0]),
            "post_contour_sample_count": int(post_samples.shape[0]),
            "target_count": total_count,
            "valid_target_count": valid_count,
            "valid_target_fraction": (
                float(valid_count / total_count) if total_count else 0.0
            ),
            "scan_offset_median_mm": median,
            "scan_offset_mad_mm": mad,
            "scan_offset_p10_mm": offset_p10,
            "scan_offset_p90_mm": offset_p90,
            "suggested_target_correction_mm": -median if median is not None else None,
            "match_confidence_median": match_confidence,
            "spatial_consistency": spatial_consistency,
            "confidence": confidence,
            "target_output_yaw_deg": output_yaw_deg,
            "contour_sample_spacing_mm": 1e3 * self.sample_spacing_m,
            "observations": observations,
        }

    def _estimate_target(
        self,
        *,
        target: Mapping[str, Any],
        pre_samples: np.ndarray,
        post_samples: np.ndarray,
        candidate_mode: str,
        bead_height_m: float,
        output_yaw_deg: float,
    ) -> dict[str, Any]:
        kind = str(target.get("kind", "dot"))
        end_pose = target.get("end") or target.get("pose") or {}
        end = self._position_in_contour_frame(end_pose, output_yaw_deg)

        if kind == "segment":
            start = self._position_in_contour_frame(
                target.get("start") or {},
                output_yaw_deg,
            )
            printed_vector = end - start
            printed_length = float(np.linalg.norm(printed_vector))
            if printed_length <= 1e-9:
                return self._invalid("degenerate_segment")
            axis = printed_vector / printed_length
            if candidate_mode == "gradient_lift" and bead_height_m > 0.0:
                expected_source = end - bead_height_m * axis
            else:
                expected_source = start
            pre_index = self._nearest_index(pre_samples, expected_source)
            pre_source = pre_samples[pre_index]
            pre_match_distance = float(np.linalg.norm(pre_source - expected_source))
        else:
            pre_index = self._nearest_index(pre_samples, end)
            pre_source = pre_samples[pre_index]
            expected_vector = end - pre_source
            expected_length = float(np.linalg.norm(expected_vector))
            if expected_length <= 1e-9:
                axis = self._axis_in_contour_frame(end_pose, output_yaw_deg)
            else:
                axis = expected_vector / expected_length
            expected_source = pre_source
            pre_match_distance = 0.0

        expected_delta = end - pre_source
        expected_gain_m = float(np.dot(expected_delta, axis))
        expected_lateral_m = float(
            np.linalg.norm(expected_delta - expected_gain_m * axis)
        )
        if expected_gain_m <= 1e-4:
            return self._invalid(
                "nonpositive_expected_gain",
                expected_gain_mm=1e3 * expected_gain_m,
            )

        scale_m = max(expected_gain_m, bead_height_m, 0.005)
        search_radius_m = max(0.010, 1.5 * scale_m)
        post_delta = post_samples - end
        axial = post_delta @ axis
        lateral = np.linalg.norm(post_delta - axial[:, None] * axis, axis=1)
        euclidean = np.linalg.norm(post_delta, axis=1)
        search_mask = (
            (axial >= -1.5 * scale_m)
            & (axial <= 0.75 * scale_m)
            & (euclidean <= search_radius_m)
        )
        search_indices = np.flatnonzero(search_mask)
        if search_indices.size == 0:
            return self._invalid(
                "no_post_contour_in_search_region",
                expected_gain_mm=1e3 * expected_gain_m,
                pre_contour_match_distance_mm=1e3 * pre_match_distance,
            )

        min_lateral_m = float(np.min(lateral[search_indices]))
        local_indices = search_indices[
            lateral[search_indices] <= min_lateral_m + self.axis_band_m
        ]
        local_offsets_m = axial[local_indices]
        scan_offset_m = float(np.median(local_offsets_m))
        residual_mad_m = float(
            np.median(np.abs(local_offsets_m - scan_offset_m))
        )
        observed_gain_m = expected_gain_m + scan_offset_m
        valid_sample_fraction = float(local_indices.size / search_indices.size)

        pre_tolerance_m = max(0.002, 0.20 * scale_m)
        lateral_tolerance_m = max(0.004, 0.25 * scale_m)
        expected_lateral_tolerance_m = max(0.002, 0.15 * scale_m)
        pre_quality = math.exp(-((pre_match_distance / pre_tolerance_m) ** 2))
        lateral_quality = math.exp(-((min_lateral_m / lateral_tolerance_m) ** 2))
        source_quality = math.exp(
            -((expected_lateral_m / expected_lateral_tolerance_m) ** 2)
        )
        dispersion_quality = math.exp(-((residual_mad_m / 0.002) ** 2))
        sample_quality = min(1.0, float(local_indices.size) / 5.0)
        confidence = float(
            pre_quality
            * lateral_quality
            * source_quality
            * dispersion_quality
            * sample_quality
        )

        valid = (
            pre_match_distance <= 2.0 * pre_tolerance_m
            and min_lateral_m <= 2.0 * lateral_tolerance_m
            and expected_lateral_m <= 2.0 * expected_lateral_tolerance_m
            and local_indices.size >= 2
        )
        return {
            "status": "valid" if valid else "low_confidence_match",
            "reason": "" if valid else "contour_match_outside_tolerance",
            "expected_gain_mm": 1e3 * expected_gain_m,
            "observed_gain_mm": 1e3 * observed_gain_m,
            "scan_offset_mm": 1e3 * scan_offset_m,
            "height_error_mm": 1e3 * scan_offset_m,
            "pre_contour_match_distance_mm": 1e3 * pre_match_distance,
            "expected_axis_lateral_error_mm": 1e3 * expected_lateral_m,
            "post_axis_lateral_distance_mm": 1e3 * min_lateral_m,
            "valid_sample_count": int(local_indices.size),
            "search_sample_count": int(search_indices.size),
            "valid_sample_fraction": valid_sample_fraction,
            "residual_dispersion_mm": 1e3 * residual_mad_m,
            "confidence": confidence,
            "axis_contour_frame": axis.tolist(),
            "expected_source_contour_frame_m": expected_source.tolist(),
            "matched_pre_source_contour_frame_m": pre_source.tolist(),
            "expected_end_contour_frame_m": end.tolist(),
        }

    def _load_contour_samples(self, raw_path: str) -> np.ndarray:
        path = Path(str(raw_path)).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"Contour PLY not found: {path}")
        try:
            import open3d as o3d
        except ImportError as exc:  # pragma: no cover - runtime dependency
            raise RuntimeError("Open3D is required to evaluate contour offsets") from exc

        line_set = o3d.io.read_line_set(str(path))
        points = np.asarray(line_set.points, dtype=np.float64)
        lines = np.asarray(line_set.lines, dtype=np.int64)
        if points.ndim != 2 or points.shape[1] != 3 or lines.size == 0:
            raise ValueError(f"Contour PLY contains no line geometry: {path}")

        samples = []
        for start_index, end_index in lines.reshape(-1, 2):
            start = points[int(start_index)]
            end = points[int(end_index)]
            delta = end - start
            length = float(np.linalg.norm(delta))
            count = max(1, int(math.ceil(length / self.sample_spacing_m)))
            fractions = np.arange(count, dtype=np.float64) / float(count)
            samples.extend(start + fractions[:, None] * delta)
            samples.append(end)
        return np.asarray(samples, dtype=np.float64)

    @classmethod
    def _position_in_contour_frame(
        cls,
        pose_record: Mapping[str, Any],
        output_yaw_deg: float,
    ) -> np.ndarray:
        position = np.asarray(pose_record.get("position_m", []), dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("Target pose has no finite position_m")
        return cls._inverse_output_yaw(position, output_yaw_deg)

    @classmethod
    def _axis_in_contour_frame(
        cls,
        pose_record: Mapping[str, Any],
        output_yaw_deg: float,
    ) -> np.ndarray:
        axis = np.asarray(pose_record.get("z_axis", []), dtype=np.float64)
        if axis.shape != (3,) or not np.all(np.isfinite(axis)):
            raise ValueError("Target pose has no finite z_axis")
        axis = cls._inverse_output_yaw(axis, output_yaw_deg)
        norm = float(np.linalg.norm(axis))
        if norm <= 1e-9:
            raise ValueError("Target pose z_axis is degenerate")
        return axis / norm

    @staticmethod
    def _inverse_output_yaw(vector: np.ndarray, yaw_deg: float) -> np.ndarray:
        yaw = math.radians(-float(yaw_deg))
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        rotation = np.asarray(
            [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        return np.asarray(vector, dtype=np.float64) @ rotation.T

    @staticmethod
    def _nearest_index(points: np.ndarray, query: np.ndarray) -> int:
        return int(np.argmin(np.linalg.norm(points - query, axis=1)))

    @staticmethod
    def _invalid(reason: str, **metrics: Any) -> dict[str, Any]:
        return {
            "status": "invalid",
            "reason": str(reason),
            "expected_gain_mm": metrics.get("expected_gain_mm"),
            "observed_gain_mm": None,
            "scan_offset_mm": None,
            "height_error_mm": None,
            "pre_contour_match_distance_mm": metrics.get(
                "pre_contour_match_distance_mm"
            ),
            "expected_axis_lateral_error_mm": None,
            "post_axis_lateral_distance_mm": None,
            "valid_sample_count": 0,
            "search_sample_count": 0,
            "valid_sample_fraction": 0.0,
            "residual_dispersion_mm": None,
            "confidence": 0.0,
        }
