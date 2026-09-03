"""Random, non-heuristic action proposals sampled from phi contours."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
import numpy.typing as npt


FloatArray = npt.NDArray[np.float64]
BoolArray = npt.NDArray[np.bool_]
IntArray = npt.NDArray[np.int32]


class PoseCollisionChecker(Protocol):
    """Minimal interface implemented by the existing extruder checker."""

    def check_pose(
        self,
        position: np.ndarray,
        z_axis: np.ndarray,
    ) -> tuple[bool, float, int]: ...


@dataclass(frozen=True)
class ActionCandidateBatch:
    """Complete bead proposals before geometric validity filtering."""

    source_points: FloatArray
    target_points: FloatArray
    z_axes: FloatArray
    heights_m: FloatArray
    width_map_m: FloatArray
    widths_m: FloatArray
    heat: FloatArray
    contour_segment_indices: IntArray
    segment_fractions: FloatArray

    @property
    def count(self) -> int:
        return int(self.source_points.shape[0])


@dataclass(frozen=True)
class CandidateValidation:
    """Contact and extruder-collision results for an action pool."""

    contact_hit_points: FloatArray
    contact_distances_m: FloatArray
    contact_source_errors_m: FloatArray
    contact_valid: BoolArray
    collision_checked: BoolArray
    collides: BoolArray
    extruder_clearance_mm: FloatArray
    extruder_hit_vertices: IntArray
    valid: BoolArray


def _points(values: npt.ArrayLike, name: str) -> FloatArray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _lines(values: npt.ArrayLike, point_count: int) -> IntArray:
    raw = np.asarray(values)
    if raw.ndim != 2 or raw.shape[1] != 2:
        raise ValueError(f"contour_lines must have shape (M, 2), got {raw.shape}")
    result = raw.astype(np.int32, copy=False)
    if result.size and (np.any(result < 0) or np.any(result >= point_count)):
        raise ValueError("contour_lines contains an invalid point index")
    return result


def _per_contour_point(
    values: npt.ArrayLike,
    point_count: int,
    name: str,
) -> FloatArray:
    result = np.asarray(values, dtype=np.float64).reshape(-1)
    if result.shape[0] != point_count:
        raise ValueError(f"{name} must contain one value per contour point")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _finite_bounds(lower: float, upper: float, name: str) -> tuple[float, float]:
    low = float(lower)
    high = float(upper)
    if not np.isfinite(low) or not np.isfinite(high) or high < low:
        raise ValueError(f"invalid {name} bounds: ({lower}, {upper})")
    return low, high


def sample_action_candidates(
    *,
    contour_points: npt.ArrayLike,
    contour_lines: npt.ArrayLike,
    contour_heat: npt.ArrayLike,
    contour_widths_m: npt.ArrayLike,
    count: int,
    rng: np.random.Generator,
    height_min_m: float,
    height_max_m: float,
    cone_max_tilt_deg: float,
    width_delta_min_m: float,
    width_delta_max_m: float,
    width_min_m: float,
    width_max_m: float,
) -> ActionCandidateBatch:
    """Sample complete actions uniformly by contour arc length.

    No heat, gradient, or width value changes the sampling probability. Heat
    and nominal width are attached only as candidate features. Cone directions
    are uniform in solid angle rather than uniform in tilt angle.
    """

    points = _points(contour_points, "contour_points")
    lines = _lines(contour_lines, points.shape[0])
    if int(count) <= 0:
        raise ValueError("count must be > 0")
    if lines.shape[0] == 0:
        raise ValueError("cannot sample actions from an empty phi contour")
    heat_at_points = _per_contour_point(
        contour_heat,
        points.shape[0],
        "contour_heat",
    )
    widths_at_points = _per_contour_point(
        contour_widths_m,
        points.shape[0],
        "contour_widths_m",
    )
    if np.any(widths_at_points <= 0.0):
        raise ValueError("contour_widths_m values must be positive")

    height_min, height_max = _finite_bounds(
        height_min_m,
        height_max_m,
        "height",
    )
    if height_min <= 0.0:
        raise ValueError("height bounds must be positive")
    delta_min, delta_max = _finite_bounds(
        width_delta_min_m,
        width_delta_max_m,
        "width delta",
    )
    width_min, width_max = _finite_bounds(width_min_m, width_max_m, "width")
    if width_min <= 0.0:
        raise ValueError("width bounds must be positive")
    cone_degrees = float(cone_max_tilt_deg)
    if not np.isfinite(cone_degrees) or not 0.0 <= cone_degrees <= 90.0:
        raise ValueError("cone_max_tilt_deg must be in [0, 90]")

    starts = points[lines[:, 0]]
    ends = points[lines[:, 1]]
    lengths = np.linalg.norm(ends - starts, axis=1)
    usable = lengths > 1e-12
    if not np.any(usable):
        raise ValueError("phi contour contains no non-degenerate segments")
    usable_indices = np.flatnonzero(usable)
    usable_lengths = lengths[usable]
    probabilities = usable_lengths / float(np.sum(usable_lengths))

    sampled_usable = rng.choice(
        usable_indices.shape[0],
        size=int(count),
        replace=True,
        p=probabilities,
    )
    segment_indices = usable_indices[sampled_usable]
    fractions = rng.random(int(count))
    line_pairs = lines[segment_indices]
    source_points = (
        (1.0 - fractions[:, np.newaxis]) * points[line_pairs[:, 0]]
        + fractions[:, np.newaxis] * points[line_pairs[:, 1]]
    )
    heat = (
        (1.0 - fractions) * heat_at_points[line_pairs[:, 0]]
        + fractions * heat_at_points[line_pairs[:, 1]]
    )
    width_map = (
        (1.0 - fractions) * widths_at_points[line_pairs[:, 0]]
        + fractions * widths_at_points[line_pairs[:, 1]]
    )

    heights = rng.uniform(height_min, height_max, int(count))
    azimuth = rng.uniform(0.0, 2.0 * np.pi, int(count))
    cos_tilt = rng.uniform(
        np.cos(np.deg2rad(cone_degrees)),
        1.0,
        int(count),
    )
    sin_tilt = np.sqrt(np.maximum(0.0, 1.0 - cos_tilt**2))
    z_axes = np.column_stack(
        (
            sin_tilt * np.cos(azimuth),
            sin_tilt * np.sin(azimuth),
            cos_tilt,
        )
    )
    targets = source_points + heights[:, np.newaxis] * z_axes

    width_deltas = rng.uniform(delta_min, delta_max, int(count))
    widths = np.clip(width_map + width_deltas, width_min, width_max)

    return ActionCandidateBatch(
        source_points=source_points,
        target_points=targets,
        z_axes=z_axes,
        heights_m=heights,
        width_map_m=width_map,
        widths_m=widths,
        heat=heat,
        contour_segment_indices=segment_indices.astype(np.int32, copy=False),
        segment_fractions=fractions,
    )


def validate_action_candidates(
    *,
    candidates: ActionCandidateBatch,
    scan_vertices: npt.ArrayLike,
    scan_faces: npt.ArrayLike,
    contact_distance_min_m: float,
    contact_distance_max_m: float,
    contact_source_tolerance_m: float,
    collision_checker: PoseCollisionChecker | None,
) -> CandidateValidation:
    """Raycast each proposed bead axis and apply the extruder collision proxy."""

    import open3d as o3d

    vertices = _points(scan_vertices, "scan_vertices")
    faces = np.asarray(scan_faces, dtype=np.int32)
    if faces.ndim != 2 or faces.shape[1] != 3:
        raise ValueError(f"scan_faces must have shape (M, 3), got {faces.shape}")
    if faces.size and (np.any(faces < 0) or np.any(faces >= vertices.shape[0])):
        raise ValueError("scan_faces contains an invalid vertex index")
    distance_min, distance_max = _finite_bounds(
        contact_distance_min_m,
        contact_distance_max_m,
        "contact distance",
    )
    tolerance = float(contact_source_tolerance_m)
    if not np.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("contact_source_tolerance_m must be finite and >= 0")

    tensor_mesh = o3d.t.geometry.TriangleMesh()
    tensor_mesh.vertex["positions"] = o3d.core.Tensor(vertices.astype(np.float32))
    tensor_mesh.triangle["indices"] = o3d.core.Tensor(faces.astype(np.int32))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(tensor_mesh)

    rays = np.column_stack(
        (
            candidates.target_points.astype(np.float32),
            (-candidates.z_axes).astype(np.float32),
        )
    )
    hit_distances = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy().astype(np.float64)
    has_hit = np.isfinite(hit_distances)
    hit_points = np.full_like(candidates.target_points, np.nan, dtype=np.float64)
    hit_points[has_hit] = (
        candidates.target_points[has_hit]
        - hit_distances[has_hit, np.newaxis] * candidates.z_axes[has_hit]
    )
    source_errors = np.full(candidates.count, np.inf, dtype=np.float64)
    source_errors[has_hit] = np.linalg.norm(
        hit_points[has_hit] - candidates.source_points[has_hit],
        axis=1,
    )
    contact_valid = (
        has_hit
        & (hit_distances >= distance_min)
        & (hit_distances <= distance_max)
        & (source_errors <= tolerance)
    )

    collision_checked = np.zeros(candidates.count, dtype=bool)
    collides = np.zeros(candidates.count, dtype=bool)
    clearance_mm = np.full(candidates.count, np.nan, dtype=np.float64)
    hit_vertices = np.zeros(candidates.count, dtype=np.int32)
    if collision_checker is not None:
        for index in np.flatnonzero(contact_valid):
            collision_checked[index] = True
            collision, clearance, hits = collision_checker.check_pose(
                candidates.target_points[index],
                candidates.z_axes[index],
            )
            collides[index] = bool(collision)
            clearance_mm[index] = float(clearance)
            hit_vertices[index] = int(hits)

    valid = contact_valid & ~collides
    return CandidateValidation(
        contact_hit_points=hit_points,
        contact_distances_m=hit_distances,
        contact_source_errors_m=source_errors,
        contact_valid=contact_valid,
        collision_checked=collision_checked,
        collides=collides,
        extruder_clearance_mm=clearance_mm,
        extruder_hit_vertices=hit_vertices,
        valid=valid,
    )
