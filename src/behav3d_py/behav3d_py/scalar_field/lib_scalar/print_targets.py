#!/usr/bin/env python3
"""Build and write oriented print targets from scalar-field candidates."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .geometry import (
    clamp_vectors_to_cone,
    project_points_to_surface,
    sample_tangent_axes_on_surface_from_scalar,
)


@dataclass(frozen=True)
class OrientedLineTargets:
    """Oriented target pairs defining one printable line segment each."""

    start_points: np.ndarray
    end_points: np.ndarray
    start_z_dirs: np.ndarray
    end_z_dirs: np.ndarray

    @property
    def count(self) -> int:
        return int(self.start_points.shape[0])

    def flattened_points_and_z_dirs(self) -> tuple[np.ndarray, np.ndarray]:
        """Return start/end pairs as an ordered target stream."""
        if self.count == 0:
            empty = np.zeros((0, 3), dtype=np.float64)
            return empty, empty

        points = np.empty((2 * self.count, 3), dtype=np.float64)
        z_dirs = np.empty((2 * self.count, 3), dtype=np.float64)
        points[0::2] = self.start_points
        points[1::2] = self.end_points
        z_dirs[0::2] = self.start_z_dirs
        z_dirs[1::2] = self.end_z_dirs
        return points, z_dirs


def _validate_points(points: np.ndarray, name: str) -> np.ndarray:
    out = np.asarray(points, dtype=np.float64)
    if out.ndim != 2 or out.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), got {out.shape}")
    return out


def _normalize_dirs(dirs: np.ndarray) -> np.ndarray:
    out = _validate_points(dirs, "z_dirs").copy()
    nrm = np.linalg.norm(out, axis=1)
    valid = nrm > 1e-12
    out[valid] /= nrm[valid, None]
    out[~valid] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return out


def _plane_string(point: np.ndarray, z_dir: np.ndarray, position_scale: float) -> str:
    p = np.asarray(point, dtype=np.float64)
    z = np.asarray(z_dir, dtype=np.float64)
    scale = float(position_scale)
    ox = scale * float(p[0])
    oy = scale * float(p[1])
    oz = scale * float(p[2])
    return f'O({ox:.2f},{oy:.2f},{oz:.2f}) Z({float(z[0]):.2f},{float(z[1]):.2f},{float(z[2]):.2f})'


def _yaw_rotation(base_to_world_yaw_deg: float) -> np.ndarray:
    yaw = np.deg2rad(float(base_to_world_yaw_deg))
    c = float(np.cos(yaw))
    s = float(np.sin(yaw))
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _apply_yaw(points: np.ndarray, z_dirs: np.ndarray, base_to_world_yaw_deg: float) -> tuple[np.ndarray, np.ndarray]:
    points_in = _validate_points(points, "points")
    z_dirs_in = _normalize_dirs(z_dirs)
    if abs(float(base_to_world_yaw_deg)) <= 1e-12:
        return points_in.copy(), z_dirs_in

    rot = _yaw_rotation(float(base_to_world_yaw_deg))
    return points_in @ rot.T, _normalize_dirs(z_dirs_in @ rot.T)


def orient_points_with_tangent(
    points: np.ndarray,
    field_vertices_world: np.ndarray,
    field_faces: np.ndarray,
    field_scalar: np.ndarray,
    *,
    tangent_sign: float = 1.0,
    clamp_to_cone: bool = False,
    cone_max_tilt_deg: float = 45.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Project points to surface and orient target Z along scalar tangent."""
    points_in = _validate_points(points, "points")
    if points_in.shape[0] == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return empty, empty

    projected = project_points_to_surface(points_in, field_vertices_world, field_faces)
    tangent_dir, _b_dir, _n_dir = sample_tangent_axes_on_surface_from_scalar(
        query_points=projected,
        mesh_vertices=field_vertices_world,
        mesh_faces=field_faces,
        vertex_scalar=field_scalar,
        tangent_sign=float(tangent_sign),
    )
    z_dirs = np.asarray(tangent_dir, dtype=np.float64)
    if bool(clamp_to_cone):
        z_dirs = clamp_vectors_to_cone(
            z_dirs,
            max_tilt_deg=float(cone_max_tilt_deg),
            cone_axis=(0.0, 0.0, 1.0),
        )
    else:
        z_dirs = _normalize_dirs(z_dirs)
    return projected, z_dirs


def build_oriented_line_targets(
    start_points: np.ndarray,
    end_points: np.ndarray,
    field_vertices_world: np.ndarray,
    field_faces: np.ndarray,
    field_scalar: np.ndarray,
    *,
    tangent_sign: float = 1.0,
    clamp_to_cone: bool = False,
    cone_max_tilt_deg: float = 45.0,
) -> OrientedLineTargets:
    """Build oriented start/end target pairs from candidate line geometry."""
    starts = _validate_points(start_points, "start_points")
    ends = _validate_points(end_points, "end_points")
    if starts.shape != ends.shape:
        raise ValueError(f"start/end shape mismatch: {starts.shape} vs {ends.shape}")

    if starts.shape[0] == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return OrientedLineTargets(empty, empty, empty, empty)

    paired_points = np.empty((2 * starts.shape[0], 3), dtype=np.float64)
    paired_points[0::2] = starts
    paired_points[1::2] = ends
    oriented_points, z_dirs = orient_points_with_tangent(
        paired_points,
        field_vertices_world,
        field_faces,
        field_scalar,
        tangent_sign=float(tangent_sign),
        clamp_to_cone=bool(clamp_to_cone),
        cone_max_tilt_deg=float(cone_max_tilt_deg),
    )
    return OrientedLineTargets(
        start_points=oriented_points[0::2],
        end_points=oriented_points[1::2],
        start_z_dirs=z_dirs[0::2],
        end_z_dirs=z_dirs[1::2],
    )


def write_point_targets_yaml(
    out_yaml: Path,
    points_world: np.ndarray,
    z_dirs_world: np.ndarray,
    position_scale: float,
    base_to_world_yaw_deg: float = 0.0,
) -> None:
    """Write flat target YAML with one plane per point."""
    points, z_dirs = _apply_yaw(points_world, z_dirs_world, float(base_to_world_yaw_deg))
    if points.shape[0] != z_dirs.shape[0]:
        raise ValueError(f"points/z_dirs size mismatch: {points.shape[0]} vs {z_dirs.shape[0]}")

    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    if points.shape[0] == 0:
        out_yaml.write_text("targets: []\n", encoding="utf-8")
        return

    lines: list[str] = ["targets:"]
    for i in range(points.shape[0]):
        plane = _plane_string(points[i], z_dirs[i], float(position_scale))
        lines.append(f"  - index: {i}")
        lines.append(f'    plane: "{plane}"')
    out_yaml.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_fixed_z_targets_yaml(
    out_yaml: Path,
    points_world: np.ndarray,
    z_dir: tuple[float, float, float],
    position_scale: float,
    base_to_world_yaw_deg: float = 0.0,
) -> None:
    """Write flat target YAML with a shared fixed Z direction."""
    points = _validate_points(points_world, "points_world")
    z_dirs = np.tile(
        np.asarray([[float(z_dir[0]), float(z_dir[1]), float(z_dir[2])]], dtype=np.float64),
        (points.shape[0], 1),
    )
    write_point_targets_yaml(
        out_yaml,
        points,
        z_dirs,
        position_scale,
        base_to_world_yaw_deg=float(base_to_world_yaw_deg),
    )


def write_line_targets_yaml(
    out_yaml: Path,
    targets: OrientedLineTargets,
    position_scale: float,
    base_to_world_yaw_deg: float = 0.0,
) -> None:
    """Write paired line targets as nested segment start/end planes."""
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    if targets.count == 0:
        out_yaml.write_text("segments: []\n", encoding="utf-8")
        return

    start_points, start_z_dirs = _apply_yaw(
        targets.start_points,
        targets.start_z_dirs,
        float(base_to_world_yaw_deg),
    )
    end_points, end_z_dirs = _apply_yaw(
        targets.end_points,
        targets.end_z_dirs,
        float(base_to_world_yaw_deg),
    )

    lines: list[str] = ["segments:"]
    for i in range(targets.count):
        start_plane = _plane_string(
            start_points[i],
            start_z_dirs[i],
            float(position_scale),
        )
        end_plane = _plane_string(
            end_points[i],
            end_z_dirs[i],
            float(position_scale),
        )
        lines.append(f"  - index: {i}")
        lines.append("    start:")
        lines.append(f'      plane: "{start_plane}"')
        lines.append("    end:")
        lines.append(f'      plane: "{end_plane}"')

    out_yaml.write_text("\n".join(lines) + "\n", encoding="utf-8")
