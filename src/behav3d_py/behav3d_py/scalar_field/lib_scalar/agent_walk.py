#!/usr/bin/env python3
"""Simple surface-walk agents for candidate generation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import (
    project_points_to_surface,
    sample_tangent_axes_on_surface_from_scalar,
    sample_vertex_scalar_on_surface,
)


@dataclass(frozen=True)
class AgentWalkConfig:
    """Parameters for the simple gradient-walk agent."""

    target_distance: float
    step_size: float = 1e-3
    max_steps: int = 32
    tangent_sign: float = 1.0
    source_min_distance: float = 0.008
    final_min_distance: float = 0.008
    final_min_phi: float = 0.004
    segment_start_fraction: float = 0.25
    clamp_to_cone: bool = False
    cone_max_tilt_deg: float = 45.0


@dataclass(frozen=True)
class AgentWalkResult:
    """Accepted agent walk endpoints and their original source indices."""

    points: np.ndarray
    source_points: np.ndarray
    segment_start_points: np.ndarray
    accepted_indices: np.ndarray
    final_phi: np.ndarray


def _clamp_direction_to_cone(
    direction: np.ndarray,
    *,
    z_axis: np.ndarray,
    cos_max: float,
    sin_max: float,
) -> np.ndarray:
    n = float(np.linalg.norm(direction))
    if n <= 1e-12:
        return z_axis.copy()
    u = direction / n
    c = float(np.dot(u, z_axis))
    if c >= cos_max:
        return u
    p = u - c * z_axis
    pn = float(np.linalg.norm(p))
    if pn <= 1e-12:
        p = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        p = p / pn
    return cos_max * z_axis + sin_max * p


def _clamp_point_to_cone(
    source_point: np.ndarray,
    walked_point: np.ndarray,
    *,
    field_vertices_world: np.ndarray,
    field_faces: np.ndarray,
    cone_max_tilt_deg: float,
) -> np.ndarray:
    disp = walked_point - source_point
    disp_norm = float(np.linalg.norm(disp))
    if disp_norm <= 1e-12:
        return walked_point.copy()

    z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    max_tilt_rad = np.deg2rad(float(np.clip(cone_max_tilt_deg, 0.0, 179.9)))
    u_clamped = _clamp_direction_to_cone(
        disp,
        z_axis=z_axis,
        cos_max=float(np.cos(max_tilt_rad)),
        sin_max=float(np.sin(max_tilt_rad)),
    )
    target = source_point + disp_norm * u_clamped
    return project_points_to_surface(target.reshape(1, 3), field_vertices_world, field_faces)[0]


def _walk_one_agent(
    source_point: np.ndarray,
    field_vertices_world: np.ndarray,
    field_faces: np.ndarray,
    field_scalar: np.ndarray,
    config: AgentWalkConfig,
) -> np.ndarray:
    target = max(0.0, float(config.target_distance))
    source = project_points_to_surface(source_point.reshape(1, 3), field_vertices_world, field_faces)[0]
    if target <= 0.0:
        return source

    step = max(1e-6, float(config.step_size))
    max_steps = max(1, int(config.max_steps))
    current = source.copy()
    eps = 1e-9

    for _ in range(max_steps):
        disp_now = float(np.linalg.norm(current - source))
        remaining = target - disp_now
        if remaining <= eps:
            break

        tangent_dir, _b_dir, _n_dir = sample_tangent_axes_on_surface_from_scalar(
            query_points=current.reshape(1, 3),
            mesh_vertices=field_vertices_world,
            mesh_faces=field_faces,
            vertex_scalar=field_scalar,
            tangent_sign=float(config.tangent_sign),
        )
        trial = current + min(step, max(remaining, 0.0)) * tangent_dir[0]
        current = project_points_to_surface(trial.reshape(1, 3), field_vertices_world, field_faces)[0]

    if bool(config.clamp_to_cone):
        current = _clamp_point_to_cone(
            source,
            current,
            field_vertices_world=field_vertices_world,
            field_faces=field_faces,
            cone_max_tilt_deg=float(config.cone_max_tilt_deg),
        )
    return current


def _segment_start_points(
    source_points: np.ndarray,
    final_points: np.ndarray,
    field_vertices_world: np.ndarray,
    field_faces: np.ndarray,
    fraction: float,
) -> np.ndarray:
    """Interpolate source->final by normalized fraction and project to surface."""
    if source_points.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    alpha = float(np.clip(float(fraction), 0.0, 1.0))
    raw = source_points + alpha * (final_points - source_points)
    return project_points_to_surface(raw, field_vertices_world, field_faces)


def run_agent_walk(
    source_points: np.ndarray,
    field_vertices_world: np.ndarray,
    field_faces: np.ndarray,
    field_scalar: np.ndarray,
    *,
    config: AgentWalkConfig,
    phi_scalar: np.ndarray | None = None,
) -> AgentWalkResult:
    """Run simple independent walk agents with source/final spacing filters."""
    if source_points.shape[0] == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return AgentWalkResult(
            points=empty,
            source_points=empty,
            segment_start_points=empty,
            accepted_indices=np.zeros((0,), dtype=np.int32),
            final_phi=np.zeros((0,), dtype=np.float64),
        )

    sources_surface = project_points_to_surface(source_points, field_vertices_world, field_faces)
    accepted_sources: list[np.ndarray] = []
    accepted_points: list[np.ndarray] = []
    accepted_indices: list[int] = []
    accepted_phi: list[float] = []

    source_min = max(0.0, float(config.source_min_distance))
    final_min = max(0.0, float(config.final_min_distance))
    final_min_phi = float(config.final_min_phi)

    for idx, source in enumerate(sources_surface):
        if accepted_sources and source_min > 0.0:
            d_source = np.linalg.norm(np.asarray(accepted_sources) - source, axis=1)
            if np.any(d_source < source_min):
                continue

        final = _walk_one_agent(
            source,
            field_vertices_world,
            field_faces,
            field_scalar,
            config,
        )

        if phi_scalar is not None:
            phi_final = float(
                sample_vertex_scalar_on_surface(
                    final.reshape(1, 3),
                    field_vertices_world,
                    field_faces,
                    phi_scalar,
                )[0]
            )
            if phi_final < final_min_phi:
                continue
        else:
            phi_final = float("nan")

        if accepted_points and final_min > 0.0:
            d_final = np.linalg.norm(np.asarray(accepted_points) - final, axis=1)
            if np.any(d_final < final_min):
                continue

        accepted_sources.append(source.copy())
        accepted_points.append(final.copy())
        accepted_indices.append(int(idx))
        accepted_phi.append(phi_final)

    if not accepted_points:
        empty = np.zeros((0, 3), dtype=np.float64)
        return AgentWalkResult(
            points=empty,
            source_points=empty,
            segment_start_points=empty,
            accepted_indices=np.zeros((0,), dtype=np.int32),
            final_phi=np.zeros((0,), dtype=np.float64),
        )

    points = np.vstack(accepted_points)
    sources = np.vstack(accepted_sources)
    segment_starts = _segment_start_points(
        source_points=sources,
        final_points=points,
        field_vertices_world=field_vertices_world,
        field_faces=field_faces,
        fraction=float(config.segment_start_fraction),
    )

    return AgentWalkResult(
        points=points,
        source_points=sources,
        segment_start_points=segment_starts,
        accepted_indices=np.asarray(accepted_indices, dtype=np.int32),
        final_phi=np.asarray(accepted_phi, dtype=np.float64),
    )
