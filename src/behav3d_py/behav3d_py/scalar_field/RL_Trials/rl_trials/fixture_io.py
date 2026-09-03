"""Loading helpers for RL experiment configurations and geometry fixtures."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
import yaml


FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int32]


def load_experiment_config(path: Path) -> dict[str, Any]:
    """Load one machine-readable RL experiment configuration."""

    with Path(path).open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"Experiment config must contain a mapping: {path}")
    for section in ("paths", "scan", "goal", "completion", "action"):
        if not isinstance(config.get(section), dict):
            raise ValueError(f"Experiment config is missing mapping section '{section}'")
    return config


def resolve_workspace_path(workspace_root: Path, configured_path: str) -> Path:
    """Resolve a config path relative to the selected workspace root."""

    value = Path(configured_path).expanduser()
    resolved = value.resolve() if value.is_absolute() else (workspace_root / value).resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"Configured fixture file does not exist: {resolved}")
    return resolved


def load_goal_from_field_state(path: Path) -> tuple[FloatArray, IntArray]:
    """Load positioned goal vertices and faces from a scalar field-state NPZ."""

    with np.load(path, allow_pickle=False) as state:
        if "field_vertices_scaled" in state:
            vertices = np.asarray(state["field_vertices_scaled"], dtype=np.float64)
        elif "field_vertices" in state and "field_scale" in state:
            scale = float(np.asarray(state["field_scale"]).reshape(-1)[0])
            vertices = np.asarray(state["field_vertices"], dtype=np.float64) * scale
        else:
            raise KeyError(
                "Field state requires field_vertices_scaled or field_vertices + field_scale"
            )
        if "field_faces" not in state or "offset_xyz" not in state:
            raise KeyError("Field state requires field_faces and offset_xyz")
        faces = np.asarray(state["field_faces"], dtype=np.int32)
        offset = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(3)
    return vertices + offset, faces


def load_scan_mesh_arrays(
    path: Path,
    *,
    scale: float,
    yaw_deg: float,
) -> tuple[FloatArray, IntArray]:
    """Load the scan and apply the configured scale and world-Z yaw."""

    import open3d as o3d

    mesh = o3d.io.read_triangle_mesh(str(path))
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.triangles, dtype=np.int32)
    if vertices.size == 0 or faces.size == 0:
        raise ValueError(f"Scan mesh is empty: {path}")
    scale_value = float(scale)
    if not np.isfinite(scale_value) or scale_value <= 0.0:
        raise ValueError(f"scan scale must be finite and > 0, got {scale}")
    vertices = vertices * scale_value
    yaw = np.deg2rad(float(yaw_deg))
    if not np.isfinite(yaw):
        raise ValueError(f"scan yaw must be finite, got {yaw_deg}")
    rotation = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return vertices @ rotation.T, faces


def load_configured_geometry(
    config: dict[str, Any],
    workspace_root: Path,
) -> tuple[FloatArray, IntArray, FloatArray, IntArray]:
    """Load goal and scan arrays referenced by an experiment config."""

    paths = config["paths"]
    field_state_path = resolve_workspace_path(workspace_root, str(paths["field_state"]))
    scan_mesh_path = resolve_workspace_path(workspace_root, str(paths["scan_mesh"]))
    goal_vertices, goal_faces = load_goal_from_field_state(field_state_path)
    scan_vertices, scan_faces = load_scan_mesh_arrays(
        scan_mesh_path,
        scale=float(config["scan"]["scale"]),
        yaw_deg=float(config["scan"]["yaw_deg"]),
    )
    return goal_vertices, goal_faces, scan_vertices, scan_faces
