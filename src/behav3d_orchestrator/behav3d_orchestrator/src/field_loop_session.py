#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import numpy as np

from .control_session import ControlAwareSession


class FieldLoopSession(ControlAwareSession):
    """Field-state helpers used by the field-oriented print loop."""

    @staticmethod
    def compute_field_center_from_state(field_state_path: str) -> tuple[float, float, float]:
        path = Path(str(field_state_path).strip()).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Field state file not found: {field_state_path}")

        with np.load(str(path), allow_pickle=False) as state:
            if "field_vertices_world" in state:
                vertices_world = np.asarray(state["field_vertices_world"], dtype=np.float64)
                if vertices_world.ndim == 2 and vertices_world.shape[1] == 3 and vertices_world.shape[0] > 0:
                    center = np.mean(vertices_world, axis=0)
                    return (float(center[0]), float(center[1]), float(center[2]))

            if "field_vertices_scaled" in state and "offset_xyz" in state:
                vertices_scaled = np.asarray(state["field_vertices_scaled"], dtype=np.float64)
                offset_arr = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(-1)
                if (
                    vertices_scaled.ndim == 2
                    and vertices_scaled.shape[1] == 3
                    and vertices_scaled.shape[0] > 0
                    and offset_arr.shape[0] >= 3
                ):
                    center = np.mean(vertices_scaled, axis=0) + offset_arr[:3]
                    return (float(center[0]), float(center[1]), float(center[2]))

            if "offset_xyz" in state:
                offset_arr = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(-1)
                if offset_arr.shape[0] >= 3:
                    return (float(offset_arr[0]), float(offset_arr[1]), float(offset_arr[2]))

        raise RuntimeError(
            "Field state does not contain enough data to infer field center "
            "(expected field_vertices_world or field_vertices_scaled+offset_xyz or offset_xyz)."
        )

    @staticmethod
    def map_field_center_xy(
        *,
        x: float,
        y: float,
        sign_x: float,
        sign_y: float,
        offset_x: float,
        offset_y: float,
    ) -> tuple[float, float]:
        return (
            float(sign_x) * float(x) + float(offset_x),
            float(sign_y) * float(y) + float(offset_y),
        )
