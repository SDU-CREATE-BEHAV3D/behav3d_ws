"""DDS-native visual geometry for RL goal-evaluation states."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from dds.geometry import PointCloud, TriangleMesh

from .goal_evaluator import GoalEvaluation


UNFILLED_COLOR = np.array([230, 126, 34], dtype=np.uint8)
FILLED_COLOR = np.array([46, 204, 113], dtype=np.uint8)
UNRESOLVED_COLOR = np.array([52, 73, 94], dtype=np.uint8)
FRONTIER_COLOR = np.array([0, 188, 212], dtype=np.uint8)


@dataclass(frozen=True)
class EvaluationScene:
    """DDS geometry objects consumed by the native DDS Viewer."""

    scan_mesh: TriangleMesh
    goal_mesh: TriangleMesh
    frontier_points: PointCloud
    phi_contour_points: npt.NDArray[np.float64]
    phi_contour_lines: npt.NDArray[np.int32]


def evaluation_vertex_colors(evaluation: GoalEvaluation) -> npt.NDArray[np.uint8]:
    """Map mutually relevant goal states to stable RGB colors."""

    colors = np.tile(UNFILLED_COLOR, (evaluation.filled.shape[0], 1))
    colors[evaluation.filled] = FILLED_COLOR
    colors[evaluation.unresolved] = UNRESOLVED_COLOR
    return colors


def build_evaluation_scene(
    *,
    goal_vertices: npt.ArrayLike,
    goal_faces: npt.ArrayLike,
    scan_vertices: npt.ArrayLike,
    scan_faces: npt.ArrayLike,
    phi_contour_points: npt.ArrayLike,
    phi_contour_lines: npt.ArrayLike,
    evaluation: GoalEvaluation,
) -> EvaluationScene:
    """Build scan, classified goal, frontier, and exact phi contour data."""

    goal_vertices_array = np.asarray(goal_vertices, dtype=np.float64)
    if goal_vertices_array.shape != (evaluation.filled.shape[0], 3):
        raise ValueError(
            "goal_vertices must have shape matching the evaluation: "
            f"{goal_vertices_array.shape} vs ({evaluation.filled.shape[0]}, 3)"
        )
    colors = evaluation_vertex_colors(evaluation)
    frontier_points = goal_vertices_array[evaluation.frontier]
    frontier_colors = np.tile(FRONTIER_COLOR, (frontier_points.shape[0], 1))
    contour_points = np.asarray(phi_contour_points, dtype=np.float64)
    contour_lines = np.asarray(phi_contour_lines, dtype=np.int32)
    if contour_points.ndim != 2 or contour_points.shape[1] != 3:
        raise ValueError("phi_contour_points must have shape (N, 3)")
    if contour_lines.ndim != 2 or contour_lines.shape[1] != 2:
        raise ValueError("phi_contour_lines must have shape (M, 2)")
    if contour_lines.size and (
        np.any(contour_lines < 0) or np.any(contour_lines >= contour_points.shape[0])
    ):
        raise ValueError("phi_contour_lines contains an invalid point index")
    return EvaluationScene(
        scan_mesh=TriangleMesh(
            vertices=np.asarray(scan_vertices, dtype=np.float64),
            faces=np.asarray(scan_faces, dtype=np.int64),
        ),
        goal_mesh=TriangleMesh(
            vertices=goal_vertices_array,
            faces=np.asarray(goal_faces, dtype=np.int64),
            vertex_colors=colors,
        ),
        frontier_points=PointCloud(
            points=frontier_points,
            colors=frontier_colors,
        ),
        phi_contour_points=contour_points,
        phi_contour_lines=contour_lines,
    )
