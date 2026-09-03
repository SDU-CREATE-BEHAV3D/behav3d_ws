"""DDS-native visual geometry for RL goal-evaluation states."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

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


def add_evaluation_scene_to_plotter(
    plotter: Any,
    scene: EvaluationScene,
    pyvista_module: Any,
    *,
    hide_scan: bool = False,
    scan_opacity: float = 0.32,
    goal_opacity: float = 0.82,
    frontier_size: float = 4.0,
) -> None:
    """Add the common scan, goal, and frontier scene to a PyVista plotter."""

    from dds.viz.converters import point_cloud_to_polydata, triangle_mesh_to_polydata

    if not hide_scan:
        plotter.add_mesh(
            triangle_mesh_to_polydata(scene.scan_mesh, pyvista_module),
            color="#95a5a6",
            opacity=scan_opacity,
            smooth_shading=False,
        )
    plotter.add_mesh(
        triangle_mesh_to_polydata(scene.goal_mesh, pyvista_module),
        scalars="vertex_colors",
        rgb=True,
        opacity=goal_opacity,
        show_edges=True,
        smooth_shading=False,
        show_scalar_bar=False,
    )
    if not scene.frontier_points.is_empty:
        plotter.add_mesh(
            point_cloud_to_polydata(scene.frontier_points, pyvista_module),
            scalars="point_colors",
            rgb=True,
            point_size=frontier_size,
            render_points_as_spheres=True,
            show_scalar_bar=False,
        )


def add_evaluation_scene_to_viewer(
    viewer: Any,
    scene: EvaluationScene,
    *,
    hide_scan: bool = False,
    scan_opacity: float = 0.32,
    goal_opacity: float = 0.82,
    frontier_size: float = 4.0,
) -> None:
    """Add the common scan, goal, and frontier scene to a DDS Viewer."""

    from dds.viz import MeshStyle, PointCloudStyle

    if not hide_scan:
        viewer.add_mesh(
            scene.scan_mesh,
            name="initial_scan",
            style=MeshStyle(
                color="#95a5a6",
                opacity=scan_opacity,
                show_edges=False,
                smooth_shading=False,
            ),
        )
    viewer.add_mesh(
        scene.goal_mesh,
        name="classified_goal",
        style=MeshStyle(
            color=None,
            opacity=goal_opacity,
            show_edges=True,
            smooth_shading=False,
        ),
    )
    if not scene.frontier_points.is_empty:
        viewer.add_point_cloud(
            scene.frontier_points,
            name="vertex_frontier",
            style=PointCloudStyle(
                color=None,
                size=frontier_size,
                render_as_spheres=True,
                opacity=1.0,
            ),
        )
