#!/usr/bin/env python3
"""DDS/PyVista visualization helpers for scalar-field debug data.

The public helpers in this module are intentionally small wrappers around the
retained DDS viewer. They accept NumPy arrays from lib_scalar and can either
create a standalone viewer or attach to an existing DDS workbench plotter.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def colors_to_uint8(colors: np.ndarray | None, count: int) -> np.ndarray | None:
    """Normalize RGB/RGBA colors to uint8 for DDS PointCloud."""

    if colors is None:
        return None

    arr = np.asarray(colors)
    if arr.ndim != 2 or arr.shape[0] != int(count) or arr.shape[1] not in (3, 4):
        raise ValueError("colors must have shape (N, 3) or (N, 4)")

    if arr.dtype == np.uint8:
        return arr.copy()

    arr_float = np.asarray(arr, dtype=np.float64)
    if not np.all(np.isfinite(arr_float)):
        raise ValueError("colors must contain finite values")

    if np.nanmax(arr_float) <= 1.0:
        arr_float = 255.0 * np.clip(arr_float, 0.0, 1.0)
    else:
        arr_float = np.clip(arr_float, 0.0, 255.0)

    return np.rint(arr_float).astype(np.uint8)


def make_point_cloud(points: np.ndarray, colors: np.ndarray | None = None):
    """Build a DDS PointCloud from NumPy arrays."""

    from dds.geometry import PointCloud

    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    return PointCloud(points=pts, colors=colors_to_uint8(colors, pts.shape[0]))


def make_triangle_mesh(
    vertices: np.ndarray,
    faces: np.ndarray,
    *,
    vertex_colors: np.ndarray | None = None,
    face_colors: np.ndarray | None = None,
):
    """Build a DDS TriangleMesh from NumPy arrays."""

    from dds.geometry import TriangleMesh

    verts = np.asarray(vertices, dtype=np.float64)
    tris = np.asarray(faces, dtype=np.int64)
    if verts.ndim != 2 or verts.shape[1] != 3:
        raise ValueError("vertices must have shape (N, 3)")
    if tris.ndim != 2 or tris.shape[1] != 3:
        raise ValueError("faces must have shape (M, 3)")

    return TriangleMesh(
        vertices=verts,
        faces=tris,
        vertex_colors=colors_to_uint8(vertex_colors, verts.shape[0]),
        face_colors=colors_to_uint8(face_colors, tris.shape[0]),
    )


def attach_viewer(plotter: Any):
    """Attach DDS retained visual management to an existing PyVista plotter."""

    from dds.viz.viewer import Viewer

    return Viewer._attach(plotter)


def make_viewer(
    *,
    title: str = "BEHAV3D DDS Viewer",
    off_screen: bool = False,
):
    """Create a standalone DDS Viewer."""

    from dds.viz.viewer import Viewer

    return Viewer(title=title, off_screen=bool(off_screen))


def add_colored_point_cloud(
    viewer,
    points: np.ndarray,
    colors: np.ndarray | None = None,
    *,
    name: str = "point_cloud",
    point_size: float = 3.0,
    render_as_spheres: bool = False,
    opacity: float = 1.0,
    color: str | tuple[float, float, float] | None = None,
):
    """Add or update a colored point cloud in a DDS Viewer."""

    from dds.viz import PointCloudStyle

    cloud = make_point_cloud(points, colors)
    style = PointCloudStyle(
        color=color,
        size=float(point_size),
        render_as_spheres=bool(render_as_spheres),
        opacity=float(opacity),
    )

    try:
        handle = viewer.get(name)
    except KeyError:
        return viewer.add_point_cloud(cloud, style=style, name=name)
    return handle.update(cloud, style=style)


def add_triangle_mesh(
    viewer,
    vertices: np.ndarray,
    faces: np.ndarray,
    *,
    name: str = "mesh",
    color: str | tuple[float, float, float] | None = "#93aec7",
    opacity: float = 1.0,
    show_edges: bool = False,
    smooth_shading: bool = True,
    vertex_colors: np.ndarray | None = None,
    face_colors: np.ndarray | None = None,
):
    """Add or update a triangle mesh in a DDS Viewer."""

    from dds.viz import MeshStyle

    mesh = make_triangle_mesh(
        vertices,
        faces,
        vertex_colors=vertex_colors,
        face_colors=face_colors,
    )
    style = MeshStyle(
        color=color,
        opacity=float(opacity),
        show_edges=bool(show_edges),
        smooth_shading=bool(smooth_shading),
    )

    try:
        handle = viewer.get(name)
    except KeyError:
        return viewer.add_mesh(mesh, style=style, name=name)
    return handle.update(mesh, style=style)


def _overlay_actors(viewer) -> dict[str, Any]:
    actors = getattr(viewer, "_behav3d_overlay_actors", None)
    if actors is None:
        actors = {}
        setattr(viewer, "_behav3d_overlay_actors", actors)
    return actors


def remove_overlay(viewer, name: str) -> None:
    """Remove one non-retained PyVista overlay actor if present."""

    actors = _overlay_actors(viewer)
    actor = actors.pop(name, None)
    if actor is not None:
        viewer.plotter.remove_actor(actor, render=False)


def add_wire_mesh(
    viewer,
    vertices: np.ndarray,
    faces: np.ndarray,
    *,
    name: str = "wire_mesh",
    color: str | tuple[float, float, float] = "#808080",
    opacity: float = 1.0,
    line_width: float = 1.0,
):
    """Add or replace a wireframe triangle mesh overlay."""

    import pyvista as pv

    verts = np.asarray(vertices, dtype=np.float64)
    tris = np.asarray(faces, dtype=np.int64)
    if verts.ndim != 2 or verts.shape[1] != 3:
        raise ValueError("vertices must have shape (N, 3)")
    if tris.ndim != 2 or tris.shape[1] != 3:
        raise ValueError("faces must have shape (M, 3)")

    remove_overlay(viewer, name)
    if verts.shape[0] == 0 or tris.shape[0] == 0:
        return None

    pv_faces = np.hstack(
        [
            np.full((tris.shape[0], 1), 3, dtype=np.int64),
            tris,
        ]
    ).ravel()
    dataset = pv.PolyData(verts, pv_faces)
    actor = viewer.plotter.add_mesh(
        dataset,
        name=name,
        color=color,
        opacity=float(opacity),
        line_width=float(line_width),
        style="wireframe",
        render=False,
        reset_camera=False,
    )
    _overlay_actors(viewer)[name] = actor
    viewer.plotter.render()
    return actor


def add_line_segments(
    viewer,
    points: np.ndarray,
    lines: np.ndarray,
    *,
    name: str = "line_segments",
    color: str | tuple[float, float, float] = "#00ffff",
    opacity: float = 1.0,
    line_width: float = 3.0,
    render_as_tubes: bool = False,
):
    """Add or replace independent line segments in a DDS/PyVista viewer."""

    import pyvista as pv

    pts = np.asarray(points, dtype=np.float64)
    segs = np.asarray(lines, dtype=np.int64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if segs.ndim != 2 or segs.shape[1] != 2:
        raise ValueError("lines must have shape (M, 2)")
    if segs.size and (segs.min() < 0 or segs.max() >= pts.shape[0]):
        raise ValueError("lines contain invalid point indices")

    remove_overlay(viewer, name)
    if pts.shape[0] == 0 or segs.shape[0] == 0:
        return None

    pv_lines = np.hstack(
        [
            np.full((segs.shape[0], 1), 2, dtype=np.int64),
            segs,
        ]
    ).ravel()
    dataset = pv.PolyData(pts)
    dataset.lines = pv_lines
    actor = viewer.plotter.add_mesh(
        dataset,
        name=name,
        color=color,
        opacity=float(opacity),
        line_width=float(line_width),
        render_lines_as_tubes=bool(render_as_tubes),
        render=False,
        reset_camera=False,
    )
    _overlay_actors(viewer)[name] = actor
    viewer.plotter.render()
    return actor


def add_vector_arrows(
    viewer,
    origins: np.ndarray,
    vectors: np.ndarray,
    *,
    name: str = "vectors",
    color: str | tuple[float, float, float] = "#2ecc71",
    opacity: float = 1.0,
):
    """Add or replace a batch of vector arrows on a DDS/PyVista viewer."""

    p0 = np.asarray(origins, dtype=np.float64)
    vec = np.asarray(vectors, dtype=np.float64)
    if p0.shape != vec.shape or p0.ndim != 2 or p0.shape[1] != 3:
        raise ValueError("origins and vectors must both have shape (N, 3)")

    remove_overlay(viewer, name)
    if p0.shape[0] == 0:
        return None

    actor = viewer.plotter.add_arrows(
        p0,
        vec,
        mag=1.0,
        name=name,
        color=color,
        opacity=float(opacity),
        render=False,
        reset_camera=False,
    )
    _overlay_actors(viewer)[name] = actor
    viewer.plotter.render()
    return actor


def show_viewer(viewer, *, show_axes: bool = True):
    """Show a standalone DDS Viewer and run its Qt event loop."""

    if show_axes:
        viewer.plotter.add_axes()
    viewer.plotter.reset_camera()
    viewer.window.show()
    viewer.app.exec()
    return viewer


def show_colored_point_cloud(
    points: np.ndarray,
    colors: np.ndarray | None = None,
    *,
    title: str = "BEHAV3D DDS Point Cloud",
    name: str = "point_cloud",
    point_size: float = 3.0,
    render_as_spheres: bool = False,
    opacity: float = 1.0,
    show_axes: bool = True,
):
    """Open a standalone DDS/PyVista viewer for a colored point cloud."""

    viewer = make_viewer(title=title)
    add_colored_point_cloud(
        viewer,
        points,
        colors,
        name=name,
        point_size=point_size,
        render_as_spheres=render_as_spheres,
        opacity=opacity,
    )
    return show_viewer(viewer, show_axes=show_axes)
