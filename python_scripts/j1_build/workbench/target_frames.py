"""Display-only right-handed XYZ glyphs from target origins and deposition Z."""
import numpy as np
import pyvista as pv

AXIS_COLORS = ("#e34b43", "#36a45b", "#367bf5")


def target_axes(z_dirs):
    z = np.array(z_dirs, dtype=float, copy=True).reshape(-1, 3)
    if not np.all(np.isfinite(z)):
        raise ValueError("Target frame directions must be finite")
    norm = np.linalg.norm(z, axis=1)
    z[norm < 1e-12] = (0, 0, 1)
    z /= np.linalg.norm(z, axis=1)[:, None]
    reference = np.tile((1.0, 0.0, 0.0), (len(z), 1))
    reference[np.abs(z[:, 0]) > 0.95] = (0, 1, 0)
    x = reference - np.sum(reference * z, axis=1)[:, None] * z
    x /= np.linalg.norm(x, axis=1)[:, None]
    y = np.cross(z, x)
    return np.stack((x, y, z), axis=1)


def select_target_frames(points, z_dirs, show_starts=False, show_endpoints=True):
    """Select display frames from the interleaved start/end target stream."""
    points = np.asarray(points).reshape(-1, 3)
    z_dirs = np.asarray(z_dirs).reshape(-1, 3)
    if len(points) != len(z_dirs) or len(points) % 2:
        raise ValueError("Expected aligned start/end pairs for target frames")
    if show_starts and show_endpoints:
        return points, z_dirs
    if show_starts:
        return points[0::2], z_dirs[0::2]
    if show_endpoints:
        return points[1::2], z_dirs[1::2]
    return points[:0], z_dirs[:0]


def frame_glyphs(points, z_dirs, length_mm, thickness=1.4):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    axes = target_axes(z_dirs)
    if len(points) != len(axes) or not np.all(np.isfinite(points)):
        raise ValueError("Target frame origins and directions must be finite and aligned")
    if not np.isfinite(length_mm) or length_mm <= 0:
        raise ValueError("Target axis length must be positive")
    if not np.isfinite(thickness) or thickness <= 0:
        raise ValueError("Target axis thickness must be positive")
    if not len(points):
        return [pv.PolyData() for _ in range(3)]
    tube = pv.Cylinder(center=(0.5, 0, 0), direction=(1, 0, 0), height=1,
                       radius=0.014 * thickness, resolution=6, capping=True)
    meshes = []
    for axis in range(3):
        seeds = pv.PolyData(points)
        seeds["direction"] = axes[:, axis, :]
        meshes.append(seeds.glyph(orient="direction", scale=False,
                                 factor=length_mm * 0.001, geom=tube))
    return meshes
