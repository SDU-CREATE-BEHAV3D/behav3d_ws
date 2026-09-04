"""Reusable Open3D raycasting scene for one material-geometry revision."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int32]


def _points(values: npt.ArrayLike, name: str) -> FloatArray:
    result = np.asarray(values, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3 or result.shape[0] == 0:
        raise ValueError(f"{name} must have non-empty shape (N, 3)")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _faces(values: npt.ArrayLike, vertex_count: int) -> IntArray:
    raw = np.asarray(values)
    if raw.ndim != 2 or raw.shape[1] != 3 or raw.shape[0] == 0:
        raise ValueError("mesh_faces must have non-empty shape (M, 3)")
    if not np.issubdtype(raw.dtype, np.integer) and not np.all(
        np.equal(raw, np.floor(raw))
    ):
        raise ValueError("mesh_faces must contain integer indices")
    result = raw.astype(np.int32, copy=False)
    if np.any(result < 0) or np.any(result >= vertex_count):
        raise ValueError("mesh_faces contains an invalid vertex index")
    return result


class MeshRaycaster:
    """Build Open3D acceleration data once and reuse it for many ray queries."""

    def __init__(
        self,
        mesh_vertices: npt.ArrayLike,
        mesh_faces: npt.ArrayLike,
    ) -> None:
        import open3d as o3d

        vertices = _points(mesh_vertices, "mesh_vertices")
        faces = _faces(mesh_faces, vertices.shape[0])
        tensor_mesh = o3d.t.geometry.TriangleMesh()
        tensor_mesh.vertex["positions"] = o3d.core.Tensor(
            vertices.astype(np.float32)
        )
        tensor_mesh.triangle["indices"] = o3d.core.Tensor(faces)
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(tensor_mesh)
        self._scene = scene

    def cast_distances(
        self,
        origins: npt.ArrayLike,
        directions: npt.ArrayLike,
    ) -> FloatArray:
        """Return first-hit distances for aligned batches of world-space rays."""

        import open3d as o3d

        ray_origins = _points(origins, "origins")
        ray_directions = _points(directions, "directions")
        if ray_origins.shape != ray_directions.shape:
            raise ValueError("origins and directions must have identical shape")
        norms = np.linalg.norm(ray_directions, axis=1)
        if np.any(norms <= 1e-12):
            raise ValueError("ray directions must be non-zero")
        unit_directions = ray_directions / norms[:, None]
        rays = np.hstack((ray_origins, unit_directions)).astype(np.float32)
        return np.asarray(
            self._scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy(),
            dtype=np.float64,
        )

    def cast_distance(
        self,
        origin: npt.ArrayLike,
        direction: npt.ArrayLike,
    ) -> float:
        """Return the first-hit distance for one world-space ray."""

        ray_origin = np.asarray(origin, dtype=np.float64).reshape(1, -1)
        ray_direction = np.asarray(direction, dtype=np.float64).reshape(1, -1)
        return float(self.cast_distances(ray_origin, ray_direction)[0])
