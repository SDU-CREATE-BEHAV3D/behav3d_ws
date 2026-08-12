#!/usr/bin/env python3
"""Minimal extruder collision probe for scalar `segments.yaml` contracts.

This is intentionally small: sample each start/end target pose, transform the
real extruder collision mesh into that TCP pose, and discard the pose if any
extruder vertex is within a threshold distance of the scan mesh.
"""

from __future__ import annotations

import argparse
import re
import time
from dataclasses import dataclass
from pathlib import Path

import collada
import numpy as np
from scipy.spatial.transform import Rotation as R


DEFAULT_EXTRUDER_MESH = (
    Path(__file__).resolve().parents[3]
    / "custom_workcell"
    / "ur20_workcell"
    / "meshes"
    / "ram_extruder_v2_simplified.dae"
)
TOOL0_TCP_XYZ = (-0.00551, 0.47304, 0.0822)
TOOL0_TCP_RPY = (1.5641113421, 0.0, 0.0)
START_ADJUST_FRACTIONS = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)
PLANE_PATTERN = re.compile(r"([A-Za-z]+)\(([^)]*)\)")


@dataclass(frozen=True)
class Segment:
    index: int
    start: np.ndarray
    end: np.ndarray
    start_z: np.ndarray
    end_z: np.ndarray


@dataclass(frozen=True)
class PoseCheck:
    segment_index: int
    sample_index: int
    alpha: float
    position: np.ndarray
    world_tool0: np.ndarray
    min_distance_mm: float
    hit_vertices: int
    collides: bool


def normalize(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return vector / norm


def parse_vec(text: str) -> np.ndarray:
    values = [float(part.strip()) for part in text.split(",")]
    if len(values) != 3:
        raise ValueError(f"Expected 3 vector components, got: {text!r}")
    return np.asarray(values, dtype=np.float64)


def parse_plane(plane: str, *, position_scale: float) -> tuple[np.ndarray, np.ndarray]:
    components = {key.upper(): parse_vec(value) for key, value in PLANE_PATTERN.findall(plane)}
    if "O" not in components:
        raise ValueError(f"Plane string is missing O(...): {plane}")
    point = components["O"] / float(position_scale)
    z_axis = normalize(components.get("Z", np.array([0.0, 0.0, 1.0], dtype=np.float64)))
    return point, z_axis


def load_segments(path: Path, *, position_scale: float = 1000.0) -> list[Segment]:
    import yaml  # type: ignore[import-untyped]

    with path.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict) or not isinstance(data.get("segments"), list):
        raise ValueError(f"YAML must contain a top-level segments list: {path}")

    segments: list[Segment] = []
    for ordinal, item in enumerate(data["segments"]):
        start = item["start"]["plane"]
        end = item["end"]["plane"]
        start_point, start_z = parse_plane(start, position_scale=position_scale)
        end_point, end_z = parse_plane(end, position_scale=position_scale)
        segments.append(
            Segment(
                index=int(item.get("index", ordinal)),
                start=start_point,
                end=end_point,
                start_z=start_z,
                end_z=end_z,
            )
        )
    return sorted(segments, key=lambda segment: segment.index)


def load_collada_mesh(path: Path) -> tuple[np.ndarray, np.ndarray]:
    document = collada.Collada(str(path))
    vertices: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    offset = 0
    for geometry in document.geometries:
        for primitive in geometry.primitives:
            if not hasattr(primitive, "vertex") or not hasattr(primitive, "vertex_index"):
                continue
            v = np.asarray(primitive.vertex, dtype=np.float64)
            f = np.asarray(primitive.vertex_index, dtype=np.int64).reshape((-1, 3))
            vertices.append(v)
            faces.append(f + offset)
            offset += int(v.shape[0])
    if not vertices:
        raise ValueError(f"No triangle geometry found in COLLADA mesh: {path}")
    return np.vstack(vertices), np.vstack(faces)


def load_open3d_mesh(path: Path) -> tuple[np.ndarray, np.ndarray]:
    import open3d as o3d

    mesh = o3d.io.read_triangle_mesh(str(path))
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.triangles, dtype=np.int32)
    if vertices.size == 0 or faces.size == 0:
        raise ValueError(f"Could not load triangle mesh: {path}")
    return vertices, faces


def make_scan_scene(vertices: np.ndarray, faces: np.ndarray):
    import open3d as o3d

    mesh = o3d.t.geometry.TriangleMesh()
    mesh.vertex["positions"] = o3d.core.Tensor(vertices.astype(np.float32))
    mesh.triangle["indices"] = o3d.core.Tensor(faces.astype(np.int32))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh)
    return scene


def any_orthonormal(vector: np.ndarray) -> np.ndarray:
    vector = normalize(vector)
    basis = np.zeros(3, dtype=np.float64)
    basis[int(np.argmin(np.abs(vector)))] = 1.0
    return normalize(basis - float(np.dot(basis, vector)) * vector)


def rotation_from_z(z_axis: np.ndarray, *, x_guess: np.ndarray) -> np.ndarray:
    z = normalize(z_axis)
    # Match the fixed-reference free roll used by the YAML target executor.
    x_reference = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(z[0])) > 0.95:
        x_reference = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    x = normalize(x_reference - float(np.dot(x_reference, z)) * z)
    y = normalize(np.cross(z, x))
    return np.column_stack((x, y, z))


def transform_from_xyz_rpy(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = R.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return transform


def target_to_world_tool0(
    position: np.ndarray,
    z_axis: np.ndarray,
    x_guess: np.ndarray,
    tcp_tool0: np.ndarray,
) -> np.ndarray:
    world_tcp = np.eye(4, dtype=np.float64)
    world_tcp[:3, :3] = rotation_from_z(z_axis, x_guess=x_guess)
    world_tcp[:3, 3] = position
    return world_tcp @ tcp_tool0


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def sample_segment(segment: Segment, sample_count: int) -> list[tuple[int, float, np.ndarray, np.ndarray]]:
    alphas = [1.0] if sample_count <= 1 else np.linspace(0.0, 1.0, sample_count)
    samples = []
    for sample_index, alpha in enumerate(alphas):
        a = float(alpha)
        point = (1.0 - a) * segment.start + a * segment.end
        z_axis = normalize((1.0 - a) * segment.start_z + a * segment.end_z)
        samples.append((sample_index, a, point, z_axis))
    return samples


def interpolate_start(segment: Segment, fraction: float) -> tuple[np.ndarray, np.ndarray]:
    fraction = float(fraction)
    point = (1.0 - fraction) * segment.start + fraction * segment.end
    z_axis = normalize((1.0 - fraction) * segment.start_z + fraction * segment.end_z)
    return point, z_axis


def check_pose(
    scan_scene,
    extruder_vertices_tool0: np.ndarray,
    world_tool0: np.ndarray,
    threshold_mm: float,
) -> tuple[bool, float, int]:
    import open3d as o3d

    vertices_world = transform_points(extruder_vertices_tool0, world_tool0)
    distances_m = scan_scene.compute_distance(o3d.core.Tensor(vertices_world.astype(np.float32))).numpy()
    threshold_m = float(threshold_mm) * 1e-3
    hit_vertices = int(np.count_nonzero(distances_m <= threshold_m))
    return bool(hit_vertices), float(np.min(distances_m) * 1e3), hit_vertices


def adjust_colliding_starts(
    segments: list[Segment],
    *,
    scan_scene,
    extruder_vertices: np.ndarray,
    tcp_tool0: np.ndarray,
    threshold_mm: float,
) -> list[Segment]:
    adjusted: list[Segment] = []
    shifted = 0
    discarded = 0

    for segment in segments:
        tangent = segment.end - segment.start
        if np.linalg.norm(tangent) < 1e-9:
            tangent = any_orthonormal(segment.end_z)

        accepted: Segment | None = None
        last_min_distance_mm = float("nan")
        last_hit_vertices = 0
        for fraction in START_ADJUST_FRACTIONS:
            start, start_z = interpolate_start(segment, fraction)
            world_tool0 = target_to_world_tool0(start, start_z, tangent, tcp_tool0)
            collides, min_distance_mm, hit_vertices = check_pose(
                scan_scene,
                extruder_vertices,
                world_tool0,
                threshold_mm,
            )
            last_min_distance_mm = min_distance_mm
            last_hit_vertices = hit_vertices
            if not collides:
                accepted = Segment(
                    index=segment.index,
                    start=start,
                    end=segment.end,
                    start_z=start_z,
                    end_z=segment.end_z,
                )
                if fraction > 0.0:
                    shifted += 1
                    print(
                        f"[adjust] segment={segment.index} start_fraction={fraction:.1f} "
                        f"min_distance_mm={min_distance_mm:.3f}",
                        flush=True,
                    )
                break

        if accepted is None:
            discarded += 1
            print(
                f"[adjust] segment={segment.index} discarded all_start_fractions_collide "
                f"last_min_distance_mm={last_min_distance_mm:.3f} "
                f"last_hit_vertices={last_hit_vertices}",
                flush=True,
            )
        else:
            adjusted.append(accepted)

    print(
        f"[adjust] input={len(segments)} kept={len(adjusted)} shifted={shifted} discarded={discarded}",
        flush=True,
    )
    return adjusted


def plane_string(point_m: np.ndarray, z_axis: np.ndarray, *, position_scale: float) -> str:
    point = np.asarray(point_m, dtype=np.float64) * float(position_scale)
    z = normalize(z_axis)
    return (
        f"O({point[0]:.6f},{point[1]:.6f},{point[2]:.6f}) "
        f"Z({z[0]:.6f},{z[1]:.6f},{z[2]:.6f})"
    )


def write_segments(path: Path, segments: list[Segment], *, position_scale: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["segments:"]
    for segment in segments:
        lines.extend(
            [
                f"  - index: {segment.index}",
                "    start:",
                f'      plane: "{plane_string(segment.start, segment.start_z, position_scale=position_scale)}"',
                "    end:",
                f'      plane: "{plane_string(segment.end, segment.end_z, position_scale=position_scale)}"',
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> tuple[list[PoseCheck], np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    segments = load_segments(Path(args.segments), position_scale=float(args.position_scale))
    if args.max_segments is not None:
        segments = segments[: int(args.max_segments)]

    scan_vertices, scan_faces = load_open3d_mesh(Path(args.scan_mesh))
    extruder_vertices, extruder_faces = load_collada_mesh(Path(args.extruder_mesh))
    scan_scene = make_scan_scene(scan_vertices, scan_faces)
    tcp_tool0 = np.linalg.inv(transform_from_xyz_rpy(TOOL0_TCP_XYZ, TOOL0_TCP_RPY))
    if args.adjust_colliding_starts:
        segments = adjust_colliding_starts(
            segments,
            scan_scene=scan_scene,
            extruder_vertices=extruder_vertices,
            tcp_tool0=tcp_tool0,
            threshold_mm=float(args.threshold_mm),
        )
        if args.out_segments:
            write_segments(Path(args.out_segments), segments, position_scale=float(args.position_scale))
            print(f"[adjust] wrote adjusted segments: {args.out_segments}", flush=True)

    print(
        "[probe] "
        f"segments={len(segments)} samples_per_segment={args.samples_per_segment} "
        f"scan_vertices={len(scan_vertices)} extruder_vertices={len(extruder_vertices)} "
        f"threshold_mm={float(args.threshold_mm):.3f}",
        flush=True,
    )

    checks: list[PoseCheck] = []
    t0 = time.perf_counter()
    for segment in segments:
        tangent = segment.end - segment.start
        if np.linalg.norm(tangent) < 1e-9:
            tangent = any_orthonormal(segment.end_z)

        for sample_index, alpha, point, z_axis in sample_segment(segment, int(args.samples_per_segment)):
            world_tool0 = target_to_world_tool0(point, z_axis, tangent, tcp_tool0)
            collides, min_distance_mm, hit_vertices = check_pose(
                scan_scene,
                extruder_vertices,
                world_tool0,
                float(args.threshold_mm),
            )
            check = PoseCheck(
                segment_index=segment.index,
                sample_index=sample_index,
                alpha=alpha,
                position=point,
                world_tool0=world_tool0,
                min_distance_mm=min_distance_mm,
                hit_vertices=hit_vertices,
                collides=collides,
            )
            checks.append(check)
            if collides or args.print_all:
                print(
                    f"[pose] segment={check.segment_index} sample={check.sample_index} "
                    f"alpha={check.alpha:.2f} collides={check.collides} "
                    f"min_distance_mm={check.min_distance_mm:.3f} "
                    f"hit_vertices={check.hit_vertices}",
                    flush=True,
                )

    elapsed_ms = 1e3 * (time.perf_counter() - t0)
    collisions = sum(1 for check in checks if check.collides)
    print(f"[summary] poses={len(checks)} collisions={collisions} elapsed_ms={elapsed_ms:.1f}", flush=True)
    return checks, scan_vertices, scan_faces, extruder_vertices, extruder_faces


def pyvista_faces(faces: np.ndarray) -> np.ndarray:
    return np.column_stack([np.full(faces.shape[0], 3, dtype=np.int64), faces]).reshape(-1)


def preview(
    checks: list[PoseCheck],
    scan_vertices: np.ndarray,
    scan_faces: np.ndarray,
    extruder_vertices: np.ndarray,
    extruder_faces: np.ndarray,
    args: argparse.Namespace,
) -> None:
    import pyvista as pv

    plotter = pv.Plotter()
    plotter.add_mesh(
        pv.PolyData(scan_vertices, pyvista_faces(scan_faces)),
        color="lightgray",
        opacity=float(args.scan_alpha),
        show_edges=True,
    )

    shown = 0
    for check in checks:
        if args.preview_collisions_only and not check.collides:
            continue
        if shown >= int(args.preview_max_poses):
            break
        vertices_world = transform_points(extruder_vertices, check.world_tool0)
        color = "red" if check.collides else "green"
        plotter.add_mesh(
            pv.PolyData(vertices_world, pyvista_faces(extruder_faces)),
            color=color,
            opacity=float(args.extruder_alpha),
        )
        plotter.add_points(
            check.position.reshape(1, 3),
            color="yellow" if check.collides else "white",
            point_size=8,
            render_points_as_spheres=True,
        )
        shown += 1

    plotter.add_axes()
    plotter.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal scalar segment extruder collision probe.")
    parser.add_argument("--segments", required=True, help="Scalar step segments.yaml file.")
    parser.add_argument("--scan-mesh", required=True, help="Scan mesh to test against.")
    parser.add_argument("--extruder-mesh", default=str(DEFAULT_EXTRUDER_MESH), help="Extruder DAE collision mesh.")
    parser.add_argument("--position-scale", type=float, default=1000.0, help="YAML O(...) scale; default mm.")
    parser.add_argument("--samples-per-segment", type=int, default=2, help="2 means start and end.")
    parser.add_argument("--threshold-mm", type=float, default=1.0)
    parser.add_argument(
        "--adjust-colliding-starts",
        action="store_true",
        help="Try fixed start fractions toward end; discard segment if all starts collide.",
    )
    parser.add_argument("--out-segments", default=None, help="Optional adjusted segment YAML output.")
    parser.add_argument("--max-segments", type=int, default=None)
    parser.add_argument("--print-all", action="store_true")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--preview-collisions-only", action="store_true")
    parser.add_argument("--preview-max-poses", type=int, default=20)
    parser.add_argument("--extruder-alpha", type=float, default=0.25)
    parser.add_argument("--scan-alpha", type=float, default=0.35)
    args = parser.parse_args()

    checks, scan_vertices, scan_faces, extruder_vertices, extruder_faces = run(args)
    if args.preview:
        preview(checks, scan_vertices, scan_faces, extruder_vertices, extruder_faces, args)


if __name__ == "__main__":
    main()
