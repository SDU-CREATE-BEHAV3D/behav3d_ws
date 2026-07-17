#!/usr/bin/env python3
"""Simulate scalar-field print loop with DDS bead accumulation.

Stage flow per loop iteration:
1) Position field over current scan (only first step)
2) Keep that field pose fixed and recompute phi vs updated scan
3) Generate print candidates (`geodesic`, `z_lift`, `gradient_lift`, or `gradient_walk`)
4) Enforce bead separation only inside current step
5) Add selected beads to a DDS Simulator
6) Extract a DDS implicit-surface proxy for the next scan intersection

Sample command:
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_loop_simulator_dds_v2.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl \
  --seed-level 1 --t-coef 2000 \
  --field-subdivide-iter 1 --field-scale 0.001 \
  --candidate-mode gradient_walk \
  --beads-per-step 7 --bead-separation-mm 16 \
  --bead-height-mm 12 --bead-shape cylinder \
  --positioning-attempts 3 \
  --search-step-x 0.01 --search-step-y 0.01 \
  --offset-distance-mm 12 --offset-geodesic-delta-mm 0.6 \
  --axis-size -1

  python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_loop_simulator_dds_v2.py \
  --field-mesh /home/lab/behav3d_ws/mesh/curved_wall_5mm.obj \
  --scan-mesh /home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl \
  --output-dir /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/output/loop_sim \
  --seed-level 1 \
  --t-coef 2000 \
  --field-subdivide-iter 1 \
  --field-scale 0.001 \
  --clearance 0.0 \
  --candidate-mode gradient_walk \
  --offset-distance-mm 12 \
  --offset-geodesic-delta-mm 0.6 \
  --beads-per-step 7 \
  --bead-separation-mm 16 \
  --bead-height-mm 12 \
  --bead-shape cylinder \
  --walk-distance-mm 12 \
  --walk-step-mm 1.0 \
  --walk-max-steps 32 \
  --walk-tangent-sign 1.0 \
  --walk-start-fraction 0.25 \
  --cone-max-tilt-deg 45 \
  --positioning-attempts 3 \
  --position-target-x 0.0 --position-target-y -0.75 \
  --search-step-x 0.01 \
  --search-step-y 0.01 \
  --base-z-offset 0.000001 \
  --axis-size -1 \
  --dds-voxel-size-mm 2.0 \
  --dds-domain-source field \
  --dds-threshold 0.5 \
  --dds-padding-mm 24 \
  --dds-surface-step-size 1 \
  --dds-view-mode surface
Controls:
- DDS window: `N` = apply current step and go next, `Q`/`Esc` = stop loop
- `--no-vis` mode: type `n` to apply+next, `q` to stop
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d

from lib_scalar.compute_heat_field import (
    choose_seed_vertex,
    choose_seed_vertices_below_level,
    compute_heat_field,
)
from lib_scalar.compute_phi_mask import evaluate_fixed_pose, make_scan_scene
from lib_scalar.geometry import load_triangle_mesh_arrays, load_triangle_mesh_legacy
from lib_scalar.loop_simulation import (
    compute_offset_contour_stage,
    generate_step_candidates,
    position_field_with_attempts,
)
from lib_scalar.print_targets import (
    OrientedLineTargets,
    build_oriented_line_targets,
    write_line_targets_yaml,
)
from lib_scalar.target_rules import (
    TARGET_NORMAL_FLIP_BEAD_HEIGHT_FRACTION,
    TARGET_NORMAL_FLIP_DOT_THRESHOLD,
    apply_secondary_target_rules,
)
from lib_scalar.viz import (
    compute_scene_bounds,
    make_point_cloud,
    yellow_to_red_colors,
)


DEFAULT_FIELD_MESH = Path("/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj")
DEFAULT_SCAN_MESH = Path("/home/lab/behav3d_ws/mesh/tsdf_surface_mesh2.stl")
DEFAULT_FIELD_STATE = Path("/home/lab/behav3d_ws/mesh/fields/field_state_init.npz")
DEFAULT_ROBOT_SCAN_MESH = Path(
    "/home/lab/behav3d_ws/captures/260317_171335/field_loop/cycle_0000/scan/reconstruct/tsdf_surface_mesh.stl"
)
OUTPUT_DIR = Path(__file__).resolve().parent / "output"
ISO_LEVEL = 0.0
BASE_Z_OFFSET = 1e-6
SEARCH_MAX_CANDIDATES = 30000
TARGET_POSITION_SCALE = 1000.0

SCALAR_OVERLAY_GROUPS: tuple[tuple[str, str, tuple[str, ...]], ...] = (
    ("field", "Scalar Field", ("field_heat_masked",)),
    ("seeds", "Heat Seeds", ("heat_seed_points",)),
    ("scan", "Scan Mesh", ("scan_wire",)),
    ("phi", "Phi Contour", ("phi_contour",)),
    ("offset", "Geodesic Offset", ("offset_contour",)),
    ("targets", "Print Targets", ("selected_points",)),
    ("sources", "Source Points", ("source_points",)),
    ("walk", "Walk Segments", ("source_to_candidate", "segment_start_points", "print_segments")),
    ("normals", "Target Normals", ("target_orientations",)),
    ("axes", "Debug Axes", ("axis_x", "axis_y", "axis_z")),
)


def default_scalar_overlay_visibility() -> dict[str, bool]:
    """Default visibility for scalar overlays in the DDS workbench."""
    return {key: True for key, _, _ in SCALAR_OVERLAY_GROUPS}


def subdivide_field_mesh_loop(
    vertices: np.ndarray,
    faces: np.ndarray,
    iterations: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Optional Loop subdivision for higher contour/geodesic resolution."""
    it = int(iterations)
    if it <= 0:
        return vertices, faces
    if it > 4:
        raise ValueError(f"field_subdivide_iter too large ({it}); use 0..4.")

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
    mesh_sub = mesh.subdivide_loop(number_of_iterations=it)
    v_sub = np.asarray(mesh_sub.vertices, dtype=np.float64)
    f_sub = np.asarray(mesh_sub.triangles, dtype=np.int32)
    return v_sub, f_sub


def resolve_heat_seed_indices(
    vertices: np.ndarray,
    *,
    seed: int | None,
    seed_level: float | None,
) -> np.ndarray:
    """Return heat-method seed vertex indices using compute_heat_field rules."""
    if seed_level is not None:
        return choose_seed_vertices_below_level(vertices, float(seed_level))

    seed_index = choose_seed_vertex(vertices) if seed is None else int(seed)
    if seed_index < 0 or seed_index >= int(vertices.shape[0]):
        raise ValueError(f"Seed index out of range: {seed_index} (num vertices={vertices.shape[0]})")
    return np.asarray([seed_index], dtype=np.int64)


@dataclass(frozen=True)
class LoadedFieldState:
    """Field state produced by the ROS field init pipeline."""

    field_vertices_scaled: np.ndarray
    field_faces: np.ndarray
    heat_norm: np.ndarray
    offset_xyz: tuple[float, float, float]
    seed_points_scaled: np.ndarray


def load_field_state_npz(field_state_path: Path) -> LoadedFieldState:
    """Load a pre-initialized scalar field state for direct loop simulation."""
    state_path = Path(field_state_path)
    if not state_path.is_file():
        raise FileNotFoundError(f"Field state not found: {state_path}")
    if state_path.suffix.lower() != ".npz":
        raise ValueError(
            f"Unsupported field state format: {state_path}. "
            "Expected a .npz state with field_vertices_scaled, field_faces, heat_norm, and offset_xyz."
        )

    state = np.load(str(state_path), allow_pickle=False)
    missing = [key for key in ("field_faces", "heat_norm", "offset_xyz") if key not in state]
    if missing:
        raise KeyError(f"Missing field state keys in {state_path}: {', '.join(missing)}")

    if "field_vertices_scaled" in state:
        field_vertices_scaled = np.asarray(state["field_vertices_scaled"], dtype=np.float64)
    elif "field_vertices" in state:
        field_vertices = np.asarray(state["field_vertices"], dtype=np.float64)
        field_scale = 1.0
        if "field_scale" in state:
            scale_arr = np.asarray(state["field_scale"], dtype=np.float64).reshape(-1)
            if scale_arr.size > 0:
                field_scale = float(scale_arr[0])
        field_vertices_scaled = field_vertices * field_scale
    else:
        raise KeyError(
            f"Missing 'field_vertices_scaled' or fallback 'field_vertices' in field state: {state_path}"
        )

    field_faces = np.asarray(state["field_faces"], dtype=np.int32)
    heat_norm = np.asarray(state["heat_norm"], dtype=np.float64).reshape(-1)
    offset_arr = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(-1)
    if field_vertices_scaled.ndim != 2 or field_vertices_scaled.shape[1] != 3:
        raise ValueError(f"field_vertices_scaled has invalid shape: {field_vertices_scaled.shape}")
    if field_faces.ndim != 2 or field_faces.shape[1] != 3:
        raise ValueError(f"field_faces has invalid shape: {field_faces.shape}")
    if heat_norm.shape[0] != field_vertices_scaled.shape[0]:
        raise ValueError(
            "heat_norm length does not match field vertices: "
            f"{heat_norm.shape[0]} vs {field_vertices_scaled.shape[0]}"
        )
    if offset_arr.size < 3:
        raise ValueError(f"offset_xyz has invalid shape: {offset_arr.shape}")

    seed_points_scaled = np.zeros((0, 3), dtype=np.float64)
    if "seed_indices" in state:
        seed_indices = np.asarray(state["seed_indices"], dtype=np.int64).reshape(-1)
        valid = (seed_indices >= 0) & (seed_indices < field_vertices_scaled.shape[0])
        seed_points_scaled = field_vertices_scaled[seed_indices[valid]]
    elif "seed_level" in state and "field_vertices" in state:
        seed_level_arr = np.asarray(state["seed_level"], dtype=np.float64).reshape(-1)
        if seed_level_arr.size > 0 and np.isfinite(seed_level_arr[0]):
            seed_indices = resolve_heat_seed_indices(
                np.asarray(state["field_vertices"], dtype=np.float64),
                seed=None,
                seed_level=float(seed_level_arr[0]),
            )
            seed_points_scaled = field_vertices_scaled[seed_indices]

    return LoadedFieldState(
        field_vertices_scaled=field_vertices_scaled,
        field_faces=field_faces,
        heat_norm=heat_norm,
        offset_xyz=(float(offset_arr[0]), float(offset_arr[1]), float(offset_arr[2])),
        seed_points_scaled=seed_points_scaled,
    )


def wait_next_terminal(step_index: int) -> bool:
    """Wait for user next/quit command in terminal mode."""
    while True:
        ans = input(f"[loop] step {step_index} ready. type 'n' to apply+next or 'q' to stop: ").strip().lower()
        if ans in ("n", ""):
            return True
        if ans == "q":
            return False
        print("[loop] invalid input. use 'n' or 'q'.")


def build_dds_domain_from_print_zone(
    field_vertices_world: np.ndarray,
    candidate_points: np.ndarray,
    *,
    source: str,
    scan_mesh: o3d.geometry.TriangleMesh | None,
    voxel_size_m: float,
    padding_m: float,
):
    """Build one fixed DDS domain around the planned print envelope."""
    from dds import Domain

    field_vertices = np.asarray(field_vertices_world, dtype=np.float64)
    candidates = np.asarray(candidate_points, dtype=np.float64)
    source_norm = str(source).strip().lower()
    if source_norm not in {"field", "scene"}:
        raise ValueError(f"Unknown DDS domain source: {source}. Use 'field' or 'scene'.")

    point_sets = [field_vertices, candidates]
    if source_norm == "scene":
        if scan_mesh is None:
            raise ValueError("scan_mesh is required when DDS domain source is 'scene'.")
        point_sets.append(np.asarray(scan_mesh.vertices, dtype=np.float64))

    bb_min, bb_max = compute_scene_bounds(*point_sets)
    pad = max(float(padding_m), float(voxel_size_m))
    return Domain.from_bounds(
        xmin=float(bb_min[0] - pad),
        xmax=float(bb_max[0] + pad),
        ymin=float(bb_min[1] - pad),
        ymax=float(bb_max[1] + pad),
        zmin=float(bb_min[2] - pad),
        zmax=float(bb_max[2] + pad),
        voxel_size=float(voxel_size_m),
        length_unit="m",
    )


def validate_deposits_inside_domain(deposits: tuple[object, ...], domain) -> None:
    """Reject deposits whose support would be clipped by the fixed DDS domain."""
    domain_min = np.asarray(domain.min_corner, dtype=np.float64)
    domain_max = np.asarray(domain.max_corner, dtype=np.float64)
    for deposit_index, deposit in enumerate(deposits):
        support_min, support_max = deposit.support_bounds(padding=0.0)
        minimum = np.asarray(support_min.to_tuple(), dtype=np.float64)
        maximum = np.asarray(support_max.to_tuple(), dtype=np.float64)
        if np.any(minimum < domain_min) or np.any(maximum > domain_max):
            raise RuntimeError(
                "DDS deposit support exceeds the fixed print-zone domain: "
                f"deposit={deposit_index} support_min={tuple(minimum)} "
                f"support_max={tuple(maximum)} domain_min={domain.min_corner} "
                f"domain_max={domain.max_corner}. Increase --dds-padding-mm "
                "or use --dds-domain-source scene."
            )


def _dds_target(point: np.ndarray, normal: np.ndarray):
    from dds import DepositionTarget

    p = np.asarray(point, dtype=np.float64)
    n = np.asarray(normal, dtype=np.float64)
    n_norm = float(np.linalg.norm(n))
    if n_norm <= 1e-12:
        n = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        n = n / n_norm
    return DepositionTarget(position=tuple(float(v) for v in p), normal=tuple(float(v) for v in n))


@dataclass(frozen=True)
class SegmentTargetSet:
    """Parsed start/end target segments in world meters."""

    start_points: np.ndarray
    end_points: np.ndarray
    start_z_dirs: np.ndarray
    end_z_dirs: np.ndarray

    @property
    def count(self) -> int:
        return int(self.end_points.shape[0])


def _fixed_z_line_targets(
    start_points: np.ndarray,
    end_points: np.ndarray,
    z_dir: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> OrientedLineTargets:
    starts = np.asarray(start_points, dtype=np.float64)
    ends = np.asarray(end_points, dtype=np.float64)
    if starts.shape != ends.shape:
        raise ValueError(f"start/end shape mismatch: {starts.shape} vs {ends.shape}")
    z = np.asarray(z_dir, dtype=np.float64)
    z_norm = float(np.linalg.norm(z))
    if z_norm <= 1e-12:
        z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        z = z / z_norm
    z_dirs = np.tile(z, (ends.shape[0], 1))
    return OrientedLineTargets(
        start_points=starts,
        end_points=ends,
        start_z_dirs=z_dirs,
        end_z_dirs=z_dirs,
    )


def build_candidate_segment_targets(
    *,
    candidate_points: np.ndarray,
    segment_start_points: np.ndarray | None,
    candidate_mode: str,
    field_vertices_world: np.ndarray,
    field_faces: np.ndarray,
    field_scalar: np.ndarray,
    tangent_sign: float,
    clamp_to_cone: bool,
    cone_max_tilt_deg: float,
) -> OrientedLineTargets:
    """Build the segment YAML contract from current scalar candidates."""

    ends = np.asarray(candidate_points, dtype=np.float64)
    if (
        segment_start_points is not None
        and np.asarray(segment_start_points).shape == ends.shape
    ):
        starts = np.asarray(segment_start_points, dtype=np.float64)
    else:
        starts = ends.copy()

    if ends.shape[0] == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return OrientedLineTargets(empty, empty, empty, empty)

    mode = str(candidate_mode).strip().lower()
    if mode in ("gradient_lift", "gradient_walk"):
        return build_oriented_line_targets(
            start_points=starts,
            end_points=ends,
            field_vertices_world=field_vertices_world,
            field_faces=field_faces,
            field_scalar=field_scalar,
            tangent_sign=float(tangent_sign),
            clamp_to_cone=bool(clamp_to_cone),
            cone_max_tilt_deg=float(cone_max_tilt_deg),
        )

    return _fixed_z_line_targets(starts, ends)


def _parse_plane_target(
    item: object,
    *,
    position_scale: float,
) -> tuple[np.ndarray, np.ndarray]:
    from dds.formats.yaml import parse_plane_string

    if not isinstance(item, dict) or "plane" not in item:
        raise ValueError(f"Segment target entry must contain a plane string. Got: {item}")
    components = parse_plane_string(str(item["plane"]))
    origin = np.asarray(components["O"], dtype=np.float64) / float(position_scale)
    z_dir = np.asarray(components.get("Z", (0.0, 0.0, 1.0)), dtype=np.float64)
    return origin, z_dir


def load_segment_targets_yaml(
    yaml_path: Path,
    *,
    position_scale: float,
) -> SegmentTargetSet:
    """Load the BEHAV3D `segments:` YAML contract in world meters."""

    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError as exc:
        raise ImportError("PyYAML is required to load segment target YAML.") from exc

    scale = float(position_scale)
    if scale <= 0.0:
        raise ValueError(f"position_scale must be > 0, got {position_scale}")

    with Path(yaml_path).open("r", encoding="utf-8") as file:
        payload = yaml.safe_load(file)
    if not isinstance(payload, dict) or not isinstance(payload.get("segments"), list):
        raise ValueError(f"YAML file must contain a top-level segments list: {yaml_path}")

    rows: list[tuple[int, int, np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
    for ordinal, item in enumerate(payload["segments"]):
        if not isinstance(item, dict):
            raise ValueError(f"Segment entry {ordinal} must be a mapping.")
        if "start" not in item or "end" not in item:
            raise ValueError(f"Segment entry {ordinal} must contain start and end targets.")

        index = int(item.get("index", ordinal))
        start_point, start_z = _parse_plane_target(item["start"], position_scale=scale)
        end_point, end_z = _parse_plane_target(item["end"], position_scale=scale)
        rows.append((index, ordinal, start_point, end_point, start_z, end_z))

    rows.sort(key=lambda row: (row[0], row[1]))
    if not rows:
        empty = np.zeros((0, 3), dtype=np.float64)
        return SegmentTargetSet(empty, empty, empty, empty)

    return SegmentTargetSet(
        start_points=np.vstack([row[2] for row in rows]),
        end_points=np.vstack([row[3] for row in rows]),
        start_z_dirs=np.vstack([row[4] for row in rows]),
        end_z_dirs=np.vstack([row[5] for row in rows]),
    )


def build_dds_step_deposits(
    segments: SegmentTargetSet,
    *,
    profile,
    mode: str,
    line_fraction: float,
) -> tuple[object, ...]:
    """Convert applied segment targets into DDS deposits.

    - dot: deposit only the segment end/candidate point.
    - line: sweep one DDS line bead over the final line_fraction of each segment.
    """
    from dds import LineDeposit, PointDeposit

    mode_norm = str(mode).strip().lower()
    if mode_norm not in {"dot", "line"}:
        raise ValueError(f"Unknown DDS deposit mode: {mode}. Use 'dot' or 'line'.")
    line_fraction_value = float(line_fraction)
    if not 0.0 <= line_fraction_value <= 1.0:
        raise ValueError(f"line_fraction must be in [0, 1], got {line_fraction}")

    starts = np.asarray(segments.start_points, dtype=np.float64)
    ends = np.asarray(segments.end_points, dtype=np.float64)
    start_normals = np.asarray(segments.start_z_dirs, dtype=np.float64)
    end_normals = np.asarray(segments.end_z_dirs, dtype=np.float64)
    if not (
        starts.shape == ends.shape == start_normals.shape == end_normals.shape
        and starts.ndim == 2
        and starts.shape[1] == 3
    ):
        raise ValueError(
            "Segment start/end points and normals must all have shape (N, 3): "
            f"starts={starts.shape} ends={ends.shape} "
            f"start_normals={start_normals.shape} end_normals={end_normals.shape}"
        )

    deposits = []
    for start, end, start_normal, end_normal in zip(
        starts,
        ends,
        start_normals,
        end_normals,
        strict=True,
    ):
        if mode_norm == "dot":
            deposits.append(
                PointDeposit(
                    target=_dds_target(end, end_normal),
                    profile=profile,
                )
            )
        else:
            material_start = end - line_fraction_value * (end - start)
            material_start_normal = end_normal + line_fraction_value * (start_normal - end_normal)
            deposits.append(
                LineDeposit(
                    start=_dds_target(material_start, material_start_normal),
                    end=_dds_target(end, end_normal),
                    profile=profile,
                )
            )
    return tuple(deposits)


def dds_surface_to_open3d_mesh(surface_mesh) -> o3d.geometry.TriangleMesh:
    """Convert a DDS TriangleMesh to an Open3D mesh for raycasting."""
    mesh = o3d.geometry.TriangleMesh()
    if surface_mesh.is_empty:
        return mesh
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(surface_mesh.vertices, dtype=np.float64).copy())
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(surface_mesh.faces, dtype=np.int32).copy())
    mesh.compute_vertex_normals()
    return mesh


def compose_scan_with_dds_proxy(
    base_scan_mesh: o3d.geometry.TriangleMesh,
    dds_result,
    *,
    threshold: float,
    mesh_step_size: int,
) -> tuple[o3d.geometry.TriangleMesh, int, int]:
    """Merge the original scan mesh with the DDS occupancy surface proxy."""
    scan_with_proxy = copy.deepcopy(base_scan_mesh)
    surface_mesh = dds_result.analysis.surface_mesh(
        threshold=float(threshold),
        step_size=int(mesh_step_size),
    )
    proxy_mesh = dds_surface_to_open3d_mesh(surface_mesh)
    proxy_faces = int(np.asarray(proxy_mesh.triangles).shape[0])
    if proxy_faces > 0:
        scan_with_proxy += proxy_mesh
    occupied = int(np.count_nonzero(dds_result.analysis.occupancy(threshold=float(threshold))))
    return scan_with_proxy, occupied, proxy_faces


def set_scalar_overlay_name_visible(overlay, name: str, visible: bool) -> None:
    """Set visibility for either a retained DDS visual or a raw PyVista overlay."""
    try:
        overlay.get(name).set_visible(bool(visible))
        return
    except KeyError:
        pass

    actors = getattr(overlay, "_behav3d_overlay_actors", {})
    actor = actors.get(name)
    if actor is not None:
        actor.SetVisibility(bool(visible))


def apply_scalar_overlay_visibility(viewer_state: dict[str, object]) -> None:
    """Apply checkbox state to all scalar overlay actor groups."""
    overlay = viewer_state.get("overlay")
    if overlay is None:
        return

    visibility = viewer_state.setdefault("scalar_overlay_visibility", default_scalar_overlay_visibility())
    for key, _, names in SCALAR_OVERLAY_GROUPS:
        visible = bool(visibility.get(key, True))
        for name in names:
            set_scalar_overlay_name_visible(overlay, name, visible)

    plotter = getattr(overlay, "plotter", None)
    if plotter is not None:
        plotter.render()


def install_scalar_overlay_controls(workbench, viewer_state: dict[str, object]) -> None:
    """Attach BEHAV3D scalar overlay checkboxes to the DDS workbench sidebar."""
    if viewer_state.get("scalar_overlay_controls") is not None:
        return

    from PySide6 import QtWidgets

    native_checkbox = getattr(workbench, "world_axes_checkbox", None)
    overlays_box = native_checkbox.parentWidget() if native_checkbox is not None else None
    overlays_layout = overlays_box.layout() if overlays_box is not None else None
    if overlays_layout is None:
        return

    visibility = viewer_state.setdefault("scalar_overlay_visibility", default_scalar_overlay_visibility())
    label = QtWidgets.QLabel("BEHAV3D Scalar", overlays_box)
    overlays_layout.addRow(label)

    controls: dict[str, object] = {}
    for key, text, _names in SCALAR_OVERLAY_GROUPS:
        checkbox = QtWidgets.QCheckBox(text, overlays_box)
        checkbox.setChecked(bool(visibility.get(key, True)))

        def _on_toggled(checked: bool, group_key: str = key) -> None:
            current = viewer_state.setdefault("scalar_overlay_visibility", default_scalar_overlay_visibility())
            current[group_key] = bool(checked)
            apply_scalar_overlay_visibility(viewer_state)

        checkbox.toggled.connect(_on_toggled)
        overlays_layout.addRow(checkbox)
        controls[key] = checkbox

    viewer_state["scalar_overlay_controls"] = controls


def show_step_window(
    step_index: int,
    viewer_state: dict[str, object],
    dds_simulator,
    dds_threshold: float,
    dds_view_mode: str,
    field_vertices_world: np.ndarray,
    heat_colors: np.ndarray,
    viable_mask: np.ndarray,
    seed_points: np.ndarray,
    scan_mesh: o3d.geometry.TriangleMesh,
    contour_points: np.ndarray,
    contour_lines: np.ndarray,
    offset_points: np.ndarray,
    offset_lines: np.ndarray,
    selected_points: np.ndarray,
    source_points: np.ndarray | None,
    segment_start_points: np.ndarray | None,
    target_points: np.ndarray | None,
    target_z_dirs: np.ndarray | None,
    axis_size: float,
) -> bool:
    """Visualize one loop step in the DDS workbench and wait for N/Q key."""
    import dds.viz
    from PySide6 import QtCore
    from dds.viz import ViewConfig
    from lib_scalar.viz_dds import (
        attach_viewer,
        add_colored_point_cloud,
        add_line_segments,
        add_vector_arrows,
        add_wire_mesh,
        remove_overlay,
    )

    view_mode = str(dds_view_mode).strip().lower()
    decision = {"next": False, "quit": False}
    viewer_state["decision"] = decision

    workbench = viewer_state.get("workbench")
    if workbench is None:
        scalar_field = None
        if view_mode == "implicit":
            scalar_field = "coverage"
        elif view_mode == "occupancy":
            scalar_field = "occupancy"
        workbench = dds.viz.show(
            dds_simulator,
            threshold=float(dds_threshold),
            initial_view=ViewConfig(
                view_mode=view_mode,
                scalar_field=scalar_field,
                show_toolpath=True,
                show_targets=False,
            ),
            off_screen=False,
        )
        overlay = attach_viewer(workbench.plotter)
        viewer_state["workbench"] = workbench
        viewer_state["overlay"] = overlay
        install_scalar_overlay_controls(workbench, viewer_state)

        def _finish(next_step: bool) -> None:
            current = viewer_state.get("decision")
            if isinstance(current, dict):
                current["next"] = bool(next_step)
                current["quit"] = not bool(next_step)
            event_loop = viewer_state.get("event_loop")
            if not bool(next_step):
                viewer_state["closed"] = True
                workbench.close()
            if isinstance(event_loop, QtCore.QEventLoop):
                event_loop.quit()

        workbench.plotter.add_key_event("n", lambda: _finish(True))
        workbench.plotter.add_key_event("N", lambda: _finish(True))
        workbench.plotter.add_key_event("q", lambda: _finish(False))
        workbench.plotter.add_key_event("Q", lambda: _finish(False))
        workbench.plotter.add_key_event("Escape", lambda: _finish(False))
    else:
        workbench.refresh(dds_simulator)
        overlay = viewer_state["overlay"]

    workbench.setWindowTitle(f"BEHAV3D DDS Scalar Loop Step {step_index}")

    overlay.clear()
    for name in (
        "field_heat_masked",
        "heat_seed_points",
        "scan_wire",
        "phi_contour",
        "offset_contour",
        "source_to_candidate",
        "print_segments",
        "target_orientations",
        "axis_x",
        "axis_y",
        "axis_z",
    ):
        remove_overlay(overlay, name)

    field_colors = heat_colors.copy()
    field_colors[~viable_mask] = np.array([0.2, 0.2, 0.2], dtype=np.float64)
    add_colored_point_cloud(
        overlay,
        field_vertices_world,
        field_colors,
        name="field_heat_masked",
        point_size=3.0,
    )

    if seed_points.shape[0] > 0:
        seed_colors = np.tile(
            np.array([0.2, 0.55, 1.0], dtype=np.float64),
            (seed_points.shape[0], 1),
        )
        add_colored_point_cloud(
            overlay,
            seed_points,
            seed_colors,
            name="heat_seed_points",
            point_size=7.0,
            render_as_spheres=True,
        )

    scan_vertices = np.asarray(scan_mesh.vertices, dtype=np.float64)
    scan_faces = np.asarray(scan_mesh.triangles, dtype=np.int64)
    add_wire_mesh(
        overlay,
        scan_vertices,
        scan_faces,
        name="scan_wire",
        color="#808080",
        line_width=1.0,
    )

    if contour_lines.shape[0] > 0:
        add_line_segments(
            overlay,
            contour_points,
            contour_lines,
            name="phi_contour",
            color="#00ffff",
            line_width=4.0,
        )

    if offset_lines.shape[0] > 0:
        add_line_segments(
            overlay,
            offset_points,
            offset_lines,
            name="offset_contour",
            color="#ff00ff",
            line_width=4.0,
        )

    if selected_points.shape[0] > 0:
        selected_colors = np.tile(
            np.array([0.0, 1.0, 0.0], dtype=np.float64),
            (selected_points.shape[0], 1),
        )
        add_colored_point_cloud(
            overlay,
            selected_points,
            selected_colors,
            name="selected_points",
            point_size=12.0,
            render_as_spheres=True,
        )

    if (
        source_points is not None
        and source_points.shape == selected_points.shape
        and selected_points.shape[0] > 0
    ):
        source_colors = np.tile(
            np.array([0.0, 1.0, 1.0], dtype=np.float64),
            (source_points.shape[0], 1),
        )
        add_colored_point_cloud(
            overlay,
            source_points,
            source_colors,
            name="source_points",
            point_size=8.0,
            render_as_spheres=True,
        )
        source_lines = np.column_stack(
            [
                np.arange(source_points.shape[0], dtype=np.int64),
                np.arange(source_points.shape[0], dtype=np.int64) + source_points.shape[0],
            ]
        )
        add_line_segments(
            overlay,
            np.vstack([source_points, selected_points]),
            source_lines,
            name="source_to_candidate",
            color="#ffff00",
            line_width=3.0,
        )

    if (
        segment_start_points is not None
        and segment_start_points.shape == selected_points.shape
        and selected_points.shape[0] > 0
    ):
        start_colors = np.tile(
            np.array([1.0, 0.55, 0.0], dtype=np.float64),
            (segment_start_points.shape[0], 1),
        )
        add_colored_point_cloud(
            overlay,
            segment_start_points,
            start_colors,
            name="segment_start_points",
            point_size=8.0,
            render_as_spheres=True,
        )
        print_segment_lines = np.column_stack(
            [
                np.arange(segment_start_points.shape[0], dtype=np.int64),
                np.arange(segment_start_points.shape[0], dtype=np.int64) + segment_start_points.shape[0],
            ]
        )
        add_line_segments(
            overlay,
            np.vstack([segment_start_points, selected_points]),
            print_segment_lines,
            name="print_segments",
            color="#ff8c00",
            line_width=7.0,
        )

    if (
        target_points is not None
        and target_z_dirs is not None
        and target_points.shape == target_z_dirs.shape
        and target_points.shape[0] > 0
    ):
        norms = np.linalg.norm(target_z_dirs, axis=1)
        valid = norms > 1e-12
        if np.any(valid):
            target_vectors = 0.008 * target_z_dirs[valid] / norms[valid, None]
            add_vector_arrows(
                overlay,
                target_points[valid],
                target_vectors,
                name="target_orientations",
                color="#21bf73",
            )

    bb_min, bb_max = compute_scene_bounds(field_vertices_world, scan_vertices)
    bb_diag = float(np.linalg.norm(bb_max - bb_min))
    bb_center = 0.5 * (bb_min + bb_max)
    axis_size_val = float(axis_size)
    if axis_size_val == 0.0:
        axis_size_val = max(1e-4, 0.15 * bb_diag)
    if axis_size_val > 0.0:
        axis_origin = bb_center.reshape(1, 3)
        add_vector_arrows(
            overlay,
            axis_origin,
            np.array([[axis_size_val, 0.0, 0.0]], dtype=np.float64),
            name="axis_x",
            color="#e74c3c",
        )
        add_vector_arrows(
            overlay,
            axis_origin,
            np.array([[0.0, axis_size_val, 0.0]], dtype=np.float64),
            name="axis_y",
            color="#27ae60",
        )
        add_vector_arrows(
            overlay,
            axis_origin,
            np.array([[0.0, 0.0, axis_size_val]], dtype=np.float64),
            name="axis_z",
            color="#2980b9",
        )

    apply_scalar_overlay_visibility(viewer_state)

    print("[viz] DDS workbench controls: N=apply+next, Q/Esc=stop")
    event_loop = QtCore.QEventLoop()
    viewer_state["event_loop"] = event_loop
    event_loop.exec()
    viewer_state.pop("event_loop", None)

    if (not decision["next"]) and (not decision["quit"]):
        return False
    return bool(decision["next"])


def run(
    field_mesh_path: Path,
    scan_mesh_path: Path,
    output_dir: Path,
    field_state_path: Path | None,
    seed: int | None,
    seed_level: float | None,
    t_coef: float,
    field_subdivide_iter: int,
    field_scale: float,
    clearance: float,
    candidate_mode: str,
    offset_distance_mm: float,
    offset_geodesic_delta_mm: float,
    beads_per_step: int,
    bead_separation_mm: float,
    bead_width_mm: float,
    bead_height_mm: float,
    walk_distance_mm: float,
    walk_step_mm: float,
    walk_max_steps: int,
    walk_tangent_sign: float,
    walk_start_fraction: float,
    clamp_to_cone: bool,
    cone_max_tilt_deg: float,
    normal_continuity_rule: bool,
    endpoint_spacing_rule: bool,
    bead_shape: str,
    positioning_attempts: int,
    position_target_xy: tuple[float, float] | None,
    search_step_x: float,
    search_step_y: float,
    search_allow_partial_hit: bool,
    base_z_offset: float,
    axis_size: float,
    visualize: bool,
    dds_voxel_size_mm: float,
    dds_domain_source: str,
    dds_deposit_mode: str,
    dds_line_fraction: float,
    dds_threshold: float,
    dds_padding_mm: float,
    dds_surface_step_size: int,
    dds_view_mode: str,
    save_dds_step_bundles: bool,
    target_position_scale: float,
) -> None:
    from dds import BeadProfile, Simulator

    initial_locked_offset_xyz: tuple[float, float, float] | None = None
    seed_points_scaled = np.zeros((0, 3), dtype=np.float64)
    if field_state_path is not None:
        field_state = load_field_state_npz(field_state_path)
        field_vertices_scaled = field_state.field_vertices_scaled
        field_faces = field_state.field_faces
        heat_norm = field_state.heat_norm
        initial_locked_offset_xyz = field_state.offset_xyz
        seed_points_scaled = field_state.seed_points_scaled
        print(
            "[field] loaded pre-initialized field state: "
            f"{field_state_path} vertices={field_vertices_scaled.shape[0]} "
            f"faces={field_faces.shape[0]} "
            f"offset=({initial_locked_offset_xyz[0]:.6f},"
            f"{initial_locked_offset_xyz[1]:.6f},"
            f"{initial_locked_offset_xyz[2]:.6f})"
        )
    else:
        field_mesh = load_triangle_mesh_arrays(field_mesh_path)
        base_vertices = field_mesh.vertices
        base_faces = field_mesh.faces
        field_vertices, field_faces = subdivide_field_mesh_loop(base_vertices, base_faces, int(field_subdivide_iter))
        heat_seed_indices = resolve_heat_seed_indices(
            field_vertices,
            seed=seed,
            seed_level=seed_level,
        )
        heat = compute_heat_field(
            vertices=field_vertices,
            faces=field_faces,
            seed=seed,
            seed_level=seed_level,
            t_coef=t_coef,
        )

        scale = float(field_scale)
        if scale <= 0.0:
            raise ValueError(f"field_scale must be > 0, got {field_scale}")
        field_vertices_scaled = field_vertices * scale
        heat_norm = np.asarray(heat.norm, dtype=np.float64)
        seed_points_scaled = field_vertices_scaled[heat_seed_indices]
    heat_colors = yellow_to_red_colors(heat_norm)

    scan_mesh_base = load_triangle_mesh_legacy(scan_mesh_path)
    scan_mesh_current = copy.deepcopy(scan_mesh_base)
    all_selected_points: list[np.ndarray] = []
    locked_offset_xyz: tuple[float, float, float] | None = initial_locked_offset_xyz
    dds_simulator: Simulator | None = None
    dds_result = None
    viewer_state: dict[str, object] = {}
    dds_bead_width_mm = float(bead_width_mm)
    if dds_bead_width_mm <= 0.0:
        raise ValueError(f"bead_width_mm must be > 0, got {bead_width_mm}")
    dds_profile = BeadProfile(
        width=1e-3 * dds_bead_width_mm,
        height=1e-3 * float(bead_height_mm),
    )
    dds_voxel_size_m = 1e-3 * float(dds_voxel_size_mm)
    dds_padding_m = 1e-3 * float(dds_padding_mm)
    if dds_voxel_size_m <= 0.0:
        raise ValueError(f"dds_voxel_size_mm must be > 0, got {dds_voxel_size_mm}")
    if not 0.0 <= float(dds_threshold) <= 1.0:
        raise ValueError(f"dds_threshold must be in [0, 1], got {dds_threshold}")
    if int(dds_surface_step_size) < 1:
        raise ValueError(f"dds_surface_step_size must be >= 1, got {dds_surface_step_size}")
    if float(target_position_scale) <= 0.0:
        raise ValueError(f"target_position_scale must be > 0, got {target_position_scale}")
    dds_deposit_mode_norm = str(dds_deposit_mode).strip().lower()
    if dds_deposit_mode_norm not in {"dot", "line"}:
        raise ValueError(f"dds_deposit_mode must be 'dot' or 'line', got {dds_deposit_mode}")
    dds_line_fraction_value = float(dds_line_fraction)
    if not 0.0 <= dds_line_fraction_value <= 1.0:
        raise ValueError(f"dds_line_fraction must be in [0, 1], got {dds_line_fraction}")

    print(
        "[config] "
        f"candidate_mode={candidate_mode} "
        f"normal_continuity_rule={bool(normal_continuity_rule)} "
        f"endpoint_spacing_rule={bool(endpoint_spacing_rule)} "
        f"proxy=dds_implicit_surface "
        f"dds_deposit_mode={dds_deposit_mode_norm} "
        f"dds_line_fraction={dds_line_fraction_value:.3f} "
        f"legacy_bead_shape_arg={bead_shape} "
        f"bead_separation_mm={float(bead_separation_mm):.3f} "
        f"bead_height_mm={float(bead_height_mm):.3f} "
        f"dds_bead_width_mm={dds_bead_width_mm:.3f} "
        f"dds_voxel_size_mm={float(dds_voxel_size_mm):.3f} "
        f"dds_domain_source={dds_domain_source} "
        f"position_target_xy={position_target_xy} "
        f"target_position_scale={float(target_position_scale):.3f} "
        f"field_state_path={field_state_path} "
        f"dds_threshold={float(dds_threshold):.3f}"
    )

    step = 0
    while True:
        step_index = step + 1

        if locked_offset_xyz is None:
            pose = position_field_with_attempts(
                scan_mesh=scan_mesh_current,
                field_vertices_scaled=field_vertices_scaled,
                heat_norm=heat_norm,
                clearance=float(clearance),
                iso_level=float(ISO_LEVEL),
                base_z_offset=float(base_z_offset),
                search_step_x=float(search_step_x),
                search_step_y=float(search_step_y),
                positioning_attempts=int(positioning_attempts),
                search_max_candidates=int(SEARCH_MAX_CANDIDATES),
                require_full_hit=not bool(search_allow_partial_hit),
                preferred_centroid_xy=position_target_xy,
            )
            locked_offset_xyz = pose.offset_xyz
            print(
                "[pose] locked field offset from step 1: "
                f"x={locked_offset_xyz[0]:.6f} y={locked_offset_xyz[1]:.6f} z={locked_offset_xyz[2]:.6f}"
            )
            if position_target_xy is not None:
                selected_centroid_xy = np.mean(pose.field_vertices_world[:, :2], axis=0)
                target_distance = float(
                    np.linalg.norm(
                        selected_centroid_xy - np.asarray(position_target_xy, dtype=np.float64)
                    )
                )
                print(
                    "[pose] preferred field XY centroid: "
                    f"target=({position_target_xy[0]:.6f},{position_target_xy[1]:.6f}) "
                    f"selected=({selected_centroid_xy[0]:.6f},{selected_centroid_xy[1]:.6f}) "
                    f"distance_m={target_distance:.6f}"
                )
        else:
            scene, z_top = make_scan_scene(scan_mesh_current)
            pose = evaluate_fixed_pose(
                scene=scene,
                z_top=z_top,
                field_vertices_scaled=field_vertices_scaled,
                offset_xyz=locked_offset_xyz,
                clearance=float(clearance),
                iso_level=float(ISO_LEVEL),
            )

        contour = compute_offset_contour_stage(
            pose=pose,
            field_faces=field_faces,
            iso_level=float(ISO_LEVEL),
            offset_distance_mm=float(offset_distance_mm),
            offset_geodesic_delta_mm=float(offset_geodesic_delta_mm),
            offset_t_coef=float(t_coef),
            scan_mesh=scan_mesh_current,
        )

        candidate = generate_step_candidates(
            contour=contour,
            field_faces=field_faces,
            field_vertices_world=pose.field_vertices_world,
            heat_norm=heat_norm,
            phi=pose.phi,
            mode=str(candidate_mode),
            beads_per_step=int(beads_per_step),
            bead_separation_mm=float(bead_separation_mm),
            bead_height_mm=float(bead_height_mm),
            walk_distance_mm=float(walk_distance_mm),
            walk_step_mm=float(walk_step_mm),
            walk_max_steps=int(walk_max_steps),
            walk_tangent_sign=float(walk_tangent_sign),
            walk_start_fraction=float(walk_start_fraction),
            clamp_to_cone=bool(clamp_to_cone),
            cone_max_tilt_deg=float(cone_max_tilt_deg),
        )

        print(
            f"[step {step_index}] "
            f"viable={pose.viable_count} "
            f"contour_segments={contour.contour_lines.shape[0]} "
            f"offset_segments={contour.offset_lines.shape[0]} "
            f"offset_z_valid={candidate.z_valid_count} "
            f"selected={candidate.points.shape[0]}"
        )

        line_targets = build_candidate_segment_targets(
            candidate_points=candidate.points,
            segment_start_points=candidate.segment_start_points,
            candidate_mode=str(candidate_mode),
            field_vertices_world=pose.field_vertices_world,
            field_faces=field_faces,
            field_scalar=heat_norm,
            tangent_sign=float(walk_tangent_sign),
            clamp_to_cone=bool(clamp_to_cone),
            cone_max_tilt_deg=float(cone_max_tilt_deg),
        )
        line_targets, secondary_stats = apply_secondary_target_rules(
            line_targets,
            candidate_mode=str(candidate_mode),
            bead_height_m=1e-3 * float(bead_height_mm),
            bead_separation_m=1e-3 * float(bead_separation_mm),
            normal_continuity_rule=bool(normal_continuity_rule),
            endpoint_spacing_rule=bool(endpoint_spacing_rule),
        )
        normal_flip_replaced = int(secondary_stats["normal_continuity_replaced"])
        endpoint_spacing_removed = int(secondary_stats["endpoint_spacing_removed"])
        if normal_flip_replaced > 0:
            print(
                f"[targets] replaced normal-discontinuity segments: "
                f"{normal_flip_replaced}/{line_targets.count} "
                f"dot_threshold={TARGET_NORMAL_FLIP_DOT_THRESHOLD:.3f} "
                f"extension_fraction={TARGET_NORMAL_FLIP_BEAD_HEIGHT_FRACTION:.3f}"
            )
        if endpoint_spacing_removed > 0:
            print(
                f"[targets] removed close endpoint segments: "
                f"{endpoint_spacing_removed} min_distance_m={1e-3 * float(bead_separation_mm):.6f}"
            )
        target_points, target_z_dirs = line_targets.flattened_points_and_z_dirs()
        step_dir = output_dir / f"step_{step_index:02d}"
        step_segments_yaml = step_dir / "segments.yaml"

        if dds_simulator is None:
            dds_domain = build_dds_domain_from_print_zone(
                field_vertices_world=pose.field_vertices_world,
                candidate_points=line_targets.end_points,
                source=str(dds_domain_source),
                scan_mesh=scan_mesh_base,
                voxel_size_m=dds_voxel_size_m,
                padding_m=max(
                    dds_padding_m,
                    1e-3 * float(offset_distance_mm),
                    1e-3 * float(bead_height_mm),
                    1e-3 * dds_bead_width_mm,
                ),
            )
            dds_simulator = Simulator(dds_domain)
            voxel_count = int(np.prod(dds_domain.grid_shape, dtype=np.int64))
            float_field_mib = voxel_count * np.dtype(np.float64).itemsize / (1024.0**2)
            print(
                f"[dds] domain source={dds_domain_source}: "
                f"min={dds_domain.min_corner} max={dds_domain.max_corner} "
                f"grid_shape={dds_domain.grid_shape} voxel_size={dds_domain.voxel_size} "
                f"voxels={voxel_count} float_field_mib={float_field_mib:.1f}"
            )

        if visualize:
            assert dds_simulator is not None
            apply_step = show_step_window(
                step_index=step_index,
                viewer_state=viewer_state,
                dds_simulator=dds_simulator,
                dds_threshold=float(dds_threshold),
                dds_view_mode=str(dds_view_mode),
                field_vertices_world=pose.field_vertices_world,
                heat_colors=heat_colors,
                viable_mask=pose.viable,
                seed_points=seed_points_scaled + np.asarray(pose.offset_xyz, dtype=np.float64),
                scan_mesh=scan_mesh_current,
                contour_points=contour.contour_points,
                contour_lines=contour.contour_lines,
                offset_points=contour.offset_points,
                offset_lines=contour.offset_lines,
                selected_points=line_targets.end_points,
                source_points=candidate.source_points,
                segment_start_points=line_targets.start_points,
                target_points=target_points,
                target_z_dirs=target_z_dirs,
                axis_size=axis_size,
            )
        else:
            apply_step = wait_next_terminal(step_index=step_index)

        if not apply_step:
            print("[loop] Stop requested by user.")
            break

        if line_targets.count == 0:
            print("[loop] No valid target segments left; stopping early.")
            break

        write_line_targets_yaml(
            out_yaml=step_segments_yaml,
            targets=line_targets,
            position_scale=float(target_position_scale),
        )
        print(
            f"[targets] step {step_index}: saved/applied segment YAML: "
            f"{step_segments_yaml} segments={line_targets.count}"
        )
        applied_segments = load_segment_targets_yaml(
            step_segments_yaml,
            position_scale=float(target_position_scale),
        )
        if applied_segments.count != line_targets.count:
            raise RuntimeError(
                "Applied segment YAML count does not match final target segments: "
                f"{applied_segments.count} vs {line_targets.count}"
            )

        step_deposits = build_dds_step_deposits(
            applied_segments,
            profile=dds_profile,
            mode=dds_deposit_mode_norm,
            line_fraction=dds_line_fraction_value,
        )

        assert dds_simulator is not None
        validate_deposits_inside_domain(step_deposits, dds_simulator.domain)
        dds_simulator.add_deposits(step_deposits)
        dds_result = dds_simulator.result(
            include_coverage=True,
            threshold=float(dds_threshold),
        )
        scan_mesh_current, occupied_voxels, proxy_faces = compose_scan_with_dds_proxy(
            base_scan_mesh=scan_mesh_base,
            dds_result=dds_result,
            threshold=float(dds_threshold),
            mesh_step_size=int(dds_surface_step_size),
        )
        step_scan_path = step_dir / "scan_with_dds_proxy.ply"
        if not o3d.io.write_triangle_mesh(str(step_scan_path), scan_mesh_current):
            raise RuntimeError(f"Failed to write step scan + DDS proxy mesh: {step_scan_path}")
        print(
            f"[dds] step {step_index}: "
            f"added_deposits={len(step_deposits)} "
            f"total_deposits={len(dds_simulator.deposits)} "
            f"occupied_voxels={occupied_voxels} "
            f"proxy_faces={proxy_faces} "
            f"implicit_max={float(dds_result.implicit_field.max()):.3f}"
        )
        print(f"[dds] saved step scan + DDS proxy mesh: {step_scan_path}")
        if save_dds_step_bundles:
            bundle_dir = step_dir / "dds_bundle"
            dds_result.save(
                bundle_dir,
                metadata={
                    "step": step_index,
                    "candidate_mode": str(candidate_mode),
                    "threshold": float(dds_threshold),
                    "bead_width_m": float(dds_profile.width),
                    "bead_height_m": float(dds_profile.height),
                },
            )
            print(f"[dds] saved step bundle: {bundle_dir}")

        all_selected_points.append(line_targets.end_points)
        step += 1

    workbench = viewer_state.get("workbench")
    if workbench is not None and not bool(viewer_state.get("closed", False)):
        workbench.close()

    output_dir.mkdir(parents=True, exist_ok=True)
    final_scan_path = output_dir / "loop_sim_scan_with_dds_proxy.ply"
    if not o3d.io.write_triangle_mesh(str(final_scan_path), scan_mesh_current):
        raise RuntimeError(f"Failed to write final scan + DDS proxy mesh: {final_scan_path}")
    print(f"[done] saved final scan + DDS proxy mesh: {final_scan_path}")

    if dds_result is not None:
        from dds.geometry import write_mesh

        final_bundle_dir = output_dir / "loop_sim_dds_bundle"
        dds_result.save(
            final_bundle_dir,
            metadata={
                "candidate_mode": str(candidate_mode),
                "threshold": float(dds_threshold),
                "bead_width_m": float(dds_profile.width),
                "bead_height_m": float(dds_profile.height),
            },
        )
        dds_surface = dds_result.analysis.surface_mesh(
            threshold=float(dds_threshold),
            step_size=int(dds_surface_step_size),
        )
        if not dds_surface.is_empty:
            surface_path = write_mesh(output_dir / "loop_sim_dds_surface.ply", dds_surface)
            print(f"[done] saved DDS surface mesh: {surface_path}")
        print(f"[done] saved DDS bundle: {final_bundle_dir}")

    if all_selected_points:
        all_points = np.vstack(all_selected_points)
        colors = np.tile(np.array([0.0, 1.0, 0.0], dtype=np.float64), (all_points.shape[0], 1))
        all_pcd = make_point_cloud(all_points, colors)
        all_points_path = output_dir / "loop_sim_all_print_points.ply"
        if not o3d.io.write_point_cloud(str(all_points_path), all_pcd):
            raise RuntimeError(f"Failed to write accumulated print points: {all_points_path}")
        print(f"[done] saved all print points: {all_points_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Simulate iterative scalar-field print loop with DDS implicit bead proxy."
    )
    parser.add_argument("--field-mesh", type=Path, default=DEFAULT_FIELD_MESH)
    parser.add_argument("--scan-mesh", type=Path, default=DEFAULT_SCAN_MESH)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR / "loop_sim")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--seed-level", type=float, default=None)
    parser.add_argument("--t-coef", type=float, default=2000.0)
    parser.add_argument("--field-subdivide-iter", type=int, default=1)
    parser.add_argument("--field-scale", type=float, default=0.001)
    parser.add_argument("--clearance", type=float, default=0.0)
    parser.add_argument(
        "--candidate-mode",
        type=str,
        choices=("geodesic", "z_lift", "gradient_lift", "gradient_walk"),
        default="geodesic",
    )
    parser.add_argument("--offset-distance-mm", type=float, default=12.0)
    parser.add_argument("--offset-geodesic-delta-mm", type=float, default=0.6)
    parser.add_argument("--beads-per-step", type=int, default=7)
    parser.add_argument("--bead-separation-mm", type=float, default=16.0)
    parser.add_argument(
        "--bead-width-mm",
        type=float,
        required=True,
        help="DDS BeadProfile width in mm. Required; independent from --bead-separation-mm.",
    )
    parser.add_argument("--bead-height-mm", type=float, default=12.0)
    parser.add_argument("--walk-distance-mm", type=float, default=12.0)
    parser.add_argument("--walk-step-mm", type=float, default=1.0)
    parser.add_argument("--walk-max-steps", type=int, default=32)
    parser.add_argument("--walk-tangent-sign", type=float, default=1.0)
    parser.add_argument("--walk-start-fraction", type=float, default=0.25)
    parser.add_argument("--clamp-to-cone", action="store_true")
    parser.add_argument("--cone-max-tilt-deg", type=float, default=45.0)
    parser.add_argument(
        "--disable-normal-continuity-rule",
        action="store_true",
        help=(
            "Disable the secondary gradient_lift target rule that replaces low "
            "start/end Z-continuity segments."
        ),
    )
    parser.add_argument(
        "--disable-endpoint-spacing-rule",
        action="store_true",
        help=(
            "Disable the secondary target rule that removes later targets whose "
            "end point is closer than --bead-separation-mm to an earlier kept end."
        ),
    )
    parser.add_argument(
        "--bead-shape",
        type=str,
        choices=("cylinder", "sphere"),
        default="cylinder",
        help="Legacy compatibility argument; DDS proxy uses BeadProfile width/height.",
    )
    parser.add_argument("--positioning-attempts", type=int, default=3)
    parser.add_argument(
        "--position-target-x",
        type=float,
        default=0.0,
        help="Preferred positioned field vertex-centroid X in world coordinates, in meters.",
    )
    parser.add_argument(
        "--position-target-y",
        type=float,
        default=-0.75,
        help="Preferred positioned field vertex-centroid Y in world coordinates, in meters.",
    )
    parser.add_argument(
        "--no-position-target",
        action="store_true",
        help="Disable XY field-center preference and use the legacy pose ranking.",
    )
    parser.add_argument("--search-step-x", type=float, default=0.01)
    parser.add_argument("--search-step-y", type=float, default=0.01)
    parser.add_argument("--search-allow-partial-hit", action="store_true")
    parser.add_argument("--base-z-offset", type=float, default=BASE_Z_OFFSET)
    parser.add_argument("--axis-size", type=float, default=0.0)
    parser.add_argument(
        "--target-position-scale",
        type=float,
        default=TARGET_POSITION_SCALE,
        help="Scale used when writing step segment YAML positions. Default 1000 writes meters as mm.",
    )
    parser.add_argument(
        "--dds-voxel-size-mm",
        type=float,
        default=2.0,
        help="DDS dense-grid voxel size in mm for the bead proxy.",
    )
    parser.add_argument(
        "--dds-domain-source",
        type=str,
        choices=("field", "scene"),
        default="field",
        help=(
            "Build the fixed DDS grid from the positioned field/print envelope "
            "(default) or from the full scan+field scene (legacy)."
        ),
    )
    parser.add_argument(
        "--dds-deposit-mode",
        type=str,
        choices=("dot", "line"),
        default="dot",
        help=(
            "DDS bead primitive built from each applied YAML segment: "
            "'dot' deposits only the segment end point, 'line' sweeps from start to end."
        ),
    )
    parser.add_argument(
        "--dds-line-fraction",
        type=float,
        default=1.0,
        help=(
            "For --dds-deposit-mode line, deposit only this final fraction of each "
            "YAML segment. 1.0 uses start->end, 0.5 uses the final half, 0.0 "
            "collapses to the end point."
        ),
    )
    parser.add_argument(
        "--dds-threshold",
        type=float,
        default=0.5,
        help="DDS implicit-field threshold used for occupancy and proxy surface extraction.",
    )
    parser.add_argument(
        "--dds-padding-mm",
        type=float,
        default=24.0,
        help="Minimum padding around scan/field bounds for the DDS domain, in mm.",
    )
    parser.add_argument(
        "--dds-surface-step-size",
        type=int,
        default=1,
        help="Marching-cubes step size for the DDS proxy surface.",
    )
    parser.add_argument(
        "--dds-view-mode",
        type=str,
        choices=("surface", "occupancy", "implicit"),
        default="surface",
        help="Initial DDS workbench representation.",
    )
    parser.add_argument(
        "--save-dds-step-bundles",
        action="store_true",
        help="Save a DDS bundle after every accepted loop step.",
    )
    parser.add_argument("--no-vis", action="store_true")
    args = parser.parse_args()
    position_target_xy = (
        None
        if args.no_position_target
        else (float(args.position_target_x), float(args.position_target_y))
    )

    run(
        field_mesh_path=args.field_mesh,
        scan_mesh_path=args.scan_mesh,
        output_dir=args.output_dir,
        field_state_path=None,
        seed=args.seed,
        seed_level=args.seed_level,
        t_coef=args.t_coef,
        field_subdivide_iter=args.field_subdivide_iter,
        field_scale=args.field_scale,
        clearance=args.clearance,
        candidate_mode=args.candidate_mode,
        offset_distance_mm=args.offset_distance_mm,
        offset_geodesic_delta_mm=args.offset_geodesic_delta_mm,
        beads_per_step=args.beads_per_step,
        bead_separation_mm=args.bead_separation_mm,
        bead_width_mm=args.bead_width_mm,
        bead_height_mm=args.bead_height_mm,
        walk_distance_mm=args.walk_distance_mm,
        walk_step_mm=args.walk_step_mm,
        walk_max_steps=args.walk_max_steps,
        walk_tangent_sign=args.walk_tangent_sign,
        walk_start_fraction=args.walk_start_fraction,
        clamp_to_cone=args.clamp_to_cone,
        cone_max_tilt_deg=args.cone_max_tilt_deg,
        normal_continuity_rule=not args.disable_normal_continuity_rule,
        endpoint_spacing_rule=not args.disable_endpoint_spacing_rule,
        bead_shape=args.bead_shape,
        positioning_attempts=args.positioning_attempts,
        position_target_xy=position_target_xy,
        search_step_x=args.search_step_x,
        search_step_y=args.search_step_y,
        search_allow_partial_hit=args.search_allow_partial_hit,
        base_z_offset=args.base_z_offset,
        axis_size=args.axis_size,
        visualize=not args.no_vis,
        dds_voxel_size_mm=args.dds_voxel_size_mm,
        dds_domain_source=args.dds_domain_source,
        dds_deposit_mode=args.dds_deposit_mode,
        dds_line_fraction=args.dds_line_fraction,
        dds_threshold=args.dds_threshold,
        dds_padding_mm=args.dds_padding_mm,
        dds_surface_step_size=args.dds_surface_step_size,
        dds_view_mode=args.dds_view_mode,
        save_dds_step_bundles=args.save_dds_step_bundles,
        target_position_scale=args.target_position_scale,
    )


if __name__ == "__main__":
    main()
