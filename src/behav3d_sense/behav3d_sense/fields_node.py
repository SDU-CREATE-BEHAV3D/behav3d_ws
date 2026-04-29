#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import copy
from pathlib import Path
from typing import Iterable, List, Optional

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node

try:
    from behav3d_interfaces.srv import GeneratePrintCandidates, InitFieldFromScan
except ImportError:  # pragma: no cover
    from behav3d_interfaces.srv._generate_print_candidates import GeneratePrintCandidates
    from behav3d_interfaces.srv._init_field_from_scan import InitFieldFromScan

from .reconstruct.service_utils import get_captures_root, resolve_session_path

try:
    from behav3d_py.scalar_field.lib_scalar.compute_heat_field import compute_heat_field
    from behav3d_py.scalar_field.lib_scalar.compute_phi_mask import evaluate_fixed_pose, make_scan_scene
    from behav3d_py.scalar_field.lib_scalar.extract_phi_contour import extract_phi_contour
    from behav3d_py.scalar_field.lib_scalar.generate_print_points import generate_print_points
    from behav3d_py.scalar_field.lib_scalar.geometry import (
        clamp_vectors_to_cone,
        load_triangle_mesh_arrays,
        sample_tangent_axes_on_surface_from_scalar,
    )
    from behav3d_py.scalar_field.lib_scalar.loop_simulation import position_field_with_attempts
    from behav3d_py.scalar_field.lib_scalar.print_targets import (
        build_oriented_line_targets,
        write_fixed_z_targets_yaml,
        write_line_targets_yaml,
        write_point_targets_yaml,
    )
    from behav3d_py.scalar_field.lib_scalar.viz import make_line_set, make_point_cloud, yellow_to_red_colors

    _SCALAR_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover
    compute_heat_field = None
    evaluate_fixed_pose = None
    make_scan_scene = None
    extract_phi_contour = None
    generate_print_points = None
    clamp_vectors_to_cone = None
    load_triangle_mesh_arrays = None
    sample_tangent_axes_on_surface_from_scalar = None
    position_field_with_attempts = None
    build_oriented_line_targets = None
    write_fixed_z_targets_yaml = None
    write_line_targets_yaml = None
    write_point_targets_yaml = None
    make_line_set = None
    make_point_cloud = None
    yellow_to_red_colors = None
    _SCALAR_IMPORT_ERROR = exc


def _resolve_input_path(raw_path: str, session_dir: Path, captures_root: Path) -> Optional[Path]:
    value = str(raw_path or "").strip()
    if not value:
        return None
    if value.startswith("file://"):
        value = value[7:] if value.startswith("file:///") else value[7:]
    if value.startswith("@session"):
        return resolve_session_path(value, False, captures_root).resolve()

    p = Path(value).expanduser()
    if p.is_absolute():
        return p.resolve()

    candidate_session = (session_dir / p).resolve()
    if candidate_session.exists():
        return candidate_session
    return (Path.cwd() / p).resolve()


def _resolve_output_dir(raw_path: str, session_dir: Path, captures_root: Path, default_value: str) -> Path:
    value = str(raw_path or "").strip()
    if not value:
        value = str(default_value or "").strip() or "@session/field_loop/init"
    if value.startswith("@session"):
        return resolve_session_path(value, False, captures_root).resolve()
    p = Path(value).expanduser()
    if p.is_absolute():
        return p.resolve()
    return (session_dir / p).resolve()


def _latest_existing(base_dir: Path, patterns: Iterable[str]) -> Optional[Path]:
    matches: List[Path] = []
    for pattern in patterns:
        try:
            for path in base_dir.glob(pattern):
                if path.is_file():
                    matches.append(path.resolve())
        except Exception:
            continue
    if not matches:
        return None
    matches.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return matches[0]


def _default_scan_mesh_path(session_dir: Path) -> Optional[Path]:
    return _latest_existing(
        session_dir,
        (
            "reconstruct/**/tsdf_surface_mesh.stl",
            "reconstruct/**/tsdf_surface_mesh.ply",
            "reconstruct/**/tsdf_surface_mesh.obj",
            "**/reconstruct/tsdf_surface_mesh.stl",
            "**/reconstruct/tsdf_surface_mesh.ply",
            "**/reconstruct/tsdf_surface_mesh.obj",
            "tsdf_surface_mesh.stl",
            "tsdf_surface_mesh.ply",
            "tsdf_surface_mesh.obj",
        ),
    )


def _default_field_state_path(session_dir: Path) -> Optional[Path]:
    return _latest_existing(
        session_dir,
        (
            "field_loop/**/field_state_init.npz",
            "**/field_loop/init/field_state_init.npz",
            "**/field_state_init.npz",
            "field_state_init.npz",
        ),
    )


def _load_scan_mesh(paths: List[Path]) -> o3d.geometry.TriangleMesh:
    if not paths:
        raise ValueError("No scan mesh paths to load.")

    merged: Optional[o3d.geometry.TriangleMesh] = None
    for path in paths:
        mesh = o3d.io.read_triangle_mesh(str(path))
        if not mesh.has_triangles():
            raise ValueError(f"Scan mesh has no triangles: {path}")
        if merged is None:
            merged = copy.deepcopy(mesh)
        else:
            merged += mesh
    assert merged is not None
    return merged


class FieldsNode(Node):
    def __init__(self):
        super().__init__("fields_node")
        self._captures_root = get_captures_root()

        self.declare_parameter("field_scale", 0.001)
        self.declare_parameter("seed_level", 1.0)
        self.declare_parameter("t_coef", 2000.0)
        self.declare_parameter("clearance", 0.0)
        self.declare_parameter("search_step_x", 0.01)
        self.declare_parameter("search_step_y", 0.01)
        self.declare_parameter("search_max_candidates", 20000)
        self.declare_parameter("positioning_attempts", 3)
        self.declare_parameter("base_z_offset", 1e-6)
        self.declare_parameter("state_filename", "field_state_init.npz")
        self.declare_parameter("debug_field_ply_filename", "field_masked_init.ply")
        self.declare_parameter("iso_level", 0.0)
        self.declare_parameter("default_cycle_output_dir", "@session/field_loop/cycle")
        self.declare_parameter("cycle_field_ply_filename", "field_masked_cycle.ply")
        self.declare_parameter("cycle_contour_ply_filename", "field_phi0_contour_cycle.ply")
        self.declare_parameter("cycle_candidates_ply_filename", "field_print_candidates_cycle.ply")
        self.declare_parameter("cycle_targets_yaml_filename", "targets.yaml")
        self.declare_parameter("beads_per_step", 7)
        self.declare_parameter("bead_separation_mm", 16.0)
        self.declare_parameter("bead_height_mm", 12.0)
        self.declare_parameter("walk_distance_mm", 12.0)
        self.declare_parameter("walk_step_mm", 1.0)
        self.declare_parameter("walk_max_steps", 32)
        self.declare_parameter("walk_start_fraction", 0.25)
        self.declare_parameter("candidate_mode_default", "z_lift")
        self.declare_parameter("orient_with_tangent_default", False)
        self.declare_parameter("tangent_sign_default", 1.0)
        self.declare_parameter("clamp_to_cone_default", False)
        self.declare_parameter("cone_max_tilt_deg_default", 40.0)
        self.declare_parameter("target_zx", 0.03)
        self.declare_parameter("target_zy", -0.01)
        self.declare_parameter("target_zz", 1.00)
        self.declare_parameter("target_position_scale", 1000.0)
        self.declare_parameter("target_base_to_world_yaw_deg", 180.0)

        self._srv_init = self.create_service(
            InitFieldFromScan,
            "/behav3d/init_field_from_scan",
            self._handle_init_field_from_scan,
        )
        self._srv_generate = self.create_service(
            GeneratePrintCandidates,
            "/behav3d/generate_print_candidates",
            self._handle_generate_print_candidates,
        )
        self.get_logger().info(
            "Fields node ready: /behav3d/init_field_from_scan, /behav3d/generate_print_candidates"
        )

    def _handle_init_field_from_scan(
        self,
        req: InitFieldFromScan.Request,
        res: InitFieldFromScan.Response,
    ) -> InitFieldFromScan.Response:
        if _SCALAR_IMPORT_ERROR is not None:
            res.success = False
            res.message = f"scalar_field import failed: {_SCALAR_IMPORT_ERROR}"
            return res

        try:
            session_dir = resolve_session_path(req.session_path, bool(req.use_latest), self._captures_root).resolve()
            res.session_dir = str(session_dir)
            if not session_dir.exists():
                raise FileNotFoundError(f"Session directory not found: {session_dir}")

            scan_paths: List[Path] = []
            for raw in list(req.scan_mesh_paths):
                p = _resolve_input_path(raw, session_dir, self._captures_root)
                if p is not None and p.is_file():
                    scan_paths.append(p)
            if not scan_paths:
                default_scan = _default_scan_mesh_path(session_dir)
                if default_scan is None:
                    raise FileNotFoundError("No scan mesh path provided and no default tsdf mesh found in session.")
                scan_paths = [default_scan]

            field_mesh_path = _resolve_input_path(req.field_mesh_path, session_dir, self._captures_root)
            if field_mesh_path is None or not field_mesh_path.is_file():
                raise FileNotFoundError(f"Field mesh not found: '{req.field_mesh_path}'")

            scan_mesh = _load_scan_mesh(scan_paths)
            field_mesh = load_triangle_mesh_arrays(field_mesh_path)
            field_vertices = np.asarray(field_mesh.vertices, dtype=np.float64)
            field_faces = np.asarray(field_mesh.faces, dtype=np.int32)

            seed_level = float(self.get_parameter("seed_level").value)
            t_coef = float(self.get_parameter("t_coef").value)
            heat_field = compute_heat_field(
                vertices=field_vertices,
                faces=field_faces,
                seed=None,
                seed_level=seed_level,
                t_coef=t_coef,
            )
            heat_dist = np.asarray(heat_field.dist, dtype=np.float64)
            heat_norm = np.asarray(heat_field.norm, dtype=np.float64)

            field_scale = float(self.get_parameter("field_scale").value)
            if field_scale <= 0.0:
                raise ValueError(f"field_scale must be > 0, got {field_scale}")
            field_vertices_scaled = field_vertices * field_scale

            position = position_field_with_attempts(
                scan_mesh=scan_mesh,
                field_vertices_scaled=field_vertices_scaled,
                heat_norm=heat_norm,
                clearance=float(self.get_parameter("clearance").value),
                iso_level=0.0,
                base_z_offset=float(self.get_parameter("base_z_offset").value),
                search_step_x=float(self.get_parameter("search_step_x").value),
                search_step_y=float(self.get_parameter("search_step_y").value),
                positioning_attempts=int(self.get_parameter("positioning_attempts").value),
                search_max_candidates=int(self.get_parameter("search_max_candidates").value),
            )
            viable = np.asarray(position.viable, dtype=bool)

            out_dir = _resolve_output_dir(
                req.state_output_dir,
                session_dir,
                self._captures_root,
                default_value="@session/field_loop/init",
            )
            out_dir.mkdir(parents=True, exist_ok=True)
            state_path = out_dir / str(self.get_parameter("state_filename").value)
            debug_field_path = out_dir / str(self.get_parameter("debug_field_ply_filename").value)

            np.savez_compressed(
                state_path,
                field_vertices=field_vertices,
                field_faces=field_faces,
                field_vertices_scaled=field_vertices_scaled,
                field_vertices_world=position.field_vertices_world,
                heat_dist=heat_dist,
                heat_norm=heat_norm,
                phi=position.phi,
                viable=viable.astype(np.uint8),
                offset_xyz=np.asarray(position.offset_xyz, dtype=np.float64),
                field_scale=np.asarray([field_scale], dtype=np.float64),
                seed_level=np.asarray([seed_level], dtype=np.float64),
                t_coef=np.asarray([t_coef], dtype=np.float64),
                clearance=np.asarray([float(self.get_parameter("clearance").value)], dtype=np.float64),
            )

            colors = yellow_to_red_colors(heat_norm)
            colors[~viable] = np.array([0.2, 0.2, 0.2], dtype=np.float64)
            pcd = make_point_cloud(position.field_vertices_world, colors)
            if not o3d.io.write_point_cloud(str(debug_field_path), pcd):
                raise RuntimeError(f"Failed to write debug field PLY: {debug_field_path}")

            res.success = True
            res.message = (
                f"Field initialized. scan_meshes={len(scan_paths)} "
                f"field_v={field_vertices.shape[0]} field_f={field_faces.shape[0]} "
                f"viable={int(np.count_nonzero(viable))}"
            )
            res.resolved_scan_mesh_path = str(scan_paths[0])
            res.resolved_field_mesh_path = str(field_mesh_path)
            res.field_state_path = str(state_path)
            res.debug_field_ply_path = str(debug_field_path)
            res.offset_x = float(position.offset_xyz[0])
            res.offset_y = float(position.offset_xyz[1])
            res.offset_z = float(position.offset_xyz[2])
            return res
        except Exception as exc:
            res.success = False
            res.message = str(exc)
            return res

    def _handle_generate_print_candidates(
        self,
        req: GeneratePrintCandidates.Request,
        res: GeneratePrintCandidates.Response,
    ) -> GeneratePrintCandidates.Response:
        if _SCALAR_IMPORT_ERROR is not None:
            res.success = False
            res.message = f"scalar_field import failed: {_SCALAR_IMPORT_ERROR}"
            return res

        try:
            session_dir = resolve_session_path(req.session_path, bool(req.use_latest), self._captures_root).resolve()
            res.session_dir = str(session_dir)
            if not session_dir.exists():
                raise FileNotFoundError(f"Session directory not found: {session_dir}")

            scan_paths: List[Path] = []
            for raw in list(req.scan_mesh_paths):
                p = _resolve_input_path(raw, session_dir, self._captures_root)
                if p is not None and p.is_file():
                    scan_paths.append(p)
            if not scan_paths:
                default_scan = _default_scan_mesh_path(session_dir)
                if default_scan is None:
                    raise FileNotFoundError("No scan mesh path provided and no default tsdf mesh found in session.")
                scan_paths = [default_scan]

            field_state_path = _resolve_input_path(req.field_state_path, session_dir, self._captures_root)
            if field_state_path is None or not field_state_path.is_file():
                default_state = _default_field_state_path(session_dir)
                if default_state is None:
                    raise FileNotFoundError(
                        "Field state not found. Provide field_state_path or run init_field_from_scan first."
                    )
                field_state_path = default_state

            scan_mesh = _load_scan_mesh(scan_paths)
            state = np.load(str(field_state_path), allow_pickle=False)
            if "field_faces" not in state:
                raise KeyError(f"Missing 'field_faces' in field state: {field_state_path}")
            if "heat_norm" not in state:
                raise KeyError(f"Missing 'heat_norm' in field state: {field_state_path}")
            if "offset_xyz" not in state:
                raise KeyError(f"Missing 'offset_xyz' in field state: {field_state_path}")

            if "field_vertices_scaled" in state:
                field_vertices_scaled = np.asarray(state["field_vertices_scaled"], dtype=np.float64)
            elif "field_vertices" in state:
                field_vertices = np.asarray(state["field_vertices"], dtype=np.float64)
                scale_arr = np.asarray(state["field_scale"], dtype=np.float64).reshape(-1) if "field_scale" in state else np.array([1.0], dtype=np.float64)
                field_vertices_scaled = field_vertices * float(scale_arr[0])
            else:
                raise KeyError(
                    f"Missing 'field_vertices_scaled' (or fallback 'field_vertices') in field state: {field_state_path}"
                )

            field_faces = np.asarray(state["field_faces"], dtype=np.int32)
            heat_norm = np.asarray(state["heat_norm"], dtype=np.float64).reshape(-1)
            if field_vertices_scaled.ndim != 2 or field_vertices_scaled.shape[1] != 3:
                raise ValueError(f"field_vertices_scaled has invalid shape: {field_vertices_scaled.shape}")
            if field_faces.ndim != 2 or field_faces.shape[1] != 3:
                raise ValueError(f"field_faces has invalid shape: {field_faces.shape}")
            if heat_norm.shape[0] != field_vertices_scaled.shape[0]:
                raise ValueError("heat_norm length does not match field vertices.")

            offset_arr = np.asarray(state["offset_xyz"], dtype=np.float64).reshape(-1)
            if offset_arr.size < 3:
                raise ValueError(f"offset_xyz has invalid shape: {offset_arr.shape}")
            offset_xyz = (float(offset_arr[0]), float(offset_arr[1]), float(offset_arr[2]))

            if "clearance" in state:
                clearance_arr = np.asarray(state["clearance"], dtype=np.float64).reshape(-1)
                clearance_used = float(clearance_arr[0]) if clearance_arr.size > 0 else float(
                    self.get_parameter("clearance").value
                )
            else:
                clearance_used = float(self.get_parameter("clearance").value)
            iso_level = float(self.get_parameter("iso_level").value)

            scene, z_top = make_scan_scene(scan_mesh)
            pose = evaluate_fixed_pose(
                scene=scene,
                z_top=z_top,
                field_vertices_scaled=field_vertices_scaled,
                offset_xyz=offset_xyz,
                clearance=clearance_used,
                iso_level=iso_level,
            )

            contour_points, contour_lines = extract_phi_contour(
                vertices=pose.field_vertices_world,
                faces=field_faces,
                scalar=pose.phi,
                iso=iso_level,
            )

            beads_per_step_used = int(req.beads_per_step)
            if beads_per_step_used <= 0:
                beads_per_step_used = int(self.get_parameter("beads_per_step").value)
            bead_separation_mm_used = float(req.bead_separation_mm)
            if bead_separation_mm_used <= 0.0:
                bead_separation_mm_used = float(self.get_parameter("bead_separation_mm").value)
            bead_height_mm_used = float(req.bead_height_mm)
            if bead_height_mm_used <= 0.0:
                bead_height_mm_used = float(self.get_parameter("bead_height_mm").value)
            walk_distance_mm_used = float(
                getattr(req, "walk_distance_mm", float(self.get_parameter("walk_distance_mm").value))
            )
            if not np.isfinite(walk_distance_mm_used) or walk_distance_mm_used <= 0.0:
                walk_distance_mm_used = float(self.get_parameter("walk_distance_mm").value)
            walk_step_mm_used = float(
                getattr(req, "walk_step_mm", float(self.get_parameter("walk_step_mm").value))
            )
            if not np.isfinite(walk_step_mm_used) or walk_step_mm_used <= 0.0:
                walk_step_mm_used = float(self.get_parameter("walk_step_mm").value)
            walk_max_steps_used = int(
                getattr(req, "walk_max_steps", int(self.get_parameter("walk_max_steps").value))
            )
            if walk_max_steps_used <= 0:
                walk_max_steps_used = int(self.get_parameter("walk_max_steps").value)
            walk_start_fraction_used = float(
                getattr(req, "walk_start_fraction", float(self.get_parameter("walk_start_fraction").value))
            )
            if not np.isfinite(walk_start_fraction_used):
                walk_start_fraction_used = float(self.get_parameter("walk_start_fraction").value)

            if bead_separation_mm_used <= 0.0:
                raise ValueError(f"bead_separation_mm must be > 0, got {bead_separation_mm_used}")
            if bead_height_mm_used < 0.0:
                raise ValueError(f"bead_height_mm must be >= 0, got {bead_height_mm_used}")
            if beads_per_step_used < 0:
                raise ValueError(f"beads_per_step must be >= 0, got {beads_per_step_used}")

            candidate_mode = str(req.candidate_mode).strip().lower()
            if not candidate_mode:
                candidate_mode = str(self.get_parameter("candidate_mode_default").value).strip().lower() or "z_lift"
            if candidate_mode not in ("z_lift", "gradient_lift", "gradient_walk"):
                raise ValueError(
                    f"Unsupported candidate_mode: '{candidate_mode}'. "
                    "Use 'z_lift', 'gradient_lift', or 'gradient_walk'."
                )

            orient_with_tangent = bool(req.orient_with_tangent)
            if not orient_with_tangent:
                orient_with_tangent = bool(self.get_parameter("orient_with_tangent_default").value)
            if candidate_mode == "gradient_walk":
                orient_with_tangent = True

            tangent_sign = float(req.tangent_sign)
            if not np.isfinite(tangent_sign) or abs(tangent_sign) < 1e-9:
                tangent_sign = float(self.get_parameter("tangent_sign_default").value)

            clamp_to_cone = bool(req.clamp_to_cone)
            if not clamp_to_cone:
                clamp_to_cone = bool(self.get_parameter("clamp_to_cone_default").value)

            cone_max_tilt_deg = float(req.cone_max_tilt_deg)
            if not np.isfinite(cone_max_tilt_deg) or cone_max_tilt_deg <= 0.0:
                cone_max_tilt_deg = float(self.get_parameter("cone_max_tilt_deg_default").value)

            targets_zx = float(req.target_zx)
            targets_zy = float(req.target_zy)
            targets_zz = float(req.target_zz)
            if not (np.isfinite(targets_zx) and np.isfinite(targets_zy) and np.isfinite(targets_zz)):
                raise ValueError("target_z direction must be finite values.")
            target_position_scale = float(req.target_position_scale)
            if target_position_scale <= 0.0:
                target_position_scale = float(self.get_parameter("target_position_scale").value)
            target_base_to_world_yaw_deg = float(req.base_to_world_yaw_deg)
            if not np.isfinite(target_base_to_world_yaw_deg):
                target_base_to_world_yaw_deg = float(self.get_parameter("target_base_to_world_yaw_deg").value)
            if (
                abs(target_base_to_world_yaw_deg) < 1e-12
                and abs(float(self.get_parameter("target_base_to_world_yaw_deg").value)) > 1e-12
            ):
                target_base_to_world_yaw_deg = float(self.get_parameter("target_base_to_world_yaw_deg").value)
            if not np.isfinite(target_base_to_world_yaw_deg):
                raise ValueError("target_base_to_world_yaw_deg must be a finite value.")

            candidates = generate_print_points(
                polyline_points=contour_points,
                polyline_lines=contour_lines,
                field_vertices_world=pose.field_vertices_world,
                field_scalar=heat_norm,
                count=beads_per_step_used,
                min_spacing=1e-3 * bead_separation_mm_used,
                point_valid_mask=None,
                extra_points=None,
                candidate_mode=candidate_mode,
                lift_height=1e-3 * bead_height_mm_used,
                field_faces=field_faces,
                walk_distance=1e-3 * walk_distance_mm_used,
                walk_step=1e-3 * walk_step_mm_used,
                walk_max_steps=walk_max_steps_used,
                walk_tangent_sign=tangent_sign,
                walk_start_fraction=walk_start_fraction_used,
                clamp_to_cone=clamp_to_cone,
                cone_max_tilt_deg=cone_max_tilt_deg,
                agent_phi_scalar=pose.phi,
            )
            candidate_points = candidates.points

            out_dir = _resolve_output_dir(
                req.output_dir,
                session_dir,
                self._captures_root,
                default_value=str(self.get_parameter("default_cycle_output_dir").value),
            )
            out_dir.mkdir(parents=True, exist_ok=True)

            debug_field_path = out_dir / str(self.get_parameter("cycle_field_ply_filename").value)
            debug_contour_path = out_dir / str(self.get_parameter("cycle_contour_ply_filename").value)
            debug_candidates_path = out_dir / str(self.get_parameter("cycle_candidates_ply_filename").value)
            targets_yaml_path = out_dir / str(self.get_parameter("cycle_targets_yaml_filename").value)

            colors = yellow_to_red_colors(heat_norm)
            colors = np.asarray(colors, dtype=np.float64)
            colors[~pose.viable] = np.array([0.2, 0.2, 0.2], dtype=np.float64)
            debug_field_pcd = make_point_cloud(pose.field_vertices_world, colors)
            if not o3d.io.write_point_cloud(str(debug_field_path), debug_field_pcd):
                raise RuntimeError(f"Failed to write debug field PLY: {debug_field_path}")

            contour_written = ""
            if contour_lines.shape[0] > 0:
                contour_ls = make_line_set(contour_points, contour_lines, color=(0.0, 1.0, 1.0))
                if not o3d.io.write_line_set(str(debug_contour_path), contour_ls):
                    raise RuntimeError(f"Failed to write contour PLY: {debug_contour_path}")
                contour_written = str(debug_contour_path)

            candidate_written = ""
            if candidate_points.shape[0] > 0:
                candidate_colors = np.tile(
                    np.array([1.0, 0.0, 1.0], dtype=np.float64),
                    (candidate_points.shape[0], 1),
                )
                candidates_pcd = make_point_cloud(candidate_points, candidate_colors)
                if not o3d.io.write_point_cloud(str(debug_candidates_path), candidates_pcd):
                    raise RuntimeError(f"Failed to write candidates PLY: {debug_candidates_path}")
                candidate_written = str(debug_candidates_path)

            if candidate_mode == "gradient_walk":
                line_targets = build_oriented_line_targets(
                    start_points=np.asarray(candidates.segment_start_points, dtype=np.float64),
                    end_points=np.asarray(candidate_points, dtype=np.float64),
                    field_vertices_world=pose.field_vertices_world,
                    field_faces=field_faces,
                    field_scalar=heat_norm,
                    tangent_sign=float(tangent_sign),
                    clamp_to_cone=bool(clamp_to_cone),
                    cone_max_tilt_deg=float(cone_max_tilt_deg),
                )
                write_line_targets_yaml(
                    out_yaml=targets_yaml_path,
                    targets=line_targets,
                    position_scale=target_position_scale,
                    base_to_world_yaw_deg=target_base_to_world_yaw_deg,
                )
            elif orient_with_tangent and candidate_points.shape[0] > 0:
                surface_points = np.asarray(candidates.surface_points, dtype=np.float64)
                if surface_points.shape != candidate_points.shape:
                    surface_points = np.asarray(candidate_points, dtype=np.float64)
                tangent_dir, _binormal_dir, _normal_dir = sample_tangent_axes_on_surface_from_scalar(
                    query_points=surface_points,
                    mesh_vertices=pose.field_vertices_world,
                    mesh_faces=field_faces,
                    vertex_scalar=heat_norm,
                    tangent_sign=float(tangent_sign),
                )
                z_dirs_base = np.asarray(tangent_dir, dtype=np.float64)
                if clamp_to_cone:
                    z_dirs_base = clamp_vectors_to_cone(
                        z_dirs_base,
                        max_tilt_deg=float(cone_max_tilt_deg),
                        cone_axis=(0.0, 0.0, 1.0),
                    )
                write_point_targets_yaml(
                    out_yaml=targets_yaml_path,
                    points_world=np.asarray(candidate_points, dtype=np.float64),
                    z_dirs_world=np.asarray(z_dirs_base, dtype=np.float64),
                    position_scale=target_position_scale,
                    base_to_world_yaw_deg=target_base_to_world_yaw_deg,
                )
            else:
                write_fixed_z_targets_yaml(
                    out_yaml=targets_yaml_path,
                    points_world=np.asarray(candidate_points, dtype=np.float64),
                    z_dir=(targets_zx, targets_zy, targets_zz),
                    position_scale=target_position_scale,
                    base_to_world_yaw_deg=target_base_to_world_yaw_deg,
                )

            res.success = True
            res.message = (
                f"Generated candidates. mode={candidate_mode} "
                f"orient_with_tangent={orient_with_tangent} "
                f"clamp_to_cone={clamp_to_cone} scan_meshes={len(scan_paths)} "
                f"viable={int(np.count_nonzero(pose.viable))} "
                f"contour_segments={int(contour_lines.shape[0])} "
                f"candidates={int(candidate_points.shape[0])} "
                f"walk_distance_mm={walk_distance_mm_used:.3f}"
            )
            res.resolved_field_state_path = str(field_state_path)
            res.resolved_scan_mesh_path = str(scan_paths[0])
            res.cycle_output_dir = str(out_dir)
            res.debug_field_ply_path = str(debug_field_path)
            res.debug_contour_ply_path = contour_written
            res.debug_candidates_ply_path = candidate_written
            res.targets_yaml_path = str(targets_yaml_path)
            res.scan_mesh_count = int(len(scan_paths))
            res.viable_count = int(np.count_nonzero(pose.viable))
            res.contour_segment_count = int(contour_lines.shape[0])
            res.candidate_count = int(candidate_points.shape[0])
            res.clearance_used = float(clearance_used)
            res.iso_level_used = float(iso_level)
            res.beads_per_step_used = int(beads_per_step_used)
            res.bead_separation_mm_used = float(bead_separation_mm_used)
            res.bead_height_mm_used = float(bead_height_mm_used)
            return res
        except Exception as exc:
            res.success = False
            res.message = str(exc)
            return res


def main(args=None):
    rclpy.init(args=args)
    node = FieldsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
