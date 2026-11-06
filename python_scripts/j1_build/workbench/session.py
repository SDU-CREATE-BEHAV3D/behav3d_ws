"""Persistent loop state. The scalar/candidate/rule implementations are imported unchanged."""
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
import hashlib
import json
import tempfile
from time import perf_counter
import uuid

import numpy as np
import open3d as o3d
from dds import BeadProfile, Domain, Simulator
from dds.geometry import TriangleMesh, write_mesh

from field_scan_loop_simulator_dds_v2 import (
    build_dds_domain_from_print_zone, build_dds_step_deposits,
    dds_surface_to_open3d_mesh, load_field_state_npz, load_segment_targets_yaml,
    rotate_mesh_about_z_origin, validate_deposits_inside_domain,
)
from lib_scalar.compute_phi_mask import evaluate_fixed_pose, make_scan_scene
from lib_scalar.extract_phi_contour import extract_phi_contour
from lib_scalar.geometry import load_triangle_mesh_legacy
from lib_scalar.loop_simulation import ContourStage, compute_offset_contour_stage, generate_step_candidates
from lib_scalar.print_targets import build_candidate_segment_targets, write_line_targets_yaml
from lib_scalar.target_rules import apply_secondary_target_rules
from .config import Parameters, SimulationOptions
from .telemetry import event


@dataclass(frozen=True)
class Preview:
    token: str
    revision: int
    parameters: Parameters
    pose: object
    contour: object
    candidates: object
    targets: object
    rules: dict
    sources: np.ndarray
    timings: dict
    geometry_cached: bool
    phantom_surface: object = None


class Session:
    def __init__(self, data_root: Path, output_root: Path, options=SimulationOptions(),
                 field_path=None, scan_path=None, scan_scale=0.001, scan_yaw=180.0):
        options.validate()
        self.options = options
        self.field_path = Path(field_path or data_root / "mesh/fields/field_state_init_subdiv1.npz")
        self.scan_path = Path(scan_path or data_root / "mesh/ScanMesh.stl")
        self.field = load_field_state_npz(self.field_path)
        self.base = load_triangle_mesh_legacy(self.scan_path)
        self.base.scale(scan_scale, center=(0, 0, 0))
        rotate_mesh_about_z_origin(self.base, scan_yaw)
        self.current_scan = self.base
        self.surface = TriangleMesh.empty()
        self.simulator = self.result = self.preview = None
        self.revision = 0
        self.history = []
        self.last_targets = np.empty((0, 3))
        self.geometry_cache = {}
        self.diagnostic_cache = {}
        self.phantom_cache = None
        self.geometry_builds = 0
        self.output = output_root / (datetime.now().strftime("session-%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6])
        self.output.mkdir(parents=True)
        (self.output / "session.json").write_text(json.dumps({
            "field": str(self.field_path), "field_sha256": hashlib.sha256(self.field_path.read_bytes()).hexdigest(),
            "scan": str(self.scan_path), "scan_sha256": hashlib.sha256(self.scan_path.read_bytes()).hexdigest(),
            "scan_scale": scan_scale, "scan_yaw": scan_yaw, "simulation": asdict(options),
        }, indent=2), encoding="utf-8")

    def prepare(self, parameters: Parameters, include_phantom=False) -> Preview:
        parameters.validate()
        start = perf_counter()
        key = (self.revision, parameters.clearance_mm)
        cached = key in self.geometry_cache
        event(self.output, "geometry", "started", cycle=self.revision, cached=cached)
        if not cached:
            scene, ztop = make_scan_scene(self.current_scan)
            pose = evaluate_fixed_pose(scene, ztop, self.field.field_vertices_scaled,
                                       self.field.offset_xyz, parameters.clearance_mm * 0.001, 0.0)
            points, lines = extract_phi_contour(pose.field_vertices_world, self.field.field_faces, pose.phi, iso=0)
            # The unused magenta diagnostic is lazy; the cyan contour is identical.
            contour = ContourStage(points, lines, np.empty((0, 3)), np.empty((0, 2), np.int32), 0.012, np.zeros(0, bool))
            self.geometry_cache = {key: (pose, contour)}
            self.geometry_builds += 1
        pose, contour = self.geometry_cache[key]
        geometry_seconds = perf_counter() - start
        event(self.output, "geometry", "finished", cycle=self.revision,
              seconds=geometry_seconds, cached=cached)
        candidate_start = perf_counter()
        event(self.output, "candidates", "started", cycle=self.revision)
        c = generate_step_candidates(
            contour, self.field.field_faces, pose.field_vertices_world, self.field.heat_norm, pose.phi,
            mode=parameters.mode, beads_per_step=parameters.count,
            bead_separation_mm=parameters.separation_mm, bead_height_mm=parameters.height_mm,
            walk_distance_mm=parameters.walk_distance_mm, walk_step_mm=parameters.walk_step_mm,
            walk_max_steps=parameters.walk_max_steps, walk_tangent_sign=parameters.tangent_sign,
            walk_start_fraction=parameters.walk_start_fraction, clamp_to_cone=parameters.clamp_to_cone,
            cone_max_tilt_deg=parameters.cone_deg, contour_sample_spacing_mm=parameters.source_spacing_mm,
        )
        targets = build_candidate_segment_targets(
            candidate_points=c.points, segment_start_points=c.segment_start_points,
            candidate_mode=parameters.mode, field_vertices_world=pose.field_vertices_world,
            field_faces=self.field.field_faces, field_scalar=self.field.heat_norm,
            tangent_sign=parameters.tangent_sign, clamp_to_cone=parameters.clamp_to_cone,
            cone_max_tilt_deg=parameters.cone_deg,
        )
        targets, rules = apply_secondary_target_rules(
            targets, candidate_mode=parameters.mode, bead_height_m=parameters.height_mm * 0.001,
            bead_separation_m=parameters.separation_mm * 0.001,
            normal_continuity_rule=parameters.normal_continuity,
        )
        keep = np.asarray(rules["kept_indices"], dtype=int)
        preview = Preview(uuid.uuid4().hex, self.revision, parameters, pose, contour, c, targets, rules,
                          c.source_points[keep], {"geometry_s": geometry_seconds,
                          "candidates_s": perf_counter() - candidate_start,
                          "preview_s": perf_counter() - start}, cached)
        self.preview = preview
        event(self.output, "candidates", "finished", cycle=self.revision,
              seconds=preview.timings["candidates_s"], targets=targets.count)
        self.phantom_cache = None
        if include_phantom:
            phantom_start = perf_counter()
            event(self.output, "phantom", "started", cycle=self.revision)
            surface = self.preview_phantom(preview.token)
            from dataclasses import replace
            preview = replace(preview, phantom_surface=surface,
                timings={**preview.timings, "phantom_s": perf_counter() - phantom_start,
                         "preview_s": perf_counter() - start})
            self.preview = preview
            event(self.output, "phantom", "finished", cycle=self.revision,
                  seconds=preview.timings["phantom_s"])
        return preview

    def _batch_deposits(self, preview, path):
        p = preview.parameters
        write_line_targets_yaml(path, preview.targets, position_scale=1000.0)
        applied = load_segment_targets_yaml(path, position_scale=1000.0)
        deposits = build_dds_step_deposits(applied,
            profiles=tuple(BeadProfile(width=p.width_mm * 0.001, height=p.height_mm * 0.001)
                           for _ in range(applied.count)), mode=p.deposit_mode, line_fraction=p.line_fraction)
        return applied, deposits

    def preview_phantom(self, token):
        """Simulate only the proposed batch in a cropped, lattice-aligned domain."""
        preview = self.preview
        if preview is None or preview.token != token or preview.revision != self.revision:
            raise ValueError("This preview is stale; regenerate it before rendering the phantom.")
        if self.phantom_cache is not None and self.phantom_cache[0] == token:
            return self.phantom_cache[1]
        if not preview.targets.count:
            return TriangleMesh.empty()
        cache = Path(__file__).parent / ".cache/phantom"
        cache.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=cache) as directory:
            _, deposits = self._batch_deposits(preview, Path(directory) / "segments.yaml")
        simulation, _ = self._new_simulator(preview, deposits)
        domain = simulation.domain
        voxel = np.asarray(domain.voxel_size)
        origin = np.asarray(domain.min_corner)
        bounds = [d.support_bounds(padding=2 * max(voxel)) for d in deposits]
        low = np.min([a.to_tuple() for a, _ in bounds], axis=0)
        high = np.max([b.to_tuple() for _, b in bounds], axis=0)
        stride = self.options.surface_step
        start = np.maximum(0, np.floor((low - origin) / voxel / stride).astype(int) * stride)
        stop = np.minimum(domain.grid_shape, np.ceil((high - origin) / voxel / stride).astype(int) * stride)
        cropped = Domain(min_corner=tuple(origin + start * voxel),
            max_corner=tuple(origin + stop * voxel), voxel_size=domain.voxel_size,
            grid_shape=tuple(int(v) for v in stop - start), length_unit="m")
        ghost = Simulator(cropped, deposits).result(include_coverage=False, threshold=self.options.threshold)
        surface = ghost.analysis.surface_mesh(threshold=self.options.threshold, step_size=stride)
        self.phantom_cache = (token, surface)
        return surface

    def offset_diagnostic(self):
        if self.preview is None:
            raise ValueError("Prepare candidates first")
        key = (self.revision, self.preview.parameters.clearance_mm)
        if key not in self.diagnostic_cache:
            self.diagnostic_cache = {key: compute_offset_contour_stage(
                self.preview.pose, self.field.field_faces, iso_level=0, offset_distance_mm=12,
                offset_geodesic_delta_mm=0.6, offset_t_coef=2000, scan_mesh=self.current_scan)}
        return self.diagnostic_cache[key]

    def domain_for_display(self):
        """Return the accepted domain, or initial planned bounds before acceptance."""
        if self.simulator is not None:
            return self.simulator.domain
        if self.preview is not None:
            return self._initial_domain(self.preview)
        return None

    def _initial_domain(self, preview):
        p = preview.parameters
        return build_dds_domain_from_print_zone(
            preview.pose.field_vertices_world, preview.targets.end_points,
            source="field", scan_mesh=None, voxel_size_m=self.options.voxel_mm * 0.001,
            padding_m=max(self.options.padding_mm, 12, p.width_mm, p.height_mm) * 0.001)

    def _new_simulator(self, preview, deposits):
        domain = self._initial_domain(preview) if self.simulator is None else self.simulator.domain
        try:
            validate_deposits_inside_domain(deposits, domain)
        except RuntimeError:
            # Expand on the existing lattice, replaying only when actually necessary.
            minimum, maximum = np.array(domain.min_corner), np.array(domain.max_corner)
            lo, hi = minimum.copy(), maximum.copy()
            for deposit in deposits:
                a, b = deposit.support_bounds(padding=self.options.voxel_mm * 0.001)
                lo = np.minimum(lo, a.to_tuple())
                hi = np.maximum(hi, b.to_tuple())
            voxel = np.asarray(domain.voxel_size)
            lo = minimum + np.floor((lo - minimum) / voxel) * voxel
            hi = maximum + np.ceil((hi - maximum) / voxel) * voxel
            domain = Domain.from_bounds(xmin=lo[0], xmax=hi[0], ymin=lo[1], ymax=hi[1],
                                        zmin=lo[2], zmax=hi[2], voxel_size=tuple(voxel), length_unit="m")
            validate_deposits_inside_domain(deposits, domain)
        if np.prod(domain.grid_shape, dtype=np.int64) > self.options.max_voxels:
            raise ValueError("Requested domain is too large. Start a session with larger voxels.")
        if self.simulator is None or domain != self.simulator.domain:
            return Simulator(domain, self.simulator.deposits if self.simulator else None), True
        return self.simulator, False

    def accept(self, token: str):
        preview = self.preview
        if preview is None or preview.token != token or preview.revision != self.revision:
            raise ValueError("This preview is stale; regenerate it before accepting.")
        if not preview.targets.count:
            raise ValueError("No valid candidates to accept")
        p = preview.parameters
        start = perf_counter()
        step_dir = self.output / f"step_{self.revision + 1:04d}"
        step_dir.mkdir(exist_ok=True)
        pending = step_dir / "segments.pending.yaml"
        # Preserve exactly the original YAML quantization and normal handling.
        applied, deposits = self._batch_deposits(preview, pending)
        old_domain = self.simulator.domain if self.simulator is not None else None
        simulation, rebuilt = self._new_simulator(preview, deposits)
        old_deposits = simulation.deposits
        deposit_start = perf_counter()
        try:
            simulation.add_deposits(deposits)
            result = simulation.result(include_coverage=False, threshold=self.options.threshold)
            deposit_seconds = perf_counter() - deposit_start
            surface_start = perf_counter()
            surface = result.analysis.surface_mesh(threshold=self.options.threshold, step_size=self.options.surface_step)
            current_scan = self.base + dds_surface_to_open3d_mesh(surface)
        except Exception:
            if simulation is self.simulator:
                self.simulator = Simulator(simulation.domain, old_deposits)
            raise
        surface_seconds = perf_counter() - surface_start
        self.simulator, self.result, self.surface, self.current_scan = simulation, result, surface, current_scan
        self.revision += 1
        self.last_targets = applied.end_points.copy()
        record = {"cycle": self.revision, "parameters": asdict(p), "targets": applied.count,
                  "deposits": len(simulation.deposits), "preview": preview.token,
                  "geometry_cached": preview.geometry_cached, "domain_rebuilt": rebuilt,
                  "domain_change": "initialized" if old_domain is None else "expanded" if rebuilt else "unchanged",
                  "domain": {"min_corner": simulation.domain.min_corner,
                             "max_corner": simulation.domain.max_corner,
                             "grid_shape": simulation.domain.grid_shape,
                             "expanded_sides": [f"{axis}{side}" for i, axis in enumerate("XYZ")
                                 for side, changed in (("-", old_domain is not None and simulation.domain.min_corner[i] < old_domain.min_corner[i]),
                                                       ("+", old_domain is not None and simulation.domain.max_corner[i] > old_domain.max_corner[i]))
                                 if changed]},
                  "timings": {**preview.timings, "deposit_s": deposit_seconds,
                              "surface_s": surface_seconds, "accept_s": perf_counter() - start}}
        pending.replace(step_dir / "segments.yaml")
        (step_dir / "parameters.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
        with (self.output / "history.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record) + "\n")
        self.history.append(record)
        self.geometry_cache.clear()
        self.diagnostic_cache.clear()
        self.preview = None
        self.phantom_cache = None
        if self.options.checkpoint_every and self.revision % self.options.checkpoint_every == 0:
            checkpoint_start = perf_counter()
            event(self.output, "checkpoint", "started", cycle=self.revision)
            self.result.checkpoint(self.output / f"checkpoint-{self.revision:04d}.npz")
            event(self.output, "checkpoint", "finished", cycle=self.revision,
                  seconds=perf_counter() - checkpoint_start)
        return record

    def export(self):
        directory = self.output / f"export-{self.revision:04d}"
        directory.mkdir(exist_ok=True)
        if not o3d.io.write_triangle_mesh(str(directory / "scan_with_dds_proxy.ply"), self.current_scan):
            raise OSError("Could not export scan mesh")
        if self.result is not None:
            self.result.checkpoint(directory / "simulation.npz")
            if not self.surface.is_empty:
                write_mesh(directory / "dds_surface.ply", self.surface)
        (directory / "history.json").write_text(json.dumps(self.history, indent=2), encoding="utf-8")
        return directory
