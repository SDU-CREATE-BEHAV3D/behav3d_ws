# lib_scalar

Reusable library for the scalar-field pipeline used in `behav3d_py/scalar_field`.

The objective is to keep each stage isolated, testable, and reusable from:
- standalone scripts,
- ROS nodes/services,
- future planning and print-path modules.

## Pipeline Stages

1. `compute_heat_field.py`
   - Builds the base scalar preference over the field mesh using `potpourri3d`.
   - This stage does not decide if a point is printable; it only provides scalar intensity.
   - Main API:
     - `compute_heat_field(vertices, faces, seed, seed_level, t_coef) -> HeatField`

2. `position_field.py`
   - Searches XY placement of the field mesh over the scan geometry.
   - For each XY candidate `(ox, oy)`, solves feasible Z using local base vertices:
     - identify vertices at the field base level,
     - require those base vertices to hit the scan,
     - `z_offset = min(z_scan_base_hit - base_local_z_hit) - base_z_offset`
   - This makes `base_z_offset` the minimum base penetration:
     - every base vertex is at least `base_z_offset` below its scan hit.
   - Then computes viability from phi:
     - `phi = z_field - z_scan - clearance`
   - Ranking inside this stage is local to pose search; a global loop-level score can be optimized outside.
   - Main API:
     - `position_field(...) -> PoseResult`
   - Utilities:
     - `default_xy_search_bounds(...)`
     - `make_axis_samples(...)`

3. `compute_phi_mask.py`
   - Builds the raycasting scene from the scan mesh.
   - Evaluates signed clearance (`phi`) and viability mask for a placed field.
   - Current model is z-up based:
     - `phi = z_field - z_scan - clearance`
   - Main APIs:
     - `make_scan_scene(scan_mesh)`
     - `compute_phi_mask(scene, z_top, field_vertices_world, offset_xyz, clearance, iso_level)`
     - `evaluate_fixed_pose(...)` (manual pose helper)

4. `extract_phi_contour.py`
   - Contour-centric module for `phi` boundaries and geodesic offsets.
   - Extracts the 3D boundary `phi = iso` by edge interpolation over field triangles.
   - Can also build geodesic offset curves from that boundary.
   - Typical use: `iso = 0` for viable/non-viable boundary.
   - Main APIs:
     - `extract_phi_contour(vertices, faces, scalar, iso=0.0)`
     - `extract_offset_phi_contour(vertices, faces, phi, iso_level, offset_distance, ...)`
     - `extract_phi_contour_with_offset(vertices, faces, phi, iso_level, offset_distance, ...)`
     - `contour_seed_vertices_from_phi(...)`
     - `compute_geodesic_from_phi_contour(...)`

5. `viz.py`
   - Converts numeric outputs to Open3D visualization objects.
   - Main APIs:
     - `yellow_to_red_colors(norm_scalar)`
     - `make_point_cloud(points, colors)`
     - `make_line_set(points, lines, color)`
     - `make_segment_line_set(start_points, end_points, color)`
     - `make_target_orientation_sticks(points, z_dirs)`
     - `compute_scene_bounds(...)`

6. `geometry.py`
   - Geometry loading and preparation utilities.
   - Ensures triangle meshes are valid and indexing is compact.
   - Includes scalar sampling on mesh surface using triangle interpolation.
   - Main APIs:
     - `load_triangle_mesh_arrays(path) -> MeshData`
     - `load_triangle_mesh_legacy(path)`
     - `compact_triangle_mesh(vertices, faces)`
     - `apply_scale_and_offset(...)`
     - `sample_vertex_scalar_on_surface(query_points, mesh_vertices, mesh_faces, vertex_scalar)`
     - `clamp_vectors_to_cone(vectors, max_tilt_deg, cone_axis=(0,0,1))`

7. `types.py`
   - Shared dataclasses used as contracts across stages.
   - `MeshData`: compacted mesh arrays + number of dropped vertices.
   - `HeatField`: raw and normalized scalar + summary stats + seed metadata.
   - `PoseResult`: positioned field, phi values, viability mask, ray-hit stats, search stats.
   - `PrintPointSet`: selected print points and spacing/selection metadata.

8. `generate_print_points.py`
   - Unified print-point generator.
   - Base selection always runs on the provided polyline graph:
     - pick the polyline vertex with minimum scalar value,
     - suppress nearby polyline vertices within minimum spacing,
     - repeat until requested count is reached.
   - Supports optional post-processing through `candidate_mode`:
     - `polyline`: keep selected points on the source polyline.
     - `z_lift`: apply vertical offset (`lift_height`) in +Z.
     - `gradient_lift`: displace selected points by `lift_height` along
       local tangent gradient direction.
     - `gradient_walk`: walk over field surface tangentially to scalar gradient
       until euclidean displacement reaches `walk_distance`.
       - Delegates the walk to `agent_walk.py`.
       - Keeps source, projected print-line start, and final points in the
         returned `PrintPointSet`.
       - `walk_start_fraction` controls the normalized `source -> final`
         interpolation used for the projected print-line start (`0.25` by
         default).
       - Applies simple agent filters:
         - source-source minimum distance: 8 mm,
         - final-final minimum distance: 8 mm,
         - optional final `phi >= 4 mm` when a phi scalar is provided.
   - Scalar sampling behavior:
     - if `field_faces` is provided and scalar length matches field vertices, scalar
       is sampled on polyline points by triangle interpolation (`sample_vertex_scalar_on_surface`),
     - otherwise falls back to nearest-vertex sampling.
   - Returns `PrintPointSet`, including:
     - `points`: final points for the selected mode,
     - `source_points`: points selected on the original polyline,
     - `segment_start_points`: projected print-line start points,
     - `surface_points`: mode-aware surface points (useful before lift/projection).
   - Main API:
     - `generate_print_points(...) -> PrintPointSet`

9. `loop_simulation.py`
   - Utilities for iterative loop simulation before robot execution.
   - Keeps loop stages explicit:
     - position field with bounded retries (`positioning_attempts`),
     - extract `phi` contour and geodesic offset contour,
     - generate print points with selectable mode:
       - `geodesic`: from geodesic offset contour (+ z-valid filter),
       - `z_lift`: from `phi=0` contour + vertical lift,
       - `gradient_lift`: from `phi=0` contour + tangent lift,
       - `gradient_walk`: from `phi=0` contour + simple agent walk,
     - update scan mesh by adding simulated bead solids (default cylinder).
   - Main APIs:
     - `position_field_with_attempts(...) -> PoseResult`
     - `compute_offset_contour_stage(...) -> ContourStage`
     - `generate_step_candidates(...) -> CandidateStage`
     - `apply_simulated_beads(...) -> (scan_mesh, bead_centers)`

10. `agent_walk.py`
   - Simple surface-walk agent used by `candidate_mode='gradient_walk'`.
   - Walks from selected source points along the scalar tangent field.
   - Exposes each accepted origin, projected print-line start (`p1`, default
     at 25% of `source -> final`), and final point (`pf`).
   - Filters accepted agents by source spacing, final spacing, and optional
     final phi clearance.
   - Main APIs:
     - `AgentWalkConfig`
     - `run_agent_walk(...) -> AgentWalkResult`

11. `print_targets.py`
   - Builds oriented point or line targets from candidate geometry.
   - Projects target points to the field surface, samples scalar-tangent
     orientation, optionally clamps target Z directions to a cone, and writes
     point YAML or nested line-segment YAML. Target-orientation visualization is standardized through
     `viz.make_target_orientation_sticks(...)` as fixed 8 mm x 1 mm rods.
   - Line target YAML schema:
     ```yaml
     segments:
       - index: 0
         start:
           plane: "O(...) Z(...)"
         end:
           plane: "O(...) Z(...)"
     ```
   - Main APIs:
     - `build_oriented_line_targets(...) -> OrientedLineTargets`
     - `write_line_targets_yaml(...)`
     - `write_fixed_z_targets_yaml(...)`

## Design Notes

- Field scalar computation and geometric viability are explicitly separated.
- `position_field` handles local candidate ranking for pose search.
- Offset generation is geodesic on the mesh, not Euclidean in free space.
- Candidate generation is centralized in `generate_print_points` (mode-driven),
  instead of split across multiple point-generator modules.
- Cone clamping can be applied in two different places:
  - point-position clamp in `generate_print_points(..., clamp_to_cone=True)`
    for limiting candidate displacement direction in `gradient_walk`.
  - orientation clamp in `print_targets.py` via
    `geometry.clamp_vectors_to_cone(...)` for limiting target-frame tilt
    without moving candidate positions.
- Full-loop/global objective design remains external to this library.
- Loop simulation keeps bead accumulation local to the simulator layer, so
  field/phi/contour stages remain reusable in ROS integration.

## Typical Script-Level Usage

1. Load field and scan meshes.
2. Compute heat field on the field mesh.
3. Position the field (manual offset or `position_field` search).
4. Compute phi and viable mask.
5. Extract `phi=0` contour.
6. Extract geodesic offset contour at desired distance (default 12 mm).
7. Select print points with `generate_print_points`:
   - from offset contour (`candidate_mode='polyline'`), or
   - from `phi=0` contour + lift (`candidate_mode='z_lift'`), or
   - from `phi=0` contour + tangent lift (`candidate_mode='gradient_lift'`), or
   - from contour with surface walk (`candidate_mode='gradient_walk'`).
8. Export and visualize.

## Current Exposure Notes

- Standalone Python scripts and `lib_scalar` support `gradient_walk`, line
  segment targets, and target-orientation visualization.
- ROS `fields_node`/`behav3d_commands` expose `z_lift`, `gradient_lift`, and
  `gradient_walk` through `GeneratePrintCandidates`.
- `gradient_walk` emits nested `segments:` YAML with `start`/`end` planes.
  The orchestrator `YamlSession` can parse both this segment schema and the
  older flat `targets:` schema.
