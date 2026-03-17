# lib_scalar

Reusable library for the scalar-field pipeline used in `python_scripts/scalar_field`.

The objective is to keep each stage isolated, testable, and reusable from:
- standalone scripts,
- future ROS nodes/services,
- future planning and print-path modules.

## Pipeline Stages

1. `compute_heat_field.py`
   - Builds the base scalar preference over the field mesh using `potpourri3d`.
   - This stage does not decide if a point is printable; it only provides scalar intensity.
   - Main API:
     - `compute_heat_field(vertices, faces, seed, seed_level, t_coef) -> HeatField`

2. `position_field.py`
   - Searches XY placement of the field mesh over the scan geometry.
   - For each XY candidate `(ox, oy)`, solves feasible Z using:
     - `base_local_z = min(field_vertices_scaled[:, 2])`
     - `z_offset = min(z_scan_hit - base_local_z) - base_epsilon`
   - This enforces base feasibility on hit points:
     - `base_world_z - z_scan <= 0`
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

7. `types.py`
   - Shared dataclasses used as contracts across stages.
   - `MeshData`: compacted mesh arrays + number of dropped vertices.
   - `HeatField`: raw and normalized scalar + summary stats + seed metadata.
   - `PoseResult`: positioned field, phi values, viability mask, ray-hit stats, search stats.
   - `PrintPointSet`: selected print points and spacing/selection metadata.

8. `generate_print_points.py`
   - Selects print points from the offset polyline.
   - For each iteration:
     - pick the polyline vertex with minimum scalar-field value,
     - require vertical proximity to scan surface (configured upstream),
     - optionally add external bridge points (e.g. from phi=0 endpoints projected in +Z),
     - suppress nearby polyline vertices within minimum spacing,
     - repeat until requested count is reached.
   - Distances are measured along the polyline graph.
   - Main API:
     - `generate_print_points(...) -> PrintPointSet`

9. `generate_print_points_phi_lift.py`
   - Alternative point-generation strategy without geodesic contour offset.
   - Uses the existing source polyline (typically `phi=0`) as candidate set.
   - Evaluates `heat.norm` on that polyline by triangle interpolation.
   - Selects lowest interpolated heat values with spacing suppression.
   - Lifts selected points in `+Z` by configured print height.
   - Main API:
     - `generate_print_points_phi_lift(...) -> LiftedPrintPointSet`

10. `loop_simulation.py`
   - Utilities for iterative loop simulation before robot execution.
   - Keeps loop stages explicit:
     - position field with bounded retries (`positioning_attempts`),
     - extract `phi` contour and geodesic offset contour,
     - generate print points with selectable mode:
       - `geodesic`: from geodesic offset contour (+ z-valid filter),
       - `z_lift`: from `phi=0` contour + vertical lift,
     - update scan mesh by adding simulated bead solids (default cylinder).
   - Main APIs:
     - `position_field_with_attempts(...) -> PoseResult`
     - `compute_offset_contour_stage(...) -> ContourStage`
     - `generate_step_candidates(...) -> CandidateStage`
     - `apply_simulated_beads(...) -> (scan_mesh, bead_centers)`

## Design Notes

- Field scalar computation and geometric viability are explicitly separated.
- `position_field` handles local candidate ranking for pose search.
- Offset generation is geodesic on the mesh, not Euclidean in free space.
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
7. Select print points from the offset contour (default 7 points, 16 mm spacing).
8. Export and visualize.

Alternative path (no geodesic offset):
1. Extract `phi=0` contour.
2. Interpolate `heat.norm` on that contour and select minima.
3. Lift selected points in `+Z` by print height.
