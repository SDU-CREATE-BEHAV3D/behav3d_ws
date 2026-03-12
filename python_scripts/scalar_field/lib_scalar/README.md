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
   - Extracts the 3D boundary `phi = iso` by edge interpolation over field triangles.
   - Typical use: `iso = 0` for viable/non-viable boundary.
   - Main API:
     - `extract_phi_contour(vertices, faces, scalar, iso=0.0)`

5. `extract_offset_phi_contour.py`
   - Builds an offset polyline from the `phi=iso` contour directly on the field mesh.
   - First it detects seed vertices adjacent to contour-crossing edges.
   - Then it solves geodesic distance from those seeds over the mesh using `potpourri3d`.
   - Geodesic distance is signed by side:
     - unprinted side: `phi > iso`
     - printed side: `phi <= iso`
   - The offset polyline is extracted as an iso-curve of this signed geodesic field.
   - Main APIs:
     - `contour_seed_vertices_from_phi(...)`
     - `compute_geodesic_from_phi_contour(...)`
     - `extract_offset_phi_contour(...)`

6. `viz.py`
   - Converts numeric outputs to Open3D visualization objects.
   - Main APIs:
     - `yellow_to_red_colors(norm_scalar)`
     - `make_point_cloud(points, colors)`
     - `make_line_set(points, lines, color)`
     - `compute_scene_bounds(...)`

7. `geometry.py`
   - Geometry loading and preparation utilities.
   - Ensures triangle meshes are valid and indexing is compact.
   - Main APIs:
     - `load_triangle_mesh_arrays(path) -> MeshData`
     - `load_triangle_mesh_legacy(path)`
     - `compact_triangle_mesh(vertices, faces)`
     - `apply_scale_and_offset(...)`

8. `types.py`
   - Shared dataclasses used as contracts across stages.
   - `MeshData`: compacted mesh arrays + number of dropped vertices.
   - `HeatField`: raw and normalized scalar + summary stats + seed metadata.
   - `PoseResult`: positioned field, phi values, viability mask, ray-hit stats, search stats.
   - `PrintPointSet`: selected print points and spacing/selection metadata.

9. `generate_print_points.py`
   - Selects print points from the offset polyline.
   - For each iteration:
     - pick the polyline vertex with minimum scalar-field value,
     - require vertical proximity to scan surface (configured upstream),
     - suppress nearby polyline vertices within minimum spacing,
     - repeat until requested count is reached.
   - Distances are measured along the polyline graph.
   - Main API:
     - `generate_print_points(...) -> PrintPointSet`

## Design Notes

- Field scalar computation and geometric viability are explicitly separated.
- `position_field` handles local candidate ranking for pose search.
- Offset generation is geodesic on the mesh, not Euclidean in free space.
- Full-loop/global objective design remains external to this library.

## Typical Script-Level Usage

1. Load field and scan meshes.
2. Compute heat field on the field mesh.
3. Position the field (manual offset or `position_field` search).
4. Compute phi and viable mask.
5. Extract `phi=0` contour.
6. Extract geodesic offset contour at desired distance (default 12 mm).
7. Select print points from the offset contour (default 7 points, 16 mm spacing).
8. Export and visualize.
