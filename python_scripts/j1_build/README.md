# J1_Build: Python-only scalar-field simulation

Run the DDS field/scan loop without ROS or a workspace build. Use Python 3.12.
From the repository root on Windows:

```powershell
py -3.12 -m venv behav3d
.\behav3d\Scripts\python.exe -m pip install -r python_scripts/j1_build/requirements.txt
.\python_scripts\j1_build\run_simulation.cmd
```

On Linux or macOS:

```bash
python3.12 -m venv behav3d
behav3d/bin/python -m pip install -r python_scripts/j1_build/requirements.txt
behav3d/bin/python python_scripts/j1_build/run_simulation.py
```

Dependencies include Open3D, Potpourri3D and DDS with its formats and visualization
extras. DDS is pinned to commit `66161c037d0d29fbb073aaa1027d436cbe2ba5f5`.
`requirements-windows-lock.txt` records the versions installed for the Windows
validation; the regular requirements file is the cross-platform starting point.

## Interactive desktop workbench

```powershell
.\python_scripts\j1_build\run_workbench.cmd
```

The desktop workbench provides live parameter panels, a non-destructive light-blue
DDS phantom, edge overlays, material/light controls, cached geometry, and on-demand
exports. See [the workbench guide](workbench/README.md).
Startup preferences are in [workbench/config.py](workbench/config.py):
`Parameters`, `SimulationOptions`, and `DisplayOptions`.
The current workbench preferences include 38 mm separation, 3.5 mm voxels and
42 x 20 mm beads. The reference launcher below keeps its original defaults.

On Linux/macOS, run `behav3d/bin/python python_scripts/j1_build/workbench/app.py`.

## Controls and defaults

- `N`: accept the current targets and advance; `Q`/`Esc`: finish.
- Add `--no-vis` for terminal stepping (`n`/`q`, then Enter).
- Candidate mode: `gradient_lift`; seven candidates per cycle.
- Contour source spacing: **3 mm** of arc length, including open endpoints.
  The last interval may be shorter; closed loops do not duplicate the seam.
- All source samples appear as cyan spheres in the `Source Samples` overlay.
- Bead width: **42 mm**; height: **20 mm**; selection separation: **16 mm**.
- DDS deposition mode: line; final segment fraction: **0.22**.
- Debug axes start hidden and are ten times smaller than the previous automatic size.

Arguments supplied to the launcher override its defaults. For example:

```powershell
.\python_scripts\j1_build\run_simulation.cmd --contour-sample-spacing-mm 2
.\python_scripts\j1_build\run_simulation.cmd --no-vis --dds-voxel-size-mm 3 --dds-surface-step-size 2
```

Each execution creates a new folder under `python_scripts/j1_build/outputs`.
Temporary files and the Matplotlib cache are stored next to the launcher to
support Windows environments with restricted default cache directories.

## Field input

Both launchers use `workbench/user_defaults.json` input preferences when present,
falling back to `InputOptions` in `workbench/config.py`. Built-in defaults load
the tracked `mesh/ScanMesh_simple_centered.stl` at scale 0.001 and yaw 0 degrees,
with `mesh/fields/field_state_centered_subdiv1.npz` as the goal.

The centered base field is `mesh/fields/field_state_centered.npz` (2,424 vertices,
4,590 triangles). The extra Loop-subdivision state has 9,436 vertices and 18,360
triangles and follows that base file for field-width mode. The original heat
distribution is preserved under the verified rigid coordinate transform; phi
has been recalculated against the centered scan. The corresponding inverse
width map is `mesh/fields/width_field_inverse_heat_centered.npz`.

`mesh/goal_field_original_mm_centered.obj` and `mesh/fault_region.stl` share the
same coordinates, in millimeters. Preparation metadata and input hashes are in
`mesh/fields/centered_field_preparation.json`. Earlier example inputs remain
available for explicit command-line overrides.

Paper, Front and Top frame the active goal. The Paper preset looks from +X/-Y
and above, with world Z upright. Saved local camera preferences override the
startup preset until removed or saved again.

## Validation

```powershell
.\behav3d\Scripts\python.exe -m unittest discover -s src/behav3d_py/test -p test_contour_sampling.py -v
```

Seven regression checks cover edge-partition independence, bends and graph arc
distance, endpoints, disconnected contours, closed loops, junctions, and source
eligibility. A terminal-controlled cycle was also validated with the extra
subdivision and 42 x 20 mm beads: seven DDS deposits, surface export and the
next contour/candidate computation. Source spheres were rendered off-screen.

DDS models deposited geometry; it does not model material flow, curing or deformation.
