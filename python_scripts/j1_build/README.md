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

The launcher uses the tracked `mesh/fields/field_state_init.npz` and
`mesh/ScanMesh.stl` example data. It scales the scan by 0.001 and rotates it
180 degrees around world Z so its XY region overlaps the stored field.
This alignment is specific to these inputs, not a robot calibration.

On first launch, `prepare_field.py` creates
`mesh/fields/field_state_init_subdiv1.npz` with one extra level of the existing
Loop subdivision: 2,424 to 9,436 vertices and 4,590 to 18,360 triangles.
Loop subdivision can slightly smooth the goal geometry. The stored field
offset is preserved and `heat_norm` is transferred by surface barycentric
interpolation. The simulation evaluates phi again against the current scan.
The original input is unchanged; the generated copy is ignored by Git.

The mesh-loading loop's `--field-subdivide-iter` option does not apply to
NPZ inputs, so this preparation happens before launching the state-based loop.
An explicit `--field-state PATH` override skips preparation and uses that input.

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
