# J1 Desktop Workbench

A persistent simulation session using PyVista/VTK, PyVistaQt, PySide6 and DDS
visualization helpers. Uses the existing `behav3d` environment; no ROS, web
server or additional dependencies are required. Production text and the UI
are in English, including numeric formatting.

## Launch from the repository root

```powershell
.\python_scripts\j1_build\run_workbench.cmd
```

The existing `run_simulation.cmd` launcher retains the reference loop.

## Controls

### Editing startup defaults

Edit `workbench/config.py` and restart the workbench:

- `Parameters`: bead width/height, source spacing, candidate count, separation,
  walk, orientation, clearance and deposition fraction.
- `SimulationOptions`: voxel size, threshold, surface step, domain padding and checkpoints.
- `DisplayOptions`: material, face/edge/phantom/domain opacity, camera controls,
  layer visibility, run count and SSAO settings. SSAO radius and bias are expressed in millimeters.

For example, set `contact_shading = False` to start without SSAO, or reduce
`ssao_radius_mm` to make its contact effect more local. Live panel edits affect
the current session but are not saved back to these startup defaults.

`DisplayOptions` also contains `light_intensity` (overall multiplier), individual
`key_light_intensity`, `fill_light_intensity`, `rim_light_intensity`, and the bead
material coefficients `surface_ambient`, `surface_diffuse`, `surface_specular`
and `surface_specular_power`. Light directions are in `workbench/appearance.py`.
Lights are restored after every scene reset, because
PyVista's `clear()` removes them along with the actors.

**Light intensity** adjusts all three lights live. **Shade goal** defaults to off
(`goal_lighting = False`), preserving the unlit scalar colors independently of
scene lighting. SSAO remains a global screen-space effect, not a per-object
physical shadow switch. Disable Contact shading separately when comparing it.
There is no gamma/exposure control or tone-mapping stage at present; intensity
and material coefficients adjust lighting rather than gamma correction.

- Editing a parameter pauses automatic execution, invalidates the old preview
  and prepares a new one after a 350 ms debounce.
- **Preview** recalculates candidates without depositing material.
- **Accept batch** applies the current preview. `N` works when the scene has focus.
- **Run** accepts the selected number of cycles, preparing a fresh preview after
  every batch. The default limit is ten cycles.
- **Pause**, or Space over the scene, prevents the next cycle from starting.
  An operation already in progress finishes normally.
- **Export** saves the combined scan, DDS surface, NPZ checkpoint, history and
  viewport screenshot without closing the session.
- Camera and layer settings persist between batches. **Paper view / frame goal**
  enables orthographic projection and frames the goal.
- **Camera controls** defaults to **Rhino**: right drag rotates, Shift + right
  drag pans, Ctrl + right drag zooms, and the wheel zooms. Ctrl + Shift + right
  drag also rotates. These bindings work in the orthographic 3D paper view too.
  Rotation orbits around world Z, keeps the horizon upright and stops short of
  the poles. This matches Rhino's Z-axis orbit convention using VTK's terrain
  interaction style; it does not reproduce Rhino's complete camera engine.
  **VTK** restores the original gestures. Switching controls preserves the view.
- Scene updates preserve the live camera position, target, up vector, projection,
  field of view and zoom, including camera changes made while a worker is busy.
  Automatic framing happens only when creating a session. The Paper view button
  remains available for an explicit reset.
  Paper framing explicitly marks the camera as configured, and snapshots read
  the renderer camera directly to avoid PyVista's lazy default-view reset.

Next-batch controls include mode, count, source spacing, target separation,
bead width and height, clearance, deposition fraction, walk parameters,
orientation and normal continuity. Changes affect future batches only.
Defaults remain 3 mm source spacing, 42 x 20 mm beads and one extra subdivision
of the stored goal field.

### Fixed and field-controlled widths

**Width mode** selects `fixed` or `field`. The panel explicitly marks the active
mode and disables the other mode's inputs:

- **Fixed mode only · mm**: Bead width and Separation. Uses the configured goal,
  including the extra subdivision in the standard launch.
- **Width field mode only · mm**: Width field NPZ, Minimum bead width, Maximum
  bead width and Within-batch overlap. Uses the original goal without the extra
  subdivision. The standard derived goal records its original file in
  `source_field_state`; custom unsubdivided goals are used directly.

Load an NPZ containing `width_norm` in `[0, 1]`, with one value per original goal
vertex in the same order. A map for the subdivided goal is rejected rather than
silently transferred. The loader checks the vertex count and numeric values;
vertex ordering remains part of the input contract.

For a trial map, load `mesh/fields/width_field_inverse_heat.npz` from the data
root. It uses the original goal's `width_norm = 1 - heat_norm`: lower heat
sources, which have earlier selection priority, produce wider beads. With the
default field range of 20–40 mm, the first batch is close to 40 mm. This is a
synthetic test distribution, not a calibrated material-width field.

The existing scalar functions map normalized values to the width range, sample
at contour sources before lift/walk, and apply pair spacing
`max(0, (width_i + width_j) / 2 - overlap)` in candidate suppression and endpoint
pruning. Heat priority, source subdivision spacing, height and orientation rules
are shared between modes. Width is constant within each individual DDS deposit.

Switching modes updates the goal and preview while retaining accepted DDS
geometry and the camera. Range/overlap edits reuse phi geometry when possible.
Preview widths are frozen and used by both phantom generation and acceptance;
changes never resize historical beads. The initial domain reserves the field's
maximum width. Each batch saves its applied widths and input hashes, with a
normalized width-map snapshot under `inputs/`. Field-mode target YAML includes
the existing nominal `volume_mm3` contract.

Voxel size, physical threshold, surface-extraction step, padding and checkpoint frequency
apply through **Create new session**. Previous session files are retained.
Closing does not trigger a heavy mesh export; accepted targets are already
stored. Use Export to obtain the final meshes.

## Surface appearance and edges

- **DDS domain** displays a translucent box with unlit faces and edges;
  **Domain opacity** defaults to 0.20. The layer starts hidden. Before the first
  accepted batch it shows the initial planned domain; afterwards it shows the
  current DDS domain, updating only when its bounds change. It is excluded from
  camera framing and does not change voxel allocation. Lighting is disabled on
  this actor; Contact shading remains a global screen-space effect.
- **DDS phantom** previews the proposed batch in translucent light blue.
  It starts enabled; **Phantom opacity** controls transparency (default 0.80).
- The phantom is a separate DDS simulation of the proposed deposits only,
  including their widths, heights, orientations, line fractions and YAML rounding.
  It uses the session's voxel resolution, threshold and surface step on a cropped
  domain aligned to the same voxel lattice. It includes overlap with existing
  material; it is not a visualization of only the newly exposed union surface.
- Phantom generation runs on the calculation worker and is cached per preview.
  It never changes the accepted deposits, history, current scan or candidate rules.
  Editing parameters or accepting a batch removes the old phantom immediately.

- **Goal edges** and **Bead edges** show the actual triangle edges as separate
  wireframe layers. Initial visibility comes from `DisplayOptions.layers`.
- **Goal edge opacity** and **Bead edge opacity** independently control those
  overlays from 0 to 1; initial values come from `DisplayOptions`.
- Edge visibility is independent of surface visibility, allowing wire-only views.
- **Bead material** selects Warm white (default), Slate, Blue or Graphite.
- Neutral key, fill and rim lighting with smooth display normals provide a
  studio appearance. **Contact shading (SSAO)** softly darkens crevices and
  contacts between beads. Its initial state comes from `DisplayOptions` and it
  can be disabled for lighter GPU rendering. These are display settings, not
  simulated physical shadows.
- Goal face opacity is separate from edge opacity.
- Edge actors are created lazily, reused for unchanged meshes and refreshed when
  the source mesh changes. Display controls do not regenerate candidates or
  modify simulation parameters, voxel data or history.

The old bead color was a near-white ivory material. The last-batch layer shows
target markers, not a separate physical bead mesh. Smooth shading affects only
display normals; the geometry used for phi is unchanged.

## Optimizations

- Load the field and initial scan once per session.
- Cache phi and contour geometry by cycle and clearance.
- Compute and cache the magenta offset only when requested as a diagnostic.
- Keep DDS accumulation incremental and omit unnecessary coverage computation.
- Extract one DDS surface per cycle and share it with the next evaluation and
  the viewer. The viewer does not request a second DDS result.
- Build the combined scan without an additional preceding deepcopy. Retain
  static scan actors across previews.
- Save YAML, parameters and timings per batch; export full meshes on demand.
  Checkpoint frequency comes from `SimulationOptions.checkpoint_every`; 0 disables them.
- Expand the goal-based domain only when a future deposit exceeds it, preserving
  the voxel lattice and replaying historical deposits. A thirty-million-voxel
  limit prevents accidental oversized allocations.

The initial domain surrounds the goal and initial targets. Its margin is
`max(padding_mm, 12, bead_width_mm, bead_height_mm)` on both sides of all three
axes, with bounds rounded by DDS to whole voxels. Expansion occurs only when
the support bounds of an incoming deposit extend beyond any current domain
face. The expanded box includes one extra voxel around those deposits and
retains the existing voxel lattice. Reaching the top or end of the goal alone
does not trigger expansion. Initial creation is distinct from expansion in the
activity log and the saved `domain_change` field (`initialized`, `expanded`,
`unchanged`). Each accepted record includes domain bounds, shape and the faces
that expanded (for example `Z+`).

Storage remains dense. Sparse blocks and a distance band around the goal are a
later optimization, using this version as the reference.

## Data and module boundaries

Workbench code lives in `workbench/`. Existing phi, contour, source sampling,
candidate generation, orientation and rule functions are imported unchanged.
The original YAML round-trip, including rounding, is preserved for deposition.

Each session creates `workbench/outputs/session-.../`:

```text
session.json                     # input paths, hashes and resolution
history.jsonl                    # one entry per accepted batch
performance.jsonl                # operation starts/ends, including unaccepted previews and viewport updates
step_0001/segments.yaml           # original target contract
step_0001/parameters.json         # exact parameters and timings
checkpoint-0010.npz              # optional DDS checkpoint
export-000N/                     # created only by Export
```

Checkpoints contain DDS geometry and deposits, readable with
`SimulationResult.load`. Reopening a full session from the GUI is not yet
implemented; targets and parameters are stored for future replay.

`performance.jsonl` records geometry, candidates, phantom generation, worker
operations, checkpoint compression and scene/render updates. An operation that
has started without a matching completion helps locate a stall before another
batch is accepted. `history.jsonl` still contains only accepted batches.
`accept_s` measures the numerical update before output writes. Checkpoint events
measure compression/output separately. Worker timings cover the complete action,
including all output writes, while viewport timings cover the main-thread scene update
and render request. These are wall-clock measurements, not GPU profiler timings.

## Validation

From the project directory:

```powershell
.\behav3d\Scripts\python.exe -m unittest discover -s python_scripts/j1_build/tests -v
.\behav3d\Scripts\python.exe python_scripts/j1_build/tests/desktop_smoke.py
```

Session tests compare three cycles against the reference: identical target YAML,
implicit fields within absolute tolerance 1e-12, and matching surface face counts.
Tests cover non-destructive previews, caching, stale tokens, historical dimensions,
domain expansion, exports and checkpoints. The Qt smoke test checks live controls,
acceptance, automatic stepping and export. Native-window rendering is checked
separately because this machine's offscreen Qt platform has no OpenGL context.
