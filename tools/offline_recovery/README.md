# Recorded recovery experiments

Two non-executable strategy comparisons use the same recording and Agent007
evidence as the inspection experiments. The original files are read only.

- **Missing region:** C5:T7 in scan 6, using its saved DDS reference samples.
  This is recorded, tolerance-sensitive evidence, not a confirmed material loss.
- **Leaning region:** a controlled smooth shear of the right-hand region of that
  full recorded reconstruction, with 0°, 4°, 8° and 12° conditions. This is a
  geometric perturbation, not a claim that the recorded artifact leaned that way.

## Run

From the workspace root, with NumPy and PyYAML available:

```sh
/Users/josephnamar/Documents/GitHub/agent007/.venv/bin/python -B \
  tools/offline_recovery/build_experiments.py \
  --agent-root '/Users/josephnamar/Documents/ChatGPT/SustaniAi sumerschool/agent007-main'

/Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python -B \
  -m unittest discover -s tools/offline_recovery -p 'test_*.py' -v
```

Default output: `outputs/offline_recovery/recorded_cycle6/viewer.html`.
Open the self-contained viewer to switch cases, strategies and lean magnitudes;
rotate or zoom the reconstruction; and compare proposals with the original scene.
No network assets or ROS imports are used.

The complete recovery test suite also covers the inward-lean extension below and
uses Open3D/SciPy from the existing DDS environment. The original cycle-6 builder
still needs only NumPy and PyYAML.

Outputs include the original and edited full mesh arrays, evidence arrays,
proposal targets, ray diagnostics, exact perturbation parameters, source hashes
and an artifact manifest. Proposal JSON is not an execution/target-YAML export.

## What is compared

The missing case compares the first two actual C6 candidates, two candidates
prioritized from that same archived batch, a local fill probe, the complete
seven-candidate batch as a cost reference, and hold/inspect. The probe borrows
C5:T7's issued 36 × 16 mm rounded-bead dimensions and axis, centred on the
unambiguously located missing-reference samples. It is not a planner-approved
repair. The C6 YAML has positions and axes but no dimensions; its bead dimensions
are explicitly borrowed from C5 for this experiment.

The leaning case compares the archived batch unchanged, deferring the two
candidates in the affected region, adapting their bases to the edited geometry,
and hold/inspect. Adaptation keeps bead dimensions and fits a new axis between
the transformed archived base and top. It does not regenerate a remaining goal
field or prove mechanical stability. The affected region is fixed before testing
the four angles. The 0° condition leaves the mesh exactly unchanged.

The anchored deformation is:

`x' = x + weight_x(x) * weight_y(y) * max(z + 0.175, 0) * tan(angle)`

Y and Z are unchanged. Smoothstep weights transition over X = 0.19–0.23 m and
over a Y band around −0.87 m (full weight within 0.065 m, zero beyond 0.095 m).
Every triangle remains; the base and regions outside this mask are unchanged.
This is a controlled shape edit without material mechanics or sensor simulation.

## Evidence fidelity

The loader validates the prior local assessment, Agent007 bundle and their source
hashes. The spatial DDS export must reproduce the saved summary and retain all
target reference samples. Next-batch YAML sign/position mapping is checked against
its saved candidate point cloud to 0.01 mm. Frame binding uses the recorded scan
manifest, not an alias inferred from Agent007's abstract world/base frame.

DDS's spatial export rounds distances to 0.001 mm. At 10 mm it has 398 samples
strictly beyond the rounding interval and 22 at the boundary. Its saved aggregate
count is 412: 14 boundary samples contribute, but the export cannot identify which.
The experiment retains the 412 count and all coordinates, scores only the 398
unambiguous samples, and saves the 22 boundary samples separately. Unknown expected
samples are also retained separately and never treated as missing. Every saved
tolerance count must be consistent with its rounding interval or the build fails.

All geometric tests use the full 290,108-triangle reconstruction. Only the viewer
is cropped/downsampled. Source hashes are checked again after generation. No scan
after cycle 6 contributes evidence; the archived C6 batch is a planning baseline,
not a later observation or proof of successful printing.

## First measured results (2026-10-09)

| Missing-region strategy | Reference samples inside proposed solids | Base Z minus upper-surface hit |
| --- | --- | --- |
| First two archived candidates | 0 / 398 | +0.27 to +0.63 mm |
| Prioritize nearby C6:T7 and C6:T6 | 0 / 398 | −0.16 to +0.12 mm |
| New local fill probe | 385 / 398 | −17.23 mm |
| Complete archived batch | 0 / 398 | −0.16 to +1.01 mm |

Reordering the current candidates does not put nominal material into this recorded
reference region. The fill probe reaches most samples but starts well below the
upper surface; sample containment alone is a poor selection objective. These are
surface-band reference points, not missing material voxels or a repair-success
percentage. Observed-sample overlap may include bonding, and sampled volume outside
the prior issued beads does not imply being outside the final product goal.

At 8° the controlled edit moves 5,728 of 148,716 mesh vertices, by at most 9.47 mm.
The two unchanged nearby candidates have base-to-upper-surface gaps of +8.33 and
+6.62 mm. Adapting their bases gives +0.12 and −0.15 mm; the largest candidate-top
change is 10.94 mm. The 0°/4°/8°/12° mesh-displacement maxima are
0 / 4.71 / 9.47 / 14.32 mm. This demonstrates geometric mismatch and its correction
under the prescribed edit, not a physically successful recovery.

## Current boundary

This is an archived-batch replay and proposal comparison. It preserves ordinary
rule-based candidate generation. The recording provides field/contour point files,
but not the original field faces/scalars, avoidance state and full runtime settings
needed for faithful planner regeneration. New or adapted candidates have
`field_goal_validity: not_replayed`, `avoidance_validity: not_replayed`, and
`executable: false`. Existing archived candidates are not recertified here either.

No scalar preference or deposition direction is modified in the ROS planner.
Supports, contact correction and structural stability still need their own models
and tests. The experiment keeps the original missing-material inspection open and
requires stability evidence for the leaning case; it does not select an executable
winner based on geometric reach alone. The required full-frontier sweep remains
part of the operating workflow.

## Higher inward lean and contact access (2026-10-09)

The next experiment uses the already assessed **C18:T7 / scan 19** and its actual
next C19 batch. It keeps the earlier cycle-6 experiment as a separate comparison.
The right-hand upper region leans toward **−X, into the gap**, starting at
Z = −75 mm. This is 100 mm higher than the old −175 mm anchor, on a taller recorded
artifact. Lower geometry remains fixed; 0°, 4°, 8° and 12° are evaluated.

```sh
/Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python -B \
  tools/offline_recovery/build_inward_lean.py \
  --agent-root '/Users/josephnamar/Documents/ChatGPT/SustaniAi sumerschool/agent007-main'
```

Output: `outputs/offline_recovery/inward_cycle19/viewer.html`. The viewer compares
the archived batch, deferring the affected region, direct deformation following,
vertical fitting of candidate bases, and a bounded local placement search.
The search tests 13 XY offsets within 8 mm of the adapted candidate, seats each
nominal footprint using its median ray gap, and ranks fixed-tolerance support then
smallest displacement. It preserves the proposed axis and bead dimensions. This
is a new offline proposal rule, not a modification to ordinary candidate generation.

### Explicit geometric screens

- Positive candidate-top clearance above the upper mesh.
- At least 6 of 9 nominal bottom-plane rays within ±3 mm of that mesh.
- At least 32 mm between candidate centres: 36 mm nominal bead proxy minus an
  assumed 4 mm overlap. This is not a recovered historical width-map setting.
- Extruder free approach of 50 mm along its axis, sampled at at most 3 mm steps,
  with a 2 mm margin against the entire recorded/edited mesh. Tool samples use the
  repository COLLADA vertices plus triangle centroids, with a 2 mm TCP exclusion.

`viability.py` imports only the existing pure numerical collision module. Its
rotation/transform functions are reused. A restricted COLLADA reader validates
metres, Z-up, triangle indexing and identity geometry instances; no additional
package installation or ROS construction is needed.

The collision module's default tool translation differs from the current workcell
URDF by **13.36 mm**. The experiment evaluates **both**, records disagreements and
requires both sampled approaches to pass. Neither transform is independently
bound to the historical robot calibration. The module's runtime defaults and the
URDF are not edited by this experiment.

### Results at 8°

The edited region contains 5,239 changed vertices, with maximum displacement
7.21 mm. Only C19:T7 in the next batch is within this experiment's fixed region.

| C19:T7 alternative | Base-centre gap | Base rays within ±3 mm | Minimum extruder clearance, URDF / checker defaults | Screen |
| --- | --- | --- | --- | --- |
| Unchanged | −3.33 mm | 2 / 9 | 14.27 / 7.83 mm | Support rejection |
| Follow deformation | +3.58 mm | 2 / 9 | 16.63 / 11.36 mm | Support rejection |
| Fit base height | 0.00 mm | 4 / 9 | 14.04 / 9.07 mm | Support rejection |
| Local placement search | −0.12 mm | 6 / 9 | 15.75 / 9.44 mm | Passes declared screens |

The local alternative is 8 mm outward relative to the directly adapted candidate,
with a +7.35 mm vertical fitting adjustment. It does not yet have a regenerated
goal/avoidance validation. Its footprint still includes rays outside the allowed
gap; 6/9 is the declared experimental screen, not proven mechanical support.

The zero control matters: the archived C19:T7 already fails this footprint screen
at 0° (3/9 rays). Across the complete batch, 2/7 archived candidates and 3/7 with
the local alternative pass at both 0° and 8°. Do not attribute all rejections to
the injected lean or infer that previously issued targets were invalid. The
proxy dimensions and thresholds need calibration against fabrication evidence.

### Contact-correction path study

The contact tab compares a compact paddle concept with the actual extruder mesh.
It tests 18 fixed contact/access alternatives on the inner face: three Y locations,
three heights above the anchor and two stand-off distances. It ranks **free approach
clearance only**. A slider shows above-gap, standby, approach end, nominal contact,
desired correction, release and retreat positions.

The paddle proxy has a 12 × 10 mm contact face, 3 mm plate thickness and a 6 mm
diameter vertical stem rising 80 mm. These are concept dimensions, not an existing
calibrated tool. Mount, wrist, edge rounding and contact compliance are not modeled.
The same path positions are interpreted as the extruder TCP for the comparison;
its tool axis points −X, so the correction approach is +X.

At 8°, the selected paddle's **free approach** has 5.88 mm minimum sampled clearance.
The existing extruder has sampled surface conflicts under both tool translations
(minimum clearances below 0.01 mm). The selected nominal contact shift toward the
pre-edit reference is **2.11 mm**, at Z = −60 mm. This is not a prediction that this
single push restores the whole upper section or its 7.21 mm maximum displacement.

The sampled free path ends **6 mm before nominal contact**. Contact search, pushing
and post-contact retreat remain explicitly unvalidated; no corrected mesh is
invented to clear those stages. JSON positions reference the paddle face centre,
not a calibrated flange/TCP, and contain no force, speed or controller commands.
Material response, force feedback, contact detection, abort limits, calibrated
tool/mount geometry, IK/full-arm collision and post-contact inspection are required
before an executable correction can be developed.

Unsigned, sampled mesh distances do not certify continuous collision, inside/outside
containment or unobserved space. A candidate passing this experiment retains
`executable: false`, unreplayed field/avoidance validity and unverified robot IK.
