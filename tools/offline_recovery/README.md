# Recorded recovery experiments

Two non-executable strategy comparisons use the same recording and Agent007
evidence as the inspection experiments. The original files are read only.

The follow-up in `../offline_planner_replay/README.md` has now located a matching
goal field in the repository and reproduced all 23 saved candidate batches.
The earlier artifacts below retain their original incomplete-state provenance;
they are not retroactively promoted to goal-validated recovery proposals.

The decision-layer replay now connects these comparisons to explicit recovery
choices. Run `build_decisions.py` (instructions below) and open
`outputs/offline_recovery/decisions/viewer.html` to inspect the recommendation,
candidate alternatives, rejection reasons and next inspection checkpoint.

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
rule-based candidate generation. At the time of this experiment only the recording's
field/contour point files had been identified. A later repository audit recovered
`mesh/fields/field_state_init1.npz`; see `../offline_planner_replay/README.md`.
The avoidance state and full runtime settings remain unresolved. These earlier new or adapted candidates have
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

## Recovery decision layer (2026-10-09)

`behav3d_orchestrator.offline_agent.recovery` is a standard-library policy module.
It takes a normalized issue, bounded proposals and a planner-state summary. It
returns a next action, a selected proposal for validation when appropriate,
alternative rejection reasons and a mandatory inspection checkpoint. There are
no ROS imports, command exports, runtime configuration changes or motion calls.

```sh
/Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python -B \
  tools/offline_recovery/build_decisions.py
python3 -B -m unittest discover -s src/behav3d_orchestrator/test -p 'test_offline*.py' -q
/Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python -B \
  -m unittest discover -s tools/offline_recovery -p 'test_*.py' -q
```

This replay uses the **committed experiment inputs**, verifies their artifact
manifests and queries their complete meshes. It does not need the external
Agent007 directory or raw recording. Its own source and output hashes are saved.
The old experiment outputs remain unchanged.

### Decision rules

1. Keep normal rule-based generation when no recovery issue is supported.
2. Inspect unresolved identity, registration, coverage, tolerance-sensitive
   discrepancies, unconfirmed failure or an unknown lean trend.
3. Defer the affected region for progressing deformation or compromised
   connections; extensive failures require reassessment of the construction goal.
4. For a stable local lean, compare bounded adaptations when the objective is
   continued growth. Restoring already printed geometry routes to a contact/goal
   study instead. Two distinct observation revisions are required for a trend;
   the policy does not estimate a trend from a single lean angle.
5. For a confirmed local deficit, compare an attached first fill bead. Unknown
   samples are not missing material. A buried or unreachable deficit can produce
   no suitable deposition option.
6. Exclude failed geometric/validator checks and options with no measured
   objective progress. Rank the remaining proposals by validated feasibility,
   objective progress, least displacement, then least nominal material. Benefit
   units must match within a decision; there is no arbitrary weighted score that
   can compensate for a failed constraint.
7. Require new readback evidence before another batch. Unchanged/worsened readback
   or the explicit attempt limit (default two) routes to reassessment. Attempt
   count and observation history are caller-supplied; this pure policy is not a
   persistent runtime state machine.

Support, contact and goal revision are reported with their missing prerequisites.
Priority changes need verified independence and progress toward the declared
objective; the replay does not infer stability benefits from printing elsewhere.
Those strategies do not acquire fabricated candidates or validation results.

### Proposal generation and limits

`proposals.py` adds two numerical searches without changing the existing planner:

- **Adaptation:** retain the unchanged candidate for comparison; test 13 XY
  offsets within 8 mm of the deformation-following seed and five axis choices
  (unchanged plus ±4° around X/Y). Fit the nominal base height to the observed
  footprint with a 12 mm Z-adjustment bound. Evaluate all 65 alternatives, including
  spacing against the reserved archived peers, before ranking. Only the selected
  single bead is proposed; the peers are not issued or recertified.
- **Fill:** sample the observed upper surface near the deficit centroid, then fit
  a nominal bead to that surface. Compare the old centroid probe and archived
  candidates too. This implements the **first attached bead**, not a simulated
  multi-bead repair chain; further deposition requires new evidence or an explicit
  validated deposition model. A candidate that touches only the upper edge of a
  deficit is not credited with filling the buried portion.

Benefits are reference samples reached for a deficit and additional base rays
within the experimental gap for adaptation. Neither measures repair volume,
structural stability, adhesion, print success or future goal compatibility. The
existing 6/9 support, ±3 mm gap, 32 mm spacing and sampled tool-clearance assumptions
remain uncalibrated. Future continuation is an explicit unresolved validation gate.
All target records and geometry checks are bound to the issue's artifact, mesh,
goal revision, frame and units; stale geometry cannot be silently reused.

### Recorded and controlled policy cases

The replay has nine cases: the real deficit and a confirmation counterfactual;
0°/4°/8°/12° geometry controls with a declared stable-trend assumption; and the same
8° geometry with unknown trend, progressing trend, or a restore-existing-shape
objective. **Confirmation, trend and intact connections in controlled cases are
test inputs, not Agent007 detections or new sensor measurements.**

Measured results:

| Case | Policy result | Geometric comparison |
| --- | --- | --- |
| Recorded C5:T7 discrepancy | Inspect the region | Failure remains unconfirmed and tolerance-sensitive |
| Same deficit, assume confirmation | Validate an attached fill proposal | 17/398 reference samples reached, 6/9 support rays, minimum sampled tool clearance 11.71 mm |
| Unmodified control | Use normal planner | Experimental proxy rejections do not trigger a fabricated failure |
| Stable 8° inward lean assumption | Validate bounded adaptation | 8/9 support rays versus 2/9 unchanged; minimum sampled tool clearance 9.83 mm |
| Same 8° geometry, unknown trend | Inspect the region | Geometry alone does not establish stability |
| Same 8° geometry, progressing trend | Defer the affected region | A good footprint does not resolve worsening lean |
| Same 8° geometry, restore printed shape | Contact/goal study | Additional deposition does not reposition existing material |

The selected 8° adaptation moves its top 7.19 mm relative to the archived target.
This replay did not use the subsequently recovered goal field, so this displacement
is **not** certified as an acceptable design deviation. The attached fill alternative reaches far fewer
reference samples than the 385/398 buried centroid probe; the latter is excluded
by feasibility rather than preferred for its higher coverage.

### Planner-state and validation interface

`planner_state.py` packages explicitly supplied inputs into a portable snapshot.
It does not infer missing inputs from contours or fill defaults from today's ROS
configuration. A complete package contains:

- Field NPZ with `field_vertices_scaled`, `field_faces`, `heat_norm`, `offset_xyz`.
- Scan NPZ with `vertices_m` and `faces`.
- Settings JSON with `effective_parameters`, `effective_request`, `field_to_scene`
  (rigid 4×4 transform in metres), and `source_revision`. All parameter reads from
  the actual fields node and all `GeneratePrintCandidates` request fields must be
  present; capture their actual values. Include resolved overrides in the request.
- Tool geometry and calibration JSON with `revision`, `status` (`verified` or
  `unverified`), `tool0_to_tcp_xyz_m`, and `tool0_to_tcp_rpy_rad`.
- Explicit avoidance mesh (`vertices_m`, `faces`) or a versioned, reasoned
  `explicitly_none` declaration; a width map with `width_norm` for field-width mode.

List the required parameter/request names with:

```sh
/Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python -B \
  tools/offline_recovery/planner_state.py --requirements
```

A specification has schema `behav3d.planner_snapshot_spec.v1`, `binding` containing
`artifact_id`, `mesh_revision`, `goal_revision`, `frame`, `units` (`m`), and `files`
mapping `field_state`, `scan_mesh`, `settings`, `tool_geometry`, `tool_calibration`
and optional `width_map`/`avoidance_map` to paths. `avoidance` has `mode`, `revision`
and a `reason` when explicitly absent. Run `planner_state.py --spec spec.json
--output snapshot_directory`; validate the package later with `--inspect
snapshot_directory/snapshot.json`. Output destinations must be empty. Sources are
copied, hash checked, and the planner source revision is bound into the manifest.

`planner_request(issue, proposal, snapshot)` binds the exact issue, targets and
snapshot hashes. A trusted numerical validator returns schema
`behav3d.recovery_validation_result.v1`, `request_sha256`, `validator`, and `gates`.
Each gate contains `status` (`pass`, `fail`, `unknown`) and supporting `evidence`.
Required gates are `goal`, `avoidance`, `support`, `spacing`, `tool_access`, `robot`,
`process` and `future_continuation`. Changing the mesh, goal, candidate, issue or
snapshot invalidates an old receipt. Receipts are trusted-adapter inputs, not
cryptographic attestations or agent-authored permission slips.

**A complete snapshot is input completeness, not a passed validation.** The saved
recording lacks a complete avoidance/settings/calibration state, so this earlier replay exports
incomplete validation requests and leaves those checks unresolved. This change
implements the packaging and result contract; automatic ROS snapshot capture and
full planner validation for recovery are still pending. The separate
`offline_planner_replay` experiment now runs numerical candidate generation against
the matched field with an explicitly inferred profile. No mocked success is inserted
into recorded results. A fully passing test receipt produces `review_bounded_batch`
while retaining `executable: false` and an empty command list.
