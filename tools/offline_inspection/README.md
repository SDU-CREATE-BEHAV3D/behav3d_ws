# Offline focused inspection prototype

## Current development scope: preserve the upper-frontier sweep

The scan supports closed-mesh reconstruction, upper-frontier/goal-field comparison,
ray intersections and valid print-candidate selection. The existing top grid sweep
and its normal height adjustment remain the required baseline. Supplemental views
must serve a named inspection or geometry-development need. They cannot crop,
replace or reorder the required sweep.

`frontier_scan.py` implements this offline decision boundary. It takes an existing
pose sequence, cached geometric assessments, explicit inspection sample IDs and
evidence references. It appends at most two poses by default (hard maximum four),
scoring only new opportunity on requested samples. Requests may originate from
Agent007 or an upstream geometry-development assessment. The latter assessment
is not yet generated automatically. The optimizer uses area, then fewer poses,
then camera-position distance and orientation change; it does not estimate sensor
information gain or robot duration. No request or no predicted relevant gain means
no additions. Inspection uncertainty remains open in every offline proposal.

The recorded replay uses **all 16 actual optical poses from scan 6**, recomposed
from its manifest and recorded extrinsics. The old nine-pose cropped grid below is
an earlier diagnostic experiment, not the workflow baseline. The adapter checks
every recorded pose and its order against the cached assessment; altered or cropped
baselines fail. Recorded poses are for replay, not a next-cycle motion plan.

`build_frontier_scan.py` reads the actual Agent007 evaluation bundle, checks bundle
and source hashes, binds suggestion/evidence/candidate identities and cycles, and
resolves the assessed label through the shared review source. Agent007's abstract
`world/base` frame is preserved as metadata; no frame alias or transform is assumed.
Planning coordinates come only from the separately bound robot-base assessment.
This stateless file adapter does not implement the provisional controller's revision
ledger or runtime transport.

The first recorded case is C5:T1. Its cap-coverage request maps to nominal top
samples, not unrestricted side area. The current model predicts 87.5% top opportunity
for both the recorded sweep and the full eligible candidate pool. It therefore
keeps all 16 baseline poses and adds zero. This does **not** resolve partial angular
coverage, fragmented footprint or insufficient complete views: depth quality,
angular diversity and repeated measurements are not scored. Other packet reviews
are explicitly listed as pending, not treated as resolved. Shape/position requests
map to top and sides; registration/identity concerns go to evidence review first.

```sh
python tools/offline_inspection/build_frontier_scan.py \
  --demo outputs/offline_inspection/c5-t1 \
  --evaluation-bundle '/path/to/agent007-main/outputs/integrated_evaluation_260728_133013_final/cycle_0006/evaluation' \
  --max-extra 2
python -m unittest discover -s tools/offline_inspection -p 'test_*.py' -v
```

Outputs: `frontier_scan.json` (full baseline, additions, reasons and source binding)
and `frontier_scan_report.html` (decision report). Existing demo outputs are retained.
Rebuild the proposal after changes to sources or local assessments; saved reports
are snapshots and do not auto-refresh. Regression tests cover preservation, relevant
supplements, no-benefit unresolved reviews, budget/clearance rejection and source
binding. Preserving the input sequence does not prove frontier mesh completeness;
effects on reconstruction and candidate generation require captured-data replay.

## Fixed-settings batch replay

`replay_frontier_cases.py` tests 12 predeclared cases across recorded scans 2, 6,
12, 19 and 23. They cover cap uncertainty, shape review, identity ambiguity,
position review and a within-tolerance comparison. C5:T1 repeats the original
case as a control. Selection is by stage and finding before scoring; it is not an
accuracy benchmark or a representative sample of all possible geometries.

```sh
python tools/offline_inspection/replay_frontier_cases.py \
  --agent-root '/path/to/agent007-main' \
  --output outputs/offline_inspection/frontier_batch
```

The adapter verifies the actual YAML target index, origin, axis, dimensions and
saved DDS sign mapping for each current bead, plus evaluation source hashes,
calibration, scan linkage and every baseline capture pose. Whole recorded meshes
remain in ray tests. Intended-neighbor envelopes cover same-cycle beads; older
recorded material remains available as scene obstruction. Settings and policy
remain fixed: 24 candidate directions, 400 mm stand-off, and at most two added
views per case. The runner never reads later-cycle geometry for an earlier case.

`case_selection.json` records the chosen cases, settings and code hashes before
scoring. `batch_results.json` records every result or error, and `report.html`
compares them. Each case has its own `local_scene.json`, `frontier_scan.json` and
decision report. These are independent proposals, not a combined scan schedule;
added views across regions have not been merged or assigned a shared budget.

Use `--reuse` only to rerender/reselect from existing assessments with unchanged
sources and code. Hash mismatches are reported as failed cases; they do not
silently reuse old geometry. A normal run recomputes the assessments.

Recorded batch observations (2026-10-07): all 12 cases completed and retained their
16-pose baseline. Six cases received two additions, with 8.1–17.8 percentage points
of extra modeled opportunity on the requested top-and-side surfaces. The camera
position order increased by 1.05–1.43 m per independent proposal. Four cap-review
cases received no additions; two identity-review cases correctly remained in the
evidence-review branch. The C5:T1 control reproduced its prior baseline poses,
decision and requested-surface metric exactly.

Two development issues are exposed, without changing policy during this test:

- C1:T4 already has within-tolerance DDS evidence, but its broad shape-review
  suggestion still triggers two extra views. A more selective scan trigger or
  priority rule is needed before accepting every geometrically beneficial view.
- Cap quality is not described by geometric access alone: C11:T4 has 100% modeled
  nominal-top opportunity but still has incomplete cap evidence. Conversely,
  C22:T1 has only 4.2% modeled top opportunity and no gain from the tested pool.
  Most of its ray obstructions lie near intended neighboring candidates, with
  material identity unverified. Neither case has been resolved by this replay.

These results support further work on useful shape/position supplements, targeted
cap-quality evidence, and scan priority. They do not validate new sensor captures,
mesh quality, physical defects, robot reachability or subsequent print candidates.

## Raw-depth holdout and qualified inspection requests

`replay_depth_holdouts.py` tests C1:T4, C5:T7 and C11:T4 using their original depth
PNGs. It reads the recorded reconstruction metadata (depth scale, integer bias,
crop, maximum depth and extrinsic convention), backprojects the valid returns,
and reruns the saved Agent007 numerical cap estimator. That single pure estimator
module is loaded read-only after checking its producer hash; bytecode writes are
disabled. No Agent007 workers/services, DDS reconstruction or ROS are launched.
The source data and Agent007 files are unchanged.

```sh
python -B tools/offline_inspection/replay_depth_holdouts.py \
  --agent-root '/path/to/agent007-main' \
  --batch outputs/offline_inspection/frontier_batch \
  --output outputs/offline_inspection/depth_holdouts
```

The runtime needs numpy, scipy, scikit-image, Pillow and PyYAML. No OpenCV is needed
for this replay. The depth projection intentionally matches Agent007's recorded
ideal-pinhole convention; it does not independently validate lens calibration.

Protocol: four fixed folds hold out every fourth manifest capture. Each trial
starts from 12 views and restores up to one/two held-out views. Ranking uses only
known capture directions, commanded target geometry and the recorded depth crop;
held-out measurements, full-scan meshes and fitted cap patches are excluded from
selection. Every subset is fitted afresh. All single/pair restorations are also
reported as retrospective diagnostics, never presented as a prospective selector.
Full-data results must reproduce the saved cap status, reasons, counts and numeric
fit before subset results are accepted.

The output also qualifies scan requests with `inspection_policy.py` and compares
zero/one/two proposed oblique additions against the full mandatory sweep. The
qualification requires current evidence and a specific unresolved shape/position
issue. A generic shape-review suggestion with within-tolerance evidence is kept
as a review record without generating extra scan poses. Cap-quality issues route
to raw-depth review, not unrestricted nominal-area optimization. This is a new
offline entry point; the original batch report remains an unchanged first-policy
comparison and the ROS workflow is not modified.

Observed results:

- All three full raw-depth replays reproduce Agent007's saved results.
- C1:T4: no qualified supplemental scan need. One of four reduced-data trials
  loses cap eligibility; restoring one selected existing view recovers it.
- C5:T7: three of four reduced-data trials lose cap eligibility. One selected
  restoration recovers two of those three; up to two recover all three. The
  recorded full scan already passes the cap gates, so this does not resolve its
  separate DDS shape concern. For that shape objective, one proposed oblique view
  raises modeled opportunity from 54.4% to 65.4% with 0.83 m added position order;
  two reach 72.2% with 1.43 m. Oblique-view depth has not been captured.
- C11:T4: the full fit has a 2.517 mm outline residual against a 2.5 mm provisional
  threshold. Some subsets pass while the full set fails. This is view/fit
  sensitivity, not proof that discarding evidence improves the physical bead.
  Shape/outline interpretation needs review before another scan is justified.

`protocol.json`, `depth_holdout_results.json` and `report.html` preserve settings,
sources, per-view diagnostics, fold results and qualified proposals. Passing a cap
gate is not a physical accuracy test, material-identity proof or clearance to print.
The 12/13/14-view subsets are analytical ablations, never a shortened frontier
sweep. Every actual proposal in this output preserves all 16 baseline poses.

## Earlier geometry experiments

Pure pose generator: `behav3d_orchestrator.offline_agent.inspection`. No ROS imports,
robot commands, hardware connections, or physical claims. Positions are metres in
an explicit frame. Orientations are optical X/Y/Z world vectors (matrix columns);
these are not flange poses. Grid retains the existing world-XY serpentine convention.
Cap uses the existing Fibonacci-cap sampling law and position-only nearest-neighbor
ordering, with explicit outward axis and no random jitter. Roll is deterministic;
it has not been tuned for robot joint limits.

## Recorded demo

`outputs/offline_inspection/c5-t1/viewer.html` embeds the scene and both plans and
works without a server or network. Rotate by dragging; scroll to zoom. Layer toggles,
pose selection, ideal frustums and focus/overview/top views are available.

Case: issued C5:T1 evaluated against recorded scan 6 of dataset 260728_133013.
Agent007 reports partial angular coverage, fragmented/noncircular footprint and
insufficient complete views. No confirmed missing material is asserted.

The builder reads Agent007 outputs and the original dataset without modifying them.
Source paths/hashes are stored in poses.json. It checks the issued YAML hash, saved
YAML sign mapping against the replay position, scan hash and N→N+1 cycle linkage.
The colored reconstruction is displayed in recorded base coordinates, without
alignment to a target or diagnostic fit. Camera transforms use recorded tool0 poses
composed with T_tool0_ir, matching Agent007's depth replay convention.
Calibration is recorded, not independently verified. Frustums use ideal pinhole
intrinsics; distortion is not simulated.

Demonstration assumptions: 400 mm stand-off, 120×120 mm grid with 3×3 poses, nine
cap poses within 60° around the commanded candidate axis, illustrative ±26 mm focus
box. These are comparison parameters, not approved camera operating limits or a
policy selected from evaluation evidence. Original capture positions are included
for reference; the proposed grid does not copy their orientation or stand-off.

The scene is a deterministic sample of 9,295 of 102,241 recorded vertices. The
observed patch is displayed separately, without assimilating its diagnostic center.
No unknown region is filled. Occlusion, coverage, depth quality, IK, tool clearance,
and transition checks remain unverified. Capture-order lines are not trajectories.
No pattern is ranked or recommended by this stage.

## Rebuild and test

Use a Python environment with numpy and PyYAML for the read-only source adapter:

```sh
python tools/offline_inspection/build_recorded_demo.py \
  --agent-root '/path/to/agent007-main' \
  --dataset '/path/to/260728_133013' \
  --output outputs/offline_inspection/c5-t1
python3 -m unittest discover -s src/behav3d_orchestrator/test -p 'test_offline*.py' -v
```

The adapter currently targets this recorded case and references its saved DDS
metrics path explicitly. General source-manifest adapters are a later step.
The pure generator and its tests need only the Python standard library.

## Stage 3: recorded mesh visibility diagnostics

`visibility.py` uses full-resolution binary STL triangles and double-sided ray /
triangle intersections with a ray/AABB broad phase. It projects every existing
observed cap point through an ideal pinhole camera. Outcomes are outside image,
blocked in mesh, endpoint supported in mesh, or unknown mesh support. The 4 mm
endpoint tolerance is an explicit diagnostic assumption, with 2/4/8 mm sensitivity
counts. A ray whose first intersection is farther than the target or absent remains
unknown; even supported endpoints do not prove the intervening space is physically
free, since the recorded mesh is incomplete and may include fusion artifacts.

This stage measures re-observability of the 212 *existing* observed samples. It
cannot estimate full-bead coverage or information gain over unobserved material.
Counts of samples supported by one / three proposed views are geometric diagnostics,
not predicted independent depth measurements or resolution of identity uncertainty.
The commanded candidate axis is used only for a diagnostic view angle; no measured
normal or sensor incidence limit is inferred. Capture-order length excludes robot
joint travel, orientation cost, approach paths and timing.

```sh
python -m unittest discover -s tools/offline_inspection -p test_visibility.py -v
python tools/offline_inspection/compare_recorded.py \
  --demo outputs/offline_inspection/c5-t1 \
  --mesh '/path/to/260728_133013/field_loop/cycle_0006/scan/reconstruct/tsdf_surface_mesh.stl'
```

The comparison validates pose-source hashes and uses the STL sibling of the bound
reconstruction PLY. It records the STL hash; this format linkage is not independent
geometric equivalence certification. Rebuilding the initial demo removes its
comparison; rerun the comparison to refresh it. No physical scan is selected or
executed. Controlled tests cover exposure, ridge/side-view occlusion, missing
geometry, image bounds, triangle winding and tolerance sensitivity.

## Separate nominal top/side inspection opportunities

The observed cap comparison does not represent the bead sides. `bead_surface.py`
therefore samples the issued nominal rounded cylinder independently: a flat top disk
and rounded side bands with analytic normals and surface-area quadrature weights.
The bottom contact face is excluded. This case uses the saved nominal 36 mm width
and 16 mm height, with the issued point at the top and the body extending down its
commanded axis. Tests check samples against the DDS rounded-cylinder SDF formula.
This nominal reference is not reconstructed or measured material.

For each nominal sample, reject outside-image, back-facing, >75° grazing, or
recorded-mesh-blocked views. Remaining views are labeled inspection opportunities
with unknown space. They do not imply observed material, calibrated depth quality,
confirmed coverage, material identity, or safe camera travel. The 75° criterion is
an explicit demonstration setting, not a camera specification. Report area-weighted
opportunities in one / three views separately for top and sides. Rounded side
bands include upper and lower shoulders; lower shoulders may be inaccessible.

The recorded scene contains any material that was reconstructed, including the
bead and neighbors, so some nominal samples may be blocked by observed material
where nominal and actual geometry differ. The endpoint tolerance influences this
classification; nominal metrics currently use 4 mm. No pattern is automatically
selected from these metrics. Per-pose overlays distinguish nominal outline samples
from solid observed-patch marks. The earlier cap-only counts must not be used to
claim that either pattern covers the whole bead.

## Local scene assessment and direction selection

Run after the recorded comparison:

```sh
python tools/offline_inspection/assess_local_scene.py \
  --demo outputs/offline_inspection/c5-t1 \
  --settings tools/offline_inspection/local_scene_settings.json
```

This creates `local_scene.json` and refreshes the same viewer. The input scan,
issued target, neighbors, settings and source revisions are checked and hashed.
It remains a case-specific offline adapter, not a live action endpoint.

- Use the entire observed mesh for sightlines and proximity. Never delete a
  triangle because it lies inside the intended bead. Classify the first blocking
  point by spatial proximity to the nominal target and neighbor shapes: possible
  self-obstruction, neighbor region, overlapping/ambiguous regions or unassigned
  scene. These are geometric associations, not verified material ownership.
- Six same-cycle intended neighbors supply conservative enclosing spheres plus an
  explicit 8 mm uncertainty margin. A ray crossing such an envelope is marked as
  a possible obstruction even if the recorded mesh misses it; exclude it from the
  geometric proposal's opportunity score. Older intended beads are not supplied,
  but all their reconstructed geometry remains in the observed scene.
- Compute point-to-triangle distances, including triangle interiors, for an
  explicitly configured union of camera-envelope spheres. Reject observed surface
  intersections and possible intended-neighbor envelope contacts. No observed
  geometry or a missing camera envelope leaves clearance unverified and excludes
  the pose from selection. Positive surface separation is not a free-space or
  solid-volume containment test; unobserved obstacles and whole-tool geometry
  remain unverified.
- Generate 24 directions at 25/50/75 degrees about the bead axis, all at 400 mm.
  Assess each direction against nominal top/side normals, pinhole FOV, all scene
  triangles, intended-neighbor envelopes and the camera envelope. Select up to nine
  via deterministic greedy new side area, side repeats (capped at three), then top
  area. This is a declared geometry objective, not LLM candidate selection or a
  measured information-gain prediction. The deposition planner is untouched.
- Compare the grid, cap and selected directions with identical local-scene criteria.
  Recorded-camera visibility is also modeled for context. It does not count as
  raw-depth confirmation. Selection order is not a robot trajectory; IK, approach,
  travel, measured sensor quality and full tool/arm clearance are still unverified.

The demo sphere (75 mm radius, 50 mm behind the optical origin, 10 mm margin) is
an assumption rather than verified Femto or tool dimensions. The checked-in workcell
includes extruder collision geometry and an external camera xacro, but those have
not been transformed/validated against this recorded setup. Do not interpret this
sphere as covering the full assembly. All settings are saved in the result.

A separate proximity diagnostic measures distances to observed triangles whose
vertices are outside the target plus its association margin. That exclusion affects
only this diagnostic, never ray/clearance tests, and its minimum is strongly tied
to the chosen margin. It is not a measured bead-to-bead gap. The viewer labels it
accordingly and can display the nearest-surface connectors.

Viewer additions: intended neighbor outlines, local proposal, full direction pool,
first-obstruction rays, per-pose explanations, camera-envelope circles and nearby
surface distance links. No-hit outcomes retain the unknown-space label; every
proposal has `execution_authorized=false` and an empty robot command list.

## Keep the grid and add a small side-view supplement

```sh
python tools/offline_inspection/build_grid_supplement.py \
  --demo outputs/offline_inspection/c5-t1
```

The source-bound local-scene assessment is reused, so this step needs no new mesh
raycasts. It checks the assessment digest, original source hashes, cycle linkage
and viewer/grid consistency before creating `grid_supplement.json`. Recomputing
visibility or local-scene assessments removes downstream proposals from the viewer;
rerun this command to regenerate them.

The baseline is the proposed nine-view cropped grid from this demo, not the
original sixteen recorded captures. Its poses and order are preserved. For each
budget from zero through four, exhaustively search the eligible direction pool for
maximum *new* nominal side area beyond the grid. Reject zero-gain poses and
redundant sets. Ties prefer fewer additions, then the shortest camera-position
order starting at the last grid pose, then summed orientation change. With at most
four additions, all append permutations can be evaluated exactly. The pool is
bounded to 32 poses. An ineligible baseline pose produces a hold; it is not silently
removed from the grid. Candidate identity, frame, shape and revision errors fail.

The selected result is the smallest set attaining the best modeled new area within
this pool and four-view budget. There is no calibrated cost/benefit threshold:
coverage is prioritized before distance. The table exposes all budgets so a user
can compare a cheaper partial gain with the maximum. Repeated side/top coverage is
reported, but an extra pose solely repeating coverage is not selected in this
experiment. It would require a distinct inspection objective. Counts of three views
remain geometric opportunity counts, not independent physical measurements.

Cost is reported as extra captures, Euclidean camera-position polyline length,
and full SO(3) orientation change. It includes the grid-to-supplement connection,
but excludes a return, approach, acceleration, robot joint travel, collision-aware
transitions and elapsed time. This experiment chooses an endpoint sequence; it
cannot certify the connecting path. Camera-envelope assumptions and unknown-space
limitations from the local-scene stage still apply.

For C5:T1, grid-only modeled side opportunity is 26.7%. One/two/three added views
reach 34.4% / 36.1% / 37.5%, with 0.43 / 0.63 / 0.94 m of additional position-order
length. A fourth view adds no new side area, so the proposal stops at three. These
are nominal geometry results for one recorded case, not physical scan accuracy.
The viewer's extra-view budget selector switches the supplement and preserves the
grid, including the connecting line from its last pose.
