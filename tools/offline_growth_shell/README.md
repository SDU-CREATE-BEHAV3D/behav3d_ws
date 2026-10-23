# Offline recovery by revising the growth shell

This experiment uses the recorded scan-19 reconstruction, the recovered goal
field and Agent007's corresponded expected/observed shell proxies. It compares
five goal proposals at 0°, 4°, 8° and 12° controlled inward lean, then runs the
existing numerical candidate planner against each proposal and the edited scan.
No ROS nodes, robot clients, target-YAML exports or controller commands are used.

Open `outputs/offline_growth_shell/scan19/viewer.html`. The default is the 8° case
with a 40 mm return. It now opens on the complete next batch, with archived
candidates in amber, new candidates in cyan and dashed endpoint-displacement
lines. The selected comparison shows coordinates, tool directions, support,
clearance and spacing for the archived target, unchanged-goal rerun and revised
goal. Choose **Whole batch** to see every placement, or **Focus pair** to inspect
one change. Switch to **Local alternatives for C19:T7** to inspect one-at-a-time
replacement options. The viewer uses equal world-axis scales.

## Comparing old and new candidates

Three candidate sets distinguish the effects of changing the goal from rerunning
the planner:

1. All seven archived C19 targets retain their original positions and axes and
   are rescored on the same edited scan used by the new candidates.
2. The unchanged-goal rerun uses the current planner and current secondary rules
   on that scan. This is the control for the geometry revision.
3. The revised-goal batch uses those same rules and scan, changing only the goal
   field. Full retained batches are saved, including source points, start/end
   directions, retained generation indices and secondary-rule counts.

For complete batches, a maximum-cardinality one-to-one spatial association within
36 mm minimizes total endpoint distance. Reordering cannot create a false shift;
unmatched targets are reported on both sides. The association does not establish
bead identity, and its gate is not a robot motion limit. Each matched pair reports
signed XYZ change, total endpoint shift, normalized tool-axis angle change, support
count difference, sampled tool-clearance difference and batch-spacing difference.
Batch spacing uses the peers in each respective batch, so it can also change when
other targets move or are removed. The archived batch predates current endpoint
filters; removed targets must not be attributed automatically to recovery.

Local alternatives have a different contract: each replaces C19:T7 alone and is
checked with the same six archived peers reserved. Its comparison to C19:T7 uses
the same scan, support parameters and tool models. Alternatives are never pooled
into a printable batch. Their ranking is for inspection, not execution selection.

`candidate_comparison.json` provides all batches and alternatives at full precision,
their comparisons, checks, field revisions and source hashes. Coordinates are in
metres in `ur20_base_link`; `axis` is the unit tool Z direction, not a complete
robot pose. The file is an offline review artifact, not controller target YAML.
The viewer displays positions in millimetres. Original archived axes retain the
recording's rounding; recomputed axes use full numerical precision.

## What was tested

- Original goal: no field change.
- Keep the fitted offset: adapt the remaining goal to the regional displacement,
  without returning toward the original shape.
- Return over 20, 40 or 60 mm: begin at the fitted frontier displacement, then
  reduce that displacement smoothly toward zero as height increases.

The input retains three separate references: issued-candidate expected shell,
observed cap-derived middle proxies, and the original final goal field. The first
two estimate deviation; the third defines where the future design should return.
Agent007's expected shell is not relabelled as original goal CAD.

Seven upper-wall pairs marked quantitatively eligible by Agent007 are used:
C10:T5, C10:T6, C12:T7, C14:T7, C15:T7, C17:T7 and C18:T7. Only C18:T7 was observed
in scan 19. Older pairs remain historical evidence; they are not silently treated
as current measurements. All 122 expected tops in the evaluation are checked
against the archived candidate point files, establishing the coordinate mapping
to `ur20_base_link`. This does not independently verify sensor registration.

For controlled lean, the existing smooth anchored deformation edits the full
recorded mesh and retained cap locations. The producer's cap-to-middle translation
is reapplied after editing caps. This is a counterfactual input experiment, not a
new scan, a rerun of Agent007's detector, or evidence of physical deformation.

## Field construction and candidate checks

Horizontal shell displacement is fitted against expected cap height. Vertical
discrepancies are retained as diagnostics; the field's Z coordinates are unchanged.
The regional displacement is assumed laterally uniform at each height. That
assumption and extrapolation across the patch require further validation.

The recorded right-wall frontier provides a Y/height chart. At each chart location,
the fitted offset connects through a below-frontier numerical collar and decreases
above the frontier according to `1 - smoothstep(height / return_height)`. The
opposite wall and lower field remain protected. The maximum allowed field revision
is 12 mm. The collar only changes the planning reference; the printed mesh never
moves except for the explicitly declared controlled lean edit.

Vertex identities, mesh topology and original scalar ordering are preserved.
Scalar values travel with their vertices; the existing planner reevaluates surface
gradients on the revised mesh. This is an explicit transported-field strategy, not
a fresh heat-method solve using guessed historical seed settings. The original
field snapshot remains unchanged. Goal revisions and complete input/source files
are hash bound.

For each proposal, existing `compute_phi_mask`, `extract_phi_contour`,
`generate_print_points` and current target rules rebuild the frontier and produce
candidate alternatives. The experiment enumerates contour vertices within 20 mm
XY of archived C19:T7, testing each as one replacement while reserving the other
six archived endpoints. This is a new, declared search scope; counts must not be
compared directly with the prior 8 mm free-space or fixed-goal experiments.

Screens retain the borrowed 36 × 16 mm bead proxy, at least 6/9 footprint rays
within ±3 mm of the upper surface, 36 mm endpoint spacing, a 2 mm sampled tool
margin under both repository TCP models, and upward source-to-target progress.
No reflected or degenerate field faces may be introduced. A sampled clearance
pass does not establish continuous tool/arm collision freedom. The source-to-top
direction and clamped tool axis are distinct production-planner quantities; the
reported growth tilt is not the commanded tool tilt.

## Measured results

The complete candidate comparison at 8° lean / 40 mm return shows:

- Seven raw proposals become five retained targets under current endpoint-spacing
  rules. Archived C19:T3 and C19:T5 are unmatched in both reruns. This is a current
  filtering effect, not an effect of revising the growth shell.
- Four revised targets are exactly unchanged from the unchanged-goal rerun.
  Only the target spatially associated with C19:T7 changes: 1.98 mm in position
  and 2.28° in tool direction. Its nominal support drops from 4/9 to 2/9, while
  sampled tool clearance stays about 10.7 mm. The revised field does not improve
  this first candidate's nominal contact score.
- Compared with the archived C19:T7, that new target shifts by 3.65 mm
  (XYZ: −3.615, +0.126, +0.523 mm), with a 2.96° tool-axis change. Both score 2/9
  support on the edited scan. Its sampled tool clearance increases from 7.83 to
  10.74 mm. Comparing only with the archived batch would hide the support loss
  relative to regenerating the unchanged goal.
- Two of five full-batch targets pass the declared geometry screens, both in
  unaffected areas, under every tested strategy. The affected target fails.
  The separately ranked local return40 alternative reaches 5/9, but is still below
  the experimental 6/9 threshold and is not part of the ordinary batch.

Thus the field revision changes the intended continuation as designed, but the
existing planner's selection order does not optimize contact support. A future
selection policy can compare the local alternatives with that ordinary output;
it should not assume that following a smoother goal produces a better next bead.

At 8° controlled lean:

| Strategy | Maximum field change | Largest face rotation from original | Return completed in sampled columns | Maximum sampled roof offset | Best nominal support |
| --- | --- | --- | --- | --- | --- |
| Original goal | 0 mm | 0° | Not applicable | Not applicable | 5/9 |
| Keep offset | 6.08 mm | 17.23° | 0/17 | 6.08 mm | 5/9 |
| Return over 20 mm | 6.06 mm | 19.96° | 17/17 | 0 mm | 4/9 |
| Return over 40 mm | 6.06 mm | 9.95° | 17/17 | 0 mm | 5/9 |
| Return over 60 mm | 6.06 mm | 11.44° | 0/17 | 1.17 mm | 5/9 |

The 40 mm proposal is a useful geometry alternative: it completes the sampled
return with less maximum face rotation than the 20 mm proposal. This is not a
selected printable strategy or a stiffness/curvature/stability assessment.
The 60 mm proposal cannot complete its return before the existing goal ends.
Completeness is checked in 17 Y columns using 6 mm vertex neighborhoods; it is
not a proof over every boundary point.

Mapped original-frontier anchors have a median distance of 3.91 mm from the edited
scan under the original goal, versus 1.29 mm after the fitted revision. This
diagnostic is measured **before regenerating contours**: it describes tracking of
the controlled change, not candidate-base support. Both planners independently
rebuild their actual candidate frontier afterward. The horizontal shell-fit RMS
residual is 0.82 mm, not a calibrated uncertainty bound.

All tested fields retain Z coordinates and contain no reversed/degenerate faces.
All generated local alternatives advance upward. None passes the declared bead
geometry screens; insufficient nominal support is the rejection reason. The
unmodified control also reaches only 5/9 at best, confirming that this support
model cannot yet be treated as a reliable printability classifier. The zero-lean
case retains real recorded shell discrepancies; it does not fabricate a failure
or automatically trigger a goal revision.

## What this establishes—and what remains open

The experiment establishes a reproducible connection from shell comparison to a
versioned growth field and real rule-based candidate generation. It makes return
height, final deviation, field distortion and first-deposit geometry inspectable.
It does not establish that the wall will stand or that a repair will succeed.

Every case keeps structural stability, material response, future deposition
support, bead-model calibration, avoidance state, IK and full tool clearance as
pending checks. No future bead is inserted into the scan to manufacture support
for the next proposal. Mesh self-intersection checks are also pending. No strategy
is selected for execution, even if a later geometry test passes.

Next development should validate the bead/contact construction against ordinary
recorded deposition and use measured material/process parameters for a structural
check. A structural result needs to consider the already leaning wall and the
added growth together. A visually smooth or upward-only shell is insufficient.
Full-frontier inspection and a readback checkpoint remain part of any subsequent
bounded recovery workflow.

## Reproduce

Uses the same numerical dependencies as `../offline_planner_replay/README.md`.
The existing DDS environment supplies NumPy, SciPy, Open3D and PyYAML; potpourri3d
was installed only into a temporary dependency directory. Omit `--dependency-path`
when it is already available in the chosen environment.

```sh
/Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python -B \
  tools/offline_growth_shell/build_experiment.py \
  --evaluation '/Users/josephnamar/Documents/ChatGPT/SustaniAi sumerschool/agent007-main/outputs/integrated_evaluation_260728_133013_final/cycle_0019/evaluation/evaluation.json' \
  --dependency-path /private/tmp/behav3d_planner_dependencies

/Users/josephnamar/Documents/GitHub/3DP-DDS/.venv/bin/python -B \
  -m unittest discover -s tools/offline_growth_shell -p 'test_*.py' -v
```

Outputs: `results.json` with every proposal/check and provenance;
`candidate_comparison.json` with complete candidate batches and pair comparisons;
`shell_evidence.json` with original corresponded inputs and evidence age;
`proposed_fields.npz` with full-precision revised fields, unchanged topology and
transported scalar; self-contained `viewer.html`; and an output hash manifest.
Numerical queries use full meshes. Display points/field coordinates are rounded
to one micrometre, and the displayed mesh wireframe is cropped/thinned only for
readability. Original source data and previous replay artifacts stay unchanged.
