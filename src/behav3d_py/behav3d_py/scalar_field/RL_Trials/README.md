# RL Trials for DDS Bead Deposition

This directory contains the isolated experimental track for reinforcement-
learning-based bead placement. Code, configurations, logs, checkpoints, and
results specific to RL must remain under `RL_Trials/` unless integration with
an existing scalar-field or DDS API is strictly necessary.

The first learning approach will be PPO trained from scratch. The current
heuristic planner will be retained only as an evaluation baseline; it will not
be used for behavior cloning or policy initialization.

## Objective

Learn a sequential policy that deposits DDS beads until the goal geometry is
reached while balancing:

- progress over the goal surface,
- the intended heat-field order,
- agreement with the width field,
- sufficient contact with existing geometry,
- limited excessive overlap and overbuild,
- collision-free and reachable target poses,
- efficient use of bead count or deposited material.

The policy will have freedom to choose bead location and orientation inside a
geometrically safe action space. Safety and geometric validity are constraints,
not behaviors that PPO must learn by trial and error.

## Current Geometry Convention

The current scalar pipeline uses:

```text
phi = z_goal - z_scan - clearance
```

For the current vertical-ray implementation:

- `phi > iso`: viable/unprinted goal region;
- `phi <= iso`: printed/reached goal region;
- `phi << iso`: possible overbuild, depending on the selected tolerance;
- no scan/raycast hit: unresolved and not completed.

Before training, the terms `unprinted`, `filled`, and `overbuilt` must be used
explicitly in the RL code instead of relying on the ambiguous word `viable`.

## Selected Development Fixture

The machine-readable configuration has the general name
`configs/default_experiment.yaml`. Although its initial values point to the
`field_state_init2` development fixture, the same file structure can be copied
for other goal/scan/width combinations without tying the configuration name to
a particular mesh.

The first deterministic fixture is based on the existing DDS scalar-loop
inputs:

```text
field state: mesh/fields/field_state_init2.npz
width field: mesh/fields/width_field.npz
scan mesh:   mesh/ScanMesh.stl
scan scale:  0.001
scan yaw:    180 degrees
width range: 16-36 mm
nominal overlap reference: 4 mm
bead/contact height range: 10-16 mm
orientation cone: 30 degrees around world +Z
```

The field state and width map both contain 5721 aligned per-vertex entries.
The goal has 11147 faces, one connected component, an area of approximately
`0.08019 m^2`, and an open boundary, so it is not a watertight target volume.
After applying the requested scan scale and yaw, every goal vertex receives an
initial vertical-ray hit.

For the first evaluator, this fixture is therefore treated as an open 2.5D
target surface. The permitted material region is the vertical sweep between
the initial scan and the goal surface. A future directional DDS/SDF evaluator
will replace this assumption when testing geometry that is not representable
as one vertical height per `(x, y)` location.

The 10-16 mm height range initially has the following precise meaning:

1. PPO selects a goal anchor and deposition direction `Z`.
2. The decoder raycasts from the proposed top target in direction `-Z`.
3. The action is contact-valid only when the first hit is 10-16 mm away.
4. The hit distance becomes that deposit's DDS `BeadProfile.height`.

This makes the DDS bead extend from its top-referenced target back to existing
geometry without a gap. Height is derived from the proposed pose and contact
ray in the first version; it is not an additional independent PPO action.

The supplied `gradient_lift` command remains the heuristic baseline. Its
`beads_per_step=7` setting does not define the RL action: the RL environment
will use one point deposit per step. Because the command does not specify
`--dds-deposit-mode`, its current DDS default is `dot`.

## Per-Vertex Observation

The goal mesh is represented by vertices connected through its triangle faces.
Each goal vertex is one local measurement location, not one independent action.

For a goal vertex `i`, the initial observation proposal is:

| Feature | Meaning | Static or dynamic |
| --- | --- | --- |
| `position_local` | Goal-vertex XYZ in a normalized goal coordinate frame | Static |
| `normal` | Local goal-surface direction | Static |
| `neighbors` | Vertices connected to `i` by goal-mesh edges | Static |
| `heat_norm` | Desired sequence priority at that location | Static |
| `width_target` | Desired nominal bead width at that location | Static |
| `gap` | Remaining distance from existing material to the goal | Dynamic |
| `filled` | Whether `gap` is within the completion tolerance | Dynamic |
| `frontier` | Whether `i` is unfilled and adjacent to a filled region | Dynamic |
| `contact_reachable` | Whether a ray can reach existing geometry within the allowed range | Dynamic |
| `local_occupancy` | Amount/pattern of DDS material near `i` | Dynamic |
| `local_overlap` | Existing repeated coverage near `i` | Dynamic |
| `distance_last` | Distance from `i` to the previous deposited target | Dynamic |

The observation also needs episode-level values such as completed-area
fraction, step count, remaining bead/material budget, consecutive non-progress
steps, and the previous action.

Vertex contributions to goal completion must be weighted by their associated
mesh area. Counting vertices directly would make the result depend on mesh
triangulation density.

## Safe Action Space

The first environment will apply one point bead per RL step. The proposed
hybrid action is:

```text
anchor_id              discrete goal/frontier location
offset_tangent_1       bounded local positional adjustment
offset_tangent_2       bounded local positional adjustment
cone_tilt              bounded orientation tilt
cone_azimuth           orientation direction around the cone axis
bead_width             bounded DDS width control
```

The action decoder will:

1. resolve the selected anchor and its local frame;
2. apply the bounded tangential offset;
3. construct the deposition `Z` direction directly inside the allowed cone;
4. raycast opposite `Z` to find pre-existing geometry;
5. require that contact to fall inside the configured distance range;
6. compute the top-referenced DDS target `O` from the contact point and bead
   height;
7. create the candidate DDS `PointDeposit`;
8. validate DDS-domain containment, collision, and later robot reachability.

The decoder must not generate an arbitrary orientation and clamp it afterward.
Direct cone parameterization preserves useful PPO action gradients and avoids
many different raw actions collapsing to the same executed action.

Invalid actions leave the fabrication state unchanged, receive a bounded
invalid-action penalty, and are reported by reason. Repeated invalid actions
may terminate the episode. Collision and domain violations will never be
executed merely because their learned reward might be favorable.

Volume will not be an action in the first version. The current DDS geometry is
controlled by `BeadProfile(width, height)`, and the scalar DDS loop does not
apply YAML `volume_mm3` to the simulated geometry. Width is therefore the first
meaningful geometric control. Volume can be added after defining and testing a
calibrated mapping from process volume to bead width and height.

## Reward Draft

All reward components must be normalized to comparable ranges and logged
separately. The initial structure is:

```text
reward =
    progress_reward
  + heat_order_reward
  - overbuild_penalty
  - overlap_band_penalty
  - width_error_penalty
  - action_cost
  - invalid_action_penalty
```

Definitions:

- `progress_reward`: increase in area-weighted completed goal fraction;
- `heat_order_reward`: favors newly completed area with the intended heat
  priority, without requiring the heuristic contour-selection rule;
- `overbuild_penalty`: new material outside the permitted goal envelope;
- `overlap_band_penalty`: hinge penalty below or above a desired contact/
  overlap interval, so zero contact and excessive overlap are both undesirable;
- `width_error_penalty`: normalized difference between selected width and the
  width field sampled at the action anchor;
- `action_cost`: small bead/material cost that discourages endless episodes;
- `invalid_action_penalty`: applied when the safety decoder rejects an action.

“Minimizing empty goal difference,” “not advancing,” and “leaving gaps” must
not be added as three independent dense penalties because they measure nearly
the same failure. Dense area progress, a small action cost, and a terminal
remaining-gap penalty cover them without accidental triple weighting.

An episode succeeds when area-weighted completion exceeds its threshold while
overbuild and overlap remain inside their allowed limits. It terminates on
success, bead/material budget exhaustion, excessive consecutive non-progress,
or repeated invalid actions.

## Step-by-Step Implementation

### Phase 0 — Freeze definitions and fixtures

- [x] Select one small goal/scan pair as the deterministic development fixture.
- [x] Record the known units, DDS defaults, cone limit, bead-height range,
  width range, scan transform, and contact-ray interpretation.
- [x] Classify this goal as an open target surface and use a vertical swept
  region from the initial scan for the first 2.5D evaluator.
- [x] Define exact filled, overbuilt, overlap, and episode-success semantics.
- [x] Add a fixed random seed and a machine-readable fixture configuration.
- [x] Confirm or revise the proposed completion, overbuild, overlap, action,
  and episode-budget values listed in the fixture configuration.

Exit condition: the same geometry always produces the same initial metrics.

### Phase 1 — Goal completion evaluator

- [x] Implement area weights for goal vertices.
- [x] Implement the current vertical-ray `phi` evaluator as a regression
  backend.
- [x] Return explicit per-vertex `gap`, `filled`, `unfilled`, `overbuilt`, and
  `frontier` arrays.
- [x] Add summary metrics for completed area, remaining gap, and overbuild.
- [x] Add tests for empty, partially completed, completed, and overbuilt cases.

Exit condition: goal completion can be measured without running a policy.

Fixture evaluation command:

```bash
PYTHONPATH=/home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/RL_Trials \
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/RL_Trials/scripts/evaluate_fixture.py
```

Initial fixture result with a 2 mm fill tolerance and 4 mm overbuild
tolerance:

```text
resolved area:  100.0000%
completed area:   3.2404%
frontier area:    1.6929%
overbuilt area:   0.7952%
mean remaining gap: 129.617 mm
maximum remaining gap: 293.998 mm
```

### What the current tests cover

The tests are small deterministic geometry checks; they do not train PPO yet.
They verify:

- triangle area is distributed correctly to goal vertices, so completion is
  area-weighted rather than dependent on vertex density;
- ray misses remain unresolved and cannot count as completed goal;
- an empty/partial state produces the expected unfilled and frontier regions;
- the fill and overbuild tolerances classify known height samples correctly;
- vertical raycasting against a known plane returns the expected gap;
- visualization data is built as DDS `TriangleMesh` and `PointCloud` objects
  with the expected state colors and frontier points.

Run them with:

```bash
cd /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/RL_Trials
PYTHONPATH=. pytest -q
```

### DDS visualization

The initial state can be inspected interactively using DDS geometry and its
viewer:

```bash
PYTHONPATH=/home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/RL_Trials \
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/RL_Trials/scripts/visualize_goal_evaluation.py
```

The colors mean:

- orange: unfilled goal;
- green: filled goal;
- purple: overbuilt goal;
- dark blue-gray: unresolved goal;
- cyan points: fill frontier;
- transparent gray: current scan mesh.

For a headless run or a reproducible image, save a screenshot under the
isolated experiment folder:

```bash
PYTHONPATH=/home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/RL_Trials \
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/RL_Trials/scripts/visualize_goal_evaluation.py \
  --screenshot /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/RL_Trials/outputs/initial_goal_evaluation.png
```

Both modes use the same DDS `TriangleMesh`/`PointCloud` scene construction and
the exact arrays returned by the completion evaluator. The screenshot mode
uses DDS's PyVista conversion path to avoid requiring an interactive Qt
window. This visualization currently shows the initial scan evaluation; it is
also the base that will be updated with accumulated DDS beads in Phase 2.

### Phase 2 — Headless incremental DDS state

- [ ] Wrap the initial scan and mutable DDS bead field in an episode state.
- [ ] Apply one DDS deposit without YAML serialization or visualization.
- [ ] Compute action-local before/after occupancy for progress and overlap.
- [ ] Avoid marching-cubes mesh extraction on every training step.
- [ ] Support cheap `reset()` without leaking state between episodes.
- [ ] Add deterministic replay of an explicit action sequence.

Exit condition: thousands of bead steps can run reproducibly in headless mode.

### Phase 3 — Safe action decoder

- [ ] Build valid anchor candidates from unfilled, contact-reachable goal
  regions without using the old heat-minimum rule.
- [ ] Implement bounded tangential offsets.
- [ ] Implement direct cone parameterization for `Z`.
- [ ] Implement contact raycasting and top-referenced `O` construction.
- [ ] Add width bounds and `BeadProfile` creation.
- [ ] Reject no-contact, out-of-domain, degenerate, and collision actions with
  typed reason codes.
- [ ] Add collision checks against initial scan plus accumulated DDS geometry.

Exit condition: every accepted action is a valid DDS point deposit touching
pre-existing geometry.

### Phase 4 — Observation builder

- [ ] Produce static per-vertex features: normalized position, normal,
  connectivity, heat, width, and area.
- [ ] Produce dynamic features: gap, fill state, frontier, contact reachability,
  local occupancy/overlap, and distance to the previous bead.
- [ ] Build a fixed-size candidate set with padding and a validity mask for PPO.
- [ ] Add global episode features and normalize every numeric channel.
- [ ] Test that two different DDS states over the same goal produce different
  observations.

Exit condition: observation shape and semantics are stable across resets and
goal meshes.

### Phase 5 — Reward and termination

- [ ] Implement each reward component independently.
- [ ] Log raw and weighted reward terms on every step.
- [ ] Test progress monotonicity and ensure excessive width cannot exploit
  vertex-only completion.
- [ ] Test desired-overlap, zero-contact, excessive-overlap, gap, and overbuild
  scenarios.
- [ ] Implement success, budget, stagnation, and invalid-action termination.
- [ ] Create a reward-audit report for deterministic hand-authored actions.

Exit condition: simple good/bad action pairs are ranked correctly for explicit
geometric reasons.

### Phase 6 — Non-learning baselines

- [ ] Implement constrained random selection.
- [ ] Implement one-step greedy selection using the exact RL reward.
- [ ] Optionally implement short-horizon CEM or beam search.
- [ ] Evaluate the current heuristic only as an external baseline under the
  same metrics; do not use its actions as PPO training data.

Exit condition: PPO has reproducible lower and upper-reference baselines.

### Phase 7 — PPO environment and training

- [ ] Add the Gymnasium-compatible environment inside `RL_Trials/`.
- [ ] Implement a masked categorical anchor head and continuous parameter
  heads for offset, orientation, and width.
- [ ] Train PPO from random initialization; no behavior cloning.
- [ ] Begin with fixed `+Z` and fixed width, then progressively enable offset,
  cone orientation, and variable width.
- [ ] Run multiple headless environments with independent deterministic seeds.
- [ ] Save configs, checkpoints, learning curves, evaluation metrics, and
  action/reward traces under `RL_Trials/outputs/`.

Exit condition: PPO consistently exceeds constrained-random performance and
does not increase invalid-action rates during evaluation.

### Phase 8 — Generalization and fidelity

- [ ] Train and evaluate on separate goal meshes.
- [ ] Randomize scan error, bead width/height response, DDS threshold, and
  initial alignment within calibrated bounds.
- [ ] Add a DDS/SDF directional goal evaluator for geometry that is not a
  vertical height field.
- [ ] Compare the fast training evaluator against the full DDS surface-mesh and
  raycasting pipeline.
- [ ] Run voxel-resolution convergence checks.

Exit condition: held-out performance remains stable and the fast evaluator
agrees with full geometric validation within selected tolerances.

### Phase 9 — Robot-facing safety and integration

- [ ] Add IK, workspace, self-collision, environment-collision, and path-swept
  collision validation.
- [ ] Keep a non-learning safety shield between policy output and execution.
- [ ] Export accepted actions through the existing target/YAML contract only
  after simulation validation.
- [ ] Validate first in dry-run and simulation modes, then with conservative
  physical trials and post-deposition scans.

Exit condition: no raw policy action can bypass robot safety validation.

## Initial Experiment Scope

The first experiment intentionally uses:

- one fixed goal and scan fixture;
- point deposits only;
- one bead per environment step;
- fixed bead height;
- initially fixed width and `+Z` orientation;
- vertical-ray goal completion;
- direct DDS incremental updates;
- PPO from random initialization;
- the old heuristic only as an evaluation baseline.

This narrow start is a curriculum, not the final policy restriction. Freedom is
added one controlled dimension at a time so failures can be attributed to the
action space, evaluator, reward, or optimizer rather than all four at once.

## Proposed Directory Layout

Create subdirectories only when their first implementation is added:

```text
RL_Trials/
  .gitignore
  README.md
  configs/
    default_experiment.yaml
  rl_trials/
    fixture_io.py
    goal_evaluator.py
    visualization.py
    env.py
    dds_state.py
    action_decoder.py
    observations.py
    rewards.py
    policies.py
  tests/
    test_goal_evaluator.py
    test_visualization.py
  scripts/
    evaluate_fixture.py
    visualize_goal_evaluation.py
  outputs/                 # ignored/generated artifacts
```

No RL runtime dependency is added to the existing ROS or scalar-field packages
until the isolated environment and its dependency boundary have been tested.
