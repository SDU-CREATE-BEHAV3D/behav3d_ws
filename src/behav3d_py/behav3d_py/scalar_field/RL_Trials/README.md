# RL Trials for Direct DDS Bead Deposition

This directory is the isolated experimental track for reinforcement-learning
control of DDS bead deposition. RL-specific code, configuration, tests,
checkpoints, and results remain under `RL_Trials/` until an integration with an
existing scalar-field or DDS API is necessary.

The policy formulation is direct and continuous. PPO does not rank or select
from a generated pool of targets.

## Architecture

The intended step pipeline is:

```text
observation
  -> PPO continuous action
  -> deterministic geometric decoder
  -> contact/domain/collision validation
  -> DDS deposit or rejection
  -> reward and next observation
```

For a fixed state and action, decoding and validation are deterministic. An
invalid action is reported by reason, is not corrected, and must not be
replaced. Width is different: `width_ctrl` is parameterized directly over the
locally feasible intersection between its configured correction and the
physical bead-width bounds. Therefore a normalized policy action cannot
produce an invalid width. The downstream validator retains a defensive check
for malformed actions created outside the decoder. Other rejected actions
leave DDS state unchanged and receive the configured penalty.

The previous generated action-pool path has been removed. The existing
`gradient_lift` planner remains only as an external evaluation baseline; it is
not policy input, training data, or behavior-cloning initialization.

## Continuous action space

One PPO step emits:

```text
a = [source_coord, height_ctrl, orient_x, orient_y, width_ctrl]
a in [-1, 1]^5
```

The components map as follows:

| Component | Mapping |
| --- | --- |
| `source_coord` | Cumulative physical arc length over canonically ordered 3D contour components |
| `height_ctrl` | Linear map to the configured bead-height interval |
| `orient_x`, `orient_y` | Concentric square-to-disk map followed by the configured orientation cone |
| `width_ctrl` | Correction mapped into the locally feasible intersection of the configured delta and physical width bounds |

`orient_x=orient_y=0` is exactly vertical. Increasing their magnitude
increases tilt continuously; their direction controls azimuth. The decoder
does not clamp arbitrary orientations into the cone because that would make
different policy outputs collapse to the same executed action.

The normalized `source_coord` maps linearly to `[0, total_contour_length]`.
Before applying it, the decoder welds duplicated endpoints from the extracted
segment soup, traces every non-branching component continuously, chooses a
canonical direction, and sorts disconnected components by their minimum 3D
coordinate. This makes the mapping independent of mesh-face iteration order.

The components are then concatenated deterministically. Crossing a component
boundary can produce a spatial jump, but the implemented observation exposes
the current contour and the coordinate assigned to every contour sample.

Given the selected height `h` and unit deposition direction `Z`, the
top-referenced DDS target is:

```text
O = S + h Z
```

The same decoded values are used by validation, DDS conversion, logging, and
visualization. `rl_trials.dds_adapter.point_deposit_from_action` is the single
conversion point from `DecodedAction` to a DDS `PointDeposit`.

The parameterization is deterministic for a fixed state. It does not use heat
or reward values to reorder the contour, so those fields inform PPO without
silently changing the meaning of its action.

## Geometry convention

The current scalar pipeline uses:

```text
phi = z_goal - z_scan - clearance
```

For the initial vertical-ray evaluator:

- `phi > iso`: unfilled goal region;
- `phi <= iso`: filled/reached goal region;
- no scan/raycast hit: unresolved and never completed.

Goal vertices are area-weighted measurement locations, not print targets. The
auxiliary mesh frontier is useful as an observation/debug signal, but the
continuous union of interpolated `phi=0` segments remains the deposition
boundary.

The selected development fixture is configured in
`configs/default_experiment.yaml`:

```text
field state:          mesh/fields/field_state_init2.npz
width field:          mesh/fields/width_field.npz
scan mesh:            mesh/ScanMesh.stl
scan scale/yaw:       0.001 / 180 degrees
bead width:           16-36 mm
bead height/contact:  10-16 mm
orientation cone:     30 degrees around world +Z
DDS voxel size:       2 mm
```

Source selection is defined on 3D contour segments: components may overlap in
`(x, y)`, vary in `z`, and be disconnected. The current fixture completion
evaluator is still based on vertical rays and therefore assumes one relevant
height per `(x, y)` sample. A directional DDS/SDF evaluator will be needed
before the complete state and reward pipeline supports arbitrary 3D surfaces.

## Validation and rejection

`validate_decoded_action` consumes exactly one `DecodedAction`. It does not
generate alternatives. Validation proceeds in this order:

1. Verify that the selected width is inside the physical bead-width range.
2. Raycast from `O` along `-Z` against the current scan/material surface.
3. Verify that the first hit lies inside the configured contact-distance range.
4. Verify that the conservative DDS bead support bounds fit in the episode
   domain.
5. Run the existing extruder collision proxy.

The sampled contour point `S` anchors construction of `O = S + hZ`; it is not
required to be the raycast hit point. The distance from the hit back to `S` is
logged only as a diagnostic. Contact accepts the first hit on any preexisting
material inside 10–16 mm. The configured 0.001 mm epsilon exists only for
floating-point comparison at the two bounds and is not geometric clearance.

The Open3D acceleration scene is cached by DDS `geometry_revision`. Rejected
actions reuse it because they do not modify material; an accepted deposit
invalidates it and the next geometric query rebuilds it once.

Stable rejection codes are:

```text
invalid_width  # defensive only; normalized policy actions cannot produce it
no_contact
contact_distance_out_of_bounds
out_of_domain
collision
```

Collision is evaluated only after width, contact, and domain checks pass. The
implemented environment contract is:

```text
valid action   -> apply its DDS PointDeposit -> update state
invalid action -> apply nothing              -> keep state -> penalize reason
```

Robot reachability, self-collision, environment collision, and swept-path
checks remain future safety layers. No raw policy action may bypass them when
robot-facing integration begins.

## Observation

Each goal vertex is a local measurement location. Proposed features are:

| Feature | Meaning | Kind |
| --- | --- | --- |
| `position_local` | Normalized goal-space XYZ | Static |
| `normal` | Goal-surface direction | Static |
| `neighbors` | Goal-mesh connectivity | Static |
| `area_weight` | Associated surface area | Static |
| `heat_norm` | Desired sequence priority | Static |
| `width_target` | Nominal local bead width | Static |
| `gap` | Remaining distance to goal | Dynamic |
| `filled` | Gap within completion tolerance | Dynamic |
| `frontier` | Unfilled vertex adjacent to filled region | Dynamic |
| `contact_reachable` | Existing material reachable inside height bounds | Dynamic |
| `local_occupancy` | Nearby DDS material | Dynamic |
| `local_overlap` | Repeated local coverage | Dynamic |
| `distance_last` | Distance to previous deposited target | Dynamic |

The current Gymnasium observation is a fixed `spaces.Dict`:

```text
contour: (256, 8) float32
global:  (12,)   float32
```

Each contour row contains normalized source coordinate, XYZ, heat, target
width, component index, and validity mask. The global vector contains
completion, accepted/attempted budgets, invalid/non-progress streaks, contour
availability/component count, and the previous action. A state with no contour
uses a zero-filled contour observation and mask.

The implemented observation exposes deterministic samples from the union
of current contour components. Each sample should include its normalized
`source_coord`, 3D position, heat, target width, component index, and a
validity mask. `ContourParameterization.sample` and
`sample_contour_observation` now implement the fixed-budget geometric, heat,
and width portion. Samples condition PPO and explain what each source interval
means in the current state; they are not candidate actions.

Goal normals, local DDS occupancy/overlap, and the larger per-vertex feature
set listed above remain planned additions rather than claims about the current
tensor.

## Reward draft

Raw reward components must be normalized and logged separately. The intended
shape is:

```text
reward =
    progress_reward
  + heat_order_reward
  - overlap_band_penalty
  - width_error_penalty
  - action_cost
  - invalid_action_penalty
  - collision_penalty
  - lambda_tilt * normalized_tilt_cost
  - lambda_cantilever * cantilever_ratio_cost
```

Every configured `*_penalty` is a positive cost magnitude. The reward assembly
subtracts it; for example, a collision applies `-10`, not a double-negative
reward.

The implemented stability costs are:

```text
normalized_tilt_cost = (theta / theta_max)^2
cantilever_ratio_cost = (lateral_shift / bead_width)^2
```

Their weights should remain weak. They prefer near-vertical deposition when
other outcomes are equal without preventing useful tilt for progress,
overlap, or collision avoidance. Completion progress, remaining gap, and
stagnation must not be represented by several redundant dense penalties.

The Gym environment currently aggregates a deliberately provisional subset:

```text
reward =
    100 * delta_completed_area_fraction
  - 1 * invalid_action
  - 10 * collision
  - 0.01 * normalized_tilt_cost
  - 0.01 * cantilever_ratio_cost
```

Stability terms apply only to accepted deposits. Collision also receives the
generic invalid-action cost, so its current total base consequence is `-11`.
Heat ordering, overlap-band quality, local occupancy, and width adaptation are
not yet aggregated; these values must be audited before a long PPO run.

## Gymnasium environment

`DirectBeadDepositionEnv` implements the standard five-value `Box(-1, 1)`
action space and the complete headless transition:

```text
reset -> current DDS/goal/contour observation
step  -> decode -> validate -> deposit or reject -> reward -> next observation
```

Success terminates at the configured completed-area fraction. Attempt/deposit
budgets and consecutive invalid/non-progress limits truncate the episode. A
collision can optionally terminate it, but the current fixture keeps the
episode alive and applies the strong penalty.

## Current implementation

Implemented:

- area-weighted goal completion and vertical-ray gap evaluation;
- explicit filled, unfilled, unresolved, and frontier state;
- deterministic five-dimensional direct action decoding;
- canonical 3D arc-length parameterization independent of face ordering;
- deterministic concatenation of disconnected contour components;
- fixed-budget component-aware contour samples whose coordinates round-trip
  through the same parameterization used by the decoder;
- direct height, cone orientation, and physically safe local width-field
  correction;
- conversion of one decoded action to one DDS `PointDeposit`;
- typed contact, DDS-domain, and extruder-collision validation;
- defensive invalid-width rejection for externally malformed decoded actions;
- mutable incremental DDS episode state with cheap reset;
- accepted-deposit application and mutation-free physical rejection;
- immutable DDS snapshots with implicit and additive-coverage fields;
- current raycast geometry built from the initial scan and DDS surface, with
  one reusable acceleration scene per geometry revision;
- goal/phi/contour recomputation from the current material geometry;
- pure tilt and cantilever reward components;
- Gymnasium `reset()`/`step()` with fixed Dict observations, reward-component
  logging, termination, truncation, and lazy collision-checker rebuilding;
- Stable-Baselines3 PPO from-scratch training with a tanh-squashed continuous
  policy, CPU defaults, per-rollout geometry metrics, checkpoints, and resume;
- deterministic or stochastic evaluation of saved PPO models;
- deterministic and seeded random headless rollout script;
- goal-state and direct-action visualization.

Not yet implemented:

- goal-normal and local DDS occupancy/overlap observation features;
- heat-order, overlap-band, width-adaptation, and action-efficiency reward
  terms;
- long PPO training, reward tuning, and held-out policy comparison;
- robot reachability and swept-path validation.

## Setup and tests

3DP-DDS requires Python 3.9 or newer. From this directory, install the local
DDS visualization extras and the scalar-pipeline dependencies in an isolated
environment. `requirements.txt` pins the CPU build of PyTorch so a CUDA toolkit
is not downloaded for this experiment:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pip install -e "../../../../../external/3DP-DDS[viz]"
```

Run the complete deterministic suite from `RL_Trials/`:

```bash
.venv/bin/python -m pytest tests -v
```

Test files are pytest modules, not visualization programs. Running
`python tests/test_visualization.py` directly does not invoke pytest or set the
project import path correctly.

The tests cover:

- area-weighted completion, frontier classification, and unresolved rays;
- vertical raycasting against known scan geometry;
- DDS-native goal visualization data;
- direct action determinism and normalized bounds;
- continuous canonical traversal of unordered contour segment soups;
- disconnected 3D components and line-order-independent source selection;
- deterministic component-aware observation samples and source round-trips;
- direct orientation, height, target, and width construction;
- DDS `PointDeposit` conversion;
- contact, domain, and collision rejection without action replacement;
- feasible width parameterization at both physical bounds;
- accepted/rejected DDS state transitions, immutable snapshots, reset, and
  current-material mesh construction;
- ray-direction normalization and raycaster reuse across unchanged geometry;
- Gymnasium API compliance, valid DDS application, safe width extremes, fixed
  observation shapes, reward accounting, and truncation;
- bounded PPO actions, rollout-size validation, metric collection, and a short
  optimizer update;
- tilt and cantilever reward costs.

Run three deterministic neutral actions on the real fixture:

```bash
.venv/bin/python scripts/run_gym_rollout.py --steps 3
```

For a faster geometry-only smoke test, add `--disable-collision`. Use
`--random --seed 7` for reproducible Box samples, or pass one explicit action:

```bash
.venv/bin/python scripts/run_gym_rollout.py \
  --steps 3 \
  --action -0.4 0.2 0.6 -0.3 -0.2
```

## PPO training and evaluation

The training policy is `MultiInputPolicy` over the Dict observation. It uses
gSDE with `squash_output=True`, so the action distribution itself maps through
`tanh` into `[-1, 1]^5`; actions are not sampled outside the geometric space
and clipped afterward. Initial gSDE exploration is configured per action:
`[-1.5, -2.5, -2.0, -2.0, -2.5]` for source, height, the two orientation
controls, and width. This keeps broad contour exploration while sampling
height and width through the interior instead of saturating their minimum or
maximum controls. The physical contact-distance check remains strictly inside
the configured 10–16 mm range, without an added boundary tolerance.
New gSDE noise is sampled every environment step so a rejected proposal is not
repeated merely because exploration noise was being held for several steps.

Run a small end-to-end smoke training with the real fixture and collision
proxy enabled:

```bash
.venv/bin/python scripts/train_ppo.py \
  --run-name smoke_ppo_mytest_seed0 \
  --total-timesteps 128 \
  --n-steps 64 \
  --batch-size 32 \
  --n-epochs 2 \
  --checkpoint-freq 64 \
  --seed 0
```

Choose a new run name when repeating an experiment, or omit `--run-name` to
generate a timestamped one automatically.

The default invocation runs 10,240 transitions with rollouts of 128 and batch
size 64:

```bash
.venv/bin/python scripts/train_ppo.py --run-name ppo_reward_draft_seed0
```

This is currently an infrastructure/reward-audit run, not a final policy run.
Heat order, overlap quality, and width adaptation are still absent from the
aggregated reward. Running millions of transitions before adding and auditing
those terms would optimize the wrong objective.

Each run is isolated under `outputs/training/<run-name>/` and contains the
exact CLI metadata, a copy of the experiment YAML, `monitor.csv`, per-rollout
CSV logs, periodic checkpoint ZIPs, the final model ZIP, and a training
summary. A non-empty run directory is never overwritten. Geometry acceptance,
typed rejection rates, and every implemented reward component are logged
separately.

Continue from a saved model into a new run directory. Here,
`--total-timesteps` means additional transitions; PPO optimizer and rollout
hyperparameters are restored from the saved model:

```bash
.venv/bin/python scripts/train_ppo.py \
  --run-name ppo_reward_draft_seed0_resume1 \
  --resume outputs/training/ppo_reward_draft_seed0/final_model.zip \
  --total-timesteps 10240
```

Evaluate the deterministic policy while keeping collision enabled:

```bash
.venv/bin/python scripts/evaluate_ppo.py \
  outputs/training/ppo_reward_draft_seed0/final_model.zip \
  --episodes 3 \
  --seed 1000
```

Use `--stochastic` only when sampling policy behavior deliberately. Both
training and evaluation accept `--disable-collision` for geometry debugging,
but such runs are not evidence of a collision-aware policy.

## Visualization

Visualize the evaluated goal and current `phi=0` contour:

```bash
.venv/bin/python scripts/visualize_goal_evaluation.py
```

Visualize one explicit direct action (the default is the neutral action):

```bash
.venv/bin/python scripts/visualize_direct_action.py --action 0 0 0 0 0
```

Example with a different source, tilt, and width correction:

```bash
.venv/bin/python scripts/visualize_direct_action.py --action -0.4 0.2 0.6 -0.3 -0.2
```

Save reproducible headless renders:

```bash
.venv/bin/python scripts/visualize_goal_evaluation.py \
  --screenshot outputs/goal_evaluation.png
.venv/bin/python scripts/visualize_direct_action.py \
  --action 0 0 0 0 0 \
  --screenshot outputs/direct_action.png
```

Direct-action colors:

- magenta: exact interpolated `phi=0` contour;
- cyan: decoded source `S`;
- green: accepted proposed action;
- blue: contact rejection;
- orange: width or DDS-domain rejection;
- red: collision rejection.

The proposed bead is rendered for diagnosis even when rejected, but the script
does not update episode state or execute the deposit.

## Roadmap

### 1. Complete observation and reward

- Compute local before/after occupancy for progress and overlap.
- Build static and dynamic per-vertex features.
- Add normals and current-material occupancy to the implemented contour
  position/heat/width samples.
- Add normalized global episode features.
- Implement and log each reward term independently.
- Audit hand-authored good/bad direct actions before a long training run.

### 2. PPO policy experiments

- Run controlled PPO seeds from scratch without behavior cloning.
- Add component-boundary and source-coordinate stability diagnostics.
- Compare only against the external heuristic under identical metrics.

### 3. Generalization and robot safety

- Separate training and held-out goal meshes.
- Add calibrated geometry/process perturbations.
- Compare the fast evaluator with full DDS surface evaluation.
- Add IK, workspace, collision, and swept-path safety before robot execution.

## Directory layout

```text
RL_Trials/
  configs/
    default_experiment.yaml
  rl_trials/
    action_decoder.py
    action_validation.py
    contour_observation.py
    contour_parameterization.py
    dds_adapter.py
    episode_state.py
    fixture_io.py
    goal_evaluator.py
    gym_env.py
    ppo_training.py
    raycasting.py
    rewards.py
    visualization.py
  tests/
    conftest.py
    test_action_decoder.py
    test_action_validation.py
    test_contour_parameterization.py
    test_episode_state.py
    test_goal_evaluator.py
    test_gym_env.py
    test_ppo_training.py
    test_raycasting.py
    test_rewards.py
    test_visualization.py
  scripts/
    evaluate_ppo.py
    evaluate_fixture.py
    run_gym_rollout.py
    train_ppo.py
    visualize_direct_action.py
    visualize_goal_evaluation.py
  requirements.txt
  outputs/                 # ignored generated artifacts
```

## Handoff note

Branch: `dev_luc_rl_direct_action`.

Implemented: the policy emits `[source, height, orient_x, orient_y, width]` in
`[-1, 1]^5`. `source` deterministically addresses the canonical 3D `phi=0`
contour by arc length, including disconnected components. Random candidates
were removed from the PPO path. Validation, incremental DDS episode state,
reset/rejection behavior, deterministic contour observations, Gymnasium
transitions, tests, rollout diagnostics, and the direct-action visualizer are
in place.

Next: add local DDS occupancy/overlap to observations and rewards, audit heat
ordering and width adaptation, then run controlled PPO comparisons. Current goal
evaluation uses vertical ray casting; the scan plus DDS surface is a triangle
soup, not a boolean mesh union. Start with `.venv/bin/python -m pytest tests -v`.
