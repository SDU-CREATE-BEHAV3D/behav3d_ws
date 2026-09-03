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
replaced. The future environment will leave DDS state unchanged and apply the
configured penalty.

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
| `width_ctrl` | Bounded correction to the width field interpolated at the source |

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
boundary can produce a spatial jump, but the future observation will expose the
current contour and the coordinate assigned to every contour sample.

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

1. Raycast from `O` along `-Z` against the current scan/material surface.
2. Verify contact distance and return proximity to the intended source `S`.
3. Verify that the conservative DDS bead support bounds fit in the episode
   domain.
4. Run the existing extruder collision proxy.

Stable rejection codes are:

```text
no_contact
contact_distance_out_of_bounds
contact_source_mismatch
out_of_domain
collision
```

Collision is evaluated only after contact and domain checks pass. The future
environment contract is:

```text
valid action   -> apply its DDS PointDeposit -> update state
invalid action -> apply nothing              -> keep state -> penalize reason
```

Robot reachability, self-collision, environment collision, and swept-path
checks remain future safety layers. No raw policy action may bypass them when
robot-facing integration begins.

## Observation draft

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

Global features will include completed-area fraction, step and material
budgets, consecutive invalid/non-progress steps, and the previous action.

The future observation should also expose deterministic samples from the union
of current contour components. Each sample should include its normalized
`source_coord`, 3D position, heat, target width, normal, local occupancy, and a
validity mask. `ContourParameterization.sample` and
`sample_contour_observation` now implement the fixed-budget geometric, heat,
and width portion. Samples condition PPO and explain what each source interval
means in the current state; they are not candidate actions.

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

The implemented stability costs are:

```text
normalized_tilt_cost = (theta / theta_max)^2
cantilever_ratio_cost = (lateral_shift / bead_width)^2
```

Their weights should remain weak. They prefer near-vertical deposition when
other outcomes are equal without preventing useful tilt for progress,
overlap, or collision avoidance. Completion progress, remaining gap, and
stagnation must not be represented by several redundant dense penalties.

## Current implementation

Implemented:

- area-weighted goal completion and vertical-ray gap evaluation;
- explicit filled, unfilled, unresolved, and frontier state;
- deterministic five-dimensional direct action decoding;
- canonical 3D arc-length parameterization independent of face ordering;
- deterministic concatenation of disconnected contour components;
- fixed-budget component-aware contour samples whose coordinates round-trip
  through the same parameterization used by the decoder;
- direct height, cone orientation, and width-field correction;
- conversion of one decoded action to one DDS `PointDeposit`;
- typed contact, DDS-domain, and extruder-collision validation;
- mutable incremental DDS episode state with cheap reset;
- accepted-deposit application and mutation-free physical rejection;
- immutable DDS snapshots with implicit and additive-coverage fields;
- current raycast geometry built from the initial scan and DDS surface;
- goal/phi/contour recomputation from the current material geometry;
- pure tilt and cantilever reward components;
- goal-state and direct-action visualization.

Not yet implemented:

- application/rejection of actions inside `env.step`;
- dynamic observations over accumulated DDS material;
- complete reward aggregation and termination;
- Gymnasium environment and PPO training;
- collision rebuilding against accumulated DDS geometry;
- robot reachability and swept-path validation.

## Setup and tests

3DP-DDS requires Python 3.9 or newer. From this directory, install the local
DDS visualization extras and the scalar-pipeline dependencies in an isolated
environment:

```bash
python -m pip install pytest open3d potpourri3d pyyaml pycollada
python -m pip install -e "../../../../../external/3DP-DDS[viz]"
```

Run the complete deterministic suite from `RL_Trials/`:

```bash
python -m pytest tests -v
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
- accepted/rejected DDS state transitions, immutable snapshots, reset, and
  current-material mesh construction;
- tilt and cantilever reward costs.

## Visualization

Visualize the evaluated goal and current `phi=0` contour:

```bash
python scripts/visualize_goal_evaluation.py
```

Visualize one explicit direct action (the default is the neutral action):

```bash
python scripts/visualize_direct_action.py --action 0 0 0 0 0
```

Example with a different source, tilt, and width correction:

```bash
python scripts/visualize_direct_action.py --action -0.4 0.2 0.6 -0.3 -0.2
```

Save reproducible headless renders:

```bash
python scripts/visualize_goal_evaluation.py \
  --screenshot outputs/goal_evaluation.png
python scripts/visualize_direct_action.py \
  --action 0 0 0 0 0 \
  --screenshot outputs/direct_action.png
```

Direct-action colors:

- magenta: exact interpolated `phi=0` contour;
- cyan: decoded source `S`;
- green: accepted proposed action;
- blue: contact rejection;
- orange: DDS-domain rejection;
- red: collision rejection.

The proposed bead is rendered for diagnosis even when rejected, but the script
does not update episode state or execute the deposit.

## Roadmap

### 1. Gymnasium integration of the headless episode state

- Connect `DDSEpisodeState` to `env.reset()` and `env.step()`.
- Rebuild contact/collision helpers from the current material mesh when its
  geometry revision changes.
- Compute local before/after occupancy for progress and overlap.
- Add deterministic replay of explicit action sequences.

### 2. Observation and reward

- Build static and dynamic per-vertex features.
- Add normals and current-material occupancy to the implemented contour
  position/heat/width samples.
- Add normalized global episode features.
- Implement and log each reward term independently.
- Audit hand-authored good/bad direct actions before training.

### 3. Gymnasium and PPO

- Implement `Box(-1, 1, shape=(5,))` as the only policy action space.
- Connect `env.step(action)` to decode, validate, DDS update/rejection, reward,
  and observation.
- Train PPO from scratch without behavior cloning.
- Compare only against the external heuristic under identical metrics.

### 4. Generalization and robot safety

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
    rewards.py
    visualization.py
  tests/
    conftest.py
    test_action_decoder.py
    test_action_validation.py
    test_contour_parameterization.py
    test_episode_state.py
    test_goal_evaluator.py
    test_rewards.py
    test_visualization.py
  scripts/
    evaluate_fixture.py
    visualize_direct_action.py
    visualize_goal_evaluation.py
  outputs/                 # ignored generated artifacts
```

## Handoff note

Branch: `dev_luc_rl_direct_action`.

Implemented: the policy emits `[source, height, orient_x, orient_y, width]` in
`[-1, 1]^5`. `source` deterministically addresses the canonical 3D `phi=0`
contour by arc length, including disconnected components. Random candidates
were removed from the PPO path. Validation, incremental DDS episode state,
reset/rejection behavior, deterministic contour observations, tests, and the
direct-action visualizer are in place.

Next: implement the Gymnasium `reset()`/`step()` environment, refresh validators
when `geometry_revision` changes, finish observation/reward assembly, and
connect PPO training. Current goal evaluation uses vertical ray casting; the
scan plus DDS surface is a triangle soup, not a boolean mesh union. Start with
`python -m pytest tests -v`.
