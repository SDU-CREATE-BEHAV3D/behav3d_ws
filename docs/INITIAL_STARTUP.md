# Initial scene review and first wall candidates

The opt-in startup workflow connects Agent007 scene evidence to a BEHAV wall
placement proposal and, after exact operator approval, a first **candidate
preview**. It never starts cameras, motion, extrusion or the normal print loop.
Native Ubuntu 24.04 and ROS 2 Jazzy remain the deployment target; Docker is
optional Mac validation tooling.

The recorded-scene startup milestone was accepted on 30 October 2026: separate
human placement approval and batch acceptance produced seven audited preview
targets and ended `OFFLINE_REVIEW_COMPLETE`, `DISARMED`, without robot commands
or fabrication authority. This is the initial scene-to-candidate review loop,
not yet the complete print-and-rescan loop.

## Agent responsibilities

Agent007's existing scene workflow detects and reconstructs objects, produces
support topography and suitability maps, and optionally interprets scene
semantics. Inventory review remains upstream. The new `ocsr.initial_scene`
adapter freezes a completed scene, its source files, geometry and producer
identities. This adapter reuses recorded perception; it does not itself launch
new detection, reconstruction or LLM interpretation. A real new scan must first
complete the existing reviewed OCSR workflow.

The robotic startup policy consumes that frozen scene and an explicit wall task.
It searches bounded XY/yaw placements on eligible primary-surface regions and
separate review views of eligible secondary substrates,
checks the entire footprint and margin against exact grid-cell intersections,
and screens available object samples throughout the wall's intended height.
Unknown cells, inconsistent duplicate cells, unsuitable surface ownership,
obstacles and low quality reject a placement. A sloping surface is not rejected
solely because its global height range exceeds 5 mm. Duplicate-layer separation,
local triangle continuity and the initialized frontier have separate bounds.
The search is not exhaustive over continuous poses; a budget-limited failure
must not be described as proof that no placement exists.

Additional existing objects are assessed but not automatically selected as
structural supports. The result abstains when intent, material compatibility,
adhesion or capacity are unverified. The print substrate is separate from
an additional structural support. Secondary substrates must be geometry-eligible
objects with confirmed tracks, classified as support or printing plane and explicitly
represented in the source's semantic support augmentation. Their surface is
derived separately from that object's observed points, not from a mixed table
height grid. This eligibility permits a reviewed geometric proposal, not a
material, adhesion or load-capacity approval.

Segmented-object semantics are part of review selection, not just display.
The handoff retains material/identity hypotheses and their source evidence,
positive print-on role hypotheses, affordances, resolved treatment, warnings and
operator review requests. Explicit resolved avoidance and rescan requirements
veto a substrate; a material label cannot override them. Among eligible review
substrates, an explicit print-on hypothesis with identified material is preferred
over less-specific support evidence or unknown material. These are categorical
semantic recommendations, not adhesion or material-compatibility scores.

`yaw_degrees` is an ordered list of allowed orientation preferences, normalized
modulo 180 while preserving first occurrence. The example `[0, 45, 90, 135]`
tries unrotated placements before rotated alternatives. Yaw zero means the
upright wall's long edge follows source-frame +X in top view; it does not lay the
wall flat or establish a robot-frame pose. No zero orientation is injected if
the task does not allow it. Within an orientation, semantic recommendation comes
before distance to the primary table's observed XY centre, then evidence quality
and height variation. Search work is shared across substrate views.

Rotation is a documented fallback, not a tie-break after a central diagonal.
A later yaw is considered only after the preferred stage has exhausted its
map-valid/native candidates without a valid initialized pose. If either search
budget leaves that stage unresolved, the result holds rather than claiming
rotation is necessary. The proposal records the orientation policy and each
stage's counts, rejections, skipped status and fallback reason. Centrality remains
a layout preference, not robot IK or reachability evidence.

The original agent's environment description and required material/reachability/
collision checks remain attached. High confidence in a visual wood identity is
not high confidence in deposition compatibility. The raw board interpretation's
review request is preserved even if the resolved ontology flag is false. Unknown
coating, unconfirmed deposition boundaries, adhesion and mechanical capacity
remain operator-review conditions. Rich semantic text is evidence, never an
instruction to execute commands or bypass the existing approval gates.

A secondary view takes the highest actual owner point in each observed XY cell;
it does not fit a plane, fill holes or create unseen support. The original map's
clearance, class and quality gates still apply. Explicit owner selection can
separate height layers, but cannot turn a blocked or unknown map cell into clear
space. The selected owner's top samples form the native intersection substrate;
other objects remain obstacles. The upper envelope is a reconstruction-derived
review assumption, not a calibrated contact surface.

PROPOSE initializes the actual goal in memory with the native `position_field`
at one proposed XY/yaw, the production heat-method field, phi and intersection
contour. The lower frontier is intentionally below the selected scan surface,
using the explicit `base_z_offset_m`. This is the actual goal mesh, not a hidden
scalar-only extension. Field-state, mesh and target batch files are not written
before approval. GENERATE rechecks that exact mesh, pose, field and intersection,
then uses native contour, gradient-lift, target-rule and YAML kernels. It never
moves or re-searches the approved pose. The requested count is a ceiling;
native rules may produce a smaller batch, reported with `partial_batch`.
The demonstration task specifies a 0.3 m wide, 0.3 m high, 0.03 m thick wall
and explicit nominal bead/search settings. These are configurable assumptions,
not learned or physically calibrated process settings.

The selected print substrate is distinct from other objects. Its represented,
unambiguous surface may intersect the buried goal. Above-surface scan variation
is allowed only within the explicit contact tolerance and contact-height ceiling.
This is not a symmetric empty-space allowance or a blanket exemption for every
object labelled support. Foreign supports, unknown objects and owner protrusions
remain screened. The obstacle volume is checked again after native Z placement.
Scene suitability/clearance and the goal's sequence heat are separate fields.

The v2 example task makes these offline assumptions explicit:

| Parameter | Example value and meaning |
| --- | --- |
| `base_z_offset_m` | 0.005 m; native lower-frontier burial |
| `support_contact_tolerance_m` | 0.005 m; owning-surface above-top tolerance |
| `max_duplicate_layer_separation_m` | 0.005 m; ambiguity between same-XY layers |
| `max_support_triangle_height_span_m` | 0.010 m; local support triangulation continuity |
| `max_frontier_height_span_m` | 0.016 m; provisional one-bead frontier variation limit |
| `heat_seed_level_m`, `heat_t_coef` | 0.005 m and 2000; native heat-method inputs |

The old `max_base_height_span_m` and `max_support_gap_m` keys are not silently
reinterpreted. Use the explicit v2 example task. A historical proposal remains
inspectable but cannot be regenerated under changed task/code identities.

## States and communication

```text
DISARMED -> explicit arm -> PROPOSE -> AWAITING_PLACEMENT_REVIEW
                                     -> exact proposal-file SHA256 approval
                                     -> GENERATE -> BATCH_PREVIEW_REVIEW_REQUIRED
            insufficient evidence -> HOLD
```

| ROS name | Type | Purpose |
| --- | --- | --- |
| `/behav3d/startup/review_wall` | `InitialWall` action | Supervised PROPOSE and GENERATE |
| `/behav3d/startup/plan_wall` | `InitialWall` action | Fixed scientific backend |
| `/behav3d/startup/set_armed` | `SetSupervisorArmed` service | Explicit startup arm/disarm |
| `/behav3d/startup/approve_placement` | `ApproveSupervisedStep` service | Approve or deny the exact persisted proposal hash |
| `/behav3d/startup/status` | `std_msgs/String` | State and pending proposal identity |

`InitialWallSession` follows the existing `ControlAwareSession -> QueueItem ->
CommandRouter -> asynchronous action -> Command.finish_flag` conventions. Its
physical command sets are empty. The shared cooperative pause gate applies;
pause is not an emergency stop. A fixed worker uses a separately configured
scientific interpreter. Geometry stays in hashed files, not ROS messages or
model prompts. No new network/control protocol is introduced.

The goal contains `phase`, `request_id`, `artifact_id`, `frame_id`,
`scene_manifest_path`, `scene_manifest_sha256`, `task_path`, `task_sha256`,
`proposal_path`, and `proposal_sha256`. PROPOSE uses empty proposal fields.
GENERATE keeps the same request identity, scene/task/frame bindings and the
exact returned proposal path/hash. This is not a fabricated deposition cycle.

The coordinator starts disarmed and owns a flocked, fsynced, hash-chained journal.
Approval is consumed durably before generation. The backend independently
requires the matching in-flight journal state, so its GENERATE endpoint cannot
bypass that gate. Restart disarms and holds unfinished work; approval is not
restored. Cancel, deny, disarm, source changes and partial journal records fail
closed. Interrupted artifacts can remain for inspection but are not accepted
receipts. Operator IDs are audit labels, not authentication.

Completed retries return historical receipts without another dispatch. They do
not revalidate the files' current contents. Hash-check artifacts before using
them; a new generation operation rechecks sources and code before and after
analysis. ROS isolation and trusted configured code/interpreters are still
required. These components are not a filesystem or execution sandbox.

## Native Linux setup

Build the existing four integration packages:

```bash
colcon build --packages-select behav3d_interfaces behav3d_utils behav3d_commands behav3d_orchestrator
source install/setup.bash
ros2 interface show behav3d_interfaces/action/InitialWall
```

Use the scientific environment described in [ROS_DOCKER.md](ROS_DOCKER.md),
with the additional production dependency `potpourri3d==1.4.0`. Keep that
environment separate from ROS system Python. The startup planner does not use
DDS reconstruction or require the separate corrected DDS checkout.

Freeze a reviewed Agent007 scene using its `initial_scene.export_scene` API;
see Agent007's `docs/INITIAL_SCENE_HANDOFF.md`. Supply the true source-frame
identity. The demonstration uses `ocsr_recorded_world`, deliberately not a
robot-frame alias. No robot transform or registration is inferred.

Copy `config/initial_startup_config.yaml` to a local config and set:

- One canonical journal path shared by both nodes.
- Existing, separate input and output roots. Input roots must cover the frozen
  scene, its original source scene and the task file; outputs must not overlap
  inputs or either code checkout.
- Explicit Agent007 and BEHAV code roots, scientific `worker_python`, and
  bounded operation/worker timeouts (the coordinator timeout must exceed the
  worker timeout and cleanup allowance).

Then launch only the review nodes:

```bash
ros2 launch behav3d_orchestrator initial_startup.launch.py config:=/absolute/path/startup.yaml
ros2 service call /behav3d/startup/set_armed behav3d_interfaces/srv/SetSupervisorArmed "{armed: true, operator_id: reviewer}"
ros2 topic echo /behav3d/startup/status
```

Submit a PROPOSE goal using `InitialWall`; inspect the returned source-bound
proposal or HOLD report. If the proposal is acceptable, approve its exact file
hash, then submit the matching GENERATE goal:

```bash
ros2 service call /behav3d/startup/approve_placement behav3d_interfaces/srv/ApproveSupervisedStep "{request_id: YOUR_REQUEST_ID, proposal_sha256: EXACT_FILE_HASH, operator_id: reviewer, approve: true}"
```

A successful preview creates `field_state_init.npz`, a 300 x 300 mm
`goal_mesh.ply` for the example task, `targets.yaml`, and a manifest with pose,
source, producer and product hashes. The scalar state and exported goal share
the same vertices and faces, including their intentionally buried portion.
A 300 mm imported mesh remains 300 mm in total height; lowering it makes its
above-support height smaller. There is no hidden guard row or silent extension.
Goal PLY coordinates use round-trip float64 serialization. Target origins remain
millimetres, with the unchanged
`O(x,y,z) Z(i,j,k)` contract. The manifest carries frame/metre identities;
no guessed target yaw or robot-frame conversion is applied.

For proposal-only offline use, `scripts/review_initial_wall.py --help` exposes
the same fixed workflow without arming, approving or generating candidates.
Agent007's `scripts/build_initial_wall_review.py` displays the frozen scene,
maps, inferred wall, alternatives and optional generated target previews.
Changing the displayed alternative does not approve or modify the proposal.
The native view separates the positioned goal's sequence colours, selected
substrate mesh, buried portion, intersection frontier and generated targets.
Selected-pose field evidence is hidden when displaying a different alternative;
it is never translated to suggest an uncomputed field there.

## Recorded offline rehearsal

`scripts/rehearse_initial_startup.py` binds an existing frozen scene and a new
task to a new exclusive run. It calls the same durable startup gate and numerical
workflow without starting ROS, cameras or physical clients. Original source
evidence stays immutable. Input roots must cover the frozen scene, original
source scene and task; the run must be separate from those inputs and code.

Use `prepare --help` for the explicit paths and identities. Then choose one mode:

```bash
scientific-python -B scripts/rehearse_initial_startup.py automatic --run /data/rehearsal-test --software-test-reviewer
scientific-python -B scripts/rehearse_initial_startup.py interactive --run /data/rehearsal-human --human-review-stdin
```

The automatic mode is a software reviewer, never the human user's approval. It
checks disarmed, unapproved, wrong-hash and direct-backend rejection, audits the
proposal, grants a labelled test approval, generates a preview and audits the
products. The interactive mode is a separate prepared run, in one long-lived
process. It waits for `approve <exact proposal SHA256>`, then separately for
`accept-preview <exact batch SHA256>`. `reject`, `abort` or EOF stop safely.
Nothing executes after accepting a preview. Restart requires a fresh run and
proposal; it never restores a pending approval.

Each run retains `configuration.json`, `events.jsonl`, the approval journal and
`summary.json`. `end_to_end_preview_validated` becomes true only after the batch
audit passes. A safely handled HOLD is reported as HOLD, not end-to-end success.
The audits compare the exact goal, heat and intersection against the reviewed
evidence, product hashes, YAML millimetres and candidate count. Partial batches
remain explicit. This is recorded-evidence replay and candidate-preview testing,
not a physics simulation or proof of printed outcomes. Old scans of a different
artifact are never presented as consequences of the newly proposed wall.

## Validation and remaining work

The scientific controls exercise the real scene exporter, durable supervisor,
startup workflow and production numerical kernels, not a mocked planner.
Synthetic scene metadata and test operators are labelled as software controls.
The separate-process ROS harness additionally exercises generated ROS types,
the command/session path, approval, backend bypass rejection, pause/cancel and
restart. It accepts test approvals only with its explicit software-test flag.

```bash
BEHAV3D_AGENT007_ROOT=/absolute/path/agent007 scientific-python -B -m unittest discover -s src/behav3d_orchestrator/test -p test_initial_joint.py -v
docker build --platform linux/amd64 -f docker/ros-review/Dockerfile.startup -t behav3d-ros-startup:jazzy-amd64 .
```

Run `scripts/ros_initial_startup.py --help` inside the existing isolated review
container with the BEHAV checkout and Agent007 `src/` mounted read-only, no
network/devices, and a separate writable results volume. Never use its synthetic
approval flag to approve a real recorded scene.

Software passing results do not establish live perception accuracy, support
capacity, adhesion, workcell clearance, reachability or physical wall stability.
Registered-plane augmentation cannot become measured support just because a
proposal fits. A fresh robot scan remains an existing, separately authorized
ScanSession operation upstream; this launch does not automate it. Automatic
existing-support selection and physical execution remain later increments.

### Initial version validation and subsequent diagnosis

The source-frozen integration passed 51 new lightweight workflow/process/transport
and harness-helper tests plus 188 selected existing BEHAV regressions on the
host. Separately, 25 numerical tests and 8 scientific two-agent tests passed on
both macOS and Linux. Agent007 passed 137 selected exporter/viewer/integration
tests on the host. These are scoped suites,
not full robot-workspace or hardware validation.

An isolated ROS 2 Jazzy amd64 run passed eight separate-process scenarios in
15.833 seconds: disarmed rejection, native proposal, approval/bypass protection,
native first-batch generation, cached retry, pause/cancel, disarmed restart and
task/frame conflicts. All three node process instances exited with code zero,
without forced kills or tracebacks. The synthetic positive scene produced five
of seven requested targets, explicitly a partial candidate preview. All test
approvals were labelled software controls, never the user's real-scene approval.

The initial recorded environment scene returned HOLD. Separate 0/45/90/135-degree
searches completed 22,028 poses without truncation, retaining the same 300 mm
wall dimensions and quality thresholds. Missing footprint coverage was the
largest first-rejection category. A subsequent exact replay found 557 poses
passed all preceding checks; 510 were rejected only by semantic support samples
in the final collision gate. The initial wrapper also pinned the goal to maximum
support height and used a scalar-only guard and analytic sequence values. Those
results did not establish that another scan was required or that the wall could
not fit. The v2 contact/initialization path corrects those mismatches. Historical
receipts remain unchanged; use fresh source-bound proposals after code changes.

Linux testing fixed startup-level numerical conditioning and a boundary-reserve
inconsistency without modifying legacy scalar kernels or relaxing bounds. The
approved pose remains in its declared world frame; target output is restored to
world coordinates. Native shutdown testing also fixed an inactive-context
WaitSet race. Recorded perception was reused; no fresh scan/model call, robot
registration, additional-support capacity or physical printing was validated.

### Corrected native initialization and recorded rehearsal

The corrected startup passed 38 native numerical tests on macOS and Linux,
8 scientific integration tests on each, 51 startup control/helper tests and
13 recorded-rehearsal tests on the host. The selected existing BEHAV regressions
passed 188 tests. Agent007 passed 154 selected exporter/viewer/integration tests,
including 55 viewer tests. New Python Ruff and whitespace checks passed.

The corrected isolated ROS run passed all eight scenarios in 21.148 seconds;
all node instances exited with code zero. A separate recorded-data native
rehearsal completed on both operating systems, producing six of seven requested
targets with matching reviewed geometry and product audits. It used a separately
labelled 8 mm owning-table contact sensitivity profile, not the unchanged 5 mm
example default. The full 5 mm run still held: sparse raw table samples exceeded
that contact assumption even in map-valid sites. The 8 mm result is an offline
parameter-sensitivity demonstration, not physical calibration or universal
placement validity.

In the recorded positive example, 22,028 sites were tested, 47 passed the site
screen, and native initialization tried 17 ranked sites to retain nine review
poses. The selected actual 300 mm mesh had 1,599/1,599 support hits, 82 frontier
segments, 3.94 mm frontier variation and 5.00–8.94 mm burial. No hidden extension
was used. The native generator returned six targets; missing requested targets
were not invented. A separate user-led run stops at exact placement approval,
then separately at batch review. Test approvals never become user approval or
authority to execute.

### Central substrate selection

The subsequent user review rejected the edge-of-table placement. A central,
map-valid pose was being rejected solely by board samples because the initial
collision policy recognized only the table owner. Eligible semantic support
members now have separate owner-only top-envelope views, including the primary
table when source membership is explicit. Original map class, clearance and
quality evidence stays separate from contact geometry. Those confidence values
are not board-specific physical-contact probabilities.

The revised recorded run selects the plywood board at the exact primary-table
XY centre, yaw 135 degrees, with the unchanged 5 mm profile. It tests 22,500 poses
without search truncation, retains 426 site-valid poses, initializes nine board
review poses and generates seven of seven preview targets. Other site-valid
poses are not all initialized. All 1,599 goal vertices hit the selected owner's
mesh; the frontier has 82 segments, 6.65 mm variation and 5.00--11.65 mm burial.
No foreign-object or underlying-table exemption was needed. The old 5 mm HOLD
above predates these separate contact views and is a historical result.

The selected upper-envelope frontier is 2.53--8.64 mm above the smoother board
patch saved from the same scan. That difference is representation uncertainty,
not independently measured physical accuracy. Centrality, raw point maxima and
a numerically consistent preview do not establish reachability, nozzle contact,
adhesion or structural capacity. At that stage, the human run remained at a
fresh placement approval gate; the later accepted milestone is recorded below.
Previous rejection and software approvals never granted consent.

Validation passed 45 native tests and 10 real-export scientific integration
tests on each of macOS and Linux, including board generation and stale-owner
evidence rejection. Agent007 passed 164 selected tests including 65 viewer
tests. The isolated ROS harness passed all eight scenarios in 19.164 seconds,
with normal node shutdown. A separate Linux recorded-data replay also produced
seven audited targets. ROS interfaces, pause semantics and target YAML remain
unchanged. No physical execution or browser/WebGL layout validation is claimed.

### Semantic and unrotated placement validation

The next source-bound revision retains rich segmented-object semantic evidence,
environment descriptions and upstream review checks, and applies the ordered yaw
stages described above. It reuses the recorded interpretation without another
model call. A fresh export preserves all ten geometry arrays bit for bit; only
the semantic handoff and producer/content identities change.

The recorded 5 mm run now selects an upright yaw-zero wall on the board, 31.7 mm
from the table-centre reference. It tests 5,625 zero-yaw poses, retains 52 site
candidates and initializes nine review poses. Rotated stages are explicitly not
run because zero yaw succeeds. Seven of seven preview targets are produced.
The goal has 1,599/1,599 support hits, 84 contour segments, 3.66 mm frontier
variation and 5.00--8.66 mm burial into the owner-point upper envelope. At the
exact centre, the zero-yaw footprint crosses the clear-region boundary; the
small translation preserves complete coverage and margin.

The board is preferred for review from its recorded wood hypothesis and print-on
affordance; the table remains unknown-material review and the box remains
blocked by resolved semantics. Visual confidence is not material compatibility.
All upstream material, reachability and complete-collision checks remain
attached. The selected upper envelope lies 3.69--7.71 mm above the smoother board
patch from the same reconstruction; physical contact remains unvalidated.

Validation passed 37 exporter tests, 56 native policy/geometry tests on each
platform and 11 scientific two-agent tests on each. Tests include semantic
source mutation rejection, review-request preservation, explicit-veto precedence,
ordered yaw and unresolved-budget HOLD. The 51 startup controls and 13 rehearsal
tests also pass on the host. Linux ROS passed eight scenarios in 17.151 seconds
with normal node shutdown, and a separate recorded-data replay produced the same
unrotated placement and seven audited targets. The previous diagonal human run
was rejected; its replacement used a separate exact proposal approval. Existing
ROS communication, pause and target-YAML conventions are unchanged.

## Accepted milestone and next development

The user accepted the resulting seven-target offline batch on 30 October 2026,
after its separate placement approval and exact product audit. The durable
rehearsal recorded both human decisions and ended `OFFLINE_REVIEW_COMPLETE`
and `DISARMED`. The selected board, yaw-zero pose, semantic restrictions,
source hashes and native field geometry were unchanged. No movement or extrusion
command was sent, and acceptance did not grant physical execution authority.
Recorded scans, journals and generated previews remain local evidence; Git
contains the implementation, portable fixtures and reproducible validation tools.
The matching Agent007 handoff/viewer implementation is commit
[`025817e`](https://github.com/alejp1998/agent007/commit/025817e7450f8c3245eebce95ce98d0ee5ca3173).
Keep both repositories on compatible revisions when reproducing this milestone.

The next development is a controlled offline closed-loop demo:

1. Connect the accepted startup goal, field and candidate batch to the existing
   expected-deposition evaluation contract, retaining frame, cycle, source and
   product identities through the existing ROS action/session path.
2. Supply an explicitly synthetic outcome or genuinely matched recorded scans
   for that geometry. Test nominal deposition, missing material, occlusion and
   deviation independently; do not call an unseen area a missing bead.
3. Let observation produce evidence and repair/continuation/rescan suggestions.
   The robotic agent must compare those options while the existing rule-based
   field/candidate algorithm remains authoritative. Missing material below the
   frontier already attracts the lowest-viable-candidate rule; expose that
   behaviour rather than replacing it with LLM-selected target coordinates.
4. Gate the selected action through human review and use a matched readback to
   evaluate whether it resolved the issue. Preserve negative tests for stale
   evidence, repeated actions, absent approval and uncertain observations.

Pass criteria are causal candidate-to-readback bindings, correct missing-versus-
occluded handling, a documented action comparison, no gate bypass, and a
post-action assessment that can report success, failure or uncertainty. A scan
of another artifact is not an outcome of this newly proposed wall.

Before moving to physical printing on Linux, validate the substrate/contact
model against independent measurements, confirm the material/deposition setup,
register the scene to the robot frame, and run plan-only IK plus complete swept-
tool/robot collision checks. The current upper envelope differs by millimetres
from the smoother reconstruction; centrality is not reachability. A subsequent
hardware session requires separate explicit approval and workcell supervision.
