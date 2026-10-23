# Offline robotic agent foundation

Initial draft, 2026-10-07. No runtime transport or physical execution is implemented.
Base: origin/robarch_2026, 59d8427268d16cedc1c0256c765f6d68675b3440.

## Agreed scanning purpose and first implementation

Scanning provides the closed mesh used to compare the upper frontier with the goal
field and ray-trace valid printing candidates. Preserve the existing upper-frontier
grid sweep, including its normal height adjustment. Agent007 inspection requests
and geometry-development findings can justify additional views. A local bead scan
must not replace the mesh-building sweep.

The current offline implementation is `tools/offline_inspection/frontier_scan.py`.
It preserves a supplied sweep verbatim and appends bounded, request-specific views.
The recorded adapter in `build_frontier_scan.py` consumes an actual Agent007 bundle
and checks source-bound symbolic identities before using local geometric evidence.
Its first case preserves all 16 recorded scan-6 poses; the earlier nine-pose grid
was a cropped visibility experiment. See `tools/offline_inspection/README.md` for
reproduction and `outputs/offline_inspection/c5-t1/frontier_scan_report.html` for the
recorded decision. No ROS runtime or candidate-planner behavior has changed.

Implemented: mandatory-sweep preservation, evidence-scoped selection, append budget,
recorded packet/pose binding and unresolved-review reporting. The second offline
entry point, `replay_depth_holdouts.py`, adds `inspection_policy.qualify_requests`
before pose selection and evaluates real recorded depth subsets with Agent007's
unchanged numerical cap estimator. It suppresses generic shape suggestions without
an unresolved issue, retains supported provisional shape/position requests, and
routes cap-quality concerns to evidence review. The earlier batch replay remains
a frozen comparison of the first policy; no ROS integration is implied.

Still pending:
automatic detection of frontier deficiencies, multiple-region scan planning,
prospective depth-quality scoring for new poses, future-cycle sweep adaptation, measured
reconstruction/candidate-plan comparison, and robot feasibility/execution.

## Next responsibilities to develop with the user

User direction: **normal candidate generation stays rule based. The agent gains
candidate-planning agency when an artifact failure calls for recovery.** Agent007
evaluates evidence; the robotic agent proposes and tests recovery strategies and
candidate batches; the existing field/mesh machinery checks candidate validity.
The ROS sessions retain physical execution responsibilities. Routine candidate
generation is not being moved into an agent decision loop.

| Responsibility | Trigger and decision | Concrete output | Development state |
| --- | --- | --- | --- |
| Assess the inspection need | New Agent007 revision: retain sweep, seek region evidence, or review identity/registration | Region, unresolved question, evidence references and next review action | First offline request filter and three raw-depth replays implemented |
| Propose inspection views | A qualified geometric inspection objective | Full required sweep plus bounded additions, purpose of each addition, one/two-view cost comparison | Regional offline selector implemented; new-view sensor benefit unvalidated |
| Triage a possible artifact failure | An inspection discrepancy may affect continued printing | Affected region, supported failure hypothesis, evidence gaps, and recovery-review or reinspection decision | Next recovery milestone; ordinary cap uncertainty alone is insufficient |
| Propose a recovery strategy | Evidence supports a specific artifact problem | Alternatives for repair, area priority, support, adapted geometry or contact correction, with hold/reinspect when needed | Deterministic offline deficit/lean policy implemented; support/contact/goal revisions expose prerequisites |
| Generate and test recovery candidates | A recovery alternative has sufficient mesh/field evidence | Candidate batches produced or checked through existing rule-based planning, with predicted effects and rejection reasons | Bounded surface-fill and adaptation searches implemented; complete planner replay pending |
| Recommend a recovery batch | Alternatives have been compared | Proposed candidate sequence, reason for selection, expected effect, scan checkpoint and stop condition | Proposal only; physical execution remains a later milestone |
| Verify recovery and resume | A recovery attempt has readback evidence | Agent007 re-evaluation and a decision to resume ordinary rule-based growth, revise recovery, or hold | Evidence/cycle linking and outcome criteria pending |

The agreed next direction is **propose and test recovery**. On 2026-10-08 the user
expanded this scope to leaning, geometric deviation, local failure and major
artifact failure, with material repair, area priority, supports, contact correction
and adaptation of the intended geometry as possible responses. The proposed
development cases are a localized material deficit and a leaning region; they
exercise different recovery objectives. Compare normal continuation with bounded
recovery alternatives and retain hold/reinspect when evidence or feasibility is
insufficient.

A recovery proposal must state what it is trying to restore, which evidence
supports that need, which candidates were tested, predicted goal progress, affected
neighboring geometry, and how success would be checked. The first acceptance test
should show that a useful, rule-valid local recovery can be proposed while an
uncertain/misidentified deficit produces an evidence request rather than blind
deposition. Current DDS 'missing' labels remain provisional; a replay can test a
failure hypothesis without claiming the physical failure has been confirmed.

Mesh/field readiness remains a prerequisite inside recovery testing. It examines
the region the existing planner actually uses:
the active upper frontier against the goal field, with the recorded avoidance and
viability constraints. A single bead's cap-fit pass cannot certify that frontier.
Stale mesh/field combinations and unresolved relevant coverage must be reported.
Physical sufficiency thresholds and reconstruction quality remain to be validated.

Existing ray intersection, distance, viability and spacing checks remain the
geometric validation basis. Recovery may propose a different region, sequence or
versioned goal from normal growth, but unsupported planner operations or rule
exceptions must be explicit and tested. No proposal silently changes speed, flow,
pressure or source geometry. If the existing planner cannot express a recovery
strategy, report that limitation and design the needed adapter before producing
executable candidates. Geometric validity alone does not establish mechanical
stability or successful contact correction.

### Failure evidence and strategy catalogue

Agent007 is intended to report the observed failure distinctions. The current
bead/cap replays do not establish that whole-artifact leaning, collapse or failure
cause detection is already implemented. The recovery interface should carry:

- Artifact/region identity, observation revision, coordinate frame and evidence.
- Affected extent: one bead, a local region, or the whole artifact.
- Displacement/lean direction and magnitude relative to the intended geometry,
  and change over consecutive observations when available.
- Missing/extra material, changed connections or support geometry, with unknown
  areas and observation/registration uncertainty kept explicit.
- Failure hypotheses and unresolved questions, distinguishing the measured shape
  from an inferred cause or predicted instability.

Leaning/deviation needs evidence over the affected structure. A good bead cap fit
or the normal upper-frontier sweep alone may not supply it. Request wider or side
observations when needed while preserving the sweep used by the ordinary planner.
Check registration and distinguish a whole-object pose shift from local deformation
before interpreting displacement as a recovery target.

| Strategy | Intended use | Proposal and comparison | Additional capability needed |
| --- | --- | --- | --- |
| Fill missing material | A supported local deficit with accessible deposition geometry | Compare normal continuation with a small repair batch; measure predicted deficit reduction and added material outside the permitted region | Deficit-to-planner adapter; a buried or side deficit may be outside the current upper-frontier planner's expressible actions |
| Change area priority or batch order | A local problem justifies changing where growth proceeds next | Prioritize a justified region, defer growth in another, and compare short sequences with inspection checkpoints | Explicit regional preference/mask adapter; changed selection must retain spacing and other feasibility constraints |
| Pause a region for material development | Further loading should be deferred while the material is given time to firm up | Exclude the affected region and candidates that load it; retain a timestamped observation, schedule reinspection, and compare deformation before considering resumption | Material-specific waiting/adhesion window, deposition age and dependency checks; elapsed time or unchanged appearance alone does not establish load capacity |
| Add supporting material | A support strategy is proposed for a leaning/deformed region | Propose support geometry, intended attachment/base regions and construction order, then generate candidates against that proposal | A separate support goal/field and connection assessment; mechanical benefit cannot be inferred from added volume alone |
| Adjust remaining geometry | The observed artifact provides a changed starting boundary for continued fabrication | Compare bounded translations, rotations or local reshaping of the unprinted remainder, anchored to observed printed geometry | Versioned goal proposals, protected design features and allowed deviation limits, updated field/width mappings and recomputed feasibility |
| Correct by robot contact | A displacement might be recoverable through controlled physical adjustment | Propose contact region, desired change and a measurable verification objective; leave forces and trajectories unspecified until modeled/tested | Material/contact characterization, contact tool and force-feedback primitive, bounded trials and post-contact inspection |
| Salvage, rebuild locally, or stop | A major failure invalidates normal continuation | Compare feasible remaining targets and recovery sequences with stopping; identify any required removal/rebuild capability | Explicit salvage objectives and assessment of remaining structure; removal and restart are not currently automated recovery actions |

These strategies can be combined. For example, a proposed sequence could defer
growth in a leaning region, build a support, inspect, then continue with an adapted
remaining target. This is a hypothesis to test, not a fixed response to a lean
direction. Printing on the leaning side or adding support volume is not, by itself,
evidence of a stabilizing effect. A major failure can make every deposition option
infeasible; the comparison must permit that outcome.

Changing a remaining target does not move the already printed artifact. Preserve
the original design and observation history, bind each alternative to a new goal
revision, and show its deviation from the original. Keep support geometry separately
identified from product geometry. Run the rule-based planner against each declared
goal/priority alternative, including its avoidance constraints; do not bypass the
original field checks to insert out-of-goal support candidates. Rules may require
an explicit extension for a new strategy, rather than silently treating it as valid.

### Lean recovery: what changes and why

The current growth-shell experiment redirects **future deposition**. It fits a
horizontal displacement from expected/observed shell pairs, fades that displacement
back to zero over a trial return height, then runs the existing frontier planner.
That planner selects the lowest eligible field values subject to spacing and
advances along the field's surface growth direction. It does not optimize a
restoring force, predict drying, or straighten the already printed wall. The
20/40/60 mm return heights are experimental choices, not mechanically derived
recovery settings. The nominal footprint screen is not a wall stability test.

Extend the recovery comparison with the following proposed sequence contracts;
these are requirements, not implemented drying or support simulations:

- **Pause and reassess:** mask the leaning region and structurally dependent
  deposition, record when it was last printed, and request another observation
  after a material-specific interval. Continue elsewhere only if independent
  loading and tool access are established. Check both deformation trend and
  process suitability for the next deposit before resuming. Continuing movement
  while paused escalates the issue; it does not trigger an automatic timed resume.
- **Build a rib or support:** identify a verified supported footing or suitable
  existing anchor, a connection region on the wall, and a permitted support
  envelope. Propose a bottom-up construction order whose intermediate stages
  also have support. Compare access, attachment, added load, future obstruction
  and a material-dependent prediction of resistance to further leaning. Support
  geometry remains a separate goal revision. Do not create floating side beads
  or infer mechanical benefit from proximity to the wall alone.
- **Resume with a revised continuation:** only after the region meets the
  relevant process/stability checks, compare the first bounded batch against
  ordinary continuation and local alternatives. Reinspect before extending it.
  Physically restoring an already displaced wall remains a separate contact or
  reconstruction objective.

On scan 19, the next offline comparisons should be continuation alone, pause then
reassess, support then reassess, and pause plus support followed by continuation.
Recorded geometry can test masks, anchor access, construction order and candidates.
Pause duration, strength gain and rib effectiveness require measured material
inputs; controlled assumptions must be labelled and cannot become observed facts.

#### Material context: ram-extruded clay and occasional heat-gun use

The operator reports clay deposited with a ram extruder, relatively dry compared
with their usual printed-clay reference, with occasional heat-gun application.
This is a qualitative process description, not a measured water content, strength
or validated waiting interval. Do not infer those quantities from the extruder type.

Treat passive waiting and assisted drying as separate recovery alternatives:

- **Passive wait:** defer loading of the affected region, record elapsed time and
  any available ambient conditions, then reassess deformation and readiness for
  the next deposit.
- **Assisted drying:** propose an intervention on a specified region using a
  characterized heat-gun procedure. Record actual application time, duration,
  setting, distance, direction/sweep and any measured surface temperature. Unknown
  values stay unknown; a gun setting is not the clay temperature. No automated
  heat-gun capability is established by the current repository experiment.
- **Support plus continuation:** separately compare whether a rib/support is
  constructible and mechanically useful, including attachment to the clay in
  its current condition. Waiting or heating does not itself restore lost geometry
  or establish that the next load can be carried.

Associate each region with deposition age, observed displacement history,
intervention history and references to any material/process tests. Do not equate
an unchanged scan or a dry-looking surface with sufficient internal strength.
After waiting/heating, remeasure the frontier before generating the next targets;
material loss of water may change its dimensions. Assess subsequent bead attachment
as well as geometric stability. Resume only against calibrated process criteria,
not an invented fixed delay or an assumed temperature-to-strength conversion.

The scan-19 replay can test candidate exclusion, intervention bookkeeping,
reinspection decisions and supported rib geometry. It cannot recover unrecorded
heating or drying history. Compare labelled hypothetical outcomes (movement stops,
continues or new deformation appears) without claiming to simulate clay response.
Calibrating that response needs comparable deposits of the actual clay and ram
settings, with passive-wait and assisted-drying observations plus controlled
subsequent-load/attachment tests.

Relevant experimental context, not parameters transferable to this clay:
[Structural build-up of 3D printed earth by drying](https://researchportal.ip-paris.fr/en/publications/structural-build-up-of-3d-printed-earth-by-drying/)
relates forced drying and water content to mechanical development for a specified
kaolin system. [The effect of interlayer drying on the ceramic paste extrusion process](https://eprints.whiterose.ac.uk/id/eprint/221584/)
reports improved shape accuracy alongside shrinkage that affects later deposition.

### Development and evaluation for recovery

Implemented first experiment on 2026-10-09 in
`tools/offline_recovery/build_experiments.py`: recorded C5:T7 discrepancy evidence
and controlled 0°/4°/8°/12° regional leaning of the same scan-6 mesh. It compares
archived next-batch candidates, changed priority, a local fill probe, deferred
growth and adapted candidate bases. See `tools/offline_recovery/README.md` and
`outputs/offline_recovery/recorded_cycle6/viewer.html` for results and reproduction.
Source files are hash checked and remain unchanged. The comparison does not import
ROS, regenerate the complete field planner or validate mechanical recovery.

The follow-up `tools/offline_recovery/build_inward_lean.py` uses the taller recorded
scan 19, shifts the lean anchor to Z = −75 mm and leans the right-hand upper region
inward. It adds full-mesh footprint/spacing/sampled-extruder screens, a bounded
local placement search, and a contact-access comparison between the repository
extruder and a compact paddle concept. It evaluates both inconsistent repository
tool-tip offsets explicitly. One local candidate passes the declared geometric
screens, and one paddle approach is sampled-clear; neither is executable or
mechanically validated. Contact/push/retreat remain unresolved and the original
runtime planner, collision helper and URDF are unchanged. See
`outputs/offline_recovery/inward_cycle19/viewer.html` for the interactive comparison.

The decision layer is now implemented in `offline_agent/recovery.py`, with a
recorded replay in `tools/offline_recovery/build_decisions.py`. It separates
uncertain evidence, stable and progressing deformation, and the objective of
continued growth versus restoring printed geometry. Failed constraints exclude
proposals before benefit ranking; missing planner/process checks lead to a bound
validation request rather than an executable batch. The searches test attached
first fill beads and bounded position/axis adaptations. Outputs preserve every
rejection reason and an inspection checkpoint with retry/stop conditions.

`tools/offline_recovery/planner_state.py` freezes explicitly supplied field,
observation, settings, avoidance and tool inputs; validation receipts bind to the
issue, proposal and snapshot hashes. Capturing a complete snapshot does not pass
any rule. The historical recording is still incomplete for full planner replay.
Automatic runtime capture, the full recovery planner adapter and calibrated
process/mechanical checks remain pending. See
`outputs/offline_recovery/decisions/viewer.html` and the recovery README for the
nine replay cases and the normalized contracts. Controlled confirmation/trend
inputs are explicitly experimental and are not claimed as Agent007 detections.

The follow-up in `tools/offline_planner_replay` located the matching repository
field state and reproduced 23/23 saved candidate batches (157 ordered positions)
with an explicitly inferred profile. Full mesh-to-contour replay still has small
unresolved differences. Bounded fixed-goal candidates on the inward-lean scene do
not pass the experimental local screens; the unmodified archived target also
fails the borrowed support model, identifying a calibration limitation rather
than proving an artifact failure. See its README and
`outputs/offline_planner_replay/recorded_260728/viewer.html`. No runtime defaults,
field geometry or execution interfaces were changed.

`tools/offline_growth_shell` now tests an explicit goal-revision strategy: fit
regional offsets from seven eligible Agent007 shell pairs, construct bounded
upward growth fields returning over 20/40/60 mm, and regenerate candidates with
the existing numerical rules. Four controlled lean angles produce 20 comparison
cases. The 40 mm return completes all sampled goal columns at 8° lean with less
maximum face rotation than the 20 mm alternative; no tested local leaning-area
alternative passes the experimental support screen. Only one fitted shell pair is from the current scan.
Evidence age, registration assumptions, borrowed bead dimensions and unverified
structural stability remain explicit. See
`outputs/offline_growth_shell/scan19/viewer.html` and its tool README. This changes
separate versioned planning proposals, not the original goal, observations or ROS
runtime. The full-frontier sweep and post-proposal inspection remain required.

The growth-shell viewer also compares complete archived, unchanged-goal and
revised-goal candidate batches on each identical edited scan. It exports full
candidate records and checks in `candidate_comparison.json`, with one-to-one
spatial associations, unmatched targets, signed position changes and tool-axis
changes. At 8° / 40 mm return, four of five retained candidates are unchanged from
the unchanged-goal rerun. The affected target moves 1.98 mm but its nominal support
drops from 4/9 to 2/9; two unaffected targets pass the declared geometry screens.
Local replacement alternatives remain separate from complete batches. This
comparison exposes first-bead limitations despite an improved continuation shape.

Continue the offline strategy comparison for a deficit and a leaning case.
Each proposal should expose its region, objective, changed priority/goal, candidate
batch or unsupported operation, predicted geometric effects, unresolved mechanical
effects and next inspection criterion. Include normal continuation and hold/reinspect
as comparison options. Keep the ordinary runtime planner unchanged.

Use recorded scenes where the inspection evidence supports the intended case.
Where no verified leaning/failure example exists, create explicitly labelled
synthetic perturbations of a recorded scene; these test responses to known geometry
changes, not Agent007's demonstrated detection accuracy or real recovery success.
Test uncertain/occluded evidence and frame misregistration as well as supported
deficits, local tilt, whole-object displacement and larger structural loss.

Compare geometric deficit/deviation, rule violations, interference with neighbors,
amount of added material, changes to the intended product and inspection cost.
Report mechanical stability as unvalidated until a suitable model and experiments
support it; do not use a purely geometric score to claim that a support or push will
correct leaning. Re-evaluate after each bounded attempted recovery before planning
the next batch. Bound retries and record whether the intended effect actually occurred.

Repository basis checked on 2026-10-08:

- `generate_print_points.py` selects the minimum scalar preference on its source
  graph and suppresses nearby candidates by graph distance. This is a possible
  basis for priority changes, not an existing recovery-priority interface. The
  scalar field also informs gradient modes, so a selection preference should not
  silently alter deposition direction; keep it separate or test both effects.
- `compute_phi_mask.py` uses `(phi > iso_level) & has_hit`; an alternative goal
  still needs recomputed ray/field viability, spacing and collision checks.
- `loop_simulation.py` updates geometry with simulated bead solids. It does not
  provide a deformation, adhesion or structural-collapse model.
- No force-controlled artifact-correction primitive was found in workspace source
  searches. The contact strategy is a separate development track, not a new use
  of ordinary free-space motion or the current nominal deposition simulator.

Later responsibilities, after those interfaces are tested:

- **Initial geometry placement:** propose a placement using observed support,
  goal geometry and the avoidance field, then check it with the existing placement
  and viability code before generating a first batch.
- **Optional inspection scheduling:** group related inspection regions and choose
  when extra views are useful. The full frontier scan required by candidate
  generation remains part of the baseline workflow.

Every proposed action needs an input revision, a bounded objective, an output
artifact and a stop condition. Repeating the same unresolved scan request should
end in evidence review rather than an unbounded rescan loop. These are proposed
responsibilities for discussion, not newly enabled robot commands.

## Existing capabilities inspected without executing

- `behav3d_commands.Session` supplies queues, result dictionaries, motion,
  camera, field and extruder commands. These are ROS interfaces, not offline adapters.
- `ControlAwareSession` adds pause/control gating to those runtime queues.
- `ScanSession` composes scan patterns, poses, capture and reconstruction.
  `sense_node.py` stores captures; world/field nodes manage reconstructed state.
- `PrintSession` consumes dot/segment YAML, converts intended volume to extrusion
  steps and synchronizes motion with extrusion. `print_node.py` connects to Modbus
  in its constructor. Neither is an offline simulation interface.
- `fields_node.py` exposes field initialization and candidate generation via ROS.
  `scalar_field/lib_scalar` contains geometry, raycasting, contour generation,
  candidate generation, spacing/target rules and collision filtering.
- DDS loop simulators serialize accepted candidates to YAML and build deposited
  geometry proxies from that intended plan. This is not measured print success.
- `compute_phi_mask.py` uses `phi > iso_level` AND a ray hit for viability;
  the scalar README's opposite inequality must not be used as an adapter rule.

## Proposed action catalogue for discussion

| Action | Existing foundation | Initial availability |
| --- | --- | --- |
| Read recorded state | NPZ field states, captures, YAML and DDS bundles | Review envelope only; file adapters pending |
| Validate candidate plan | Target YAML contracts and secondary rules | Pending parser adapter; must preserve mm plane origins and explicit frame |
| Invoke offline planner | lib_scalar raycasting, viability, candidate selection | Numerical offline replay verified against 23 saved batches; complete runtime validation remains pending |
| Propose focused scan | Scan patterns and ScanSession | Offline mandatory-sweep-plus-supplement selector; recorded single-region adapter |
| Simulate focused scan | Geometry/raycasting utilities | Recorded-mesh visibility model; sensor quality and physical accuracy unverified |
| Propose deposition | Existing dot/segment YAML | Pending data-only proposal; no motion/flow changes |
| Simulate deposition | DDS loop workbench | Pending safe adapter; intended geometry only |
| Record evidence-based decision | New offline controller | Implemented: hold and non-executed evidence-review proposal |

The LLM may explain evidence and draft proposals. Existing spatial-field,
ray-tracing, viability and spacing rules remain the geometric validation basis.
Normal candidate selection remains unchanged; recovery priority or goal changes
must be explicit alternatives that are tested through the applicable rules.
A suggestion grants no execution authority; mesh coloring grants no safety
certification. Unsupported operations fail closed through `execute()`.

## Provisional local envelope

The joint Agent007 interface now specifies serialized fields. This earlier
controller scaffold still does **not** accept an actual export directly. The new
scan replay adapter reads the real bundle separately and does not extend this ledger.
The fixture in `test/test_offline_agent.py` specifies the local normalization target:

- `schema`: `agent007.robot_review_packet.v1`
- Nonempty `artifact_id`, `evaluation_id` (immutable revision identity), `scan_job_id`
- Integer `candidate_cycle=N`, `scan_cycle=N+1`; `previous_revision` is null for
  the first local revision or the current accepted evaluation ID
- Exact configured `frame`, `units` (`m` or `mm`); no guessed frame or conversion
- Unique nonempty `evidence_ids`, `summary.uncertainty` as nonempty descriptive text
- Suggestions with an allowed review `category`, bound `evidence_ids`, literal
  `executable=false`, `fabrication_authorized=false`, `requires_operator_review=true`

The caller supplies a separate trusted recorded-source manifest containing
artifact, scan job, both cycles, frame, units and available evidence IDs. Validation
checks packet values against that manifest. Trust/authentication and original
YAML/DDS content hashes are not yet established by this scaffold. Extra packet
fields are hashed and retained in the digest but never interpreted as commands.

`DryRunController.review(packet, request_id=..., source=...)` returns a decision
linked to the revision, cycles, evidence and canonical packet SHA-256, with explicit
uncertainty and an empty `robot_commands` list. All checks precede ledger updates.
Returned data and ledger reads are detached copies.

Replay/request tracking and revision heads are in memory, scoped to one controller
instance. Restart persistence, concurrent access, transport, numeric geometry
validation and source-file adapters remain future work. Do not use this ledger as
an operating controller. Tests use synthetic records, not a real scan packet.

## Offline verification

From the repository root:

```sh
python3 -m unittest discover -s src/behav3d_orchestrator/test -p test_offline_agent.py -v
```

Only the standard-library offline package is imported. No ROS processes, service
calls, robot connections, scans, deposits, uploads or parameter changes are needed.

The scan replay now binds one actual recorded packet to a preserved sweep proposal.
Integration into this controller's revision ledger remains a separate milestone.
No robot commands are part of the current implementation.
