# Supervised observation review in ROS 2

The `feature/supervised-agent-print-loop` branch adds the first ROS-native
integration boundary between BEHAV3D and the observation agent. After explicit
operator approval it can verify an existing Agent007 evaluation, or generate
new local geometry evidence from a frozen RGB-D scan with the opt-in
`fresh_geometry` backend. Neither backend executes a recovery decision.
The normal scanning, field planning and printing entry points are unchanged.

Deployment remains native Linux and ROS 2 using the existing workspace
conventions. Docker is optional local validation tooling, not a dependency of
the coordinator, observation bridge or Linux workcell.

## Current capabilities

The coordinator starts disarmed. A new review requires arming, a typed action
goal, and approval of the exact proposal SHA256. The approval covers the
request identity, cycle, frame, preceding evaluation and source manifest hash;
it authorizes evidence review only. No physical command handler is registered
or constructed in `ObservationSession`.

The bridge runs a fixed, server-configured Python worker in an isolated process.
`backend_mode=recorded` remains the default and verifies a saved evaluation,
shell, robot review packet, source hashes, cycle mapping, frame and safety
fields. It confines bundle paths to `evidence_root` and source paths to
explicitly configured read-only roots.

`backend_mode=fresh_geometry` first validates a prepared input manifest, then
copies its frozen inputs into a new server-owned job. Existing Agent007 and DDS
code reconstruct the current and baseline scans, compare the issued candidates
against observed voxels, estimate bead caps, pair before/after evidence and
extend the planned and observed central shells. The worker independently
verifies the generated evaluation bundle before returning the same ROS result.
A changed source, invalid packet or unsupported event fails closed. Both paths
return `REVIEW_REQUIRED`, `requires_review=true` and
`fabrication_authorized=false`; neither establishes physical accuracy.

Fresh geometry requires `semantic_mode=offline_mock`. Its reports and
suggestions are deterministic local rules, not LLM interpretation, scene-object
detection or a new inventory. No model account, external upload, robot command
or candidate planner is invoked.

## Communication and execution

```text
operator -> supervised ROS review action -> exact hash approval service
         -> Supervisor journal -> ObservationSession
         -> QueueItem -> CommandRouter -> asynchronous ROS action client
         -> observation bridge -> isolated server-selected worker
                              -> recorded verification OR fresh geometry
         <- typed result and immutable artifact references
```

ROS actions and services carry lifecycle and bounded identities. Geometry and
reports stay in their existing artifact files, referenced by absolute path and
SHA256. Worker stdin/stdout is an internal dependency boundary, not another
robot-control bus. There is no HTTP server, socket protocol or directory watcher.

| ROS name | Type | Purpose |
| --- | --- | --- |
| `/behav3d/supervised_agent/review_observation` | `behav3d_interfaces/action/EvaluateObservation` | Human-approved review entry point |
| `/behav3d/supervised_agent/set_armed` | `behav3d_interfaces/srv/SetSupervisorArmed` | Explicitly arm or disarm review |
| `/behav3d/supervised_agent/approve_step` | `behav3d_interfaces/srv/ApproveSupervisedStep` | Approve or deny the exact current request and proposal hash |
| `/behav3d/supervised_agent/status` | `std_msgs/msg/String` | Transient-local JSON state and pending proposal hash |
| `/behav3d/evaluate_observation` | `behav3d_interfaces/action/EvaluateObservation` | Review/analysis backend called by the command layer |

The action goal contains `request_id`, `artifact_id`, `observation_id`,
`event_type`, `candidate_cycle`, `scan_cycle`, `previous_evaluation_id`,
`frame_id`, `coordinate_units`, `source_manifest_path` and
`source_manifest_sha256`. These must match the selected backend's manifest
exactly. In recorded mode, `source_manifest_path` refers to an Agent007
evaluation `manifest.json`. In fresh mode it refers to a prepared
`agent007.fresh_observation_input.v1` manifest, not a generic camera manifest.
Backend selection, Python executable, code roots and writable output directory
are server configuration; an action goal cannot change them.

For deposition readback, candidate cycle N is evaluated against scan N+1, with
coordinates in metres. Frame names are exact identities: no `world/base` to
`ur20_base_link` alias or transform is inferred. Target YAML remains unchanged
and keeps the existing millimetre-origin and material-volume conventions.
Inspection and intervention events use candidate cycle `-1`; the interface
recognizes them, but both current backends return `UNSUPPORTED`. Do not invent
deposition cycles for an inspection scan.

`ObservationCommands` produces the usual command result with `kind`, `ok`,
`phase=plan`, `planned_only=true`, `error` and `metrics`. `metrics` contains
the typed action result. The coordinator normalizes local command failures to
the action status vocabulary rather than treating an internal HOLD as success.

## Approval and failure behaviour

An append-only, fsynced, hash-chained JSONL journal is stored at
`<session_path>/supervised_agent/review_journal.jsonl`. Exclusive file ownership
prevents two coordinators from sharing a journal. An operation's START is
durable before the worker is invoked. Identical completed requests return
cached results without re-running the worker; changed content under the same
request ID is rejected. Later observations must reference the recorded head
for that artifact and advance its scan cycle.

Disarming, denying, cancelling or shutting down invalidates pending approval.
A late in-flight result is held for reconciliation and cannot advance the
artifact head. Restart always disarms; pending or running work is held rather
than automatically replayed. An interrupted review needs operator
reconciliation and a new request identity, not an automatic retry loop.
The journal detects accidental corruption; it is not a signed trust boundary
against a writer who can replace the journal and recompute its hashes.

### Node process recovery

Coordinator and bridge can run as separate ROS node processes. The journal,
not a surviving client future or a DDS discovery entry, determines whether a
review was accepted after a coordinator restart.

| Durable phase before coordinator exit | State after restart | Retry behaviour |
| --- | --- | --- |
| `PENDING` or `APPROVED` | Disarmed with the request in `HOLD`; approval removed | No dispatch or automatic replay |
| `RUNNING` | Disarmed with the request in `HOLD`; result uncertain | A late bridge result cannot advance the artifact head |
| `COMPLETED` | Disarmed with the successful receipt and artifact head retained | The identical request returns its cached receipt without another dispatch |

To reconcile a held request through the ROS interface, explicitly disarm the
coordinator, inspect the journal and evidence, then arm and submit a newly
reviewed request with a new ID. Reusing the held ID is rejected. If the process
exits while writing a journal entry and leaves a truncated record, startup
fails closed instead of treating the journal as a clean recovery.

A bridge outage is bounded by the command deadline. It cannot authorize
fabrication or promote an unaccepted evaluation. A failed request retains a
failure receipt; repeating its ID returns that receipt rather than silently
retrying the worker. A new attempt requires a new request and approval.
Graceful bridge cancellation cleans the worker's owned process group. An
uncatchable bridge `SIGKILL` cannot run that cleanup; a worker in its separate
process session may continue verification or fresh analysis. An active worker
after uncatchable parent death is not a validated recovery case. Process
supervisors must account for those children when stopping the whole review
service. Fresh jobs may retain incomplete or completed analysis artifacts;
only the coordinator journal establishes whether a review was accepted.

Sessions share the existing `/behav3d/control_state` pause gate. Cancellation
is checked while paused and again before dispatch. This cooperative pause is
not an emergency stop. The existing `/behav3d/control` `stop` command toggles
state; it must not be used as an idempotent hardware stop. Legacy debug input
can continue without a TTY and is not used for these approvals.
For local disarm, denial or shutdown, the typed result is `CANCELLED` while
the ROS goal terminates as `ABORTED`; `CANCELED` is reserved for an accepted
client cancellation, following the [ROS 2 action lifecycle](https://design.ros2.org/articles/actions.html).

`operator_id` is an audit label, not authenticated proof of a human. ROS access
control and deployment isolation are required before adding any hardware
executor. The bridge is independently callable without coordinator approval;
fresh mode can therefore compute artifacts through that backend endpoint.
Use the supervised entry point for gated work and restrict backend access at
deployment. Neither endpoint has fabrication authority.

## Build and review on a ROS workstation

The Mac uses Linux containers for ROS rather than a macOS ROS installation;
see [ROS review validation in Docker](ROS_DOCKER.md) for the recorded transport
and separate-process validation results. On a BEHAV ROS workstation, with its usual ROS
environment and dependencies available:

```bash
colcon build --packages-select behav3d_interfaces behav3d_utils behav3d_commands behav3d_orchestrator
source install/setup.bash
ros2 interface show behav3d_interfaces/action/EvaluateObservation
```

Use normal Linux package permissions: installed ament metadata and Python
modules must be readable by the account running ROS, and console scripts must
be executable. The optional Docker image normalizes its installed workspace
permissions; this does not change source files or impose a Mac-specific
permission scheme on the Linux checkout.

Copy `src/behav3d_orchestrator/config/supervised_agent_loop_config.yaml` to a
local configuration and set an explicit existing `session_path` and
`evidence_root`. Add every required external source directory to `source_roots`.
For the July 28 recording, these are the Agent007 output directory, the
`260728_133013` dataset and the 3DP-DDS source directory. Historical evaluation
inputs are also checked. Empty configuration deliberately refuses startup or
rejects goals. Use a trusted Python executable for `worker_python`; no worker
path or shell command can be supplied in an action goal.

```bash
ros2 launch behav3d_orchestrator supervised_agent_review.launch.py config:=/absolute/path/review.yaml
ros2 service call /behav3d/supervised_agent/set_armed behav3d_interfaces/srv/SetSupervisorArmed "{armed: true, operator_id: reviewer}"
ros2 topic echo /behav3d/supervised_agent/status
```

This launch starts only the two review nodes, not `print_move`, cameras,
controllers, homing or extrusion. Submit `EvaluateObservation` to the
supervised review action using the exact recorded fields. Once its pending
request and hash appear on the status topic, review the evidence and call:

```bash
ros2 service call /behav3d/supervised_agent/approve_step behav3d_interfaces/srv/ApproveSupervisedStep "{request_id: YOUR_REQUEST_ID, proposal_sha256: EXACT_PENDING_HASH, operator_id: reviewer, approve: true}"
```

The review action should return `REVIEW_REQUIRED`. A denial, timeout or failure
does not authorize continuing the print. No physical steps are available in
this entry point, even after approval.

## Fresh geometry from a completed scan

The fresh backend consumes a completed scan, its immediately preceding scan,
calibration, and the issued YAML for candidate cycle N. Scan N+1 evaluates that
plan; the YAML is the expected print plan, not independent evidence that each
bead was deposited. Keep the original target syntax and units unchanged.

Agent007's `ocsr.printed_artifacts.ros_fresh_analysis prepare` freezes these
inputs without running reconstruction or analysis. Its manifest contains exact
observation/cycle/frame bindings, a complete relative file-to-SHA256 inventory,
the adapter hash, and optional preceding generated-job and evaluation-manifest
bindings. Frozen provenance also binds the analysis producer and DDS source
fingerprints. A code change after preparation requires a new submission.

Use the corrected DDS source tree: its comparison module must declare
`OBSERVED_TSDF_BLOCK_LAYOUT = "open3d_x_fastest_v1"`, and generated comparison
metrics must attest the same layout. Older DDS code swapped X and Z within
Open3D blocks. The adapter rejects it instead of silently accepting a different
comparison. A Git commit alone is insufficient provenance: the main and
corrected trees can share a commit while having different local source files.
The integration imports existing DDS code; it does not copy or patch DDS core.

### Native Linux analysis environment

Keep the ROS system interpreter unchanged and select a separate trusted Python
environment for geometry. On Ubuntu 24.04 amd64, one explicit setup is:

```bash
sudo apt-get install python3-venv libegl1 libgl1 libgomp1
python3 -m venv /data/venvs/behav3d-analysis
/data/venvs/behav3d-analysis/bin/python -m pip install \
  'numpy==1.26.4' 'scipy>=1.10' 'PyYAML>=6' 'Pillow>=10' \
  'open3d-cpu==0.19.0' 'opencv-python-headless==4.10.0.84' \
  'scikit-image==0.23.2' 'tyro==1.0.13' 'plotly==6.8.0'
```

The CPU wheel needs no robot or GPU device. Official Open3D 0.19 wheels do not
support Linux ARM64; a custom ARM64 backend needs separate validation. This
restriction concerns geometry dependencies, not the existing ROS protocol.
Record the resolved package versions for the deployed environment.

For the July 28 candidate cycle 4, prepare scan 5 with explicit absolute paths:

```bash
agent007_root=/data/agent007
dds_root=/data/dds-corrected
recording_root=/data/260728_133013
analysis_python=/data/venvs/behav3d-analysis/bin/python
PYTHONPATH="${agent007_root}/src:${agent007_root}" "${analysis_python}" \
  -m ocsr.printed_artifacts.ros_fresh_analysis prepare \
  --output /data/fresh/submitted/cycle_0005 \
  --scan "${recording_root}/field_loop/cycle_0005/scan" \
  --baseline-scan "${recording_root}/field_loop/cycle_0004/scan" \
  --calibration "${recording_root}/config" \
  --targets "${recording_root}/field_loop/cycle_0004/candidates/targets.yaml" \
  --profile-path /home/lab/behav3d_ws/src/behav3d_orchestrator/config/fresh_observation_profile.example.json \
  --pipeline-config /home/lab/behav3d_ws/src/behav3d_orchestrator/config/fresh_observation_pipeline.example.yaml \
  --dds-root "${dds_root}" --artifact-id july28-artifact \
  --observation-id july28-scan-0005 --candidate-cycle 4 \
  --expected-target-count 7 --frame-id world/base
```

The example profile carries July 28 settings, including depth bias, crop,
bead dimensions and axis flips. Review those settings for another recording;
they are not universal calibration. Preparation requires a new output folder
and prints its manifest path, SHA256 and binding. For a later consecutive
cycle, pass `--previous-job-directory` with the preceding generated job, not
the submitted input folder or an independently reconstructed past state.

Create separate existing directories for submitted inputs, generated jobs and
the coordinator session. Configure a local copy of the ROS YAML, for example:

```yaml
behav3d_supervised_agent_loop:
  ros__parameters:
    session_path: /data/fresh/session
    approval_timeout_s: 300.0
    observation_timeout_s: 930.0
behav3d_observation_bridge:
  ros__parameters:
    backend_mode: fresh_geometry
    evidence_root: /data/fresh/submitted
    source_roots: []
    agent007_root: /data/agent007
    dds_root: /data/dds-corrected
    analysis_output_root: /data/fresh/jobs
    worker_python: /data/venvs/behav3d-analysis/bin/python
    worker_timeout_s: 900.0
```

`analysis_output_root` must not overlap any input or trusted source root. The
coordinator's observation timeout must exceed the worker's whole-analysis
deadline so cleanup and result delivery have time to finish. No writable path
or model setting comes from the action goal.

Launch the normal review nodes with this configuration, arm review, and send
`EvaluateObservation` with `event_type=deposition_readback`, the printed
binding, and the prepared manifest path/hash. Use a new `request_id` and
`previous_evaluation_id=""` for the initial submission. Approve its exact
pending proposal through `approve_step`, as above. Completed retries return
the journal's cached result without creating another geometry job.

New jobs contain the frozen inputs, DDS current/baseline TSDF exports,
comparison metrics and HTML, semantic projections, and an immutable
`evaluation/` bundle with bead-cap, paired-change and central-shell evidence.
The typed result references the generated evaluation, not the input manifest.
`analysis_status.json` uses `INCOMPLETE`, `UNACCEPTED`, or
`COMPLETE_REVIEW_REQUIRED`, with `supervisor_acceptance=not_recorded`. Even the
last marker is analysis provenance, not approval or a coordinator receipt.
After cancellation, timeout or a coordinator interruption, retain these jobs
for inspection; never promote them by editing the marker or journal.

## Offline demonstration and tests

`scripts/supervised_review_demo.py` provides a software-only reproduction with
the same Supervisor and isolated evidence worker. It does not run ROS or models.
Without `--offline-demonstration-approval` it stops disarmed; that explicit flag
simulates an operator for the offline demonstration only, not the ROS runtime.
Use a fresh absolute output directory and an existing recorded evaluation:

```bash
python3 -B scripts/supervised_review_demo.py --manifest /data/agent007/outputs/run/cycle_0006/evaluation/manifest.json --evidence-root /data/agent007/outputs --source-root /data/260728_133013 --source-root /data/3DP-DDS --output /tmp/supervised-review-new --offline-demonstration-approval
```

Inspect the generated report, typed result, receipt and journal. A successful
demo verifies saved evidence and proves cached retries do not invoke the worker
again. It is not a new object detection, reconstruction, removal detection,
LLM reasoning or physical robot run.

The isolated process runner requires POSIX process-group semantics, supported
by the Linux ROS workstation and development Mac. Other platforms return
`UNSUPPORTED`. The worker is trusted code, not a security sandbox, and must
not daemonize or escape its process group.

Run the portable standard-library tests from the workspace root:

```bash
python3 -B -m unittest discover -s src/behav3d_commands/test -p test_observation_commands.py -q
python3 -B -m unittest discover -s src/behav3d_orchestrator/test -p 'test_observation*.py' -q
python3 -B -m unittest discover -s src/behav3d_orchestrator/test -p 'test_supervised*.py' -q
```

The command and wiring suites use isolated ROS-shaped test doubles. Worker
tests verify synthetic immutable bundles and adversarial process behaviour.
A real July 28 cycle 6 bundle also passed recorded-evidence verification.
Position and shape findings remain provisional; successful hash verification
does not calibrate registration or validate missing material physically.

The initial host validation on 23 October 2026 passed 175 selected tests: 110 new command,
supervisor, worker, process, mocked ROS wiring and offline demo tests, plus
65 existing offline-agent, contour, extrusion and scalar-field tests. The
orchestrator and command tests used the Agent007 Python environment; scalar
tests used the DDS Python environment with its existing geometry dependencies
and a temporary potpourri3d dependency directory. No dependencies were installed
into either repository. Python syntax, changed XML/YAML and diff whitespace
checks passed. A full style-lint run still reports style recommendations and
deliberate broad exception handling at asynchronous boundaries; the selected
syntax and undefined-name lint checks passed. Subsequent Docker validation
compiled the four review packages and passed 122 selected tests plus eight
real ROS transport cases. The later separate-process milestone passed 143
selected unit/configuration tests, two YAML compatibility tests, seven process
recovery cases and eleven recorded-replay checks; see `ROS_DOCKER.md` for those
historical run details. These counts describe the saved-evidence milestone,
not validation of the new fresh backend.

A host fresh-analysis check using corrected DDS source on scan 5 produced seven
candidate DDS results: six within tolerance and one missing. Its exact
reference comparison counted 23,418 matched, 2,055 missing and 294 extra voxels;
cap processing found five position-eligible observations and two requiring
cap-quality review.
These are outcomes under the configured geometry rules, not independently
measured defect labels or physical accuracy.

The later Linux amd64 fresh run passed two consecutive scans through separate
ROS coordinator, bridge and client processes. It made two analysis dispatches,
reused receipts on retry, accumulated fourteen candidate records, and finished
disarmed with clean node exits. The selected Linux review suite passed 188
tests plus two YAML compatibility checks; eight ROS transport cases and seven
process-recovery cases also passed. Agent007's scoped adapter/core tests passed
65 tests in its existing DDS environment. These are separate suites, not a
full-workcell build or physical validation. See the fresh run in `ROS_DOCKER.md`.

## Next integration milestones

1. Add unchanged, deliberately removed and occluded controls to the passing
   fresh consecutive-scan path. Verify
   registration and visibility before calling a change missing material.
2. Add inspection and intervention events for re-scans outside the N to N+1
   deposition sequence, including deformation of earlier observed geometry.
3. Add initial scene perception and, separately, optional semantic LLM review.
   Neither is supplied by the model-free fresh geometry adapter.
4. Present two rule-based planner alternatives: accept its normal lowest viable
   repair-prioritized candidates, or apply an independent region/dependency mask
   to continue elsewhere. Do not repaint the scalar field used for directions.
5. Add validated planning, collision-scene receipts and per-step human gates
   before connecting physical execution. Evidence review must never serve as
   a motion or extrusion authorization. Re-scan to evaluate the outcome.
