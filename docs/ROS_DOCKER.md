# ROS 2 review validation in Docker

This Docker environment builds the supervised observation integration on
Ubuntu 24.04 and ROS 2 Jazzy, then tests its real ROS actions and services
without connecting a robot. It is the Mac development environment for the
review-only milestone described in [Supervised observation review](SUPERVISED_AGENT_LOOP.md).

Native Linux and ROS remain the deployment target. These Docker files are
optional development and validation tooling; the runtime packages still build
with `colcon` and run with the existing Linux ROS launch conventions.

The image builds four packages: `behav3d_interfaces`, `behav3d_utils`,
`behav3d_commands` and `behav3d_orchestrator`. It does not build the complete
workcell. The default platform is `linux/arm64` for Apple Silicon; Mac Python
virtual environments are neither copied nor used inside Linux.

## Verified recorded review milestone

On 23 October 2026, Docker Desktop 4.96.3 ran this image natively as
Linux ARM64 on the MacBook. All four selected packages compiled, including
the observation action and approval services. That saved-evidence milestone passed:

- 143 selected unit/configuration tests, plus two existing YAML compatibility tests.
- Eight real ROS transport cases with coordinator, bridge and client nodes in one process.
- Seven separate-process cases, including coordinator crashes, cached recovery and backend outage.
- Eleven recorded-replay checks: saved scans 5, 6 and 7 advanced one shared artifact history, with eight rejection/retry controls.

The recorded replay used separate coordinator, bridge and client processes,
made exactly three successful worker dispatches, and finished disarmed. Two
additional dispatches in an independent journal deliberately tested invalid
frame/hash bindings and failed without advancing a head. Bundle metadata was
unchanged; no fresh scan analysis, model calls or robot commands occurred.

These checks found and fixed unreadable installed ament metadata and duplicate
ROS context shutdown after SIGTERM. The initial harness also allowed only
1.5 seconds for nodes that permit 10 seconds of executor drainage. The final
harness permits a bounded 12 seconds and still fails on forced cleanup. The
post-outage coordinator exited zero in 3.074 seconds in the final run.

Host regressions passed 208 tests plus 108 unittest subtests; the 143 review
tests overlap the Linux suite and must not be added as unique coverage. This
is selected review validation, not full-workspace `colcon test`, cross-machine
DDS, MoveIt/physics simulation or physical defect-detection validation.

The validated image ID is
`sha256:51677467a31409b828f8f5b0c95af0abe6e75c0fc0c5e2d7a7eba109a2e65c9b`.
The official ROS base is pinned by digest in both the Dockerfile and
Compose defaults. Installed Linux package versions and source hashes are
saved with each test run. Results were exported to the shared workspace's
`validation_multiprocess_20261023T152500Z` directory, outside this checkout.
It retains the initial failing shutdown check as well as the final passing
reports; the machine-readable overview is `validation-summary.json`.

## Build and run

Start Docker Desktop and open a terminal in this worktree:

```bash
cd /Users/josephnamar/Documents/GitHub/behav3d-supervised-loop
docker version
docker compose -f compose.ros-review.yaml config
docker compose -f compose.ros-review.yaml build
```

The first build downloads the official ROS base image and Linux packages, so
it requires internet access and several gigabytes of free disk space. The
container runs without external networking after the build.

Run the checks in a named container so its results remain easy to retrieve:

```bash
review_container="behav3d-review-$(date +%Y%m%dT%H%M%S)"
docker compose -f compose.ros-review.yaml run --name "${review_container}" ros-review
```

The command exits nonzero if a check fails. Each run creates a separate
`/results/review-validation-*` directory in the persistent
`ros-review-results` Compose volume. It contains the environment and source
hashes, installed package versions, unit-test logs, and
`ros-validation/summary.json` and `ros-multiprocess/summary.json`. Successful
summaries have `passed: true`; individual ROS cases are listed in `cases`.

Copy results to a directory outside the repository:

```bash
review_export="/Users/josephnamar/Downloads/${review_container}-results"
docker cp "${review_container}:/results/." "${review_export}"
```

Rebuild the image after editing any of the four packages. The validation
script compares the mounted checkout with the source hashes captured during
the build and rejects changed or missing included files. Adding new files
also requires a rebuild; the hash check only covers files already included
in the image. Changes to the smoke script are recorded separately in the run log.

## What the checks exercise

The unit suites cover the supervisor journal, approval and cancellation
logic, the saved-evidence worker, process limits, command routing, and
container configuration. The transport smoke test uses actual `rclpy`,
generated ROS interfaces, CycloneDDS, the command queue and an isolated
subprocess worker. It creates synthetic immutable evidence locally rather
than needing the July dataset.

The ROS cases check default-disarmed rejection, malformed goals, exact-hash
approval, a successful saved-evidence review, a cached retry without another
dispatch, client cancellation, operator denial, an approval deadline, and
disarming while the shared cooperative pause gate is active. Test approvals
are explicitly generated by the test client; the production nodes do not
auto-approve work.

A successful result verifies this software path. It does not establish
physical measurement accuracy or demonstrate a newly acquired scan being
analysed. These default tests exercise `backend_mode=recorded`. The opt-in
fresh geometry backend below runs additional dependencies and generates new
analysis; its results must not be conflated with recorded transport checks.

## Separate node process validation

`scripts/ros_review_multiprocess.py` puts the installed coordinator and bridge
in separate OS processes, with a third process acting only as a ROS client.
It uses generated interfaces and synthetic immutable evidence. The default
validation command includes this stage. To run it alone after rebuilding the
image:

```bash
review_process_run="behav3d-process-$(date +%Y%m%dT%H%M%S)"
docker compose -f compose.ros-review.yaml run --name "${review_process_run}" ros-review \
  python3 -B /workspace/behav3d_ws/scripts/ros_review_multiprocess.py \
  --repo /workspace/behav3d_ws --output "/results/${review_process_run}"
```

The seven cases cover exact-hash approval, cached retries, a coordinator
`SIGKILL` while waiting for approval, durable completed-receipt recovery,
a coordinator `SIGKILL` after START while the shared pause gate blocks
backend dispatch, an idle bridge outage, and explicit recovery with a new
request and approval. Coordinator-interrupted requests restart disarmed in
`HOLD` without automatic retry. This does not test abruptly killing a bridge
while its separately sessioned worker is actively verifying evidence.

The output contains `summary.json`, bounded stdout/stderr logs for each node
generation, process IDs and the journal transitions. The test deliberately
kills only its own review processes; it never starts workcell bringup. All
approvals are software test inputs, not confirmation by a human operator.
Graceful shutdown has a 12-second allowance; nonzero exits, unexpected forced
kills, tracebacks, truncated logs and undrained log readers fail the run.
The scenario deadline is 140 seconds, reserving cleanup time beneath the
default runner's 180-second outer deadline.

## Container isolation

The default Compose service has no devices, published ports, host networking
or Docker socket. It runs as a non-root user with all Linux capabilities
dropped, a read-only root filesystem and a read-only source checkout. Only
temporary storage and the results volume are writable.

ROS discovery is confined to loopback inside this container using
CycloneDDS and domain 231. The smoke test publishes cooperative pause state
only in this isolated domain. Do not run it against the workcell or remove
the network isolation to make a test pass.

The build context is an allowlist of the four source packages and the
review Docker configuration. Recordings, generated outputs, Git metadata,
environment files and virtual environments are excluded. The image
installs `moveit-msgs` for interface definitions, not MoveIt planning,
Gazebo, a UR driver or a camera/extruder server. This is not yet a robot
motion or physics simulator.

## Optional recorded evidence mounts

The default tests need no external recordings. To inspect an existing
Agent007 evaluation later, add the optional evidence override. The saved
manifests bind original absolute source paths, so the override preserves
those paths inside the container and mounts them read-only:

```bash
export BEHAV3D_AGENT007_ROOT='/Users/josephnamar/Documents/ChatGPT/SustaniAi sumerschool/agent007-main'
export BEHAV3D_DDS_ROOT='/Users/josephnamar/Documents/GitHub/3DP-DDS'
export BEHAV3D_RECORDING_ROOT='/Users/josephnamar/Downloads/260728_133013'
docker compose -f compose.ros-review.yaml -f compose.ros-review.evidence.yaml config
docker compose -f compose.ros-review.yaml -f compose.ros-review.evidence.yaml run ros-review
```

This exposes only the Agent007 and DDS `outputs` directories and the
explicit recording directory. It does not expose their source code,
credentials or Mac virtual environments. Mounts alone do not dispatch a
recorded review or grant an approval; the default command still runs the
synthetic test suite. Configure explicit evidence and source roots using
the review-node instructions before submitting a recorded observation.

### Consecutive recorded cycles through ROS

The opt-in recorded replay submits scan cycles 5, 6 and 7 through the same
separate coordinator and bridge processes. It retains their original
candidate-cycle mapping of 4 to 5, 5 to 6 and 6 to 7, and their exact preceding
evaluation identities. It starts a new journal rather than altering a saved
evaluation or inventing a review of earlier cycles.

With the three evidence variables above set and the image rebuilt, run:

```bash
review_replay_run="behav3d-recorded-$(date +%Y%m%dT%H%M%S)"
docker compose -f compose.ros-review.yaml -f compose.ros-review.evidence.yaml \
  run --name "${review_replay_run}" ros-review \
  python3 -B /workspace/behav3d_ws/scripts/ros_recorded_cycle_replay.py \
  --manifest "${BEHAV3D_AGENT007_ROOT}/outputs/integrated_evaluation_260728_133013_final/cycle_0005/evaluation/manifest.json" \
  --manifest "${BEHAV3D_AGENT007_ROOT}/outputs/integrated_evaluation_260728_133013_final/cycle_0006/evaluation/manifest.json" \
  --manifest "${BEHAV3D_AGENT007_ROOT}/outputs/integrated_evaluation_260728_133013_final/cycle_0007/evaluation/manifest.json" \
  --evidence-root "${BEHAV3D_AGENT007_ROOT}/outputs" \
  --source-root "${BEHAV3D_DDS_ROOT}/outputs" \
  --source-root "${BEHAV3D_RECORDING_ROOT}" \
  --output "/results/${review_replay_run}" --offline-demonstration-approval
```

`--offline-demonstration-approval` explicitly enables programmatic approvals
for this read-only protocol test. It is not human approval, LLM reasoning or
permission for robot execution. The replay checks exact evaluation, packet,
manifest, frame and cycle identities; advances a shared artifact head;
checks cached retries across process restart; and rejects stale or conflicting
requests. Wrong-frame and wrong-manifest-hash checks use a separate journal
without editing the evidence.

Inspect `report.json`, `requests.json`, the output `manifest.json`, the main
and negative-test journals, and `process-logs/`. `report.json` distinguishes
three accepted saved cycles from deliberate failed-binding tests and records
whether the bundle metadata stayed unchanged. The worker separately verifies
the underlying source hashes during each uncached request. All evidence is
mounted read-only, and the replay must finish disarmed.

These runs verify already saved analysis through ROS, not fresh RGB-D
processing, bead-removal detection or physical accuracy. For Linux recordings,
configure native absolute paths when generating new evaluation bundles. Do
not silently rewrite the source paths in existing content-bound manifests.

## Optional fresh RGB-D analysis image

`Dockerfile.fresh` adds a CPU geometry environment to an amd64 review image.
The existing ARM64 image remains suitable for recorded transport tests.
Open3D 0.19 has no official Linux ARM64 wheel, so fresh validation explicitly
uses `linux/amd64`, including emulation on Apple Silicon. This is optional Mac
validation tooling; native Linux uses an explicit analysis venv and the normal
ROS configuration described in `SUPERVISED_AGENT_LOOP.md`.

The analysis venv is `/opt/behav3d_analysis`. It adds pinned NumPy, Open3D CPU,
headless OpenCV, scikit-image, tyro and Plotly wheels while retaining access to
the base scientific packages. It does not change the ROS system interpreter
or prepend the venv to `PATH`. The build checks CPU imports and records
`fresh-python-packages.txt` and `fresh-apt-packages.txt` alongside the base
source and package inventories. No GUI, GPU device, camera driver, LLM SDK or
robot executor is added.

Build the matching base first, then the optional layer. Set all external roots
to explicit existing directories on the host; the DDS root must contain the
tested `open3d_x_fastest_v1` block-layout correction:

```bash
export BEHAV3D_AGENT007_ROOT=/data/agent007
export BEHAV3D_DDS_ROOT=/data/dds-corrected
export BEHAV3D_RECORDING_ROOT=/data/260728_133013
docker build --platform linux/amd64 -f docker/ros-review/Dockerfile \
  -t behav3d-ros-review:jazzy-amd64 .
docker compose -f compose.ros-review.yaml -f compose.ros-review.fresh.yaml config
docker compose -f compose.ros-review.yaml -f compose.ros-review.fresh.yaml build
```

Replace the `/data/...` examples with actual host paths, including on a Mac.
Only Agent007 `src/` and `scripts/`, DDS `src/` and `examples/`, and the selected
recording are mounted, read-only at the same absolute paths. Complete external
repositories, Git metadata, credentials, outputs and host virtual environments
are not mounted. The base checkout stays read-only. Network isolation,
non-root execution, dropped capabilities and the results volume are unchanged.
Do not combine this override with the saved-output evidence override unless a
separate task explicitly requires those additional mounts.

The opt-in harness freezes candidate cycles 4 and 5 with their baseline and
next scans, then submits them to separate ROS coordinator and bridge processes:

```bash
fresh_run="behav3d-fresh-$(date +%Y%m%dT%H%M%S)"
docker compose -f compose.ros-review.yaml -f compose.ros-review.fresh.yaml \
  run --name "${fresh_run}" ros-review \
  /opt/behav3d_analysis/bin/python -B /workspace/behav3d_ws/scripts/ros_fresh_evaluation.py \
  --agent007-root "${BEHAV3D_AGENT007_ROOT}" \
  --dds-root "${BEHAV3D_DDS_ROOT}" --dataset "${BEHAV3D_RECORDING_ROOT}" \
  --profile /workspace/behav3d_ws/src/behav3d_orchestrator/config/fresh_observation_profile.example.json \
  --output "/results/${fresh_run}" \
  --worker-python /opt/behav3d_analysis/bin/python \
  --candidate-cycle 4 --candidate-cycle 5 --offline-demonstration-approval
```

This flag enables software test approvals only. The harness checks that no
analysis starts while disarmed or with a wrong proposal hash, that approved
requests produce fresh TSDF, DDS, cap/pair/shell artifacts, and that completed
retries use cached receipts rather than recomputing geometry. Both cycles use
one accepted artifact history; the final state must be disarmed.

Each new output contains `submitted/` frozen inputs, `jobs/` generated geometry,
`session/` coordinator state, node `logs/`, and `report.json`. Reported
`input_mode=fresh_raw_depth_scan_job` distinguishes new computation from saved
replay. An `analysis_status.json` marker, including complete analysis, is not
a Supervisor acceptance receipt. Inspect the journal and typed result as well
as the generated evidence. Incomplete and unaccepted jobs remain available for
failure inspection.

### Verified fresh analysis run

On 23 October 2026 the Linux amd64 image passed fresh scans 5 and 6 through
separate ROS processes under Apple Silicon emulation. There were exactly two
analysis dispatches; completed retries reused their receipts. The shared
history contained fourteen candidates after scan 6. Both node processes
exited zero without forced cleanup, and the final review state was disarmed.
The selected regression suite passed 188 tests, two existing YAML checks,
eight single-process ROS cases and seven separate-process recovery cases.

Scan 5 reported six DDS candidates within tolerance and one missing, with five
centre observations passing provisional evidence gates. Scan 6 reported two
within tolerance and five missing, with three centre observations eligible.
Those DDS labels are surface-comparison findings, not confirmed physical
defects. Both cycles' DDS target records match the corrected saved reference
when floating-point values are rounded to twelve decimal places; scan 6 also
matches without rounding. The host scan-5 recomputation matched exactly.

The run exposed deployment issues now covered by guards or regressions:
the old DDS block layout is rejected; native stdio is flushed before restoring
the worker JSON channel; accepted bridge cancellation suppresses late success;
and an executor RCLError is ignored only when the ROS context is already shut
down. Active-context errors still propagate. The recovery test now waits for
the restarted bridge and gives its command eight seconds, longer than the
five-second worker budget; the deliberately absent bridge still times out.

The image is
`sha256:f1e8b331027676012c01d29a804829e2d8048e8babec7b25be7a896e91335112`.
Reports, exported DDS viewers and initial failed attempts are retained in
the shared workspace's `fresh_analysis_validation_20261023` directory.
The complete frozen/generated jobs remain in the named results volume;
exported product bundles keep their original Linux source references.

This establishes the fresh geometry software path, not independent defect
labels, registration accuracy, active-worker `SIGKILL` cleanup, scene
perception, LLM reasoning, structural evaluation or robot execution. Docker
remains optional validation tooling, not a native Linux runtime dependency.

## Troubleshooting

- If Docker cannot find the daemon, wait for Docker Desktop to finish
  starting and rerun `docker version`.
- If the source-hash check fails, rebuild the image from this worktree.
- If a bind mount fails, check Docker Desktop file sharing and the exact
  paths. The configuration deliberately refuses to create missing source
  directories.
- If ROS discovery times out, inspect `ros-transport.log` and the summary.
  Keep `network_mode: none` and the loopback middleware configuration.
- On an Intel Mac, set `BEHAV3D_REVIEW_PLATFORM=linux/amd64` before building.
  ARM64 is the platform validated for the recorded transport milestone;
  the optional fresh-analysis override explicitly selects amd64 on either Mac.
- If fresh analysis rejects the producer or block layout, use the configured
  corrected DDS tree and prepare a new frozen submission. Matching Git commits
  do not imply matching source fingerprints. Do not rewrite an old manifest.

The next simulation milestone is an additional, separately scoped image
for UR mock hardware and MoveIt planning. That work must keep physical
drivers, camera access and extrusion disabled and validate collision
geometry in the same frame as the observed mesh.
