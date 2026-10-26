#!/usr/bin/env bash
set -euo pipefail

if [[ "${BEHAV3D_REVIEW_CONTAINER:-}" != 1 \
      || "${ROS_LOCALHOST_ONLY:-}" != 1 \
      || "${ROS_DOMAIN_ID:-}" != 231 \
      || "${RMW_IMPLEMENTATION:-}" != rmw_cyclonedds_cpp \
      || ! -f /.dockerenv && ! -f /run/.containerenv ]]; then
    echo "Refusing to run ROS control-topic tests outside the isolated review container." >&2
    exit 64
fi

review_repo=/workspace/behav3d_ws
review_tests="${review_repo}/src/behav3d_orchestrator/test"
if [[ ! -f "${review_repo}/scripts/ros_review_smoke.py" \
      || ! -f "${review_tests}/test_observation_worker.py" ]]; then
    echo "Mount the complete current BEHAV checkout read-only at ${review_repo}." >&2
    exit 66
fi
review_run_dir="$(mktemp -d /results/review-validation-XXXXXX)"
echo "Review-only validation output: ${review_run_dir}"

# A read-only source checkout must match the compiled/installed image overlay.
# Rebuild after source edits rather than accidentally testing stale ROS nodes.
if ! (cd "${review_repo}" && sha256sum --check --status /opt/behav3d_ws/source-sha256.txt); then
    echo "Review image source differs from the mounted checkout; rebuild the image." >&2
    exit 78
fi
cp /opt/behav3d_ws/apt-packages.txt "${review_run_dir}/apt-packages.txt"

# Imports alone do not prove that installed packages are discoverable by ROS.
for review_package in behav3d_interfaces behav3d_utils behav3d_commands behav3d_orchestrator; do
    ros2 pkg prefix "${review_package}"
done | tee "${review_run_dir}/installed-package-prefixes.txt"

{
    python3 -B --version
    uname -m
    echo "ROS_DISTRO=${ROS_DISTRO}"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
    echo "Scope: saved-evidence verification and real ROS transport; no fresh scan analysis or fabrication."
    sha256sum "${review_repo}/scripts/ros_review_smoke.py"
    cat /opt/behav3d_ws/source-sha256.txt
} | tee "${review_run_dir}/environment.txt"

# These processes are separate so ROS mocks in a unit test cannot contaminate
# either the stdlib supervisor tests or the actual rclpy transport test.
for review_pattern in \
    test_supervised_agent.py \
    test_observation_worker.py \
    test_observation_process.py \
    test_supervised_review_demo.py \
    test_supervised_ros_wiring.py \
    test_ros_review_multiprocess_helpers.py \
    test_recorded_cycle_replay_helpers.py \
    test_fresh_observation.py \
    test_fresh_docker_config.py \
    test_docker_review_config.py; do
    if [[ ! -f "${review_tests}/${review_pattern}" ]]; then
        echo "Required review test missing: ${review_pattern}" >&2
        exit 66
    fi
    python3 -B -m unittest discover -s "${review_tests}" -p "${review_pattern}" -v \
        2>&1 | tee "${review_run_dir}/${review_pattern%.py}.log"
done
python3 -B -m unittest discover \
    -s "${review_repo}/src/behav3d_commands/test" \
    -p test_observation_commands.py -v \
    2>&1 | tee "${review_run_dir}/test_observation_commands.log"

# This imports installed generated interfaces and real rclpy, runs isolated
# bridge/coordinator nodes, and writes an explicit machine-readable summary.
timeout --signal=TERM --kill-after=10s 120s \
    python3 -B "${review_repo}/scripts/ros_review_smoke.py" \
    --output "${review_run_dir}/ros-validation" \
    2>&1 | tee "${review_run_dir}/ros-transport.log"

# Installed bridge and coordinator now run in separate OS process groups;
# the client imports no runtime node classes. Crash tests remain review-only.
timeout --signal=TERM --kill-after=10s 180s \
    python3 -B "${review_repo}/scripts/ros_review_multiprocess.py" \
    --repo "${review_repo}" --output "${review_run_dir}/ros-multiprocess" \
    2>&1 | tee "${review_run_dir}/ros-multiprocess.log"

echo "Review-only checks passed. ROS summaries: ${review_run_dir}/ros-validation/summary.json and ${review_run_dir}/ros-multiprocess/summary.json"
echo "No robot, reconstruction, fresh model analysis, motion or printing was executed."
