#!/usr/bin/env bash
set -eo pipefail

# This image has no physical executor. Isolation is also enforced by Compose:
# network_mode:none, no devices, read-only checkout/rootfs and a non-root user.
if [[ "${BEHAV3D_REVIEW_CONTAINER:-}" != 1 \
      || "${ROS_LOCALHOST_ONLY:-}" != 1 \
      || "${ROS_DOMAIN_ID:-}" != 231 \
      || "${RMW_IMPLEMENTATION:-}" != rmw_cyclonedds_cpp \
      || ! -f /.dockerenv && ! -f /run/.containerenv ]]; then
    echo "Refusing: isolated review-container environment is required." >&2
    exit 64
fi
if [[ "$(id -u)" == 0 ]]; then
    echo "Refusing: review commands must run as the non-root image user." >&2
    exit 64
fi

# ROS setup scripts may inspect unset environment variables.
source /opt/ros/jazzy/setup.bash
source /opt/behav3d_ws/install/setup.bash
set -u
mkdir -p "${ROS_LOG_DIR}"
if [[ "$#" == 0 ]]; then
    set -- /opt/behav3d_review/validate.sh
fi
exec "$@"
