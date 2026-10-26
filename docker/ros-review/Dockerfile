# syntax=docker/dockerfile:1
# Review transport only: no UR driver, camera, extrusion server or workcell.
# Official base digest verified during the first native ARM64 build.
ARG ROS_IMAGE=ros:jazzy-ros-base@sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410
FROM ${ROS_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive PYTHONDONTWRITEBYTECODE=1

# Deliberately curated review dependencies instead of full-workspace rosdep:
# handeye_interfaces is a legacy commands exec_depend, not imported here.
# moveit-msgs supplies interface definitions, not a MoveIt planning runtime.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-matplotlib \
    python3-numpy \
    python3-pytest \
    python3-scipy \
    python3-setuptools \
    python3-yaml \
    ros-jazzy-action-msgs \
    ros-jazzy-ament-cmake \
    ros-jazzy-ament-index-python \
    ros-jazzy-builtin-interfaces \
    ros-jazzy-control-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-moveit-msgs \
    ros-jazzy-rclpy \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-default-runtime \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-ros-py \
    ros-jazzy-trajectory-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/behav3d_ws
COPY src/behav3d_interfaces/ src/behav3d_interfaces/
COPY src/behav3d_utils/ src/behav3d_utils/
COPY src/behav3d_commands/ src/behav3d_commands/
COPY src/behav3d_orchestrator/ src/behav3d_orchestrator/

RUN find src -type f -print0 | sort -z | xargs -0 sha256sum \
       > /opt/behav3d_ws/source-sha256.txt \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build --merge-install --executor sequential \
       --packages-select behav3d_interfaces behav3d_utils \
                         behav3d_commands behav3d_orchestrator \
       --event-handlers console_direct+ \
       --cmake-args -DBUILD_TESTING=OFF \
    && dpkg-query -W > /opt/behav3d_ws/apt-packages.txt

# Mac checkout permissions can be 0600 even for Git's 100644 resource files.
# Installed ROS metadata and code must be readable by the non-root test user.
RUN chmod -R a+rX /opt/behav3d_ws/install

RUN groupadd --gid 10001 behav3d-review \
    && useradd --uid 10001 --gid 10001 --create-home \
       --shell /bin/bash behav3d-review \
    && mkdir -p /results /etc/behav3d_review /opt/behav3d_review \
    && chown 10001:10001 /results

COPY docker/ros-review/cyclonedds.xml /etc/behav3d_review/cyclonedds.xml
COPY --chmod=0755 docker/ros-review/entrypoint.sh /opt/behav3d_review/entrypoint.sh
COPY --chmod=0755 docker/ros-review/validate.sh /opt/behav3d_review/validate.sh

ENV BEHAV3D_REVIEW_CONTAINER=1 \
    ROS_DOMAIN_ID=231 \
    ROS_LOCALHOST_ONLY=1 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    CYCLONEDDS_URI=file:///etc/behav3d_review/cyclonedds.xml \
    ROS_LOG_DIR=/results/ros-log \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    OPENBLAS_NUM_THREADS=1 \
    OMP_NUM_THREADS=1 \
    MKL_NUM_THREADS=1

USER 10001:10001
WORKDIR /workspace/behav3d_ws
ENTRYPOINT ["/opt/behav3d_review/entrypoint.sh"]
CMD ["/opt/behav3d_review/validate.sh"]
