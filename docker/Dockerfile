# One Dockerfile, three targets that share their lower layers:
#
#   runtime  The Jetson (arm64). Builds and runs the driver: ros2_control, the ROS lint set,
#            pymycobot. No MoveIt, Gazebo, RViz or clang tools (4 GB RAM, small disk).
#   ci       GitHub Actions. runtime's packages + clang tools + MoveIt. No Gazebo, no GUI.
#   dev      Laptops and the dev container. ci + Gazebo, RViz, rqt: the old single image.
#
#   docker build --target <name> docker        (the last stage, dev, is the default)
#
# Rules for editing it:
#  - Stages go from stable to volatile: core -> tools -> sim. Inside a stage, one apt layer
#    per theme, so adding a package re-downloads that layer and not the whole image.
#  - The uid/gid steps stay in the last lines of each target. A changed build arg
#    invalidates the cache of every instruction after the ARG.
#  - Add a package to the lowest stage that needs it. Jetson needs it? core. Only
#    CI and laptops? tools. Only laptops? sim.
#  - No BuildKit-only syntax (RUN --mount, "# syntax="): the Jetson builds with its own
#    Docker 24.0.7, which may use the classic builder.

ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# core: everything the driver needs to build and run. Shared by all targets.
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS core

ARG ROS_DISTRO

# AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS: lets the ament cppcheck test run with the cppcheck
# version Ubuntu ships. Same for everyone, so it lives in the image, not in .env.
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PYTHONUNBUFFERED=1 \
    AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

# ament-lint-* are needed even to *build*: every package's CMakeLists asks for
# ament_lint_auto when BUILD_TESTING is on (colcon's default).
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo python3-pip \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-control-cmake \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-ament-lint-auto \
      ros-${ROS_DISTRO}-ament-lint-common \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pymycobot

# ---------------------------------------------------------------------------
# tools: what building and checking our code needs beyond running it (CI and laptops).
# ---------------------------------------------------------------------------
FROM core AS tools

ARG ROS_DISTRO

RUN apt-get update && apt-get install -y --no-install-recommends \
      gdb ccache clang-tidy clangd \
      ros-${ROS_DISTRO}-ament-cmake-clang-format \
      ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
    && rm -rf /var/lib/apt/lists/*

# Pinned so every machine formats identically, whatever the Ubuntu version.
RUN pip3 install --no-cache-dir clang-format==18.1.8

# Last in this stage: it is the layer most likely to grow as the packages arrive.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-moveit \
      ros-${ROS_DISTRO}-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# sim: simulation and GUI tools. Laptops only (never the Jetson, never CI for now).
# ---------------------------------------------------------------------------
FROM tools AS sim

ARG ROS_DISTRO

# The biggest download of the image, in its own layer.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      x11-utils mesa-utils \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rqt \
      ros-${ROS_DISTRO}-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Targets. Each one only adds the container user (the host's uid/gid, see the Makefile),
# so a different uid never invalidates the layers above.
# ---------------------------------------------------------------------------
FROM core AS runtime

ARG ROS_DISTRO
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

COPY setup-user.sh /tmp/setup-user.sh
RUN bash /tmp/setup-user.sh "${USERNAME}" "${USER_UID}" "${USER_GID}" "${ROS_DISTRO}" \
    && rm /tmp/setup-user.sh

USER ${USERNAME}
WORKDIR /ws
CMD ["bash"]

FROM tools AS ci

ARG ROS_DISTRO
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

COPY setup-user.sh /tmp/setup-user.sh
RUN bash /tmp/setup-user.sh "${USERNAME}" "${USER_UID}" "${USER_GID}" "${ROS_DISTRO}" \
    && rm /tmp/setup-user.sh

USER ${USERNAME}
WORKDIR /ws
CMD ["bash"]

# Keep dev last: it is what `docker build` makes when no --target is given.
FROM sim AS dev

ARG ROS_DISTRO
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

COPY setup-user.sh /tmp/setup-user.sh
RUN bash /tmp/setup-user.sh "${USERNAME}" "${USER_UID}" "${USER_GID}" "${ROS_DISTRO}" \
    && rm /tmp/setup-user.sh

USER ${USERNAME}
WORKDIR /ws
CMD ["bash"]
