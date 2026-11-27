# One Dockerfile, three targets that share their lower layers:
#
#   runtime  The Jetson (arm64). Builds and runs the driver: ros2_control, the ROS lint set,
#            pymycobot. No MoveIt, Gazebo, RViz or clang tools (4 GB RAM, small disk).
#   ci       GitHub Actions. runtime's packages + clang tools + MoveIt. No Gazebo, no GUI.
#   dev      Laptops and the dev container. ci + Gazebo, RViz, rqt: the old single image.
#
#   docker build --target <name> docker        (the last stage, dev, is the default)
#
# Stage order: core -> runtime -> tools -> ci -> sim -> dev.
#
# Rules for editing it:
#  - Package stages go from stable to volatile: core -> tools -> sim. Inside a stage, one apt
#    layer per theme, so adding a package re-downloads that layer and not the whole image.
#  - Each target sits right after the stage it is built from, and runtime right after core.
#    The classic builder builds EVERY stage above the --target, needed or not: with runtime
#    further down, the Jetson would install MoveIt and Gazebo before reaching it. BuildKit
#    skips unused stages, so CI would never notice.
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

# Temporary: make build compiles positronic_mission on the Jetson,
# which needs MoveIt's planning interface
# The mission node never runs on the Jetson.
# The proper fix is a COLCON_DEFAULTS_FILE that skips positronic_mission there,
# after which this block goes.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pymycobot

# The wrist camera on the Jetson (camera.launch.py): OpenCV reads the USB camera (V4L2, MJPG)
# and turns the frames; image_transport_plugins lets a laptop subscribe to the compressed image
# over Wi-Fi (RViz, rqt), and lets `image_transport republish` run anywhere.
# OpenCV from pip, headless: Ubuntu's python3-opencv drags GUI and VTK libraries the Jetson's
# disk can't spare. --no-deps and the apt numpy: pip must not install a second numpy over the
# one the ROS packages use. 4.10.0.84 accepts numpy >= 1.26 (Ubuntu 24.04 ships 1.26.4).
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-numpy \
      ros-${ROS_DISTRO}-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir --no-deps opencv-python-headless==4.10.0.84

# ---------------------------------------------------------------------------
# runtime target: core + the container user. Right after core, see the rules above.
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

# ---------------------------------------------------------------------------
# tools: what building and checking our code needs beyond running it (CI and laptops).
# ---------------------------------------------------------------------------
FROM core AS tools

ARG ROS_DISTRO

# lcov and python3-pytest-cov: line coverage of C++ and Python (scripts/coverage.sh, make coverage).
# libcpanel-json-xs-perl: lcov finds it by itself and reads its JSON about 8x faster than JSON::PP.
# ccache: CI's builds (.github/workflows/ci.yml).
RUN apt-get update && apt-get install -y --no-install-recommends \
      gdb ccache clang-tidy clangd \
      lcov libcpanel-json-xs-perl python3-pytest-cov \
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
# ci target: tools + the container user.
# ---------------------------------------------------------------------------
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

# Camera calibration with a checkerboard (camera_calibration, a GUI), on laptops. It needs
# Ubuntu's python3-opencv with its GUI: the headless pip OpenCV of core comes first on the path
# and has no windows, so it goes. wrist_camera.py then uses Ubuntu's OpenCV here.
RUN pip3 uninstall -y opencv-python-headless \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-camera-calibration \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# dev target: sim + the container user. Keep it last: it is what `docker build` makes
# when no --target is given.
# ---------------------------------------------------------------------------
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
