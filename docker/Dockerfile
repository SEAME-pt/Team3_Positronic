ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo gdb ccache python3-pip clang-tidy x11-utils \
      ros-${ROS_DISTRO}-ros2-control \
      ros-${ROS_DISTRO}-ros2-controllers \
      ros-${ROS_DISTRO}-moveit \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-ament-lint-auto \
      ros-${ROS_DISTRO}-ament-lint-common \
      ros-${ROS_DISTRO}-ament-cmake-clang-format \
      ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
    && rm -rf /var/lib/apt/lists/*

# Pinned so every machine formats identically, whatever the Ubuntu version.
RUN pip3 install --no-cache-dir clang-format==18.1.8 pymycobot

# The container user gets the host user's uid/gid (see Makefile).
RUN if getent passwd ubuntu > /dev/null; then userdel ubuntu && rm -rf /home/ubuntu; fi \
    && if ! getent group "${USER_GID}" > /dev/null; then groupadd --gid "${USER_GID}" "${USERNAME}"; fi \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" \
    && usermod -aG dialout,video "${USERNAME}" \
    && mkdir -p /etc/sudoers.d \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

USER ${USERNAME}

RUN cd "$HOME" \
	&& rosdep update --rosdistro "${ROS_DISTRO}" \
    && colcon mixin add default \
         https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update default \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> ~/.bashrc

WORKDIR /ws
CMD ["bash"]
