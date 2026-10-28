#!/usr/bin/env bash
# Creates the container user with the host's uid/gid and sets up its ROS environment.
# Run as root by the Dockerfile, once per target, as the LAST step: the heavy layers above
# never depend on the uid/gid, so they are cached the same way on every machine.
#
# Usage: setup-user.sh <username> <uid> <gid> <ros_distro>
set -euo pipefail

username=$1
uid=$2
gid=$3
distro=$4

# The Ubuntu base image ships a user "ubuntu" with uid 1000. Remove it so the host's uid is free.
if getent passwd ubuntu > /dev/null; then
  userdel ubuntu
  rm -rf /home/ubuntu
fi
if ! getent group "${gid}" > /dev/null; then
  groupadd --gid "${gid}" "${username}"
fi
useradd --uid "${uid}" --gid "${gid}" --create-home --shell /bin/bash "${username}"
# dialout: the arm's serial port. video: the camera.
usermod -aG dialout,video "${username}"

mkdir -p /etc/sudoers.d
echo "${username} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${username}"
chmod 0440 "/etc/sudoers.d/${username}"

# rosdep and colcon keep their data under $HOME. Run as root with the user's HOME,
# then hand the whole home over to the user.
export HOME="/home/${username}"
cd "${HOME}"
rosdep update --rosdistro "${distro}"
colcon mixin add default \
  https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
echo "source /opt/ros/${distro}/setup.bash" >> "${HOME}/.bashrc"
echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> "${HOME}/.bashrc"
chown -R "${uid}:${gid}" "${HOME}"
