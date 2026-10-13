#!/usr/bin/env bash
# One-time setup so your editor's clangd shows errors and clang-tidy warnings (make setup-editor).
# Installs, on YOUR machine (not in Docker): the ROS 2 Jazzy headers we compile against,
# clangd and a compiler. The build itself still runs only in Docker.
# Safe to run again: every step skips itself when already done.
set -euo pipefail

say() { printf '\n\033[1m== %s\033[0m\n' "$1"; }

# ROS 2 Jazzy has binary packages only for Ubuntu 24.04 (noble).
# shellcheck source=/dev/null
. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "noble" ]; then
  echo "This needs Ubuntu 24.04 (noble); you have ${PRETTY_NAME:-unknown}." >&2
  echo "You can still build with make build; only the editor integration is unavailable." >&2
  exit 1
fi

say "ROS 2 apt repository"
if dpkg -s ros2-apt-source > /dev/null 2>&1; then
  echo "already configured"
else
  # Official steps: docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  version=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.noble_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  rm -f /tmp/ros2-apt-source.deb
fi

say "clangd, compiler and ROS headers"
sudo apt-get update
if ! sudo apt-get install -y clangd build-essential \
  ros-jazzy-ros-base ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit; then
  echo >&2
  echo "apt could not install the packages. On a fresh Ubuntu 24.04 this is usually an" >&2
  echo "out-of-date system. Run 'sudo apt update && sudo apt full-upgrade', reboot, then" >&2
  echo "run make setup-editor again." >&2
  exit 1
fi

say "ros2 command in new terminals"
# Not needed by clangd (it reads the headers directly); this is for ros2 / ros2 pkg create.
line='source /opt/ros/jazzy/setup.bash'
if grep -qxF "$line" ~/.bashrc; then
  echo "already in ~/.bashrc"
else
  echo "$line" >> ~/.bashrc
  echo "added to ~/.bashrc (open a new terminal to use ros2)"
fi

say "VS Code clangd extension"
if command -v code > /dev/null; then
  code --install-extension llvm-vs-code-extensions.vscode-clangd
else
  echo "VS Code not found, skipped (Zed already includes clangd)"
fi
