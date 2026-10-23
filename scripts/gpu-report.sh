#!/usr/bin/env bash
# Positronic GPU report: what each machine's GPU does for Gazebo and RViz in our dev container.
# It changes nothing on your machine: the container it uses is throwaway (--rm).
#
#   bash scripts/gpu-report.sh            asks for your name
#   bash scripts/gpu-report.sh felipe     or pass it
#
# Run it in a normal terminal on your machine (NOT inside a container). It needs the dev image
# (make image) and a desktop session, and takes about 3 minutes. A Gazebo window opens twice
# for 45 seconds each: don't touch it.
# Result: scripts/gpu-report-<name>.txt. Commit and push that file (the script prints how).
set -u
cd "$(dirname "$0")/.." || exit 1

if [ -e /.dockerenv ]; then echo "Run this on your machine, not inside a container."; exit 1; fi
if [ ! -f compose.yaml ]; then echo "compose.yaml not found: run this from inside the repo."; exit 1; fi
if ! docker image inspect "positronic-dev:${ROS_DISTRO:-jazzy}" > /dev/null 2>&1; then
  echo "The dev image is missing. Run 'make image' first."; exit 1
fi

NAME="${1:-}"
if [ -z "$NAME" ]; then read -r -p "Your name (used in the file name): " NAME; fi
NAME=$(printf '%s' "$NAME" | tr '[:upper:]' '[:lower:]' | tr -c 'a-z0-9\n' '-' | sed 's/^-*//; s/-*$//' | cut -c1-30)
if [ -z "$NAME" ]; then echo "A name is needed."; exit 1; fi

OUT="scripts/gpu-report-${NAME}.txt"
echo "Writing $OUT"
exec > >(tee "$OUT") 2>&1

sec() { printf '\n===== %s =====\n' "$*"; }
run() { printf '$ %s\n' "$*"; "$@" 2>&1; echo "[exit $?]"; }

echo "Positronic GPU report for ${NAME}, $(date -Is)"

sec "A. Host"
run uname -m
run bash -c 'grep PRETTY_NAME /etc/os-release'
run bash -c 'echo "session=${XDG_SESSION_TYPE:-unset} DISPLAY=${DISPLAY:-unset} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-unset}"'
run nproc
run bash -c 'lspci | grep -iE "vga|3d|display"'
run nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
run ls -l /dev/dri
run bash -c 'stat -c "%n gid=%g (%G)" /dev/dri/* 2>/dev/null'
run docker --version
run docker compose version
run bash -c 'docker info 2>/dev/null | grep -i runtimes'
run nvidia-ctk --version

sec "B. Container: plain dev image, no GPU options (the same as 'make shell')"
# Same X access and uid/gid as the Makefile.
if command -v xhost > /dev/null; then xhost +SI:localuser:"$(id -un)" > /dev/null 2>&1; fi
export USER_UID="$(id -u)" USER_GID="$(id -g)"
if [ "$USER_UID" = 0 ]; then USER_UID=1000; USER_GID=1000; fi

docker compose run --rm -T dev bash -s << 'INSIDE' 2>&1
echo "DISPLAY=${DISPLAY:-unset}"
echo '$ ls -l /dev/dri'; ls -l /dev/dri 2>&1

sudo apt-get update -qq > /dev/null 2>&1
sudo apt-get install -y -qq --no-install-recommends mesa-utils > /dev/null 2>&1 || echo "mesa-utils install FAILED"
PAT='direct rendering|OpenGL renderer|OpenGL version|core profile version|rror|nable|ailed'

echo; echo '$ glxinfo -B   (default rendering)'
glxinfo -B 2>&1 | grep -E "$PAT"
echo; echo '$ LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B   (software rendering)'
LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B 2>&1 | grep -E "$PAT"

# gz_test <label> <VAR=value>: runs the shapes world for 45 s and reads the real-time factor.
gz_test() {
  label=$1; shift
  echo; echo "\$ gz sim -r shapes.sdf   ($label, 45 s)"
  rm -f /tmp/gz.log
  ( env "$@" timeout -s INT 45 gz sim -r shapes.sdf > /tmp/gz.log 2>&1 & )
  sleep 30
  topic=$(gz topic -l 2>/dev/null | grep -m1 '/stats$')
  echo "stats topic: ${topic:-none found}"
  if [ -n "$topic" ]; then
    timeout 10 gz topic -e -t "$topic" -n 3 2>&1 | grep -E "real_time_factor|sim_time|paused"
  fi
  sleep 16
  echo "gz processes still running: $(pgrep -fc 'gz sim' || true)"
  echo "--- errors in the gz log (first 5 distinct)"
  grep -E "\[Err\]|OGRE EXCEPTION|Segmentation" /tmp/gz.log | sort -u | head -5
  pkill -f 'gz sim' 2> /dev/null || true
  sleep 2
}
gz_test "default rendering" POSITRONIC_TEST=default
gz_test "software rendering" LIBGL_ALWAYS_SOFTWARE=1
INSIDE

sec "Done"
echo "In the Gazebo windows you saw: were they drawn, and what RTF did the GUI show?"
echo "Now share the file:"
echo "  git pull && git add $OUT && git commit -m \"chore(gpu): add gpu report for ${NAME}\" && git push"
sleep 1
