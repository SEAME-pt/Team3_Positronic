#!/usr/bin/env bash
set -u

failed=0
check() {
  local name="$1"; shift
  if out=$("$@" 2>&1); then
    printf '  \033[32m[ok]\033[0m   %-22s %s\n' "$name" "$(echo "$out" | head -n1)"
  else
    printf '  \033[31m[fail]\033[0m %-22s %s\n' "$name" "$(echo "$out" | tail -n1)"
    failed=1
  fi
}

echo "Positronic dev environment"
check "ROS 2"            bash -c 'echo "${ROS_DISTRO:?not sourced}"; command -v ros2 > /dev/null'
check "colcon"           command -v colcon
check "colcon mixins"    bash -c 'colcon mixin show 2> /dev/null | grep -q coverage-gcc && echo "coverage-gcc available"'
check "ros2_control"     ros2 pkg prefix controller_manager
check "MoveIt 2"         ros2 pkg prefix moveit_ros_move_group
check "clang-format"     clang-format --version
check "clang-tidy"       bash -c 'clang-tidy --version | grep -i version'
check "pymycobot"        python3 -c 'import importlib.metadata as m; print(m.version("pymycobot"))'
check "repo writable"    bash -c 'f=/ws/.doctor_$$ && touch "$f" && rm "$f" && echo "uid $(id -u) can write /ws"'

check "DDS pub/sub" python3 - <<'EOF'
import time
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("positronic_doctor")
got = []
node.create_subscription(String, "/positronic_doctor", lambda m: got.append(m.data), 10)
pub = node.create_publisher(String, "/positronic_doctor", 10)
start = time.time()
while not got and time.time() - start < 5.0:
    pub.publish(String(data="ping"))
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
print(f"round trip {'ok' if got else 'timed out'}")
raise SystemExit(0 if got else 1)
EOF

if [ -n "${DISPLAY:-}" ]; then
  check "display ${DISPLAY}" bash -c 'xdpyinfo | grep "name of display"'
else
  printf '  [skip]   %-22s %s\n' "display" "DISPLAY not set, GUI apps will not open"
fi

# GPU: Gazebo's renderer needs OpenGL 3.3 core. glxinfo ships in the image (mesa-utils).
if [ -n "${DISPLAY:-}" ] && command -v glxinfo > /dev/null; then
  gl=$(glxinfo -B 2> /dev/null)
  renderer=$(echo "$gl" | sed -n 's/^OpenGL renderer string: //p')
  core=$(echo "$gl" | sed -n 's/^OpenGL core profile version string: \([0-9]*\.[0-9]*\).*/\1/p')
  compat=$(echo "$gl" | sed -n 's/^OpenGL version string: \([0-9]*\.[0-9]*\).*/\1/p')
  if [ -z "$renderer" ]; then
    printf '  \033[31m[fail]\033[0m %-22s %s\n' "OpenGL" "no OpenGL context on ${DISPLAY} (run it from make shell, not over ssh)"
    failed=1
  elif [ -z "$core" ]; then
    printf '  \033[31m[fail]\033[0m %-22s %s\n' "OpenGL" "no core profile (${renderer}, OpenGL ${compat:-?}), Gazebo will crash. Set GPU=sw in .env"
    failed=1
  elif ! awk -v v="$core" 'BEGIN { exit !(v >= 3.3) }'; then
    printf '  \033[31m[fail]\033[0m %-22s %s\n' "OpenGL" "core ${core} < 3.3, Gazebo will crash. Set GPU=sw in .env"
    failed=1
  elif [[ "$renderer" == *llvmpipe* ]] && [ "${LIBGL_ALWAYS_SOFTWARE:-}" != "1" ]; then
    printf '  \033[33m[warn]\033[0m %-22s %s\n' "OpenGL" "software rendering (${renderer}), Gazebo will be slow. Set GPU=dri or GPU=nvidia in .env"
  else
    printf '  \033[32m[ok]\033[0m   %-22s %s\n' "OpenGL" "${renderer} (core ${core})"
  fi
fi

if [ -n "${ARM_IP:-}" ]; then
  check "arm ${ARM_IP}:${ARM_PORT:-22}" python3 -c "
import socket
s = socket.create_connection(('${ARM_IP}', int('${ARM_PORT:-22}')), timeout=3)
s.close(); print('reachable')"
else
  printf '  [skip]   %-22s %s\n' "arm" "set ARM_IP to test the connection to the Jetson"
fi

exit "$failed"
