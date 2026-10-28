#!/usr/bin/env bash
# Copyright 2026 Team Positronic. MIT license, see LICENSE at the repository root.
#
# Gazebo pick verification: the arm picks a cube off a pedestal with the gripper, while the wrist
# camera and a fixed camera show it in two windows. PASS when the cube was lifted.
#
# From the workspace root, inside the dev container, after make build:
#   make verify V=gazebo_pick      (or: bash docs/03_verification/gazebo_pick/run.sh)
# Logs and the camera frames of each step go to log/gazebo_pick/<date_time>/.
set -eo pipefail   # no -u: the ROS setup.bash reads variables that may be unset

DIR=$(cd "$(dirname "$0")" && pwd)
OUT=log/gazebo_pick/$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Every background process is stopped on exit, whatever the reason. Launch files stop their
# own children when they get the signal.
PIDS=()
cleanup() { kill "${PIDS[@]}" 2> /dev/null || true; wait 2> /dev/null || true; }
trap cleanup EXIT

echo "Gazebo with workcell_pick.sdf..."
ros2 launch positronic_bringup gazebo.launch.py world:=workcell_pick.sdf > "$OUT/gazebo.log" 2>&1 &
PIDS+=($!)

# The spawners give up after 5 s, and without a GPU the camera rendering makes the activation
# slower than that. Wait, then activate by hand whatever is still inactive.
active() { timeout 5 ros2 control list_controllers 2> /dev/null | grep -c " active" || true; }
for _ in $(seq 1 45); do sleep 2; [ "$(active)" = 3 ] && break; done
if [ "$(active)" != 3 ]; then
  for c in joint_state_broadcaster arm_controller gripper_action_controller; do
    ros2 control set_controller_state "$c" inactive > /dev/null 2>&1 || true
    ros2 control set_controller_state "$c" active > /dev/null 2>&1 || true
  done
fi
[ "$(active)" = 3 ] || { echo "FAIL: controllers not active, see $OUT/gazebo.log"; exit 1; }
echo "controllers active"

echo "move_group with the planning scene..."
ros2 launch positronic_moveit_config move_group.launch.py rviz:=false use_sim_time:=true \
  > "$OUT/move_group.log" 2>&1 &
PIDS+=($!)
for _ in $(seq 1 30); do sleep 2; grep -q "loaded into the planning scene" "$OUT/move_group.log" && break; done
grep -q "loaded into the planning scene" "$OUT/move_group.log" \
  || { echo "FAIL: planning scene not loaded, see $OUT/move_group.log"; exit 1; }

# The fixed camera of workcell_pick.sdf is bridged here, not in the launch: it only exists in
# this world. Then one window per camera.
ros2 run ros_gz_bridge parameter_bridge '/scene_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image' \
  > "$OUT/bridge.log" 2>&1 &
PIDS+=($!)
ros2 run rqt_image_view rqt_image_view /wrist_camera/image_raw > /dev/null 2>&1 &
PIDS+=($!)
ros2 run rqt_image_view rqt_image_view /scene_camera/image_raw > /dev/null 2>&1 &
PIDS+=($!)

echo "pick..."
if python3 "$DIR/pick.py" --out "$OUT" > "$OUT/pick.log" 2>&1; then result=PASS; else result=FAIL; fi
grep -vE "^\[(INFO|WARN)\]" "$OUT/pick.log" || true
echo "$result: frames and logs in $OUT"
[ "$result" = PASS ]
