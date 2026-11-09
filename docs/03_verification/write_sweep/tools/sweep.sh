#!/usr/bin/env bash
# Sweep of the serial-thread timings on the real arm (#32).
#
# Runs inside the Jetson container (make nano-shell), with the arm powered on and the bench
# clear. For every point P:S (moving_period_ms:send_settle_ms) it starts the driver with those
# timings, activates forward_position_controller, streams a command at 30 Hz for DURATION
# seconds (joint 1 swings smoothly by DELTA rad, 4 s per cycle, so the target changes every
# tick and the serial thread stays in its moving mode) while joint_states_rate.py records the
# interval between new poses, then stops the driver and waits COOLDOWN seconds. Results go to
# /ws/log/sweep_<date>/<P>_<S>/: summary.txt, intervals.csv, driver.log.
#
#   bash docs/03_verification/write_sweep/tools/sweep.sh            # the default points
#   POINTS="50:30 40:30" COOLDOWN=60 bash docs/03_verification/write_sweep/tools/sweep.sh
#
# Ctrl+C stops the driver and leaves the arm where it is.

POINTS=${POINTS:-"50:30 40:30 35:25 30:20"}
DURATION=${DURATION:-120}
COOLDOWN=${COOLDOWN:-120}
DELTA=${DELTA:-0.1745}  # 10 deg on joint 1
PORT=${PORT:-/dev/ttyTHS1}
OUT=/ws/log/sweep_$(date +%Y-%m-%d_%H%M)

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u  # after the ROS setup files, which read variables they do not define

driver_pid=""
stop_driver() {
  if [ -n "$driver_pid" ] && kill -0 "$driver_pid" 2>/dev/null; then
    kill -INT -- "-$driver_pid" 2>/dev/null
    wait "$driver_pid" 2>/dev/null
  fi
  driver_pid=""
}
trap 'echo; echo "interrupted"; stop_driver; exit 1' INT TERM

# The arm must be in fresh mode 1 for streamed commands (#20). Set it once, before the driver
# takes the port, and read the rest pose at the same time: the six angles come in protocol
# order, J1 to J6, which is the order of the controller's joints. Radians, comma separated.
rest=$(python3 - "$PORT" <<'PY'
import math
import sys
from pymycobot import MyCobot280
arm = MyCobot280(sys.argv[1], 1000000)
arm.power_on()
arm.set_fresh_mode(1)
angles = arm.get_angles()
print("fresh mode read-back:", arm.get_fresh_mode(), " angles:", angles, file=sys.stderr)
if not isinstance(angles, list) or len(angles) != 6:
    sys.exit("no angles from the arm")
print(','.join(f'{math.radians(a):.4f}' for a in angles))
PY
) || exit 1

# Streams the command at 30 Hz for $2 seconds: joint 1 swings from the rest pose "$1" by
# DELTA rad and back, 4 s per cycle, the other joints stay. One publisher for the whole run:
# a `ros2 topic pub --once` per message loses most of them to DDS discovery.
stream() {
  python3 - "$1" "$2" "$DELTA" <<'PY'
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rest = [float(x) for x in sys.argv[1].split(',')]
duration = float(sys.argv[2])
delta = float(sys.argv[3])
rclpy.init()
node = Node('sweep_stream')
pub = node.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
start = time.monotonic()
while pub.get_subscription_count() == 0 and time.monotonic() - start < 10:
    time.sleep(0.1)
start = time.monotonic()
while time.monotonic() - start < duration:
    t = time.monotonic() - start
    target = list(rest)
    target[0] = rest[0] + delta * (1 - math.cos(2 * math.pi * t / 4)) / 2
    pub.publish(Float64MultiArray(data=target))
    time.sleep(1 / 30)
pub.publish(Float64MultiArray(data=rest))
node.destroy_node()
rclpy.try_shutdown()
PY
}

mkdir -p "$OUT"
echo "results in $OUT"
for point in $POINTS; do
  period=${point%%:*}
  settle=${point##*:}
  dir="$OUT/${period}_${settle}"
  mkdir -p "$dir"
  echo
  echo "=== moving_period_ms=$period send_settle_ms=$settle ==="

  setsid ros2 launch positronic_bringup driver.launch.py port:="$PORT" \
    moving_period_ms:="$period" send_settle_ms:="$settle" >"$dir/driver.log" 2>&1 &
  driver_pid=$!

  # Wait for the driver to be up: a first /joint_states.
  if ! timeout 30 bash -c 'source /opt/ros/jazzy/setup.bash; ros2 topic echo --once /joint_states >/dev/null'; then
    echo "no /joint_states within 30 s, see $dir/driver.log"
    stop_driver
    continue
  fi
  ros2 run controller_manager spawner forward_position_controller >>"$dir/driver.log" 2>&1

  echo "rest  $rest  (joint 1 swings +$DELTA rad)"

  # Rate of new poses on /joint_states. Stalls are counted on the serial line instead: run
  # proxy_tap.py first and point PORT at its pty, then tap_stalls.py on the tap.
  ros2 run positronic_bringup joint_states_rate.py --duration "$DURATION" \
    --out "$dir" >"$dir/rate.log" 2>&1 &
  rate_pid=$!
  sleep 2
  stream "$rest" "$((DURATION - 4))"
  wait "$rate_pid"
  stop_driver
  cat "$dir/summary.txt"

  if [ "$point" != "${POINTS##* }" ]; then
    echo "cooldown $COOLDOWN s"
    sleep "$COOLDOWN"
  fi
done

echo
echo "=== all points ==="
for d in "$OUT"/*/; do
  echo "$(basename "$d"): $(tr '\n' ' ' <"$d/summary.txt")"
done
