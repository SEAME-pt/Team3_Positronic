#!/usr/bin/env bash
# Copyright 2026 Positronic
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# AI-generated (Claude, Anthropic).
# Prompt:
#   Write the runner for the #124 runs, for the Jetson container with the driver already up:
#   start vmstat, the thread sampler on ros2_control_node and rt_record.py, then for 10 minutes
#   send a T2 goal every ~23 s through the ros2 CLI (variant A), or start and stop a throwaway
#   participant every 23 s with no goal (G), or send the goals from goals_node.py (H). Kill the
#   recorders with SIGTERM at the end and write start and end stamps.

# #124: 10 minutes of the driver with something arriving every ~23 s. Run inside the Jetson
# container with driver.launch.py already up (serial or fake backend):
#   bash run124.sh A    T0, then a T2 goal per `ros2 action send_goal` process: reproduces the pause
#   bash run124.sh G    no goal, a throwaway participant (`ros2 node list`) every 23 s
#   bash run124.sh H    T2 goals from one persistent node (goals_node.py)
# A and H MOVE THE ARM on the serial backend (joints 1 and 2, 20 deg at 5 deg/s): bench clear,
# two people, hand on the plug. Output in $OUT (default /ws/log/rt/124_<variant>_<time>), read
# with: python3 tools/pauses_124.py $OUT

set -uo pipefail
V=${1:?variant A, G or H}; DUR=${2:-600}
OUT=${OUT:-/ws/log/rt/124_${V}_$(date +%H%M)}; mkdir -p "$OUT"
HERE=$(cd "$(dirname "$0")" && pwd)
TR=/ws/docs/03_verification/jtc_on_the_arm/trajectories
pids=()
cleanup() { for p in "${pids[@]:-}"; do [ -n "$p" ] && kill -TERM "$p" 2>/dev/null; done; }
trap cleanup EXIT INT TERM
ros2 topic list | grep -qx /joint_states || { echo "driver not up"; exit 1; }
CM=$(pidof ros2_control_node | cut -d" " -f1)
[ -n "$CM" ] || { echo "no ros2_control_node process"; exit 1; }
date +%s.%N > "$OUT/start.txt"
vmstat 1 > "$OUT/vmstat.txt" 2>&1 & pids+=($!)
python3 "$HERE/../thread_sampler.py" --pid "$CM" --out "$OUT/threads.csv" \
  --duration $((DUR + 40)) > "$OUT/sampler.out" 2>&1 & pids+=($!)
python3 "$HERE/../rt_record.py" --out "$OUT" --duration $((DUR + 15)) \
  > "$OUT/rt_record.txt" 2>&1 & pids+=($!)
sleep 5
END=$(( $(date +%s) + DUR )); n=0
send() {
  ros2 action send_goal /arm_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "$(cat "$TR/$1.yaml")"
}
case "$V" in
  A) { send T0_to_start
       while [ "$(date +%s)" -lt "$END" ]; do
         n=$((n+1)); echo "goal $n T2 $(date +%s.%N)" >> "$OUT/goals.txt"; send T2; sleep 10
       done; } > "$OUT/goals_out.txt" 2>&1 ;;
  G) while [ "$(date +%s)" -lt "$END" ]; do
       n=$((n+1)); echo "participant $n start $(date +%s.%N)" >> "$OUT/churn.txt"
       ros2 node list > /dev/null 2>&1; sleep 23
     done ;;
  H) python3 "$HERE/goals_node.py" --trajectory "$TR/T2.yaml" --period 23 --duration "$DUR" \
       --log "$OUT/goals.txt" > "$OUT/goals_out.txt" 2>&1 ;;
  *) echo "unknown variant $V"; exit 1 ;;
esac
sleep 20
cleanup; wait
date +%s.%N > "$OUT/end.txt"
echo "done: $OUT"
