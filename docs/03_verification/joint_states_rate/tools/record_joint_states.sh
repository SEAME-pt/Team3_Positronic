#!/usr/bin/env bash
# Copyright 2026 Positronic. MIT license, see LICENSE at the repository root.
#
# Record /joint_states for the #23 acceptance run: a rosbag2 recording plus the output of
# ros2 topic hz, side by side, for 10 minutes and 30 s of margin.
#
# Run it inside the Jetson container while driver.launch.py is running:
#   bash docs/03_verification/joint_states_rate/tools/record_joint_states.sh [duration_s] [out_dir]
#
# Read-only: it only subscribes. The output goes under /ws, the only folder that outlives the
# container. Copy it to your laptop before the next make nano-sync, which deletes it.
#
# AI-generated (Claude, Anthropic).
# Prompt:
#   Write a bash script for the #23 acceptance run, to run in the Jetson container while the
#   driver is up. Write run_info.txt (date, host, update rate, hardware components, first
#   message), then record /joint_states with ros2 bag for 10 min 30 s and log ros2 topic hz
#   alongside. Close the bag with a clean SIGINT so metadata.yaml is written, never a SIGKILL:
#   run the recorder under job control (set -m) in its own process group, disable its keyboard
#   controls and take stdin from /dev/null so it is not stopped on the terminal, and wait for it
#   to finish writing. Fail within 10 s if the bag folder never appears, forward a Ctrl+C to the
#   recorder so an early stop still closes cleanly, warn if metadata.yaml is missing, and print
#   the check and copy commands at the end.
set -euo pipefail

TOPIC=/joint_states
DURATION_S="${1:-630}"
OUT_DIR="${2:-/ws/joint_states_runs/$(date +%Y%m%d_%H%M%S)}"

if ! ros2 topic list | grep -qx "${TOPIC}"; then
  echo "${TOPIC} is not published. Start the launch first:" >&2
  echo "  ros2 launch positronic_bringup driver.launch.py update_rate:=<#20 rate>" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"
{
  echo "date: $(date -Iseconds)"
  echo "host: $(hostname)"
  echo "duration_s: ${DURATION_S}"
  echo "update_rate: $(ros2 param get /controller_manager update_rate 2>&1)"
  echo "hardware components:"
  ros2 control list_hardware_components 2>&1 | sed 's/^/  /'
  echo "first message:"
  ros2 topic echo --once "${TOPIC}" 2>&1 | sed 's/^/  /'
} > "${OUT_DIR}/run_info.txt"

echo "Recording ${TOPIC} for ${DURATION_S} s into ${OUT_DIR} (Ctrl+C stops early)..."

# ros2 topic hz, for the rate log. A background job of a script starts with SIGINT ignored and
# Python keeps it ignored, so it is stopped with SIGTERM at the end.
ros2 topic hz "${TOPIC}" --window 500 > "${OUT_DIR}/topic_hz.log" 2>&1 &
HZ_PID=$!
trap 'kill -TERM "${HZ_PID}" 2> /dev/null || true' EXIT

# ros2 bag writes metadata.yaml only on a clean SIGINT; a SIGKILL (what the old timeout
# --kill-after sent) leaves a bag the checker cannot read. 'set -m' (job control) puts the
# recorder in its own process group with SIGINT at its default, so kill -INT reaches it the same
# way Ctrl+C does; without it a background job ignores SIGINT. We then wait for it to finish
# writing, however long that takes, instead of killing it.
#
# Own process group means background: if the recorder touched the terminal it would be stopped
# (SIGTTIN/SIGTTOU) before recording anything. Its keyboard controls (SPACE to pause) do that,
# so they are disabled and stdin comes from /dev/null.
set -m
ros2 bag record --output "${OUT_DIR}/bag" --topics "${TOPIC}" --disable-keyboard-controls \
  < /dev/null > "${OUT_DIR}/bag_record.log" 2>&1 &
BAG_PID=$!
set +m

stop_bag() {
  kill -INT "${BAG_PID}" 2> /dev/null || true
  wait "${BAG_PID}" 2> /dev/null || true
}

# Fail in seconds, not after 10 minutes: the recorder creates the bag folder as it starts.
for _ in $(seq 1 10); do
  [ -d "${OUT_DIR}/bag" ] && break
  sleep 1
done
if [ ! -d "${OUT_DIR}/bag" ]; then
  echo "ERROR: ros2 bag did not start recording within 10 s. Its log:" >&2
  cat "${OUT_DIR}/bag_record.log" >&2 || true
  kill -KILL "${BAG_PID}" 2> /dev/null || true
  exit 1
fi
echo "Bag started in ${OUT_DIR}/bag. Recording until $(date -d "+${DURATION_S} seconds" +%H:%M:%S)."

# A Ctrl+C on the script goes to the script's group, not the recorder's own group, so forward it
# and still close the bag cleanly before leaving.
trap 'echo; echo "Stopping early..."; stop_bag; kill -TERM "${HZ_PID}" 2> /dev/null || true; \
      exit 130' INT TERM

sleep "${DURATION_S}"
stop_bag

kill -TERM "${HZ_PID}" 2> /dev/null || true
wait "${HZ_PID}" 2> /dev/null || true
trap - INT TERM EXIT

if [ ! -f "${OUT_DIR}/bag/metadata.yaml" ]; then
  echo "WARNING: ${OUT_DIR}/bag/metadata.yaml is missing: the bag did not close cleanly." >&2
  echo "See ${OUT_DIR}/bag_record.log." >&2
fi

echo
echo "Done. Last ros2 topic hz lines:"
tail -n 5 "${OUT_DIR}/topic_hz.log" || true
echo
echo "Check it (nominal rate = the #20 value):"
echo "  python3 docs/03_verification/joint_states_rate/tools/check_joint_states.py ${OUT_DIR}/bag --rate <hz> \\"
echo "    --reference-deg '<pymycobot get_angles>' --report ${OUT_DIR}/report.md"
echo
echo "Then copy it to your laptop BEFORE any make nano-sync:"
echo "  scp -r nano:positronic/<you>/${OUT_DIR#/ws/} ."
