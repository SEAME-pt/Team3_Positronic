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
# A background job of a script starts with SIGINT ignored, and Python keeps it ignored, so
# ros2 topic hz is stopped with SIGTERM. PYTHONUNBUFFERED (set in the image) keeps its log whole.
ros2 topic hz "${TOPIC}" --window 500 > "${OUT_DIR}/topic_hz.log" 2>&1 &
HZ_PID=$!
trap 'kill -TERM "${HZ_PID}" 2> /dev/null || true' EXIT

# SIGINT lets ros2 bag close the recording cleanly; timeout exits 124 when time is up.
timeout --signal=INT --kill-after=10 "${DURATION_S}" \
  ros2 bag record --output "${OUT_DIR}/bag" "${TOPIC}" || true

kill -TERM "${HZ_PID}" 2> /dev/null || true
wait "${HZ_PID}" 2> /dev/null || true
trap - EXIT

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
