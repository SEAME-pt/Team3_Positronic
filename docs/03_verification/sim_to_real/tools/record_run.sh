#!/usr/bin/env bash
# Copyright 2026 Positronic. MIT license, see LICENSE at the repository root.
#
# Record one sim-to-real run for #37: one command, one run, one folder.
#   bash docs/03_verification/sim_to_real/tools/record_run.sh sim 1    # -> runs/sim_1/
#   bash docs/03_verification/sim_to_real/tools/record_run.sh real 1   # -> runs/real_1/
#
# It records /joint_states and /arm_controller/controller_state while run_trajectory.py sends
# trajectory.yaml, then closes the bag. A run whose sender failed is renamed <world>_<n>_FAILED
# so compare_runs.py never reads it by mistake. Set RUNS_DIR to write somewhere else (on the
# Jetson: under /ws, the only folder that outlives the container).
#
# "real" MOVES THE ARM. Follow the safety checklist on the Sprint 02 page first.
#
# AI-generated (Claude, Anthropic).
# Prompt:
#   Write record_run.sh for #37: "record_run.sh <sim|real> <n>" makes runs/<world>_<n>/. Refuse
#   to start if ROS_DOMAIN_ID does not match the world (18 sim, 17 real), if the folder exists,
#   or if the topics are not published. Write run_info.txt for the report's setup table. Start
#   ros2 bag record in the background the way record_joint_states.sh does (set -m, SIGINT, wait),
#   wait 1 s, run run_trajectory.py, wait 1 s, stop the recorder cleanly. Forward Ctrl+C so the
#   bag still closes, exit with the sender's code, and rename a failed run to _FAILED.
#   Then: wait up to 30 s for a message on each topic instead of only checking that the topic
#   exists, so a Gazebo that is still loading does not fail the run.
set -euo pipefail

TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${TOOLS_DIR}/../../../.." && pwd)"
TOPICS=(/joint_states /arm_controller/controller_state)

usage() {
  echo "Usage: $0 <sim|real> <run number>   e.g. $0 sim 1" >&2
  exit 1
}

[ $# -eq 2 ] || usage
WORLD="$1"
RUN="$2"
case "${WORLD}" in
  sim) EXPECTED_DOMAIN=18 ;;
  real) EXPECTED_DOMAIN=17 ;;
  *) usage ;;
esac
[[ "${RUN}" =~ ^[0-9]+$ ]] || usage

# The script cannot tell Gazebo from the real arm: both offer the same action. ROS_DOMAIN_ID
# is what picks the world, so a sim run must never start on the real arm's domain.
if [ "${ROS_DOMAIN_ID:-}" != "${EXPECTED_DOMAIN}" ]; then
  echo "ROS_DOMAIN_ID is '${ROS_DOMAIN_ID:-unset}', but ${WORLD} uses ${EXPECTED_DOMAIN}." >&2
  echo "  export ROS_DOMAIN_ID=${EXPECTED_DOMAIN}" >&2
  exit 1
fi

OUT_DIR="${RUNS_DIR:-${REPO_DIR}/runs}/${WORLD}_${RUN}"
if [ -e "${OUT_DIR}" ] || [ -e "${OUT_DIR}_FAILED" ]; then
  echo "${OUT_DIR} already exists: pick another run number." >&2
  exit 1
fi

# Refuse a bad trajectory before anything starts, so no empty run folder is left behind.
python3 "${TOOLS_DIR}/run_trajectory.py" --check-only

# Wait for one real message on each topic, not only for the topic to exist: a topic shows up as
# soon as a node announces it, while Gazebo can still be loading and sending nothing (sim_1, the
# sender found no /joint_states within its 5 s). Goes on as soon as a message arrives; gives up
# after DATA_TIMEOUT_S. Short tries in a loop, because ros2 topic echo can exit at once when the
# topic does not exist yet instead of waiting for it.
DATA_TIMEOUT_S=30
for topic in "${TOPICS[@]}"; do
  deadline=$((SECONDS + DATA_TIMEOUT_S))
  until timeout 5 ros2 topic echo --once "${topic}" > /dev/null 2>&1; do
    if [ "${SECONDS}" -ge "${deadline}" ]; then
      echo "No message on ${topic} for ${DATA_TIMEOUT_S} s: is the ${WORLD} launch running" \
        "with arm_controller active?" >&2
      exit 1
    fi
    echo "Waiting for data on ${topic}..." >&2
    sleep 1
  done
done

mkdir -p "${OUT_DIR}"
{
  echo "date: $(date -Iseconds)"
  echo "world: ${WORLD}"
  echo "run: ${RUN}"
  echo "ros_domain_id: ${ROS_DOMAIN_ID}"
  echo "host: $(hostname)"
  echo "commit: $(git -C "${REPO_DIR}" rev-parse --short HEAD 2> /dev/null || echo unknown)"
  echo "trajectory_sha256: $(sha256sum "${TOOLS_DIR}/trajectory.yaml" | cut -d' ' -f1)"
  echo "update_rate: $(ros2 param get /controller_manager update_rate 2>&1)"
} > "${OUT_DIR}/run_info.txt"

# Same recorder setup as joint_states_rate/tools/record_joint_states.sh, which explains it:
# own process group (set -m) so kill -INT reaches it, no keyboard controls and stdin from
# /dev/null so it is not stopped on the terminal, and SIGINT (never SIGKILL) so metadata.yaml
# is written.
set -m
ros2 bag record --output "${OUT_DIR}/bag" --topics "${TOPICS[@]}" --disable-keyboard-controls \
  < /dev/null > "${OUT_DIR}/bag_record.log" 2>&1 &
BAG_PID=$!
set +m

stop_bag() {
  kill -INT "${BAG_PID}" 2> /dev/null || true
  wait "${BAG_PID}" 2> /dev/null || true
}

# A Ctrl+C reaches run_trajectory.py (it cancels the goal) and this script, not the recorder's
# own group: forward it, so the bag still closes, and keep the run marked as failed.
trap 'echo; echo "Stopping..."; stop_bag; mv "${OUT_DIR}" "${OUT_DIR}_FAILED"; exit 130' INT TERM HUP

for _ in $(seq 1 10); do
  [ -d "${OUT_DIR}/bag" ] && break
  sleep 1
done
if [ ! -d "${OUT_DIR}/bag" ]; then
  echo "ERROR: ros2 bag did not start recording within 10 s. Its log:" >&2
  cat "${OUT_DIR}/bag_record.log" >&2 || true
  kill -KILL "${BAG_PID}" 2> /dev/null || true
  mv "${OUT_DIR}" "${OUT_DIR}_FAILED"
  exit 1
fi

sleep 1  # the bag has the arm still before the motion
# With pipefail the pipe's code is the sender's. tee -i ignores Ctrl+C, so the sender can still
# print while it cancels the goal.
SENDER_EXIT=0
python3 "${TOOLS_DIR}/run_trajectory.py" 2>&1 | tee -i "${OUT_DIR}/sender.log" \
  || SENDER_EXIT=$?
sleep 1  # and the last messages after it
stop_bag
trap - INT TERM HUP

echo "sender_exit: ${SENDER_EXIT}" >> "${OUT_DIR}/run_info.txt"
if [ ! -f "${OUT_DIR}/bag/metadata.yaml" ]; then
  echo "WARNING: ${OUT_DIR}/bag/metadata.yaml is missing: the bag did not close cleanly." >&2
  SENDER_EXIT=1
fi

if [ "${SENDER_EXIT}" -ne 0 ]; then
  mv "${OUT_DIR}" "${OUT_DIR}_FAILED"
  echo "Run FAILED (exit ${SENDER_EXIT}): kept as ${OUT_DIR}_FAILED, see sender.log there." >&2
  exit 1
fi
echo "Run OK: ${OUT_DIR}"
