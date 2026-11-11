#!/usr/bin/env bash
# Copyright 2026 Positronic. MIT license, see LICENSE at the repository root.
#
# The three runs of the real-time profile (#39), inside the Jetson container, with
# driver.launch.py already running in another shell.
#
#   bash docs/03_verification/rt_profile/tools/run_rt_profile.sh idle    # 10 min, read-only
#   bash docs/03_verification/rt_profile/tools/run_rt_profile.sh load    # 2 min, CPU saturated
#   bash docs/03_verification/rt_profile/tools/run_rt_profile.sh steps   # 20 steps, MOVES THE ARM
#   bash docs/03_verification/rt_profile/tools/run_rt_profile.sh all     # the three, in order
#
# Output: $OUT/<run>/ with cycles.csv, run_info.txt, and for load also load.txt, for steps
# also steps.csv. OUT defaults to /ws/log/rt_<date>: /ws is the synced repo folder on the
# Jetson and log/ is excluded from make nano-sync, so the results outlive both the container
# and the next sync. Copy them to your laptop anyway, nothing backs the Jetson up.
#
# AI-generated (Claude, Anthropic).
# Prompt:
#   Write the bash runner for #39, for the Jetson container with the driver already up. Three
#   modes plus "all": a 10 minute read-only run; a 2 minute read-only run with every core
#   saturated, using stress-ng when it is installed and plain shell busy loops otherwise, and
#   writing evidence that the CPU really was busy (nproc, load average before and after, and
#   the busy percentage computed from /proc/stat over the run); and a run of 20 commanded
#   steps that moves one joint, which first checks that a controller owns the command
#   interfaces, prints a safety banner with a countdown, and stops the load and the steps
#   cleanly on Ctrl+C. Each run writes run_info.txt with the date, host, update rate and the
#   hardware components. Kill every background job on exit, never leave a busy loop behind,
#   and print the analyze_rt.py command at the end.
set -uo pipefail

RUN=${1:-all}
OUT=${OUT:-/ws/log/rt_$(date +%Y-%m-%d_%H%M)}
IDLE_DURATION=${IDLE_DURATION:-630}
LOAD_DURATION=${LOAD_DURATION:-120}
STEPS=${STEPS:-20}
STEP_INTERVAL=${STEP_INTERVAL:-3}
DELTA_RAD=${DELTA_RAD:-0.0873}  # 5 deg on the stepped joint
JOINT=${JOINT:-0}               # 0 is J1, at the base
COMPONENT=${COMPONENT:-positronic_arm}  # the <ros2_control> name in positronic_real.urdf.xacro
TOOLS=$(cd "$(dirname "$0")" && pwd)

load_pids=()
stop_load() {
  for pid in "${load_pids[@]:-}"; do
    [ -n "${pid}" ] && kill -TERM "${pid}" 2> /dev/null
  done
  load_pids=()
}
trap 'echo; echo "interrupted"; stop_load; exit 130' INT TERM
trap stop_load EXIT

if ! ros2 topic list 2> /dev/null | grep -qx /joint_states; then
  echo "/joint_states is not published. In another shell, start the driver first:" >&2
  echo "  ros2 launch positronic_bringup driver.launch.py" >&2
  exit 1
fi

# The busy percentage of all cores between two reads of /proc/stat.
cpu_busy_percent() {
  awk -v before="$1" -v after="$2" 'BEGIN {
    split(before, b); split(after, a)
    for (i = 1; i <= 8; i++) { total += a[i] - b[i] }
    idle = (a[4] - b[4]) + (a[5] - b[5])
    if (total > 0) { printf "%.1f", 100 * (total - idle) / total } else { print "unknown" }
  }'
}

cpu_fields() {
  awk '/^cpu /{ $1 = ""; print; exit }' /proc/stat
}

write_run_info() {
  {
    echo "run: $1"
    echo "date: $(date -Iseconds)"
    echo "host: $(hostname)"
    echo "cores: $(nproc)"
    echo "update_rate: $(ros2 param get /controller_manager update_rate 2>&1)"
    echo "hardware components:"
    ros2 control list_hardware_components 2>&1 | sed 's/^/  /'
    echo "controllers:"
    ros2 control list_controllers 2>&1 | sed 's/^/  /'
  } > "$2/run_info.txt"
}

record() {  # record <name> <duration> [extra rt_record.py arguments...]
  local name=$1 duration=$2
  shift 2
  local dir="${OUT}/${name}"
  mkdir -p "${dir}"
  write_run_info "${name}" "${dir}"
  echo "=== ${name}: ${duration} s into ${dir}"
  python3 "${TOOLS}/rt_record.py" --out "${dir}" --duration "${duration}" "$@" \
    2>&1 | tee "${dir}/record.log"
}

run_idle() {
  record idle "${IDLE_DURATION}"
}

run_load() {
  local dir="${OUT}/load"
  mkdir -p "${dir}"
  local cores before after
  cores=$(nproc)
  echo "=== load: saturating ${cores} cores for ${LOAD_DURATION} s"
  if command -v stress-ng > /dev/null; then
    stress-ng --cpu "${cores}" --timeout "$((LOAD_DURATION + 15))"s > "${dir}/stress.log" 2>&1 &
    load_pids+=("$!")
    echo "stress-ng --cpu ${cores}" > "${dir}/load_tool.txt"
  else
    # No stress-ng in the image: one busy shell per core does the same to the scheduler.
    for _ in $(seq 1 "${cores}"); do
      sh -c 'while :; do :; done' &
      load_pids+=("$!")
    done
    echo "${cores} shell busy loops (stress-ng not installed)" > "${dir}/load_tool.txt"
  fi
  sleep 5  # let the load settle before the measurement starts
  before=$(cpu_fields)
  { echo "load tool: $(cat "${dir}/load_tool.txt")"
    echo "cores: ${cores}"
    echo "loadavg at start: $(cat /proc/loadavg)"; } > "${dir}/load.txt"
  record load "${LOAD_DURATION}"
  after=$(cpu_fields)
  { echo "loadavg at end: $(cat /proc/loadavg)"
    echo "cpu busy during the run: $(cpu_busy_percent "${before}" "${after}") %"; } \
    >> "${dir}/load.txt"
  stop_load
  cat "${dir}/load.txt"
}

run_steps() {
  if ! ros2 control list_controllers 2> /dev/null | grep -q 'forward_position_controller.*active'
  then
    echo "forward_position_controller is not active. With the bench clear, spawn it:" >&2
    echo "  ros2 control set_controller_state arm_controller inactive  # launch spawns it active (#110)" >&2
    echo "  ros2 run controller_manager spawner forward_position_controller" >&2
    echo "The arm must also be in fresh mode 1, set before the driver took the port." >&2
    exit 1
  fi
  # Since #36 write() returns at once while the hardware component is inactive, so the steps
  # would be recorded, nothing would move, and the latency would come out as "no usable
  # sample" with no reason given. Check it here instead.
  if ! ros2 control list_hardware_components 2> /dev/null \
    | grep -A5 "name: ${COMPONENT}" | grep -q 'label=active'
  then
    echo "the ${COMPONENT} hardware component is not active: write() sends nothing (#36)." >&2
    echo "  ros2 control set_hardware_component_state ${COMPONENT} active" >&2
    exit 1
  fi
  echo
  echo "!!! THE ARM WILL MOVE: joint index ${JOINT} by ${DELTA_RAD} rad, ${STEPS} times."
  echo "!!! Bench clear? Ctrl+C now to stop."
  for second in 5 4 3 2 1; do
    printf '\r    starting in %s s ' "${second}"
    sleep 1
  done
  echo
  record steps "$((STEPS * STEP_INTERVAL + 10))" --steps "${STEPS}" --joint "${JOINT}" \
    --delta-rad "${DELTA_RAD}" --step-interval "${STEP_INTERVAL}"
}

mkdir -p "${OUT}"
case "${RUN}" in
  idle) run_idle ;;
  load) run_load ;;
  steps) run_steps ;;
  all) run_idle; run_load; run_steps ;;
  *) echo "usage: $0 [idle|load|steps|all]" >&2; exit 2 ;;
esac

echo
echo "Results in ${OUT}. Analyse them (on this machine or on your laptop):"
echo "  python3 docs/03_verification/rt_profile/tools/analyze_rt.py \\"
echo "    ${OUT}/idle/cycles.csv ${OUT}/load/cycles.csv ${OUT}/steps/cycles.csv \\"
echo "    --rate <update_rate> --figure-dir docs/03_verification/rt_profile/figures"
echo
echo "Copy it to your laptop (make nano-sync excludes log/, so it survives a sync):"
echo "  scp -r nano:positronic/<you>/${OUT#/ws/} ."
