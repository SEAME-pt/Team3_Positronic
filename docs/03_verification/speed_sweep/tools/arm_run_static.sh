#!/bin/bash
# AI-generated (Claude, Anthropic).
# Prompt:
#   #133 spike, approach "adaptive": the arm run, one person at the keyboard, one hand on the
#   switch. Per mode (fixed or adaptive byte at 30, 50, 70, never above 70): the tap of every
#   byte on the serial link, the driver with that mode, a bag, the 5 deg first move, the #33
#   base and wrist trajectories 3 times each, 5 stops at 15 deg/s, three 5 deg steps. Every step
#   that moves the arm says so and counts down first. Reuse the #32 tap, the #33 trajectories and
#   waypoint_error.py, the #39 steps run.
# Follow-up prompt:
#   One person alone: he types and keeps a hand on the switch. One terminal (the driver runs in
#   the background), one command per mode (`run`) that waits for Enter between stages, a longer
#   countdown, and the checks a second person would have done (fresh mode, the send speed the
#   driver logged, the by-name rt_record) made by the script, which stops on a wrong one.
# Follow-up prompt (arm day, 24 Nov):
#   With camera + gripper the wrist servos (J4, J5) heat ~1.5 degC/min just holding the pose and
#   reached the driver's 55 degC activation gate in the middle of the stops. Refuse a mode that
#   starts above 45 degC, do the stops right after the first move, and end the mode if the
#   resume after a stop fails.
# Follow-up prompt (Felipe, 24 Nov):
#   The servos heat because the tip is left tilted after every move; upright (all zeros) they
#   don't. Between tests the arm goes to a standing pose where gravity does not load the servos.
# Follow-up prompt (approach "static", 24 Nov):
#   The same run for the static driver (send_speed fixed at launch, fresh mode 1 sent by the
#   driver before every power_on) over Felipe's full range: bytes 5, 10, 30, 50, 70, 90, 100,
#   in three tip setups. Lift the 70 cap safely: high bytes step up only, each only after the
#   stop distance at the byte before it was measured in the same setup (step_up_check.py), a
#   byte above 70 is typed twice, and the rest pose's J4 is given for every setup but the first.
#
# Run INSIDE the positronic-arm container on the Jetson (make nano-shell), from /ws, ONE
# terminal: `up` leaves the tap, the driver and the bag running in the background. Nothing here
# touches Wi-Fi.
#
#   bash $T/arm_run_static.sh run MODE [STAGES]   one mode in one go: up, then each stage
#                                            after Enter (s skips it, q ends the mode), then
#                                            down. STAGES default: start first stops base steps
#   bash $T/arm_run_static.sh cool           no driver up: wait (arm holding still) until every
#                                            servo is <= MAX_START_C (45); `run` does it first
#   bash $T/arm_run_static.sh up MODE        tap + driver + bag for MODE (no motion; the
#                                            servos are powered on and hold their pose)
#   bash $T/arm_run_static.sh start          MOVES: REST, upright, J4 where the tip balances
#   bash $T/arm_run_static.sh first          MOVES: T1, J1 5 deg out and back at 1.7 deg/s
#   bash $T/arm_run_static.sh base           MOVES: T3, T6, T4 (J1+J2, 10/15/20 deg/s), 3 each
#   bash $T/arm_run_static.sh wrist          MOVES: T11, T12, T13 (J3-J6, 10/15/17.5), 3 each
#   bash $T/arm_run_static.sh stops          MOVES: 5 x J1 at 15 deg/s, stopped mid-way
#   bash $T/arm_run_static.sh steps          MOVES: 6 x J1 5 deg step and back (#39 steps run)
#   bash $T/arm_run_static.sh down           bag, driver, tap stopped; summary printed
#   bash $T/arm_run_static.sh state          component, controllers, last summary (no motion)
#
#   T=docs/03_verification/speed_sweep/tools
#   DRY=1 runs the same flow on the fake backend, without pymycobot and the tap: the desk check
#   of the script (no arm, no serial port).
#   MODE: fixed5 fixed10 fixed30 fixed50 fixed70 fixed90 fixed100, in that order (step up only).
#   SETUP=setup1|setup2|setup3 (required by `up`): the tip, and the folder log/133s/SETUP/MODE.
#   Above 30 a byte starts only after the 5 stops of the byte before it (30 -> 50 -> 70 -> 90 ->
#   100) in the same setup, every travel <= STOP_MAX_DEG (4); above 70 also CONFIRM_BYTE=<byte>.
#   REST_J4_DEG: required for setup2 and setup3 (measure it per tip), 39.8 for setup1.
#   Each mode starts with `first` (stops, base, wrist, steps refuse before it), and `stops` is
#   done before the next speed is tried.

set -m
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

ROOT=/ws/log/133s
TOOLS=/ws/docs/03_verification/speed_sweep/tools
TRAJ=/ws/docs/03_verification/jtc_on_the_arm/trajectories
WAYPOINT=/ws/docs/03_verification/jtc_on_the_arm/tools/waypoint_error.py
TAP=/ws/docs/03_verification/write_sweep/tools/proxy_tap.py
RT=/ws/docs/03_verification/rt_profile/tools/run_rt_profile.sh
# After the goal is accepted: the leg starts at 2 s, so 3.0 puts the stop 1 s (15 deg) into the
# 2.67 s leg at 15 deg/s. The trigger is a node already connected to the service, a few ms; the
# ros2 CLI took 0.8 to 2.3 s on the desk, too loose to aim with. The tap has the true moment.
STOP_AFTER_S=${STOP_AFTER_S:-3.0}
WAIT_BETWEEN_S=${WAIT_BETWEEN_S:-4} # the arm stands still before the next goal
COUNTDOWN_S=${COUNTDOWN_S:-5}       # one person: time to move the hand from the keys to the switch
# The rest pose's J4: where the tip balances with J4 released, measured per tip setup. Camera +
# gripper: +39.8 deg (24 Nov; at 0 J4 still held the camera and stayed at 56 degC, at 39.8 it went
# 45 -> 39 degC in 5 min). The other joints rest at 0. Another tip moves the centre of mass, so
# setup2 and setup3 must say it (`up` refuses without); setup1 defaults to 39.8.
if [ -z "${REST_J4_DEG:-}" ] && [ "${SETUP:-}" = setup1 ]; then REST_J4_DEG=39.8; fi
STOP_MAX_DEG=${STOP_MAX_DEG:-4} # step up: every stop at the byte before travelled at most this
mkdir -p "$ROOT"

countdown() {
  echo ">>> $1"
  for s in $(seq "$COUNTDOWN_S" -1 1); do echo "    $s ... hand on the switch"; sleep 1; done
}

current_mode() { cat "$ROOT/current_mode" 2> /dev/null; }
mode_dir() { echo "$ROOT/$(current_mode)"; }

need_mode() {
  if [ -z "$(current_mode)" ]; then
    echo "no mode up: bash $0 up MODE" >&2
    exit 1
  fi
}

# Every byte starts with the 5 deg first move (#133): no other motion stage before T1 succeeded.
need_first() {
  if ! grep -q "status: SUCCEEDED" "$(mode_dir)/goal_T1_r1.txt" 2> /dev/null; then
    echo "!!! no successful first move (T1) in $(current_mode): bash $0 first" >&2
    exit 1
  fi
}

# The launch arguments of a mode: Felipe's range (24 Nov), nothing else. Which byte may start
# now is step_up_check.py's call, in `up`.
mode_args() {
  case "$1" in
    fixed5 | fixed10 | fixed30 | fixed50 | fixed70 | fixed90 | fixed100) echo "send_speed:=${1#fixed}" ;;
    *) return 1 ;;
  esac
}

# What the driver's INFO line must say for a mode.
expected_speed_line() { echo "send speed ${1#fixed}\."; }

# T6, T4 (from T3) and T12, T13 (from T11): the same goals with other times (#33 README).
make_trajectories() {
  local out="$ROOT/traj"
  mkdir -p "$out"
  python3 - "$TRAJ" "$out" "$REST_J4_DEG" << 'PY'
import sys, re, math
src, out, rest_j4_deg = sys.argv[1], sys.argv[2], float(sys.argv[3])
def retime(name, base, seg):
    text = open(f'{src}/{base}.yaml').read()
    times = iter([2.0, 2.0 + seg, 2.0 + 2 * seg])
    def sub(_):
        t = next(times)
        return f'time_from_start: {{sec: {int(t)}, nanosec: {int(round((t - int(t)) * 1e9))}}}'
    text = re.sub(r'time_from_start: \{[^}]*\}', sub, text)
    open(f'{out}/{name}.yaml', 'w').write(text)
for name, base, seg in (('T3', 'T3', 2.0), ('T6', 'T3', 20 / 15), ('T4', 'T3', 1.0),
                        ('T11', 'T11', 2.0), ('T12', 'T11', 20 / 15), ('T13', 'T11', 20 / 17.5)):
    retime(name, base, seg)
# TS: J1 40 deg out at 15 deg/s (2.67 s) and back, for the stops. Start pose of the #33 files.
start = [0.2822, 0.0061, -0.0106, -0.2330, 0.0075, -0.0106]
out_pose = list(start); out_pose[0] += math.radians(40)
names = ('[joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, '
         'joint6_to_joint5, joint6output_to_joint6]')
def pt(p, t):
    return (f'  - positions: [{", ".join(f"{v:.4f}" for v in p)}]\n'
            f'    time_from_start: {{sec: {int(t)}, nanosec: {int(round((t - int(t)) * 1e9))}}}\n')
open(f'{out}/TS.yaml', 'w').write(
    f'trajectory:\n  joint_names: {names}\n  points:\n' + pt(start, 2.0)
    + pt(out_pose, 2.0 + 40 / 15) + pt(start, 2.0 + 80 / 15))
# REST: upright, J4 where the tip balances, in 6 s like T0_to_start: gravity does not load the
# wrist there. Never past 60 deg: a typo must not swing the tip around.
assert abs(rest_j4_deg) <= 60, rest_j4_deg
rest = [0.0, 0.0, 0.0, math.radians(rest_j4_deg), 0.0, 0.0]
open(f'{out}/REST.yaml', 'w').write(
    f'trajectory:\n  joint_names: {names}\n  points:\n' + pt(rest, 6.0))
PY
}

send() {  # send <file> <out>: one goal, its feedback kept in <out>; fails unless SUCCEEDED
  ros2 action send_goal --feedback /arm_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "$(cat "$1")" > "$2" 2>&1
  tail -1 "$2"
  grep -q "status: SUCCEEDED" "$2"
}

# Felipe (24 Nov): tilted, the wrist holds the tip against gravity and heats (J4/J5 42 -> 59 degC
# in 11 min, camera + gripper); upright it does not. The arm waits upright (REST, all zeros) and
# every goal is rest -> start pose (T0_to_start, 6 s) -> goal -> rest. The files are move_*.txt,
# so waypoint_error.py (goal_T*_r[123].txt) sees only the goals.
to_start() { send "$TRAJ/T0_to_start.yaml" "$(mode_dir)/move_start_$1.txt"; }
to_rest() { send "$ROOT/traj/REST.yaml" "$(mode_dir)/move_rest_$1.txt"; }

# The last servo temperatures in the tap (the driver's get_servo_temps replies, every few s).
tap_temps() {
  grep -o "fefe08e5[0-9a-f]\{12\}" "$(mode_dir)/tap.csv" 2> /dev/null | tail -1 \
    | python3 -c "import sys; h=sys.stdin.read().strip()[8:]; print(*[int(h[i:i+2], 16) for i in range(0, 12, 2)])"
}

# Before a goal, at rest: if a servo is at HOT_C or more, wait there until all are at COOL_C or
# less. One goal is ~14 s away from the rest pose, so it cannot start hot enough to reach the
# driver's 60 degC power-off. No tap (DRY): no wait.
cool_wait() {
  [ -f "$(mode_dir)/tap.csv" ] || return 0
  local temps max
  temps=$(tap_temps)
  [ -n "$temps" ] || return 0
  max=$(echo "$temps" | tr ' ' '\n' | sort -n | tail -1)
  [ "$max" -ge "${HOT_C:-54}" ] || return 0
  echo ">>> servos $temps degC: waiting at rest until all <= ${COOL_C:-48}" | tee -a "$(mode_dir)/cool_waits.txt"
  local start=$SECONDS
  while [ "$max" -gt "${COOL_C:-48}" ]; do
    sleep 10
    temps=$(tap_temps)
    max=$(echo "$temps" | tr ' ' '\n' | sort -n | tail -1)
  done
  echo ">>> servos $temps degC after $((SECONDS - start)) s" | tee -a "$(mode_dir)/cool_waits.txt"
}

goal() {  # goal <name> <run>: one #33 goal from the rest pose, recorded for waypoint_error.py
  local name=$1 run=$2 file="$ROOT/traj/$1.yaml"
  [ -f "$file" ] || file="$TRAJ/$1.yaml"
  to_start "${name}_r${run}" || return 1
  # A JTC abort is a result (#33), not a reason to stop: no check on the goal itself.
  send "$file" "$(mode_dir)/goal_${name}_r${run}.txt"
  to_rest "${name}_r${run}" || return 1
  sleep "$WAIT_BETWEEN_S"
}

case "$1" in
  up)
    MODE=$2
    ARGS=$(mode_args "$MODE") || { echo "MODE: fixed5 fixed10 fixed30 fixed50 fixed70 fixed90 fixed100" >&2; exit 2; }
    [ -n "$(current_mode)" ] && { echo "mode $(current_mode) is still up: bash $0 down" >&2; exit 1; }
    case "${SETUP:-}" in
      setup1 | setup2 | setup3) ;;
      *) echo "SETUP=setup1 (camera + gripper), setup2 (gripper) or setup3 (bare)" >&2; exit 2 ;;
    esac
    [ -n "${REST_J4_DEG:-}" ] || { echo "REST_J4_DEG=<deg> for $SETUP: measure where the tip balances" >&2; exit 2; }
    BYTE=${MODE#fixed}
    # Step up only (Felipe, 24 Nov): above 30, the byte before must have its 5 stops here.
    DRY_FLAG=""
    [ "${DRY:-0}" = 1 ] && DRY_FLAG=--dry
    # shellcheck disable=SC2086
    STEP_UP=$(python3 "$TOOLS/step_up_check.py" "$ROOT/$SETUP" "$BYTE" \
      --max-travel-deg "$STOP_MAX_DEG" $DRY_FLAG 2>&1) || { echo "!!! $STEP_UP" >&2; exit 1; }
    echo "$STEP_UP"
    # Above 70 the byte is typed twice: in MODE and in CONFIRM_BYTE.
    if [ "$BYTE" -gt 70 ] && [ "${CONFIRM_BYTE:-}" != "$BYTE" ]; then
      echo "!!! byte $BYTE is above 70: CONFIRM_BYTE=$BYTE to start it (hand on the switch)" >&2
      exit 1
    fi
    DIR="$ROOT/$SETUP/$MODE"
    [ -e "$DIR" ] && DIR="$DIR-$(date +%H%M%S)"
    mkdir -p "$DIR"
    echo "$STEP_UP" > "$DIR/step_up.txt"
    echo "$SETUP/$(basename "$DIR")" > "$ROOT/current_mode"
    make_trajectories || { rm -f "$ROOT/current_mode"; exit 1; }
    echo "$SETUP, rest pose J4 $REST_J4_DEG deg, others 0" > "$DIR/rest.txt"
    if [ "${DRY:-0}" = 1 ]; then
      echo "driver: DRY, fake backend $ARGS" | tee "$DIR/args.txt"
      # shellcheck disable=SC2086
      ros2 launch positronic_bringup driver.launch.py backend:=fake $ARGS > "$DIR/launch.log" 2>&1 &
      echo $! > "$ROOT/launch.pid"
      sleep 15
      grep -m1 "send speed" "$DIR/launch.log" | grep -q "$(expected_speed_line "$MODE")" \
        || { echo "!!! wrong send speed in $DIR/launch.log" >&2; bash "$0" down > /dev/null 2>&1; exit 1; }
      ros2 bag record -o "$DIR/bag" /joint_states /dynamic_joint_states \
        /arm_controller/controller_state /forward_position_controller/commands > "$DIR/bag.log" 2>&1 &
      echo $! > "$ROOT/bag.pid"
      timeout 5 ros2 control list_controllers
      exit 0
    fi
    # Fresh mode 1 before the driver takes the port (#20, sweep.sh); power on holds the pose.
    python3 - << 'PY' 2>&1 | tee "$DIR/fresh_mode.txt"
from pymycobot import MyCobot280
arm = MyCobot280('/dev/ttyTHS1', 1000000)
arm.power_on()
arm.set_fresh_mode(1)
print('fresh mode read-back:', arm.get_fresh_mode(), ' angles:', arm.get_angles())
print('servo temps degC:', arm.get_servo_temps())
PY
    # The driver will not re-activate after a stop above 55 degC (#115); a mode is ~9 min powered
    # and the wrist gains ~1.5 degC/min with camera + gripper: start at 45 at most.
    if ! python3 - "$DIR/fresh_mode.txt" "${MAX_START_C:-45}" << 'PY'
import ast, re, sys
m = re.search(r'servo temps degC: (\[[^]]*\])', open(sys.argv[1]).read())
temps = ast.literal_eval(m.group(1)) if m else None
if not temps or len(temps) != 6 or max(temps) > float(sys.argv[2]):
    sys.exit(f'!!! servo temps {temps}: need all <= {sys.argv[2]} degC to start a mode, let it cool')
PY
    then
      rm -f "$ROOT/current_mode"
      exit 1
    fi
    rm -f /tmp/proxy_port
    python3 "$TAP" "$DIR/tap.csv" > "$DIR/tap.log" 2>&1 &
    echo $! > "$ROOT/tap.pid"
    for _ in $(seq 50); do [ -s /tmp/proxy_port ] && break; sleep 0.1; done
    [ -s /tmp/proxy_port ] || { echo "proxy_tap did not start, see $DIR/tap.log" >&2; exit 1; }
    echo "driver: port $(cat /tmp/proxy_port) $ARGS" | tee "$DIR/args.txt"
    # shellcheck disable=SC2086
    ros2 launch positronic_bringup driver.launch.py port:="$(cat /tmp/proxy_port)" $ARGS \
      > "$DIR/launch.log" 2>&1 &
    echo $! > "$ROOT/launch.pid"
    sleep 15
    # The checks a second person would make, made here: the driver runs the mode asked for.
    if ! grep -q "fresh mode read-back: 1 " "$DIR/fresh_mode.txt"; then
      echo "!!! fresh mode is not 1 (see $DIR/fresh_mode.txt): stopping this mode" >&2
      bash "$0" down > /dev/null 2>&1
      exit 1
    fi
    if ! grep -m1 "send speed" "$DIR/launch.log" | grep -q "$(expected_speed_line "$MODE")"; then
      echo "!!! the driver did not log '$(expected_speed_line "$MODE")' (see $DIR/launch.log): stopping" >&2
      bash "$0" down > /dev/null 2>&1
      exit 1
    fi
    # Static's step zero: the driver itself sends fresh mode 1 (FE FE 03 16 01 FA) before its
    # power_on. The INFO line says how many the Atom acknowledged (the open question).
    grep -m1 -o "Fresh mode 1 before power_on: .*" "$DIR/launch.log" | tee -a "$DIR/fresh_mode.txt"
    if ! grep -q "Fresh mode 1 before power_on: [1-9]" "$DIR/launch.log" \
      || ! grep -q "fefe031601fa" "$DIR/tap.csv"; then
      echo "!!! the driver did not send fresh mode 1 before power_on (launch.log, tap.csv): stopping" >&2
      bash "$0" down > /dev/null 2>&1
      exit 1
    fi
    grep -m1 -o "send speed[^.]*\.[0-9]*[^.]*" "$DIR/launch.log"
    ros2 bag record -o "$DIR/bag" /joint_states /dynamic_joint_states \
      /arm_controller/controller_state /forward_position_controller/commands > "$DIR/bag.log" 2>&1 &
    echo $! > "$ROOT/bag.pid"
    timeout 5 ros2 control list_controllers
    ;;
  start)
    need_mode
    countdown "MOVES: to the rest pose, upright, $(cut -d, -f2 "$(mode_dir)/rest.txt" | xargs), 6 s"
    to_rest start || exit 1
    ;;
  first)
    need_mode
    cool_wait
    countdown "MOVES: start pose, T1 (J1 5 deg out and back, 1.7 deg/s, the first move of this mode), rest"
    goal T1 1 || exit 1
    ;;
  base)
    need_mode
    need_first
    for name in T3 T6 T4; do
      for run in 1 2 3; do
        cool_wait
        countdown "MOVES: start pose, $name run $run (J1 and J2 20 deg out and back), rest"
        goal "$name" "$run" || exit 1
      done
    done
    ;;
  wrist)
    need_mode
    need_first
    for name in T11 T12 T13; do
      for run in 1 2 3; do
        cool_wait
        countdown "MOVES: start pose, $name run $run (J3 to J6 20 deg out and back, raises J3: nothing above), rest"
        goal "$name" "$run" || exit 1
      done
    done
    ;;
  stops)
    need_mode
    need_first
    for run in ${STOP_RUNS:-1 2 3 4 5}; do  # STOP_RUNS="5": redo one stop only
      cool_wait
      countdown "MOVES: start pose, then stop $run of 5: J1 40 deg at 15 deg/s, stopped about 1 s in"
      to_start "stop$run" || exit 1
      rm -f "$ROOT/trigger_ready" "$ROOT/trigger_go"
      python3 - "$ROOT" "$(mode_dir)/stop_r${run}_trigger.txt" << 'PY' &
import os, sys, time
import rclpy
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State
root, out = sys.argv[1], sys.argv[2]
rclpy.init()
node = rclpy.create_node('stop_trigger_133')
client = node.create_client(SetHardwareComponentState,
                            '/controller_manager/set_hardware_component_state')
client.wait_for_service()
open(f'{root}/trigger_ready', 'w').close()
while not os.path.exists(f'{root}/trigger_go'):
    time.sleep(0.002)
sent = time.time()
future = client.call_async(SetHardwareComponentState.Request(
    name='positronic_arm', target_state=State(id=State.PRIMARY_STATE_INACTIVE, label='inactive')))
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
ok = future.result().ok if future.result() else None
open(out, 'w').write(f'{sent:.6f} {time.time():.6f} ok={ok}\n')
print(f'stop requested, answered after {(time.time() - sent) * 1000:.0f} ms, ok={ok}')
PY
      TRIGGER=$!
      for _ in $(seq 200); do [ -f "$ROOT/trigger_ready" ] && break; sleep 0.05; done
      ros2 action send_goal /arm_controller/follow_joint_trajectory \
        control_msgs/action/FollowJointTrajectory "$(cat "$ROOT/traj/TS.yaml")" \
        > "$(mode_dir)/stop_r${run}_goal.txt" 2>&1 &
      GOAL=$!
      for _ in $(seq 100); do
        grep -q "Goal accepted" "$(mode_dir)/stop_r${run}_goal.txt" 2> /dev/null && break
        sleep 0.05
      done
      sleep "$STOP_AFTER_S"
      # The supervisor's halt order (#45): hardware inactive (the driver sends the stop),
      # then the goal, then the controller.
      touch "$ROOT/trigger_go"
      wait "$TRIGGER"
      kill -INT "$GOAL" 2> /dev/null
      ros2 control set_controller_state arm_controller inactive
      sleep 2
      echo ">>> resume: hardware active (holds), controller active (holds)"
      ros2 control set_hardware_component_state positronic_arm active
      ros2 control set_controller_state arm_controller active
      sleep 1
      # The ros2 CLI takes several seconds here: 20 s, 3 tries (5 s gave a false failure).
      resumed=""
      for _ in 1 2 3; do
        timeout 20 ros2 control list_controllers 2> /dev/null | grep -E "^arm_controller .* active" && { resumed=1; break; }
      done
      if [ -z "$resumed" ]; then
        echo "!!! resume after stop $run failed (temperature gate? see launch.log): ending the stops" >&2
        exit 1
      fi
      countdown "MOVES: back to the start pose, then rest"
      to_start "stop${run}_back" || exit 1
      to_rest "stop$run" || exit 1
    done
    ;;
  steps)
    need_mode
    need_first
    # #171: the deployed rt_record.py must read the joints by name, or step 1 moves five joints.
    if ! grep -q "ARM_JOINTS" /ws/docs/03_verification/rt_profile/tools/rt_record.py; then
      echo "!!! rt_record.py reads /joint_states by index (#171): steps refused" >&2
      exit 1
    fi
    countdown "MOVES: controllers switched to forward_position_controller (no motion yet)"
    ros2 control set_controller_state arm_controller inactive
    ros2 run controller_manager spawner forward_position_controller --param-file \
      /ws/install/positronic_bringup/share/positronic_bringup/config/jetson_controllers.yaml
    # From the rest pose (all zeros), where the previous stage left the arm.
    countdown "MOVES: J1 5 deg step and back, 3 times, 3 s apart, from the rest pose"
    # run_rt_profile.sh checks the controllers with the ros2 CLI, which sometimes answers empty
    # on the Jetson right after the switch; a refused check moves nothing, so try up to 3 times.
    for attempt in 1 2 3; do
      STEPS=6 RUN=steps OUT="$(mode_dir)/rt" bash "$RT" steps 2>&1 | tee -a "$(mode_dir)/steps.log"
      [ -s "$(mode_dir)/rt/steps/steps.csv" ] && break
      echo ">>> steps attempt $attempt refused by a check, trying again" | tee -a "$(mode_dir)/steps.log"
      sleep 3
    done
    ros2 control set_controller_state forward_position_controller inactive
    ros2 control set_controller_state arm_controller active
    # run_rt_profile.sh refuses on a failed check (stderr) without moving: say so here.
    [ -s "$(mode_dir)/rt/steps/steps.csv" ] || { echo "!!! no steps recorded, see $(mode_dir)/steps.log" >&2; exit 1; }
    ;;
  down)
    need_mode
    DIR=$(mode_dir)
    for p in bag launch tap; do
      [ -f "$ROOT/$p.pid" ] && kill -INT "$(cat "$ROOT/$p.pid")" 2> /dev/null
      sleep 3
      rm -f "$ROOT/$p.pid"
    done
    pkill -INT -f ros2_control_node 2> /dev/null
    rm -f "$ROOT/current_mode"
    {
      [ -f "$DIR/tap.csv" ] && python3 "$TOOLS/tap_speed.py" "$DIR/tap.csv" --csv "$DIR/tap_rows.csv"
      ls "$DIR"/goal_T*_r[123].txt > /dev/null 2>&1 && python3 "$WAYPOINT" "$DIR"/goal_T*_r[123].txt
    } 2>&1 | tee "$DIR/summary.txt"
    echo "kept: $DIR (tap.csv, bag/, goal_*.txt, stop_*, rt/, launch.log, summary.txt)"
    ;;
  run)
    MODE=$2
    shift 2
    STAGES=${*:-start first stops base steps}
    bash "$0" cool || exit 1
    bash "$0" up "$MODE" || exit 1
    for stage in $STAGES; do
      case "$stage" in
        start) what="to the rest pose, upright, ~10 s" ;;
        first) what="T1, J1 5 deg, from and back to rest, ~30 s" ;;
        base) what="T3 T6 T4 x3, J1+J2 20 deg, each from and back to rest, ~5 min" ;;
        wrist) what="T11 T12 T13 x3, J3-J6 20 deg, J3 goes UP, each from and back to rest, ~5 min" ;;
        stops) what="5 stops at 15 deg/s on J1 40 deg, each from and back to rest, ~4 min" ;;
        steps) what="6 x J1 5 deg step and back, from rest, ~1 min" ;;
        *) echo "unknown stage $stage" >&2; continue ;;
      esac
      answer=""
      if [ "${AUTO:-0}" != 1 ]; then
        read -r -p "Next: $stage ($what). Enter = go, s = skip, q = end this mode: " answer
      fi
      case "$answer" in
        q) break ;;
        s) continue ;;
      esac
      bash "$0" "$stage" || { echo "!!! $stage failed: ending this mode" >&2; break; }
    done
    bash "$0" down
    echo ">>> mode $MODE done. The arm holds the rest pose; the next run waits until it is cool."
    ;;
  cool)
    # Between modes (handover, 24 Nov): the arm holds the rest pose and the wrist cools ~50 -> 40
    # degC in ~10 min. `up` refuses above MAX_START_C; this waits for it instead. No driver may
    # be up (it owns the port); power_on only holds the pose. DRY: nothing to wait for.
    [ "${DRY:-0}" = 1 ] && exit 0
    [ -n "$(current_mode)" ] && { echo "mode $(current_mode) is up: bash $0 down first" >&2; exit 1; }
    python3 - "${MAX_START_C:-45}" "${COOL_TIMEOUT_S:-1800}" << 'PY'
import sys, time
from pymycobot import MyCobot280
limit, timeout = float(sys.argv[1]), float(sys.argv[2])
arm = MyCobot280('/dev/ttyTHS1', 1000000)
arm.power_on()
start = time.time()
while True:
    temps = arm.get_servo_temps()
    ok = isinstance(temps, list) and len(temps) == 6 and all(0 < t <= limit for t in temps)
    print(f'{time.time() - start:5.0f} s  servo temps degC: {temps}', flush=True)
    if ok:
        sys.exit(0)
    if time.time() - start > timeout:
        sys.exit(f'!!! not cool after {timeout:.0f} s: stop for the day or wait longer')
    time.sleep(30)
PY
    ;;
  state)
    echo "mode up: $(current_mode)"
    timeout 5 ros2 control list_hardware_components 2> /dev/null | grep -iE "positronic_arm|state" | head -4
    timeout 5 ros2 control list_controllers 2> /dev/null
    ;;
  *)
    sed -n "/^#   bash \$T\/arm_run_static.sh run/,/^#   done before the next speed/p" "$0"
    exit 2
    ;;
esac
