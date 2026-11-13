# Safety supervisor on the arm: fault injection (#45)

How to show, with numbers, that the safety supervisor halts the real arm. What the supervisor
is and how it decides: `src/positronic_safety/README.md`.

**Status: run on the arm on 2026-11-13; results in the last section.** Before that, the
supervisor passed its tests without the arm (unit tests, the node against a fake controller
manager) and the procedure had a dry run with `backend:=fake`.

## Safety

These tests move the real arm and stop it on purpose.

- Two people. One runs the commands; the other stands at the arm with a **hand on the power
  switch** and does nothing else.
- Bench clear, nobody inside the arm's reach, one person commands the arm at a time.
- Small moves at 15 deg/s or less (the working limit of `jtc_on_the_arm`), inside the joint
  limits, starting from the pose of `T0_to_start.yaml`.
- Safe state: supervisor INACTIVE, hardware component inactive, the arm holding its pose.
  To stop at any moment: `ros2 service call /safety_supervisor/stop std_srvs/srv/Trigger`.
- **Software cannot stop an arm whose serial line is dead**, and nothing stops the arm if the
  supervisor's own process dies (test 7 shows it). In both cases: the power switch.
- Activating the supervisor powers the servos on. They hold their pose; nothing moves until a
  goal is sent.

## Setup

Two conditions the heartbeat timeout depends on (`docs/03_verification/heartbeat_link`): the
mission node publishes at **25 Hz** (it still publishes at 10 Hz in `positronic_mission`:
change it first), and the Jetson's **Wi-Fi power saving is off**. From a laptop host terminal,
before every session (it comes back on after a reboot):

```bash
ssh nano 'iwconfig wlan0 | grep -i "power management"'   # must say off
ssh -t nano 'sudo iwconfig wlan0 power off'              # if it says on
```

On the Jetson (from a laptop, with the `nano` alias of `docs/build.md`):

```bash
make nano-build && make nano-shell
ros2 launch positronic_bringup supervised.launch.py
```

A second shell in the same container (`ssh nano`, then `docker exec -it positronic-arm bash`),
left running for the whole session:

```bash
source install/setup.bash
python3 docs/03_verification/safety_supervisor/tools/halt_latency.py | tee halts.csv
```

On the laptop that commands the arm: the mission node, configured and activated (its
heartbeat is what the supervisor waits for), then, with the bench clear:

```bash
ros2 lifecycle set /safety_supervisor activate
ros2 topic echo /safety_supervisor/status --once      # mode: 2
```

The trajectory for every test is `T1.yaml` of `docs/03_verification/jtc_on_the_arm/`, sent as
that folder's README says. After each halt: check the arm stands still, then activate the
supervisor again.

Try the whole setup once with `backend:=fake` before going to the arm, as the dry run in the
last section did.

## Tests

| # | What to do, mid-trajectory unless said | Expected | Criterion of #45 |
|---|---|---|---|
| 1 | Nothing: let the arm stand, ACTIVE, for 5 minutes, while someone runs `ros2 topic list`, `ros2 node list` and `ros2 topic echo` now and then | No halt. Count the HEARTBEAT_LOST halts | none; confirms the 300 ms heartbeat timeout on the arm |
| 2 | `ros2 service call /safety_supervisor/stop std_srvs/srv/Trigger`, 5 times | STOP_SERVICE, `hardware_stopped` 1 | `rest_ms` under 100 |
| 3 | `kill -9` the mission node, 5 times | HEARTBEAT_LOST | `heartbeat_to_halt_ms + rest_ms`: about 320 ms of detection plus the braking. Report the numbers; the #45 line "300 ms" needs rewriting |
| 4 | Take the laptop off the network (Wi-Fi off) | HEARTBEAT_LOST, same as test 3 | same as test 3 |
| 5 | Arm standing: `ros2 control switch_controllers --deactivate joint_state_broadcaster` | FAULT_DATA_LOST: the supervisor lost sight of the driver | halts |
| 6 | After a halt, wait one minute with the mission node alive | Still stopped; only `ros2 lifecycle set /safety_supervisor activate` resumes | stays stopped |
| 7 | `kill -9` the supervisor, during a small move | **The arm finishes its move.** This is the known limit, shown on purpose | written limit |

`halt_latency.py` prints one line per halt; its columns are explained at the top of the file.
`rest_ms` is the time from the trigger until every joint is slower than 0.02 rad/s.

Not on this list: a **serial link fault** (`driver/fault_code` = 1) cannot be injected into the
running driver yet. It is covered without the arm (`test_load_robot_system` latches the code,
`test_safety_supervisor` halts on it), and `hil_link_timeout` shows what the arm does on a
silent line. Injecting it on the arm needs a hook in the driver: a follow-up issue.

## What we expect to learn

- **Test 2 may miss 100 ms.** The lifecycle measurements of the driver say a move at 30 % speed
  stops about 8 degrees after the request. If `rest_ms` is over 100 with `halt_ms` of a few
  ms, the time is the arm braking, not the supervisor, and the criterion has to be rewritten
  with the measured number.
- **Test 1 should show no false halts.** The heartbeat link campaign
  (`docs/03_verification/heartbeat_link`) measured a longest gap of 157 ms, and one of at least
  204 ms with the real mission node, so the timeout is 300 ms. A false halt here means the
  conditions above are not met, or the link is worse on that day: write it down with the
  time.

## Results

### Dry run, 2026-11-12, `backend:=fake`

The setup above with the fake backend and the heartbeat timeout then in force (200 ms), one
trial per trigger, to check the procedure and `halt_latency.py` against ROS. The fake joints
do not move, so `rest_ms`, `speed_deg_s` and `travel_deg` mean nothing here.

| Trigger | `hardware_stopped` | `halt_ms` | `heartbeat_to_halt_ms` |
|---|---|---|---|
| `STOP_SERVICE` | 1 | 42.1 | 44.5 |
| `HEARTBEAT_LOST` | 1 | 66.7 | 211.0 |
| `FAULT_DATA_LOST` | 1 | 47.3 | 89.0 |

The hardware-inactive request, the first of the halt, was answered within 1 to 2 ms (the
supervisor's log); `halt_ms` is the whole halt, from the trigger to the three requests
answered.

### On the arm, 2026-11-13

Raw: `2026-11-13_halts.csv` (one line per halt). Serial backend, two people, power saving off,
**mission node at 10 Hz** (the 25 Hz change is not merged yet) except the rerun of test 1. No Gazebo run before it: the
stop path had run end to end against the controller manager with the fake backend, and Gazebo
would add physics, not stop logic.

| # | Trials | Result | Numbers | vs #45 |
|---|---|---|---|---|
| 1 | 2 | 10 Hz: **fail**, one `HEARTBEAT_LOST` after about 3 min with nothing wrong (301.9 ms without a heartbeat). Rerun at 25 Hz (mission node timer changed locally to 40 ms, not committed): **pass**, 5 min, no halt | | 25 Hz: pass |
| 2 | 15 (5 mid-motion: 1, 11, 12, 13, 15) | every stop halted; travel after the stop at most 0.53° | mid-motion: `halt_ms` 51-76, `rest_ms` 16, 91, 108, 251, 259 | **2 of 5 under 100 ms** |
| 3 | 5 | every kill halted | heartbeat to halt 304-322 ms; `halt_ms` 60-106; last heartbeat to rest 315-507 ms | over 300 ms, as expected |
| 4 | 1 | Wi-Fi off: `HEARTBEAT_LOST`, same as test 3 | 301.7 ms; the arm was not moving at that instant | halts |
| 5 | 1 | `FAULT_DATA_LOST` | `halt_ms` 63 | halts |
| 6 | 1 | stayed stopped about 3 min with the mission node alive | | pass |
| 7 | 1 | supervisor killed mid-move: **the arm finished its move** | | the written limit |

Trials 2-10 of test 2 hit the arm before or without motion (speed at the stop near 0) and are
not counted; trial 14 too.

Also seen: the driver refused to activate with J3 at 154° (limit ±150°), and the supervisor
turned the failed resume into a halt (`RESUME_FAILED`, first line of the CSV).

What it means:

- **The 300 ms timeout needs the mission node at 25 Hz** (test 1): it halted falsely at
  10 Hz and held 5 minutes at 25 Hz. One 5-minute run is little evidence: repeat test 1, longer,
  once the 25 Hz change is merged.
- **100 ms to rest is not met** in 3 of 5 (test 2). The halt itself takes 51-76 ms; the rest
  is the arm settling. The criterion needs the measured numbers or a faster halt.
- **300 ms from a kill to the halt is not possible** with a 300 ms timeout: 304-322 ms
  measured. The criterion needs rewriting.
