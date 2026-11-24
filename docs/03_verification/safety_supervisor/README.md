# Safety supervisor on the arm: fault injection (#45)

Evidence that the safety supervisor (`src/positronic_safety/README.md`) halts the real arm.
Raw data: `2026-11-13_halts.csv`, one line per halt; `2026-11-23_deadman.csv` for tests 8 and 9
(#136), one line per trial.

**`halt_latency.py` before #171 is wrong after #151.** Since #151 `/joint_states` starts with
the gripper mock, whose velocity is NaN, so the old script never saw the arm at rest: `rest_ms`
and `travel_deg` came out empty and `speed_deg_s` too. It now keeps only the six arm joints, by
name, and stops with an error if one is missing. Lines recorded with it between #151
(2026-11-18) and #171 are not valid. The results below (2026-11-13) are from before #151 and
stand.

## How it was run

Two people, one with a hand on the power switch; bench clear; trajectory `T1.yaml` of
`../jtc_on_the_arm/` (J1 out 5° and back, about 4°/s at most), from the pose of `T0_to_start.yaml`.
Jetson: `ros2 launch positronic_bringup supervised.launch.py` and the halt logger (branch
`45-verification-tools`); laptop: the mission node, then
`ros2 lifecycle set /safety_supervisor activate`. Jetson Wi-Fi power saving off; mission node at
10 Hz except where said.

Columns: `halt_ms` trigger to the three halt requests answered; `rest_ms` trigger to every joint
under 0.02 rad/s; `speed_deg_s` fastest joint at the trigger; `travel_deg` motion after it;
`heartbeat_to_halt_ms` last heartbeat to the trigger.

## Results, 2026-11-13

| # | Test | Trials | Result |
|---|---|---|---|
| 1 | 5 min, nothing wrong | 2 | 10 Hz: **one false `HEARTBEAT_LOST`** after ~3 min. 25 Hz (the rate the mission node publishes since #45): no halt |
| 2 | Stop service mid-motion | 5 counted | all halted; `halt_ms` 51-76; `rest_ms` 16, 91, 108, 251, 259; travel ≤ 0.53° |
| 3 | `kill -9` the mission node | 5 | all halted, 304-322 ms after the last heartbeat |
| 4 | Laptop Wi-Fi off | 1 | `HEARTBEAT_LOST`, 301.7 ms |
| 5 | Deactivate `joint_state_broadcaster` | 1 | `FAULT_DATA_LOST`, `halt_ms` 63 |
| 6 | Wait after a halt, mission node alive | 1 | stayed stopped ~3 min |
| 7 | `kill -9` the supervisor mid-move | 1 | **the arm finished its move**: fixed by #136, see test 8 |

Test 2 had 15 trials; 10 hit the arm before it moved (speed at the stop near 0) and are not
counted. Also seen: the driver refused to activate with J3 at 154° (limit ±150°), and the
supervisor turned the failed resume into a halt (`RESUME_FAILED`, first CSV line).

## Dead-man check, 2026-11-23 (#136)

The driver halts the arm when the supervisor's heartbeat (`/safety_supervisor/heartbeat`) stops
for `supervisor_timeout_ms` (500), and the supervisor halts the arm before it exits on Ctrl+C.

| # | Test | Trials | Result |
|---|---|---|---|
| 8 | `kill -9` the supervisor mid-move, dead-man check on | 5 counted | all halted with fault 4, 509-528 ms after the last heartbeat (504-542 ms after the kill); travel ≤ 0.52°; each resumed with one activate of a restarted supervisor |
| 9 | Ctrl+C (SIGINT) on the supervisor alone mid-move | 3 | all halted at once: hardware inactive 82 ms after the signal (49-58 ms after the halt started); travel ≤ 0.35°; no fault 4 |

How it was run: one person with a hand on the power switch; the goal and the signal were sent
by a script on the Jetson after a 3 s countdown. `supervised.launch.py` on the Jetson, a
stand-in mission heartbeat on the Jetson at 50 Hz (`ros2 topic pub /mission/heartbeat`, so no
Wi-Fi in the loop), and `ros2 bag record` of the heartbeat, `/safety_supervisor/status`,
`/joint_states`, `/dynamic_joint_states` and `/rosout`. The signal came about 3.6 s into the goal,
with J1 moving at 1.2-2.3°/s (test 8) and 0.6-0.8°/s (test 9), from its positions over the last
0.6 s. Times come from the `/rosout` stamps and the heartbeat's own stamp, one clock.
`/joint_states` stops at fault 4 (the component returns ERROR), so the travel in test 8 is the
pose at the resume minus the last pose before the fault; in test 9 it is the motion after the
halt.

CSV columns: `goal_to_trigger_s` goal accepted to the signal; `trigger_to_event_ms` signal to the
fault 4 line (test 8) or to "hardware component is inactive" (test 9), from the script's
timestamp just before it ran `kill`, so up to ~15 ms early (starting a process on the Nano takes
that long): in test 8 `heartbeat_to_fault4_ms` is the measure to use; `heartbeat_to_fault4_ms`
last supervisor heartbeat to fault 4; `halt_to_inactive_ms` the supervisor's `HALT` line to
"inactive"; `j1_speed_deg_s` J1 over the 0.6 s before the event; `travel_deg` as above.

Not like the #45 runs:
- The trajectory is `T1` from pose 0 (every joint at 0; J1 out 5° in 3 s and back), not from the
  pose of `T0_to_start.yaml`: same amplitude and speed. The arm rested folded and was moved to 0
  slowly first.
- Gripper and camera were off the arm (taken off after a servo overheat earlier that morning).
- Supervisor bench timeouts: `heartbeat_timeout_ms` 1000 (not 300) and, from trial 5 of test 8,
  `fault_timeout_ms` 1000 (not 200). A ros2 CLI starting on the Nano stalled the stand-in
  heartbeat and `/dynamic_joint_states` by about 350 ms. Two attempts of test 8 were halted that
  way and are not counted (the CSV has them): in one the kill hit an arm already halted, in the
  other the goal was rejected and nothing moved. Both are mission-side triggers; the driver's
  `supervisor_timeout_ms` stayed 500.

Also seen: in test 9 trial 3 the supervisor exited after the hardware stop and before it switched
`arm_controller` off. The Ctrl+C hook only waits for the hardware stop, so this is as designed: the
component was inactive and the goal cancelled.

## Gazebo, 2026-11-13

`gazebo.launch.py`, the mission node, and the supervisor with `-p fault_topic:=/sim_fault`; a
30 Hz script published `driver/fault_code` = 0 there, because Gazebo has no driver fault code.

| Test | Result |
|---|---|
| Stop service mid-`T1` | `STOP_SERVICE`, `hardware_stopped` true, the arm stopped |
| `kill -9` the mission node mid-`T1` | `HEARTBEAT_LOST`, `hardware_stopped` true, the arm stopped |

## What it means for #45

- The 300 ms timeout needs the mission node at 25 Hz, which it publishes since #45 (at 10 Hz it
  halted falsely once).
- "At rest within 100 ms of a stop" is met 2 times of 5: the halt takes 51-76 ms, the rest is
  the arm settling. New target or a faster halt: team decision.
- "Halt within 300 ms of a kill" cannot hold with a 300 ms timeout: 304-322 ms measured.
