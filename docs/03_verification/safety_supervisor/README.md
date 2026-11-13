# Safety supervisor on the arm: fault injection (#45)

Evidence that the safety supervisor (`src/positronic_safety/README.md`) halts the real arm.
Raw data: `2026-11-13_halts.csv`, one line per halt.

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
| 1 | 5 min, nothing wrong | 2 | 10 Hz: **one false `HEARTBEAT_LOST`** after ~3 min. 25 Hz (local change, not committed): no halt |
| 2 | Stop service mid-motion | 5 counted | all halted; `halt_ms` 51-76; `rest_ms` 16, 91, 108, 251, 259; travel ≤ 0.53° |
| 3 | `kill -9` the mission node | 5 | all halted, 304-322 ms after the last heartbeat |
| 4 | Laptop Wi-Fi off | 1 | `HEARTBEAT_LOST`, 301.7 ms |
| 5 | Deactivate `joint_state_broadcaster` | 1 | `FAULT_DATA_LOST`, `halt_ms` 63 |
| 6 | Wait after a halt, mission node alive | 1 | stayed stopped ~3 min |
| 7 | `kill -9` the supervisor mid-move | 1 | **the arm finished its move**: the known limit |

Test 2 had 15 trials; 10 hit the arm before it moved (speed at the stop near 0) and are not
counted. Also seen: the driver refused to activate with J3 at 154° (limit ±150°), and the
supervisor turned the failed resume into a halt (`RESUME_FAILED`, first CSV line).

Not run: Gazebo (follow-up), and a serial link fault on the arm (needs a driver hook; covered by
unit tests).

## What it means for #45

- The 300 ms timeout needs the mission node at **25 Hz**; repeat test 1, longer, after that change.
- "At rest within 100 ms of a stop" is met 2 times of 5: the halt takes 51-76 ms, the rest is
  the arm settling. New target or a faster halt: team decision.
- "Halt within 300 ms of a kill" cannot hold with a 300 ms timeout: 304-322 ms measured.