# Evidence: /joint_states at the measured rate from the Jetson container (#23)

## Verdict

**30 Hz is the chosen rate and passes all criteria** on a 10 minute run on the real arm
(myCobot 280 Jetson Nano). 50 Hz was also tried and fails the period criterion, which is why the
driver defaults to 30 Hz.

| Criterion (#23) | 30 Hz (accepted) | 50 Hz (rejected) |
|---|---|---|
| Duration >= 10 min | PASS 630.1 s | PASS 630.6 s |
| Rate = nominal | PASS mean 30.00 Hz | PASS mean 49.26 Hz |
| No period above 2x nominal | PASS max 41.9 ms (limit 66.7) | **FAIL** 88 periods above 40 ms, max 615.9 ms |
| Joint names = URDF | PASS | PASS |
| Values in radians | PASS | PASS |
| Reference (pymycobot) | PASS worst 0.35 deg | PASS worst 0.00 deg |

**Why 30 Hz.** #20 found 30 Hz writes clean and 50 Hz writes prone to stalls; the read alone was
clean at 101 Hz (section 6 of `docs/03_verification/serial_measurement/docs/serial_link.md`). Both rates were run
here for 10 minutes. At 30 Hz no period exceeded 2x nominal. At 50 Hz the first 9.5 minutes were
clean, then the control loop stalled in a burst over the last minute (up to 616 ms). The stalls
show on the controller-manager stamps too (max 604 ms), so the loop itself was starved, not the
serial read, which fits a Jetson Nano running the loop without real-time priority (the container
logs `Could not enable FIFO RT scheduling policy`). 30 Hz leaves the headroom this needs.

## How this was produced

Reproducible from the bag with standard ROS 2 tools, and with the checker in
`docs/03_verification/joint_states_rate/tools/`:

1. On the Jetson, with the arm powered: read the reference angles with `pymycobot`
   (`MyCobot280('/dev/ttyTHS1', 1000000).get_angles()`), before starting the driver.
2. `ros2 launch positronic_bringup driver.launch.py update_rate:=<rate>` (serial backend, read-only).
3. `bash docs/03_verification/joint_states_rate/tools/record_joint_states.sh` records
   `/joint_states` for 10 min 30 s plus the `ros2 topic hz` log.
4. `python3 docs/03_verification/joint_states_rate/tools/check_joint_states.py <bag> --rate <rate>
   --reference-deg '<angles>'` checks duration, mean rate, every gap against 2x the nominal
   period, the joint names against the URDF, every position finite and inside the URDF limits, and
   the first positions against the pymycobot reference.

## Setup

| | 30 Hz run | 50 Hz run |
|---|---|---|
| Date | 2026-10-28 18:09 (Jetson clock) | 2026-10-29 12:39 (Jetson clock) |
| Launch | `driver.launch.py update_rate:=30` | `driver.launch.py update_rate:=50` |
| Duration | 630.1 s (18905 messages) | 630.6 s (31063 messages) |
| Reference (pymycobot, deg) | [16.87, -16.17, 25.48, -10.45, 1.23, 0.26] | [1.14, -16.96, 17.84, 0.35, -8.96, 21.97] |

| | |
|---|---|
| Operator | <name> |
| Arm | myCobot 280 Jetson Nano |
| Commit | <git rev-parse --short HEAD on the laptop that ran make nano-build> |
| Image | positronic-dev:nano, built <date> |

Safety: arm powered, no motion commands, bench clear, nothing else on `/dev/ttyTHS1`. The driver
exports no command interface, so no controller can move the arm.

## 30 Hz run (accepted)

```text
/ws/joint_states_runs/20261028_180929/bag: 18905 messages on /joint_states

PASS  duration   630.1 s recorded, 600 s required
PASS  rate       mean 30.00 Hz, nominal 30 Hz (accepted 28.50 to 31.50)
PASS  periods    0 periods above 66.7 ms; p50 33.3 ms, p99 35.2 ms, max 41.9 ms at 20.7 s
PASS  names      all 18905 messages carry the 6 URDF joints
PASS  radians    every position finite and inside the URDF limits
PASS  reference  worst joint2_to_joint1: +16.52 deg published vs +16.87 deg from pymycobot (tolerance 0.5 deg)
info  header     p50 33.3 ms, p99 34.0 ms, max 39.0 ms (controller manager stamps, nominal 33.3 ms)

OVERALL: PASS
```

`ros2 topic hz` last lines: `average rate: 30.000`, min 0.028 s, max 0.037 s, std dev 0.0012 s.

## 50 Hz run (rejected)

```text
/ws/joint_states_runs/20261029_123941/bag: 31063 messages on /joint_states

PASS  duration   630.6 s recorded, 600 s required
PASS  rate       mean 49.26 Hz, nominal 50 Hz (accepted 47.50 to 52.50)
FAIL  periods    88 periods above 40.0 ms; p50 20.0 ms, p99 23.0 ms, max 615.9 ms at 592.8 s; first: 228.3 ms at 570.3 s, 235.1 ms at 570.5 s, 189.9 ms at 570.7 s, 130.0 ms at 570.8 s, 85.4 ms at 570.9 s
PASS  names      all 31063 messages carry the 6 URDF joints
PASS  radians    every position finite and inside the URDF limits
PASS  reference  worst joint3_to_joint2: -16.96 deg published vs -16.96 deg from pymycobot (tolerance 0.5 deg)
info  header     p50 20.0 ms, p99 20.9 ms, max 604.4 ms (controller manager stamps, nominal 20.0 ms)

OVERALL: FAIL
```

The stalls are a single burst at the end: the first 570 s (90% of the run) had no period above
40 ms, then from 570 s to the end they came in bursts (28 in the 570 s window, tapering to the
end), 88 in total, 11.7 s of lost time, worst 616 ms. Everything else passes, including the
reference (the published angles match pymycobot exactly). `ros2 topic hz` shows the effect too:
mean around 48.5 Hz with max period 0.091 s.

## Observations

- **The stall is in the control loop, not the serial read.** The controller-manager stamps carry
  the same ~0.6 s gaps as the receive times, so the ros2_control_node update cycle itself was
  delayed; the serial read runs in its own thread and only copies a lock-free buffer. On a Jetson
  Nano without real-time scheduling, 50 Hz plus the bag recording and DDS occasionally starve the
  loop. 30 Hz has the headroom and stayed clean.
- **Neither run moved the arm.** Positions are constant and velocity is 0 throughout. The #23
  criteria do not need motion and pass, but a static recording cannot tell a live sample from a
  frozen one (the driver does not detect a silent link yet). The reference check shows the data
  was live at the start of each run.
- No non-finite positions, no leading all-zero messages.
- The bags (`bag_0.mcap`, 7.5 MB and 12 MB) are not in git. Kept by the operator at <location>.