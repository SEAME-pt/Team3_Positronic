# Evidence: /joint_states at the measured rate from the Jetson container (#23)

## Verdict

All criteria pass on a 10 minute run on the real arm.

| Criterion (#23) | Result | Measured |
|---|---|---|
| Rate equal to the chosen nominal rate (30 Hz, see note below) | PASS | mean 30.00 Hz |
| 10 min without a period above 2x nominal (66.7 ms) | PASS | max 41.9 ms, 0 periods above the limit |
| Joint names equal to the URDF | PASS | all 18905 messages carry the 6 URDF joints |
| Values in radians | PASS | worst joint2_to_joint1: +16.52 deg published vs +16.87 deg from pymycobot |

**Note on the rate.** #20 measured `get_angles` alone at 101 Hz on average (one call up to
21 ms) but did not fix a read rate for the driver. This run uses 30 Hz, which is also the command
rate #20 proposes (section 6). Confirmed by the team as the #23 nominal rate: <yes / pending>.

## Setup

| | |
|---|---|
| Date | 2026-10-28 18:09 (Jetson clock) |
| Operator | <name> |
| Arm | myCobot 280 Jetson Nano |
| Commit | <git rev-parse --short HEAD on the laptop that ran make nano-build> |
| Image | positronic-dev:nano, built <date> |
| Launch | `ros2 launch positronic_bringup driver.launch.py update_rate:=30` |
| Nominal rate | 30 Hz (see the note above; #20 evidence in `docs/03_verification/serial_measurement/docs/serial_link.md`) |
| Duration | 630.1 s recorded (18905 messages) |
| Reference (pymycobot get_angles, deg) | [16.87, -16.17, 25.48, -10.45, 1.23, 0.26] |

Safety: arm powered, no motion commands, bench clear, nothing else on `/dev/ttyTHS1`. The driver
exports no command interface (`run_info.txt`: empty command interfaces).

## Checker report

Output of `check_joint_states.py ... --rate 30 --reference-deg ... --report report.md`:

| Criterion | Result | Detail |
|---|---|---|
| duration | PASS | 630.1 s recorded, 600 s required |
| rate | PASS | mean 30.00 Hz, nominal 30 Hz (accepted 28.50 to 31.50) |
| periods | PASS | 0 periods above 66.7 ms; p50 33.3 ms, p99 35.2 ms, max 41.9 ms at 20.7 s |
| names | PASS | all 18905 messages carry the 6 URDF joints |
| radians | PASS | every position finite and inside the URDF limits |
| reference | PASS | worst joint2_to_joint1: +16.52 deg published vs +16.87 deg from pymycobot (tolerance 0.5 deg) |

Full console output:

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

## ros2 topic hz

Last lines of `topic_hz.log`:

```text
average rate: 30.003
	min: 0.028s max: 0.037s std dev: 0.00120s window: 500
average rate: 30.000
	min: 0.028s max: 0.037s std dev: 0.00123s window: 500
average rate: 30.000
	min: 0.028s max: 0.037s std dev: 0.00120s window: 500
```

## Run information

`run_info.txt` (update rate, hardware component state, first message):

```text
date: 2026-10-28T18:09:32+00:00
host: nano
duration_s: 630
update_rate: Integer value is: 30
hardware components:
  Hardware Component 1
  	name: positronic_arm
  	type: system
  	plugin name: positronic_hardware/RobotSystem
  	state: id=3 label=active
  	read/write rate: 30 Hz
  	is_async: False
  	command interfaces
first message:
  header:
    stamp:
      sec: 1793210983
      nanosec: 390556867
    frame_id: base_link
  name:
  - joint2_to_joint1
  - joint3_to_joint2
  - joint4_to_joint3
  - joint5_to_joint4
  - joint6_to_joint5
  - joint6output_to_joint6
  position:
  - 0.28832839242946323
  - -0.28221974004748307
  - 0.44470989340815514
  - -0.18099064343181195
  - 0.021467549799530253
  - 0.004537856055185256
  velocity:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  effort:
  - .nan
  - .nan
  - .nan
  - .nan
  - .nan
  - .nan
```

## Observations

- **The control loop kept time.** On the controller manager stamps the longest period was
  39.0 ms (p99 34.0 ms) against a nominal 33.3 ms; on the receive time it was 41.9 ms. The few
  extra milliseconds are on the way to the recorder, not in `read()`.
- **The arm did not move during the run.** All six positions are constant for the 630 s and the
  velocity is 0. The four #23 criteria do not need motion and pass, but a static recording cannot
  tell live samples from a frozen last value (a silent link is not detected yet, see the known
  limitations in `tools/README.md`). The data was live at the start: joint2_to_joint1 and
  joint5_to_joint4 had moved since the reference reading taken earlier (16.87 to 16.52 deg and
  -10.45 to -10.37 deg), and the reference check passes. A short run moving the arm by hand would
  show the positions and the velocity following it.
- No leading all-zero messages, no non-finite position, no lost message reported by the recorder.
- `ros2_control_node` warns `Could not enable FIFO RT scheduling policy (Operation not permitted)`:
  the container has no real-time priority. The rate held without it.
- The bag (`bag_0.mcap`, 7.5 MB) is not in git. It is kept by the operator at <location>.
