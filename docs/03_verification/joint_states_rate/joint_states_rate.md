# Evidence: /joint_states at the measured rate from the Jetson container (#23)

## Verdict

All criteria pass on a 10 minute run on the real arm (myCobot 280 Jetson Nano).

| Criterion (#23) | Result | Measured |
|---|---|---|
| Rate equal to the chosen nominal rate (30 Hz, see note below) | PASS | mean 30.00 Hz |
| 10 min without a period above 2x nominal (66.7 ms) | PASS | max 41.9 ms, 0 periods above the limit |
| Joint names equal to the URDF | PASS | all 18905 messages carry the 6 URDF joints |
| Values in radians | PASS | worst joint2_to_joint1: +16.52 deg published vs +16.87 deg from pymycobot |

**Note on the rate.** #20 measured `get_angles` alone at 101 Hz on average (one call up to
21 ms) but did not fix a read rate for the driver. This run uses 30 Hz, which is also the command
rate #20 proposes (section 6 of `docs/03_verification/serial_measurement/docs/serial_link.md`).
Confirmed by the team as the #23 nominal rate: <yes / pending>.

## How this was produced

A one-off recording and check, not committed code (the driver and its tests are in
`positronic_bringup` and `positronic_hardware`). Anyone can reproduce it from the bag with the
standard ROS 2 tools:

1. On the Jetson, with the arm powered: read the reference angles with
   `pymycobot` (`MyCobot280('/dev/ttyTHS1', 1000000).get_angles()`), before starting the driver so
   nothing else holds the port.
2. `ros2 launch positronic_bringup driver.launch.py update_rate:=30` (serial backend, read-only).
3. Record for 10 min 30 s: `ros2 bag record --topics /joint_states` and, alongside,
   `ros2 topic hz /joint_states --window 500`.
4. Check the bag: recorded duration; mean rate; the gap between consecutive messages against
   2x the nominal period; the joint names against the URDF; every position finite and inside the
   URDF joint limits; and the first positions against the pymycobot reference (in degrees). The
   numbers below are that check.

The recording and checking were done with a throwaway script kept with the run, not in the repo.

## Setup

| | |
|---|---|
| Date | 2026-10-28 18:09 (Jetson clock) |
| Operator | Leandro Gertrudes |
| Arm | myCobot 280 Jetson Nano |
| Launch | `ros2 launch positronic_bringup driver.launch.py update_rate:=30` |
| Nominal rate | 30 Hz (see the note above) |
| Duration | 630.1 s recorded (18905 messages) |
| Reference (pymycobot get_angles, deg) | [16.87, -16.17, 25.48, -10.45, 1.23, 0.26] |

Safety: arm powered, no motion commands, bench clear, nothing else on `/dev/ttyTHS1`. The driver
exports no command interface, so no controller can move the arm.

## Check result

| Criterion | Result | Detail |
|---|---|---|
| duration | PASS | 630.1 s recorded, 600 s required |
| rate | PASS | mean 30.00 Hz, nominal 30 Hz (accepted 28.50 to 31.50) |
| periods | PASS | 0 periods above 66.7 ms; p50 33.3 ms, p99 35.2 ms, max 41.9 ms at 20.7 s |
| names | PASS | all 18905 messages carry the 6 URDF joints |
| radians | PASS | every position finite and inside the URDF limits |
| reference | PASS | worst joint2_to_joint1: +16.52 deg published vs +16.87 deg from pymycobot (tolerance 0.5 deg) |

The controller-manager stamps (when each message was published, as opposed to when it was
recorded) give p50 33.3 ms, p99 34.0 ms, max 39.0 ms, against a nominal 33.3 ms.

## ros2 topic hz

Last lines of the live `ros2 topic hz /joint_states` output during the run:

```text
average rate: 30.003
	min: 0.028s max: 0.037s std dev: 0.00120s window: 500
average rate: 30.000
	min: 0.028s max: 0.037s std dev: 0.00123s window: 500
average rate: 30.000
	min: 0.028s max: 0.037s std dev: 0.00120s window: 500
```

## Run information

Update rate, hardware component state, and the first `/joint_states` message, read on the Jetson:

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

- **The control loop kept time.** The longest period on the publish stamps was 39.0 ms against a
  nominal 33.3 ms; on the receive time it was 41.9 ms. The few extra milliseconds are on the way
  to the recorder, not in `read()`.
- **The arm did not move during the run.** All six positions are constant for the 630 s and the
  velocity is 0. The #23 criteria do not need motion and pass, but a static recording cannot tell
  live samples from a frozen last value (the driver does not detect a silent link yet). The data
  was live at the start: joint2_to_joint1 and joint5_to_joint4 had moved since the reference
  reading (16.87 to 16.52 deg and -10.45 to -10.37 deg), and the reference check passes. A short
  run moving the arm by hand would show the positions and the velocity following it.
- No leading all-zero messages, no non-finite position, no lost message reported by the recorder.
- `ros2_control_node` warns `Could not enable FIFO RT scheduling policy (Operation not permitted)`:
  the container has no real-time priority. The rate held without it.
- The bag (`bag_0.mcap`, 7.5 MB) is not in git. It is kept by the operator at <location>.
