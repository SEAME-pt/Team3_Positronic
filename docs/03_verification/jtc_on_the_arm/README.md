# joint_trajectory_controller on the real arm (#33)

Part 2 of #33: the `arm_controller` (a `joint_trajectory_controller`, JTC) configured in part 1
driving the real arm through the `follow_joint_trajectory` action. The three criteria of the
issue are measured here: a 3-point trajectory executes on hardware, the tracking error per joint
at each waypoint stays below 3 deg, and the update rate equals the one fixed by the serial
measurement (30 Hz).

Run A and the update rate are the three criteria. The tolerances, the speed curve, the runs
without the gripper and the wrist joints are measurements beyond the issue, done while the bench
was up: they fix the joint velocity limits MoveIt will need and the rest error the pick will
meet, in sprint 3.

| File | What it is |
|---|---|
| [`results.md`](results.md) | The per-run tables the sections below summarise. |
| `trajectories/T1.yaml` to `T3.yaml`, `T11.yaml` | The goals sent, as `ros2 action send_goal` takes them. T4 to T9 are T3 with other times, T10, T12 and T13 are T11 with other times (table below). |
| `trajectories/T0_to_start.yaml` | One slow point to the start pose, for the beginning of a session. |
| `tools/waypoint_error.py` | Reads a `send_goal --feedback` recording and prints the error per joint at each waypoint, in degrees. |
| `tools/test_waypoint_error.py` | Its unit tests, no ROS needed: `python3 -m pytest docs/03_verification/jtc_on_the_arm/tools`. |

## Setup

- Arm with the gripper and the camera mounted, the configuration MoveIt will drive in sprint 3.
- The driver on the Jetson, `driver.launch.py` with the serial backend, `update_rate` 30.
- `arm_controller` active, tolerances at their defaults (0, no check) for run A. The
  tolerances in `jetson_controllers.yaml` come from run A and are checked in run B.
- Start pose read from `/joint_states` before the first goal and written as the first point of
  every trajectory, so the controller never starts with a jump:
  `[0.2822, 0.0061, -0.0106, -0.2330, 0.0075, -0.0106]` rad.

## Safety protocol

There is no e-stop in the cell. Every goal was sent under these conditions, and the same apply
to any repeat:

1. Two people. One at the keyboard, one with a hand on the power plug, in place before the
   launch starts: the controller is active as soon as the driver is up.
2. Bench clear in the reach of the arm, nothing in front of it (T2 to T9 lean joint 2 forward)
   and nothing above it (T10 to T13 raise joint 3).
3. First goal of the session: the current pose, one joint by 5 deg, back. Only then larger ones.
4. Pull the plug on wrong motion: more than the commanded amplitude, another joint moving,
   towards an obstacle, or not stopping at the last point. No motion or a stop midway is not a
   reason to pull: read the driver log instead (the plugin refuses an out-of-limit target, #34).
5. Between goals, wait for the arm to stand still and confirm the pose with
   `ros2 topic echo --once /joint_states`.
6. At the end, `ros2 control set_controller_state arm_controller inactive` before closing the
   launch.

## How to run

Terminal 1, the driver (from the laptop, `NANO` is the Jetson's ssh target):

```bash
NANO=<user@jetson> make nano-build
NANO=<user@jetson> make nano-shell
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 launch positronic_bringup driver.launch.py
```

Terminal 2, inside the same container:

```bash
ssh -t <user@jetson> docker exec -it positronic-arm bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 action send_goal --feedback /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "$(cat docs/03_verification/jtc_on_the_arm/trajectories/T1.yaml)" | tee /ws/log/jtc_T1_r1.txt
```

Terminal 3, the update rate during the goals: `ros2 topic hz /joint_states --window 300`.

Then, on the laptop, over the copied recordings:

```bash
python3 docs/03_verification/jtc_on_the_arm/tools/waypoint_error.py jtc_T1_r1.txt jtc_T1_r2.txt ...
```

## Trajectories

| | Joints | Amplitude | Segment | Speed |
|---|---|---|---|---|
| T1 | 1 | 5 deg | 3 s | 1.7 deg/s |
| T2 | 1 and 2 | 20 deg each | 4 s | 5 deg/s |
| T3 | 1 and 2 | 20 deg each | 2 s | 10 deg/s |
| T6 | 1 and 2 | 20 deg each | 1.33 s | 15 deg/s |
| T9 | 1 and 2 | 20 deg each | 1.14 s | 17.5 deg/s |
| T4 | 1 and 2 | 20 deg each | 1 s | 20 deg/s |
| T8 | 1 and 2 | 20 deg each | 0.89 s | 22.5 deg/s |
| T7 | 1 and 2 | 20 deg each | 0.8 s | 25 deg/s |
| T5 | 1 and 2 | 20 deg each | 0.5 s | 40 deg/s |
| T11 | 3, 4, 5, 6 | 20 deg each | 2 s | 10 deg/s |
| T12 | 3, 4, 5, 6 | 20 deg each | 1.33 s | 15 deg/s |
| T13 | 3, 4, 5, 6 | 20 deg each | 1.14 s | 17.5 deg/s |
| T10 | 3, 4, 5, 6 | 20 deg each | 1 s | 20 deg/s |

Each is three points: the start pose at 2 s, the displaced pose, the start pose again. Run A is
T1 to T3, three runs of each plus a first T1 as the opening goal of the session, ten goals in
all. T4 to T9 look for the speed the tolerances refuse on the base and T10 to T13 on the wrist,
all of them in run B only. The numbers are the order the goals were written in, not the order of
speed. Only T0 to T3 and T11 are kept as files: the others differ only in `time_from_start`.

## Run A, default tolerances

All ten goals ended in `SUCCEEDED`. The error per joint at every waypoint, at the last feedback
and the largest of the run is in [results.md](results.md#run-a-default-tolerances), one row per
waypoint. Positive means the arm is ahead of the command, negative behind. Largest errors, in
degrees:

| | joint 1, moving | joint 2, moving from T2 | joint 4, never commanded |
|---|---|---|---|
| T1, 1.7 deg/s | 1.0 to 1.6 | 0 | 1.2 to 2.6 |
| T2, 5 deg/s | 1.3 to 1.9 | 0.8 to 1.2 | 1.6 to 2.7 |
| T3, 10 deg/s | 2.1 to 2.8 | 1.2 to 2.0 | 1.5 to 2.7 |
| largest seen | 2.93 | 2.32 | 2.73 |

Update rate during the goals: `/joint_states` at 30.0 Hz, interval 28 to 39 ms, standard
deviation 1 ms over windows of 300 messages, with the pauses of about 0.3 s noted under run B.

### What the numbers say

- **The 3 deg criterion holds at every waypoint, by 0.07 deg at the fastest.** Joint 1 reached
  2.93 deg at 10 deg/s. A faster trajectory passes 3 deg: the tolerances below are what make
  that an abort instead of a silent miss.
- **Joint 4 sits 1.2 to 2.7 deg off without ever being commanded to move**, and at rest every
  moved joint stays 1.2 to 2.8 deg short of its last point (the `last` rows). The command and the
  reply carry the same value, so it is the servo not closing on its target, not the driver. The
  goal tolerance has to allow for that or every trajectory ends in `ABORTED` once tolerances are
  on. The sections on run B and on the gripper take this further.

## Tolerances chosen for `jetson_controllers.yaml`

| Parameter | Value | Why |
|---|---|---|
| `constraints.goal_time` | 1.0 s | Time after the last point for the servos to close the last degree. |
| `<joint>.trajectory` | 0.087 rad, 5 deg | 2 deg over the worst seen while moving (2.93 deg). Still aborts a joint that stops following within 0.5 s at 10 deg/s. |
| `<joint>.goal` | 0.070 rad, 4 deg | 1.2 deg over the worst rest error (2.83 deg). |

The same values for the six joints: only joints 1, 2 and 4 showed error in run A, but the
others were never loaded.

## Run B, with the tolerances

Same arm, same start pose, gripper and camera mounted, the tolerances above in
`jetson_controllers.yaml`. T1 to T3 three times each, then faster goals to find the speed the
trajectory tolerance refuses: T6 (15 deg/s) three times, T4 (20 deg/s) four times, T8
(22.5 deg/s) three times, T7 (25 deg/s) and T5 (40 deg/s) once.

Per-run `last` and `max abs`: [results.md](results.md#run-b-with-the-tolerances).

- **The nine trajectories still end in `SUCCEEDED`** with the tolerances on. At the waypoints
  every error stays below 3 deg (largest 2.66 deg, joint 1, T3). Between waypoints joint 1
  touched 3.12 deg once at 10 deg/s, well inside the 5 deg trajectory tolerance.
- **The limit is 20 deg/s, with no margin.** Largest error on joint 1, per run:

  | deg/s | runs | largest error, deg | result |
  |---|---|---|---|
  | 10 (T3) | 3 | 2.7, 2.7, 3.1 | pass |
  | 15 (T6) | 3 | 3.7, 4.0, 3.8 | pass |
  | 17.5 (T9, without the gripper) | 3 | 3.9, 4.2, 4.2 | pass |
  | 20 (T4) | 4 | 4.5, 4.9, 4.8, 4.7 | pass, 0.15 deg from the tolerance |
  | 22.5 (T8) | 3 | crossed 5 | abort, all three |
  | 25 (T7) | 1 | crossed 5 at 0.27 s | abort |
  | 40 (T5) | 1 | crossed 5 at 0.10 s | abort |

  The error grows with speed and runs of the same goal differ by up to 0.4 deg, so 20 deg/s
  passes every time but one more run could not. 15 deg/s keeps 1 deg of margin.
- **The aborts behave.** "Aborted due to path tolerance violation", the arm stops where it is,
  2 to 4 deg past the start pose, and both controllers stay `active`: the driver did not drop, a
  new goal can follow. This is what the tolerances buy: a trajectory the arm cannot follow ends
  in an abort and a standing arm, not in a silent miss.
- **Joint 4 again 2.3 to 2.7 deg off** without being commanded, as in run A.

Update rate over the three sessions: 30.0 Hz, intervals of 27 to 39 ms, the criterion holds. A
few pauses of about 0.3 s in the control loop were also seen during the sessions. The loop
period and its worst cases are measured in detail over a 10-minute run in #39.

## Without the gripper and the camera

The same set, before mounting them, to tell the weight's share of the errors. Arm cold, first
goals of the session. T6, T8 and T9 were added after the loaded runs, with the gripper taken off
again.

Per-run `last` and `max abs`: [results.md](results.md#without-the-gripper-and-the-camera).

| | with the gripper (run B) | without |
|---|---|---|
| Joint 1 error at the waypoints, T3 | 2.2 to 2.7 deg | 2.2 to 2.6 deg |
| Joint 4 at rest, never commanded | 2.3 to 2.7 deg | 0.5 to 0.9 deg |
| Joint 5 at rest, never commanded | 0.8 deg | 1.4 deg |
| T6 at 15 deg/s, largest error | 3.7 to 4.0 deg | 3.5 deg |
| T4 at 20 deg/s, largest error | 4.5 to 4.9 deg | 4.7 deg |
| T8 at 22.5 deg/s | aborted, 3 of 3 | aborted, 3 of 3 |
| T5 at 40 deg/s | aborted 0.10 s into the segment | aborted 0.07 s into the segment |

Two things come out of the comparison:

- **The tracking error does not depend on the weight.** At 15, 20 and 22.5 deg/s the two sets
  give the same errors and the same aborts, and the difference between them is smaller than the
  difference between two runs of the same goal. The lag is the servo's, not the load's.
- **The rest error does.** Joint 4 sits 2 deg further off its command with the gripper's weight
  on it. Every joint that moved during the session settles 0.5 to 1.5 deg from its command, with
  or without load. The weight adds to that on the joints it pulls on. It is the servo stopping
  short of its target, not the driver: the command and the reply carry the same value.

## The wrist joints

T1 to T9 only moved the base (joint 1) and the shoulder (joint 2). T10 to T13 move joints 3, 4,
5 and 6 by 20 deg each, together, with joints 1 and 2 still, gripper and camera mounted: at
20 deg/s (T10), 10 deg/s (T11), 15 deg/s (T12) and 17.5 deg/s (T13).

Per-run `last` and `max abs`: [results.md](results.md#the-wrist-joints).

| deg/s | joint 3 | joint 4 | joint 5 | joint 6 | result |
|---|---|---|---|---|---|
| 10 (T11) | 2.2 to 2.4 | 3.6 to 3.8 | 3.5 to 4.2 | 2.0 | pass, 3 of 3 |
| 15 (T12) | 2.8 to 3.1 | 4.0 to 4.3 | 3.6 to 4.1 | 2.4 to 2.8 | pass, 3 of 3 |
| 17.5 (T13) | 3.1 to 3.5 | 4.4 to 4.7 | 4.2 to 4.7 | 2.8 to 3.0 | pass, 3 of 3, 0.3 deg to spare |
| 20 (T10) | | | | | abort, 2 of 2 |

- **The wrist is slower than the base.** At 10 deg/s joint 1 runs 2.7 to 3.1 deg behind,
  joints 4 and 5 run 3.5 to 4.2. At 20 deg/s the base still passes. The wrist aborts 0.2 s into
  the segment, before any of its joints had moved a degree: the command was 4 deg away and the
  servos had not started.
- **On the wrist the error grows slowly, then the servo does not start in time.** From 10 to
  17.5 deg/s joint 4 gains one degree and still passes. What costs is the start, the time the
  servo takes to leave its dead band, and that time is the same at any speed. At 20 deg/s the
  command is out of tolerance before the servo has left.
- **Joint 4 pays for the gripper twice.** It starts 2.3 to 2.7 deg off (the rest error above)
  and the lag adds on top. Half of its tolerance is spent before it moves.

Two things for whoever stops the arm (#45): a wrist servo takes about 0.2 s to start moving
after the command changes, in both directions. And an abort of the controller does not freeze
the arm: it is up to 5 deg behind the command when the controller gives up, and the firmware
finishes the last target it was sent before the hold takes over, so the arm moves those degrees
and stops. The stop that freezes the arm is the driver's `halt()`, not the controller's abort.

## What the tolerances are now

Checked by run B, with the gripper mounted:

| Parameter | Value | Checked by |
|---|---|---|
| `constraints.goal_time` | 1.0 s | every goal under 20 deg/s on the base and under 20 on the wrist ends in `SUCCEEDED` |
| `<joint>.trajectory` | 0.087 rad, 5 deg | 20 deg/s stays inside (4.5 to 4.9 deg), 22.5 deg/s aborts |
| `<joint>.goal` | 0.070 rad, 4 deg | at rest the arm is 2.7 deg off at most, on joint 4 under the gripper |

A trajectory planned for this arm should stay at or under 15 deg/s on every joint (0.26 rad/s):
it passes everywhere with 0.7 deg to spare on the wrist and 1 deg on the base. 17.5 deg/s still
passes on every joint, but with 0.3 deg to spare on the wrist, and 20 aborts on the wrist before
the servos have started (the base takes 20 with 0.15 deg left and aborts at 22.5). So: 15 is the
working limit for MoveIt's joint velocity limits (sprint 3), 17.5 the most to ask of this arm, 20
the abort. For the 3 deg of the issue at every waypoint, 10 deg/s on the base. The wrist joints
4 and 5 are above 3 deg already at 10.
