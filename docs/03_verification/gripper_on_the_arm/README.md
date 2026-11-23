# Gripper on the real arm (#123)

The adaptive gripper opening and closing on the real arm through the same `GripperCommand`
action as in Gazebo and MoveIt (`gripper_action_controller`). Three checks: the frames recorded
with pymycobot match the library, 15 action goals on the arm, and open and closed from RViz.
The mapping and the tolerance are in the `positronic_hardware` README, section Gripper.

## Setup

- Arm with the gripper and the camera mounted.
- The driver on the Jetson, `driver.launch.py` with the serial backend.
- `gripper_action_controller` active (`ros2 control list_controllers`), `goal_tolerance` and
  `allow_stalling` as in `jetson_controllers.yaml`.

## Safety protocol

1. Two people. One at the keyboard, one with a hand on the power plug.
2. Nothing between the fingers of the gripper, and no hand near them while a goal runs.
3. The arm stands still: only the gripper is commanded. Pull the plug if a joint of the arm
   moves.
4. Only the commands of this page: `get_gripper_value` and `set_gripper_value` (0 to 100,
   speed 20) for the recording, `GripperCommand` goals inside the URDF limits for the runs.

## 1. Recorded frames

On the Jetson host (system pymycobot), with the driver stopped, since the port is exclusive.
`capture_gripper.py` imports `capture_frames.py`, so both go to the same folder:

```bash
scp scripts/capture_frames/capture_frames.py scripts/capture_frames/capture_gripper.py <user@jetson>:~/captures/
ssh <user@jetson>
cd ~/captures && python3 capture_gripper.py --operator <name> --arm <label>
```

Sequence: 5 reads, then open (100), half (50), closed (0) and open again (100), each followed by
3 reads. Every move waits for Enter; `q`, Ctrl+C or any error sends a stop.

Then, on the laptop:

```bash
scp <user@jetson>:~/captures/frames_gripper_*.txt src/positronic_protocol/test/data/
make build && make test
```

`test_recorded_gripper` checks that every frame sent equals the encoder output and that the
decoder finds in the raw RX the value pymycobot returned.

| File | Date | Operator | Arm | `test_recorded_gripper` |
|---|---|---|---|---|
| `frames_gripper_20261123_063722.txt` | 2026-11-23 | none | elephant | 5 of 5 passed |

Values read back (`get_gripper_value`): 0 at the start, 99 after open (100), 50 after half,
0 after closed, 99 after open again. Open stops one step short, as in the action goals below.

## 2. Action goals on the arm

Terminal 1, the driver:

```bash
NANO=<user@jetson> make nano-build
NANO=<user@jetson> make nano-shell
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 launch positronic_bringup driver.launch.py
```

Terminal 2, inside the same container, one goal per run:

```bash
ssh -t <user@jetson> docker exec -it positronic-arm bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.15}}"
```

Targets in turn, so every goal moves the gripper: open `0.15`, closed `-0.74`, half `-0.295`.
Write down the status and the `position`, `stalled` and `reached_goal` of the result. Error =
result position - target.

| Run | Target | Target (rad) | Status | Position (rad) | Error (rad) | stalled | reached_goal |
|---|---|---|---|---|---|---|---|
| 1 | open | 0.15 | SUCCEEDED | 0.1411 | -0.0089 | false | true |
| 2 | closed | -0.74 | SUCCEEDED | -0.74 | 0 | false | true |
| 3 | half | -0.295 | SUCCEEDED | -0.3039 | -0.0089 | false | true |
| 4 | open | 0.15 | SUCCEEDED | 0.1411 | -0.0089 | false | true |
| 5 | closed | -0.74 | SUCCEEDED | -0.74 | 0 | false | true |
| 6 | half | -0.295 | SUCCEEDED | -0.3039 | -0.0089 | false | true |
| 7 | open | 0.15 | SUCCEEDED | 0.1411 | -0.0089 | false | true |
| 8 | closed | -0.74 | SUCCEEDED | -0.74 | 0 | false | true |
| 9 | half | -0.295 | SUCCEEDED | -0.3039 | -0.0089 | false | true |
| 10 | open | 0.15 | SUCCEEDED | 0.1411 | -0.0089 | false | true |
| 11 | closed | -0.74 | SUCCEEDED | -0.74 | 0 | false | true |
| 12 | half | -0.295 | SUCCEEDED | -0.3039 | -0.0089 | false | true |
| 13 | open | 0.15 | SUCCEEDED | 0.1411 | -0.0089 | false | true |
| 14 | closed | -0.74 | SUCCEEDED | -0.7311 | +0.0089 | false | true |
| 15 | half | -0.295 | SUCCEEDED | -0.3039 | -0.0089 | false | true |

Largest error per target:

| Target | Largest error (rad) | Firmware value reached |
|---|---|---|
| open | 0.0089 | 99 for 100 |
| closed | 0.0089 | 0, once 1 |
| half | 0.0089 | 49 for 50 (-0.295 rad is 49.4) |

15 of 15 goals `SUCCEEDED` with `reached_goal: true` and no stall. Every error is at most one
firmware step (0.0089 rad), inside the default `goal_tolerance` of 0.01 rad.

## 3. Open and closed from RViz

With the driver of section 2 running, on the laptop (`make shell`, same `ROS_DOMAIN_ID`):

```bash
ros2 launch positronic_moveit_config move_group.launch.py
```

In the MotionPlanning panel: Planning Group `gripper`, Goal State `open`, Plan & Execute; then
`closed`, Plan & Execute.

| Goal state | Executed | Real gripper moved |
|---|---|---|
| open | yes | yes |
| closed | yes | yes |
