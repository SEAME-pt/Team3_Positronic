# Pose age on the arm (#120)

The driver publishes `driver/pose_age_s`: how old the pose of `/joint_states` is, in seconds, from
the arrival of the serial reply that carried it to the `read()` that published it. This run
records it for 60 s with the arm moving under the `joint_trajectory_controller` and gives the
min, mean and max that go in the README of `positronic_hardware` (section "Pose age").

| File | What it is |
|---|---|
| `trajectories/moving_60s.yaml` | J1 10 deg out and back in 4 s legs (2.5 deg/s), seven times, 60 s in all. |
| `tools/pose_age_stats.py` | Subscribes to `/dynamic_joint_states` for 60 s and prints the Markdown table of the ages, in ms. |
| `tools/test_pose_age_stats.py` | Its unit tests, no ROS needed: `python3 -m pytest docs/03_verification/pose_age/tools`. |

## Safety

The arm moves. Same protocol as `jtc_on_the_arm`:

1. Two people. One at the keyboard, one with a hand on the power plug, in place before the
   launch starts: the controller is active as soon as the driver is up.
2. Bench clear in the reach of the arm.
3. Pull the plug on wrong motion: more than 10 deg, another joint moving, or not stopping at
   the last point.
4. At the end, `ros2 control set_controller_state arm_controller inactive` before closing the
   launch.

## How to run

Terminal 1, from the repository on the laptop (`NANO` defaults to the `nano` ssh host). The
build must finish without errors: `nano-shell` only syncs the sources, so without it the
Jetson runs the old plugin and every age reads NaN.

```bash
make nano-build
make nano-shell
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 launch positronic_bringup driver.launch.py
```

Terminal 2, inside the same container. Check the interface is there, bring the arm to the
start pose, then start the recording:

```bash
ssh -t nano docker exec -it positronic-arm bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 topic echo --once /dynamic_joint_states   # "driver" has fault_code and pose_age_s
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "$(cat docs/03_verification/jtc_on_the_arm/trajectories/T0_to_start.yaml)"
python3 docs/03_verification/pose_age/tools/pose_age_stats.py --duration 60 \
  --csv /ws/log/pose_age.csv
```

Terminal 3, right after the recording started:

```bash
ros2 action send_goal --feedback /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "$(cat docs/03_verification/pose_age/trajectories/moving_60s.yaml)"
```

The goal lasts 60 s, as the recording. The table the tool prints goes in the README of
`positronic_hardware`, with the date and the commit of the driver. The CSV is evidence for the
PR, not for the repository.

## What to expect

The serial thread publishes a pose at the end of every turn that got one: while the arm moves
that is the end of the sync-write, one turn per controller write (33 ms at 30 Hz, #170). The
controller's `read()` comes at its own 30 Hz, not in step with the thread, so the age of a
fresh pose spreads between ~0 and one turn. Ages well above one turn are lost or late replies:
the pose then waits for the next turn, or for a `get_angles` after it. Near 200 ms the link
timeout (#35) is close.
