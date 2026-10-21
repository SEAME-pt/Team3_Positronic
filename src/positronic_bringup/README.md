# positronic_bringup

Launch files and the controllers config:

- `launch/gazebo.launch.py`: the arm in Gazebo Harmonic with ros2_control (#26).

Files:

- `urdf/positronic_gazebo.urdf.xacro`: includes the arm URDF from `mycobot_description` and adds
  the `ros2_control` tag (6 joints, position command and state) and the Gazebo plugin. Moves to
  `positronic_description` after #25 lands.
- `config/ros2_controllers.yaml`: controller_manager at 100 Hz, `joint_state_broadcaster` and
  `arm_controller` (a `JointTrajectoryController`). Same file on the real arm: the hardware
  plugin is in the xacro, not here.

## Arm in Gazebo

Until the dev image has `ros_gz` and `gz_ros2_control`, this runs on your machine, not in the
container. You need ROS 2 Jazzy with `ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control`
and `ros-jazzy-ros2-controllers` installed.

```bash
cd <repo root>
colcon build --symlink-install --packages-select mycobot_description positronic_bringup
source install/setup.bash
ros2 launch positronic_bringup gazebo.launch.py
```

Gazebo opens with the arm standing in an empty world. The launch ends when you close Gazebo.
In a second terminal, after `source install/setup.bash`:

```bash
ros2 control list_controllers      # both must say "active"
```

A 3-point trajectory, 9 s, through the action the MoveIt side will use:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {
  joint_names: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3,
  joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6],
  points: [
  {positions: [0.8, -0.6, 0.9, 0.3, 0.5, 0.0], time_from_start: {sec: 3}},
  {positions: [-0.8, 0.4, -0.5, -0.3, -0.5, 1.0], time_from_start: {sec: 6}},
  {positions: [0, 0, 0, 0, 0, 0], time_from_start: {sec: 9}}]}}"
```

It must end with `Goal finished with status: SUCCEEDED`. The real-time factor is in the
bottom right of the Gazebo window, or in the terminal with `gz topic -e -t /stats`. The
launch log is in `~/.ros/log/<date>/launch.log`, one line per process, with time stamps.

## Troubleshooting

| Problem | Cause and fix |
| --- | --- |
| Real-time factor below 0.9 | Power profile too low. `powerprofilesctl set performance` (balanced is enough with the charger plugged in) |
| Controllers `unconfigured` or spawner times out | Gazebo took more than 10 s to load the arm. Launch again, or close other heavy programs |
| Arm has no meshes in Gazebo | Gazebo cannot find `package://mycobot_description`. The launch sets `GZ_SIM_RESOURCE_PATH`; check `mycobot_description` is built and sourced |
| `package 'positronic_bringup' not found` when running xacro | Build the package and `source install/setup.bash` |
