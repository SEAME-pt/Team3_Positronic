# positronic_bringup

Launch files and the controllers config:

- `launch/gazebo.launch.py`: the arm in Gazebo Harmonic with ros2_control (#26).
- `launch/mirror.launch.py`: the real arm mirrored in RViz on a laptop (#24).

Files:

- `positronic_description/urdf/positronic_gazebo.urdf.xacro` (in the description package): includes
  the arm URDF and adds the `ros2_control` tag (6 joints, position command and state) and the
  Gazebo plugin. The Gazebo launch reads it from there.
- `config/ros2_controllers.yaml`: controller_manager at 100 Hz, `joint_state_broadcaster` and
  `arm_controller` (a `JointTrajectoryController`). Same file on the real arm: the hardware
  plugin is in the xacro, not here.
- `rviz/mirror.rviz`: RViz config of the mirror launch: grid, robot model, fixed frame `world`.
- `scripts/joint_states_delay.py`: prints every second the number of `/joint_states` messages, the
  average and the maximum delay, as laptop time minus the message stamp.

## Arm in Gazebo

In the dev container (`make build`, then `make shell`):

```bash
source install/setup.bash
ros2 launch positronic_bringup gazebo.launch.py
```

On your machine, with ROS 2 Jazzy, `ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control` and
`ros-jazzy-ros2-controllers` installed, build outside the repo: `build/` and `install/` in the repo
belong to the container, and the two do not mix (CMake caches the paths).

```bash
cd <repo root>
colcon build --symlink-install --packages-select positronic_description positronic_bringup \
  --build-base ../Positronic_host/build --install-base ../Positronic_host/install
source ../Positronic_host/install/setup.bash
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

## Real arm in RViz

The Jetson driver publishes `/joint_states` (#23), this launch only draws it. With the driver running
on `ROS_DOMAIN_ID=17`:

```bash
ros2 launch positronic_bringup mirror.launch.py
```

Release the servos, move the arm by hand, and the model follows. Every joint must turn the same way
as the arm: a mirrored joint is a sign error in the driver or in the URDF. In a second terminal:

```bash
ros2 run positronic_bringup joint_states_delay.py
```

Each line is one second: messages received, average and maximum delay in ms. The delay is only
valid if the laptop and the Jetson clocks are synchronised: check with `timedatectl` on both (NTP
active and synchronised). Without the driver, the same launch can be fed by a simulation or by
`joint_state_publisher_gui`, but not by two of them at once.

## Troubleshooting

| Problem | Cause and fix |
| --- | --- |
| Real-time factor below 0.9 | Power profile too low. `powerprofilesctl set performance` (balanced is enough with the charger plugged in) |
| Controllers `unconfigured` or spawner times out | Gazebo took more than 10 s to load the arm. Launch again, or close other heavy programs |
| Arm has no meshes in Gazebo | Gazebo cannot find `package://positronic_description`. The launch sets `GZ_SIM_RESOURCE_PATH`; check `positronic_description` is built and sourced |
| `package 'positronic_bringup' not found` | Build the package and source the matching `install/setup.bash` (container or `../Positronic_host`) |
| CMake error `CMakeCache.txt directory ... is different` | A build made in the container is being reused on the host, or the reverse. Use the two build folders above |
| Model in RViz shows only the base, white | No `/joint_states` arriving: driver not running, or a different `ROS_DOMAIN_ID`. `ros2 topic hz /joint_states` |
| Model in RViz has no meshes | RViz cannot find `package://positronic_description`: `source install/setup.bash` before `rviz2` |
| Delay in the billions of ms | The stamp is zero or in simulation time: driver not stamping, or a simulation publishing `/joint_states` (close Gazebo) |
| Model jumps around | Two publishers on `/joint_states`: close `joint_state_publisher_gui` or the simulation |
