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

Gazebo opens with the arm standing on the bench of `worlds/workcell.sdf`. The launch ends when
you close Gazebo. `world:=<file>` loads another world from `worlds/`. The wrist camera of the URDF is bridged to
`/wrist_camera/image_raw` and `/wrist_camera/camera_info` (`ros2 run rqt_image_view
rqt_image_view` shows it). The five finger joints of the gripper follow `gripper_controller`:
they are listed in `positronic_gazebo.urdf.xacro` with no command interface, which is what makes
`gz_ros2_control` drive them from the URDF mimic tags. In a second terminal, after
`source install/setup.bash`:

```bash
ros2 control list_controllers      # the three must say "active"
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

The Jetson driver publishes `/joint_states` (#23); `mirror.launch.py` only draws it on a laptop.
Evidence of the run on the real arm: `docs/03_verification/rviz_mirror/`.

**On the Jetson** (from the laptop, with the `nano` alias of `docs/build.md`):

```bash
make nano-build        # sync this repo to the Jetson and build it there
make nano-shell        # a shell in the container, with /dev/ttyTHS1
```

Inside that shell, with a hand on the arm, because it gives when the servos release:

```bash
python3 -c "from pymycobot import MyCobot280; m = MyCobot280('/dev/ttyTHS1', 1000000); print(m.get_angles()); m.release_all_servos()"
ros2 launch positronic_bringup driver.launch.py
```

The six angles printed, in degrees, are the reference for the radians check. The launch must end
with `Successful 'activate' of hardware` and `Configured and activated joint_state_broadcaster`.
Do it in this order: the driver locks the port, so `pymycobot` cannot read once it runs.

**On the laptop**, same network, native ROS 2 Jazzy, built outside the repo as above:

```bash
export ROS_DOMAIN_ID=17            # the Jetson container uses 17 (Makefile)
ros2 topic hz /joint_states        # 50 Hz expected
ros2 topic echo --once /joint_states   # six URDF names, positions = reference in radians
ros2 launch positronic_bringup mirror.launch.py
```

RViz opens with the model in the pose of the real arm. Move the joints by hand, one at a time
from the base to the flange: the model must follow and turn the same way. A mirrored joint is a
sign error in the driver or in the URDF. In a second terminal, same `ROS_DOMAIN_ID`:

```bash
ros2 run positronic_bringup joint_states_delay.py --duration 300 --out log/delay
```

Each line is one second: messages received, average and maximum delay in ms, from the header
stamp of each message to its arrival here. After `--duration` seconds (or Ctrl+C) it prints the
summary of the run: messages, average, worst delay and when, how many were above `--limit-ms`
(100 by default, the #24 criterion), and PASS or FAIL. `--out` keeps `delays.csv`, one line per
message, and `summary.txt`. The number is only valid if both machines have NTP active and
synchronised: `timedatectl` on the laptop and `ssh nano timedatectl`. Start the measurement once
everything is running. Over wifi the model follows, but the delay depends on the day: the
laboratory wifi goes from a few ms to hundreds of ms of ping, and a 2 s burst above 100 ms is
enough to fail the criterion. Measure over a cable.

Expected: two `robot_state_publisher` nodes publish `/tf` for the same links (the driver's and
the mirror's). RViz warns and tolerates it. Without the driver, the same launch can be fed by
a simulation or by `joint_state_publisher_gui`, but not by two of them at once.

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
