# positronic_bringup

Launch files and the controllers config:

- `launch/gazebo.launch.py`: the arm in Gazebo Harmonic with ros2_control (#26).
- `launch/mirror.launch.py`: the real arm mirrored in RViz on a laptop (#24).
- `launch/driver.launch.py`: the real arm driver on the Jetson (#23).
- `launch/supervised.launch.py`: the driver under the safety supervisor (#45).
- `launch/camera.launch.py`: the real wrist camera on the Jetson, on the Gazebo topics (#138).

Files:

- `positronic_description/urdf/positronic_gazebo.urdf.xacro` (in the description package): includes
  the arm URDF and adds the `ros2_control` tag (6 joints, position command and state) and the
  Gazebo plugin. The Gazebo launch reads it from there.
- `config/ros2_controllers.yaml`: controller_manager at 100 Hz, `joint_state_broadcaster` and
  `arm_controller` (a `JointTrajectoryController`). Same file on the real arm: the hardware
  plugin is in the xacro, not here.
- `config/safety_supervisor.yaml`: parameters of the safety supervisor in
  `supervised.launch.py` (#45); what each one does is in the `positronic_safety` README.
- `rviz/mirror.rviz`: RViz config of the mirror launch: grid, robot model, TF, the wrist camera
  image (compressed), fixed frame `world`.
- `scripts/joint_states_delay.py`: prints every second the number of `/joint_states` messages, the
  average and the maximum delay, as laptop time minus the message stamp.
- `scripts/wrist_camera.py`: the camera node of `camera.launch.py`: reads the USB camera with
  OpenCV, turns the frame upright and publishes image, `camera_info` and the jpeg.
- `scripts/thermal_park.py`: parks the arm at its rest pose before the over-temperature cut;
  started by `driver.launch.py` (see Park before the over-temperature cut).
- `config/wrist_camera.yaml`: the camera calibration (`camera_calibration`'s `ost.yaml`), read by
  `camera.launch.py`. Without it `camera_info` carries only the image size.

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

## Real arm under the safety supervisor

`supervised.launch.py` (#45) is `driver.launch.py` with `start_inactive:=true`, plus the safety
supervisor of `positronic_safety`, configured and left inactive. The driver reads the arm and
publishes `/joint_states`, but the hardware component and `arm_controller` are inactive:
nothing can move the arm until a person activates the supervisor. After any halt the
supervisor is inactive again, and the same command resumes. What it watches, its parameters
(`config/safety_supervisor.yaml`) and its limits are in the `positronic_safety` README.

Without the arm, in the dev container (`backend:=fake` opens no serial port):

```bash
ros2 launch positronic_bringup supervised.launch.py backend:=fake
```

In a second terminal:

```bash
ros2 control list_hardware_components     # positronic_arm: inactive
ros2 control list_controllers             # arm_controller inactive, joint_state_broadcaster active
ros2 run positronic_mission mission_node &
ros2 lifecycle set /mission_node configure && ros2 lifecycle set /mission_node activate
ros2 lifecycle set /safety_supervisor activate
ros2 topic echo /safety_supervisor/status --once          # mode: 2, the arm may move
ros2 service call /safety_supervisor/stop std_srvs/srv/Trigger
ros2 topic echo /safety_supervisor/status --once          # mode: 0, reason: 1
```

The numbers of `mode` and `reason` are in `positronic_interfaces/msg/SafetyStatus.msg`. The
supervisor refuses to activate without the heartbeat of an active mission node.

On the arm, activating the supervisor powers the servos on (they hold their pose) and lets
`arm_controller` take goals: do it with the bench clear, one person at the arm with a hand on
the power switch. The procedure of the fault-injection tests is in
`docs/03_verification/safety_supervisor/`.

`driver.launch.py` alone still starts everything active, with no supervisor: for bench tests
only.

## Park before the over-temperature cut

`driver.launch.py` starts `thermal_park.py` next to the driver, so every way of bringing the
arm up (`supervised.launch.py` included) has it, with no terminal of its own, and it reads the
temperatures on the same machine as the driver.

- It reads the servo temperatures the driver publishes on `/diagnostics`
  (`temperature_c/<joint>`, once a second).
- When one servo stays at `park_warn_c` (63 degC) for 1 s, it publishes one trajectory on
  `/arm_controller/joint_trajectory` to the rest pose, at 0.078 rad/s (0.3 of the working
  limit of `arm_controller`). J1 and J6 stay where they are. **No collision check**: keep the
  path to the rest pose clear.
- Once the arm is there (or after the move time plus 5 s), it calls `/safety_supervisor/stop`:
  the arm holds the pose and takes no goal until it is activated again, which the driver allows
  only under the cut minus 5 degC. With `driver.launch.py` alone there is no supervisor: the
  arm stays parked, active, and it warns.
- One park per hot spell: again only after every servo went under `park_rearm_below_c`
  (60 degC).
- The driver's cut (65 degC, `servo_temp_limit_c`) stays the last line of defence. The rest
  pose is folded so the arm stays put when the servos lose their torque.

To try it on a cool arm, lower both, the park under the cut:

```bash
ros2 launch positronic_bringup driver.launch.py servo_temp_limit_c:=55 park_warn_c:=48 park_rearm_below_c:=46
```

`thermal_park:=false` leaves it out; only the driver's cut remains. The rest pose is also the
`rest` group_state of `positronic_moveit_config`, to try the path in RViz.

## Real wrist camera

`camera.launch.py` publishes the USB camera on the wrist on the topics of the Gazebo camera,
`/wrist_camera/image_raw`, `/wrist_camera/camera_info` and `/wrist_camera/image_raw/compressed`,
with `frame_id` `camera_link`: the driver's `robot_state_publisher` puts that frame under joint 6
(`positronic_real.urdf.xacro` includes the camera URDF). The raw image of this camera lies on its
side; the node turns it 90 deg counterclockwise (#97), so 640 x 480 asked of the camera is
published as 480 x 640, gripper at the bottom. Evidence and numbers:
`docs/03_verification/camera_real/`.

**Off by default.** Nothing starts the camera unless asked. With the driver, on the Jetson
(`make nano-shell` maps `/dev/video0` when the camera is plugged in):

```bash
ros2 launch positronic_bringup driver.launch.py camera:=true       # or supervised.launch.py
ros2 launch positronic_bringup driver.launch.py camera:=true width:=320 height:=240
```

The camera's arguments (`device:=`, `width:=`, `height:=`, `fps:=`, ...) pass through. Or alone,
in its own container next to the driver's:

```bash
make nano-build
make nano-camera                                  # /dev/video0; NANO_CAMERA=/dev/videoN otherwise
ros2 launch positronic_bringup camera.launch.py   # width:=, height:=, fps:=, device:=
```

Every 10 s the node logs the rate it achieves (`30.0 fps over 10 s, ... ms per frame`). The jpeg
is only encoded while someone subscribes to `image_raw/compressed`.

**On the laptop**, `ROS_DOMAIN_ID=17`: over Wi-Fi use the compressed topic only (the raw one is
~27 MB/s). `mirror.launch.py` shows it in RViz next to the model;
`ros2 run rqt_image_view rqt_image_view /wrist_camera/image_raw/compressed` shows it alone.

**Calibration** (once, and again if the camera or the size changes). Checkerboard flat on a card,
the arm still. `camera_calibration` reads the raw image: over a cable, give it
`/wrist_camera/image_raw` directly; over Wi-Fi, decode the jpeg on the laptop first and give it
that topic:

```bash
ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw \
  -r in/compressed:=/wrist_camera/image_raw/compressed -r out:=/wrist_camera_local/image_raw
ros2 run camera_calibration cameracalibrator --size <COLSxROWS> --square <m> \
  --ros-args -r image:=/wrist_camera_local/image_raw -p camera:=/wrist_camera
```

`--size` counts the inner corners. Move the board until the four bars are green, CALIBRATE,
SAVE, then copy `ost.yaml` out of `/tmp/calibrationdata.tar.gz` to `config/wrist_camera.yaml`
and run `make nano-build` again. The node refuses a file made for another image size, with a
warning, rather than publish a wrong `camera_info`.

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
| `thermal_park` never parks | No `temperature_c/...` keys on `/diagnostics` (`ros2 topic echo /diagnostics --field status`), or `park_warn_c` at or above `servo_temp_limit_c`: the cut comes first |
| Camera node: `cannot open /dev/video0` | Camera unplugged, another process has it, or another number: `v4l2-ctl --list-devices` on the Jetson, then `make nano-camera NANO_CAMERA=/dev/videoN` and `device:=/dev/videoN` |
| Camera node: `calibration ... not used` | No `config/wrist_camera.yaml` yet, or made for another size than the published one (480 x 640 by default). Calibrate again at that size |
| Wrist camera image black or missing in RViz on the laptop | No `compressed_image_transport` on the laptop (`ros-jazzy-image-transport-plugins`, in the dev image), or a different `ROS_DOMAIN_ID` |
