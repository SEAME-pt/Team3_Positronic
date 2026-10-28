# positronic_hardware

The ros2_control hardware interface of the myCobot 280 (Jetson Nano): the plugin
`positronic_hardware/RobotSystem` (#22).

It reads the six joint angles from the arm over the serial port and publishes, for each joint,
a `position` (rad) and a `velocity` (rad/s) state interface. **It is read-only:** it has no
command interface and `write()` sends nothing, so it cannot move the arm. Commands come in #32.

## How it fits together

```
controller_manager ── read() ──> RobotSystem (plugin)
                                    │  read_angles(), has_sample(): copy only, never blocks
                                    ▼
                                 ArmBackend
                                 ├─ FakeArm      fixed pose, no port (tests, CI)
                                 └─ SerialArm    own thread: ask, decode, publish
                                       │            through a realtime_tools::RealtimeBuffer
                                       ▼
                                 ReplyAssembler  bytes -> get_angles reply (positronic_protocol)
                                       │
                                 SerialTransport
                                 └─ PosixSerialPort   /dev/ttyTHS1, raw 8N1, exclusive
```

- The serial thread sends `get_angles`, waits up to 0.1 s for the reply, keeps the newest pose,
  sleeps 10 ms, and starts again.
- `read()` copies the newest pose out of the buffer and runs the velocity filter. It does no
  serial I/O, no allocation, no logging and throws nothing.

## Use it in a URDF

The `<ros2_control>` tag lists the six joints in protocol order, J1 (base) to J6 (flange), each
with `position` and `velocity` state interfaces and **no** command interface. `on_init` refuses
anything else.

```xml
<ros2_control name="PositronicArm" type="system">
  <hardware>
    <plugin>positronic_hardware/RobotSystem</plugin>
    <param name="backend">serial</param>
    <param name="port">/dev/ttyTHS1</param>
  </hardware>
  <joint name="joint2_to_joint1">
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- the same for joint3_to_joint2, joint4_to_joint3, joint5_to_joint4,
       joint6_to_joint5 and joint6output_to_joint6 -->
</ros2_control>
```

The Gazebo xacro in `positronic_description` declares a `position` command interface, so it
cannot be reused as is for the real arm.

## Parameters

All optional. An unknown name (a typo) or a bad value makes `on_init` fail with a message that
says which one.

| Name                    | Default        | Accepted values      | Meaning                          |
|-------------------------|----------------|----------------------|----------------------------------|
| `backend`               | `serial`       | `serial`, `fake`     | Where the angles come from       |
| `port`                  | `/dev/ttyTHS1` | any non-empty path   | Serial device                    |
| `baud`                  | `1000000`      | `115200`, `1000000`  | Baud rate                        |
| `velocity_filter_alpha` | `0.3`          | (0, 1]               | Low-pass weight of the velocity: closer to 1 follows faster but is noisier |

The default backend is `serial` on purpose: a URDF that forgets the parameter on the robot fails
loudly if the port cannot be opened, instead of silently showing a fake arm that stands still.

## Lifecycle

| Transition  | What happens                                                                     |
|-------------|----------------------------------------------------------------------------------|
| `on_init`   | Checks the joints and the parameters.                                             |
| `on_configure` | Creates the backend. `serial` opens and locks the port and starts the thread; fails if the port cannot be opened. States are set to NaN ("unknown"). |
| `read()` (inactive or active) | Publishes position and velocity once the arm has answered; until then the states stay NaN. |
| `on_activate` | Waits up to 1 s for the arm's first reply and fails if none comes ("Is it powered and on /dev/ttyTHS1?"). |
| `on_cleanup`, `on_shutdown` | Destroys the backend: stops the thread, unlocks and closes the port. |

The port is opened with `TIOCEXCL`: while the driver holds it, any other program that opens it
gets "Device or resource busy". Close a `screen`/`minicom` on the port before starting the driver.
Root is not stopped by this lock.

## Tests

```bash
make build
make test
```

No arm is needed:

| Test                       | What it checks                                                      |
|----------------------------|---------------------------------------------------------------------|
| `test_load_robot_system`   | Loads the plugin through a `ResourceManager` like the controller_manager does: joints, interfaces, parameters, lifecycle; a missing port; a silent arm on a pseudo-terminal (NaN states, activation refused). |
| `test_serial_arm`          | `SerialArm` replaying UART bytes recorded on the real arm; `PosixSerialPort` exclusivity on a pseudo-terminal. |
| `test_reply_assembler`     | Split replies, servo-bus noise, request echo, newest reply wins, `clear()`. |
| `test_velocity_estimator`  | The velocity filter: first sample, constant rate, gaps, `reset()`.  |

Passing tests do not prove the driver is right on the arm. On the robot, check that activation
succeeds, that the first published pose is the real one, and that the velocity looks sane while
the arm is moved by hand (`release_all_servos`, see `positronic_protocol`).

## Known limits

- The velocity uses the control loop `period` as dt, not the time between two serial samples.
  When the controller runs faster than the arm answers (about every 16 ms), some `read()` calls
  see the same pose twice and the velocity is underestimated.
- A read error and a read timeout look the same, and samples carry no timestamp, so a link that
  stops answering after activation is not detected yet.

## Files

- `include/positronic_hardware/positronic_hardware.hpp`, `src/positronic_hardware.cpp`: the plugin.
- `arm_backend.hpp`: the `ArmBackend` interface, `NUM_JOINTS`, `JointPositions`.
- `fake_arm.hpp`: `FakeArm`.
- `serial_arm.hpp`, `src/serial_arm.cpp`: `SerialArm` and `PosixSerialPort`.
- `serial_transport.hpp`: the `SerialTransport` interface.
- `reply_assembler.hpp`: `ReplyAssembler`.
- `velocity_estimator.hpp`: `VelocityEstimator`.
- `positronic_hardware.xml`: registers the plugin with pluginlib.
