# positronic_hardware

The ros2_control hardware interface of the myCobot 280 (Jetson Nano): the plugin
`positronic_hardware/RobotSystem` (#22).

It reads the six joint angles from the arm over the serial port and publishes, for each joint,
a `position` (rad) and a `velocity` (rad/s) state interface. It takes a `position` command
(rad) per joint and sends the six targets to the arm as one `send_angles` frame at 30 % of the
firmware speed (#32). Deactivating the component stops the arm and holds it; activating it
powers the servos on and resumes (#36).

## How it fits together

```
controller_manager ── read() / write() ──> RobotSystem (plugin)
                                              │  read_angles(), write_angles(), has_sample():
                                              │  copy in or out of a buffer, never blocks
                                              ▼
                                           ArmBackend
                                           ├─ FakeArm      at every target at once, no port (tests, CI)
                                           └─ SerialArm    own thread: ask, decode, publish, send
                                                 │            two realtime_tools::RealtimeBuffer:
                                                 │            newest pose out, newest command in
                                                 ▼
                                           ReplyAssembler  bytes -> get_angles reply (positronic_protocol)
                                                 │
                                           SerialTransport
                                           └─ PosixSerialPort   /dev/ttyTHS1, raw 8N1, exclusive
```

- One turn of the serial thread: a pending stop or power on first, if any; then `get_angles`,
  up to 50 ms for the reply; then, if the arm answered and there is a new target, one
  `send_angles` and a settle of `send_settle_ms`. A turn lasts 10 ms while the arm stands still
  and `moving_period_ms` after a send. The arm drops a frame that arrives while it is still
  answering, so nothing is sent before the reply.
- `read()` copies the newest pose out of the buffer and runs the velocity filter. `write()`
  copies the six commands into the buffer. Neither does serial I/O, allocates, logs or throws.
- A target equal to the last one sent is not sent again. A target with a NaN or an angle out
  of the protocol range is skipped and the previous one stays.

## Use it in a URDF

The `<ros2_control>` tag lists the six joints in protocol order, J1 (base) to J6 (flange), each
with `position` and `velocity` state interfaces and exactly one `position` command interface.
`on_init` refuses anything else. `positronic_description/urdf/positronic_real.urdf.xacro` is
the real one, with the parameters as xacro arguments.

```xml
<ros2_control name="PositronicArm" type="system">
  <hardware>
    <plugin>positronic_hardware/RobotSystem</plugin>
    <param name="backend">serial</param>
    <param name="port">/dev/ttyTHS1</param>
  </hardware>
  <joint name="joint2_to_joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- the same for joint3_to_joint2, joint4_to_joint3, joint5_to_joint4,
       joint6_to_joint5 and joint6output_to_joint6 -->
</ros2_control>
```

The Gazebo xacro in `positronic_description` loads `gz_ros2_control/GazeboSimSystem` instead
of this plugin, so it cannot be reused as is for the real arm.

## Parameters

All optional. An unknown name (a typo) or a bad value makes `on_init` fail with a message that
says which one.

| Name                    | Default        | Accepted values      | Meaning                          |
|-------------------------|----------------|----------------------|----------------------------------|
| `backend`               | `serial`       | `serial`, `fake`     | Where the angles come from       |
| `port`                  | `/dev/ttyTHS1` | any non-empty path   | Serial device                    |
| `baud`                  | `1000000`      | `115200`, `1000000`  | Baud rate                        |
| `velocity_filter_alpha` | `0.3`          | (0, 1]               | Low-pass weight of the velocity: closer to 1 follows faster but is noisier |
| `moving_period_ms`      | `50`           | 20 to 500            | One serial turn after a `send_angles`: a `get_angles` and the send. Measured: 50 held five 2-minute runs with no stall, 40 stalled once |
| `send_settle_ms`        | `30`           | 0 to 200, below `moving_period_ms` | Wait after a `send_angles` before the next `get_angles`: the arm ignores the port while it passes the command to the servos. Measured: 30 gave 0 stalls in 4464 requests, 25 gave 43, 20 froze the arm once for 20 s |

The default backend is `serial` on purpose: a URDF that forgets the parameter on the robot fails
loudly if the port cannot be opened, instead of silently showing a fake arm that stands still.

## Lifecycle

The component is not a ROS node: its state changes with
`ros2 control set_hardware_component_state positronic_arm <state>`, not with `ros2 lifecycle`.

| Transition  | What happens                                                                     |
|-------------|----------------------------------------------------------------------------------|
| `on_init`   | Checks the joints and the parameters.                                             |
| `on_configure` | Drops a backend left over from an earlier error, then creates a new one. `serial` opens and locks the port and starts the thread; fails if the port cannot be opened. States are set to NaN ("unknown"). |
| `read()` (inactive or active) | Publishes position and velocity once the arm has answered; until then the states stay NaN. |
| `on_activate` | Waits up to 1 s for the arm's first reply and fails if none comes ("Is it powered and on /dev/ttyTHS1?"). Then sends `power_on`, so the servos hold their pose, and fails if it did not leave the port within 300 ms. Every command starts at the measured pose: the first thing a controller sees is "stay where you are". |
| `write()` (active only) | Hands the newest command to the serial thread. Returns at once when the component is not active, so no command reaches the arm while inactive. |
| `on_deactivate` | Sends a stop: the arm halts where it is, and the serial thread holds every target until the next activate. Returns ERROR if the stop did not leave the port within 300 ms, because a component that could not stop the arm must not stay active. |
| `on_error` | Reached from any transition that returned ERROR, and from a `write()` that returned ERROR, on the control loop. Asks the serial thread for a stop without waiting, keeps the backend alive (it holds every target and keeps reading the pose) and takes the component to UNCONFIGURED, so `configure` and `activate` bring it back. It never joins the thread on the control loop. |
| `on_cleanup`, `on_shutdown` | Destroys the backend: stops the thread, unlocks and closes the port. |

**Stopping and resuming from a supervisor.** To stop: set the hardware component inactive
first (the stop leaves at once), then deactivate the controllers. To resume: set the component
active first, then activate the controllers. In the other order the controllers rewrite their
old target as soon as the component is active, and the arm moves towards it. Measured on the
arm, 25 runs: inactive in 6 to 16 ms and active in 4 to 15 ms, service round trip included; a
move at 30 % speed stops about 8 degrees after the request; nothing moves after activate.

The port is opened with `TIOCEXCL`: while the driver holds it, any other program that opens it
gets "Device or resource busy". Close a `screen`/`minicom` on the port before starting the driver.
Root is not stopped by this lock. After an error the port stays held until the next configure or
cleanup.

## Tests

```bash
make build
make test
```

No arm is needed:

| Test                       | What it checks                                                      |
|----------------------------|---------------------------------------------------------------------|
| `test_load_robot_system`   | Loads the plugin through a `ResourceManager` like the controller_manager does: joints, interfaces, parameters, lifecycle; a missing port; a silent arm on a pseudo-terminal (NaN states, activation refused, the component configurable again); deactivate holds the commands and activate resumes them from the measured pose. |
| `test_serial_arm`          | `SerialArm` replaying UART bytes recorded on the real arm: the pose, the send_angles frame, the stop and the power on (the stop goes out and holds every target, a command stored before it never goes out, `power_on()` sends once, `request_halt()` returns at once, a port that refuses writes, a silent arm); `PosixSerialPort` exclusivity on a pseudo-terminal. |
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
- `halt()` and `power_on()` report true when their frame is on the wire, not when the firmware
  acknowledged it (#103). With a silent arm a serial turn can last about 150 ms, so a stop can
  take that long to reach the port; the 300 ms deadline allows two such turns.

## Files

- `include/positronic_hardware/positronic_hardware.hpp`, `src/positronic_hardware.cpp`: the plugin.
- `arm_backend.hpp`: the `ArmBackend` interface, `NUM_JOINTS`, `JointPositions`.
- `fake_arm.hpp`: `FakeArm`.
- `serial_arm.hpp`, `src/serial_arm.cpp`: `SerialArm` (`SerialTiming`, `POLL_PERIOD`, `SEND_SPEED`, `REPLY_TIMEOUT`, `REQUEST_TIMEOUT`) and `PosixSerialPort`.
- `serial_transport.hpp`: the `SerialTransport` interface.
- `reply_assembler.hpp`: `ReplyAssembler`.
- `velocity_estimator.hpp`: `VelocityEstimator`.
- `positronic_hardware.xml`: registers the plugin with pluginlib.
