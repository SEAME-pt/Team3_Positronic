# positronic_hardware

The ros2_control hardware interface of the myCobot 280 (Jetson Nano): the plugin
`positronic_hardware/RobotSystem` (#22).

It reads the six joint angles from the arm over the serial port and publishes, for each joint,
a `position` (rad) and a `velocity` (rad/s) state interface. It takes a `position` command
(rad) per joint and sends the six targets to the arm as one `send_angles` frame at 30 % of the
firmware speed (#32), after checking them against the joint limits of the URDF (#34): a
command past a limit, NaN or with the wrong joint count is refused and the arm halts.
Deactivating the component stops the arm and holds it; activating it powers the servos on and
resumes (#36).

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
  checks the six commands against the joint limits and copies them into the buffer. Neither
  does serial I/O, allocates or throws; the only log is the one line of a refused command.
- A target equal to the last one sent is not sent again. A target with a NaN or an angle out
  of the protocol range is skipped and the previous one stays.

## Use it in a URDF

The `<ros2_control>` tag lists the six joints in protocol order, J1 (base) to J6 (flange), each
with `position` and `velocity` state interfaces and exactly one `position` command interface.
`on_init` refuses anything else. Each URDF joint needs a revolute `<limit>`: the plugin reads it. `positronic_description/urdf/positronic_real.urdf.xacro` is
the real one, with the parameters as xacro arguments.

```xml
<ros2_control name="PositronicArm" type="system">
  <hardware>
    <plugin>positronic_hardware/RobotSystem</plugin>
    <param name="backend">serial</param>
    <param name="port">/dev/ttyTHS1</param>
  </hardware>
  <joint name="joint2_to_joint1">
    <limits enable="false"/>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- the same for joint3_to_joint2, joint4_to_joint3, joint5_to_joint4,
       joint6_to_joint5 and joint6output_to_joint6 -->
  <gpio name="driver">
    <state_interface name="fault_code"/>
  </gpio>
</ros2_control>
```

`<limits enable="false"/>` is part of the joint limit check, see below. A `driver` gpio with
a `fault_code` state interface is required too (other gpios and other `driver` interfaces are
allowed; a `fault_code` command interface is not): it is how the driver reports a fault to
the supervisor (#35, see "Link timeout"). The Gazebo xacro in
`positronic_description` loads `gz_ros2_control/GazeboSimSystem` instead of this plugin, so it
cannot be reused as is for the real arm.

## Joint limits (#34)

The plugin is the last gate before the serial port: MoveIt, a misconfigured controller, a test
script and a bug all pass through `write()`. So every command is checked there, before anything
reaches the arm.

- **Where the limits come from.** The `<limit lower upper>` tags of the URDF, read once in
  `on_init` (`load_limits()`), never a table in the code. A joint without a revolute `<limit>`,
  or with `lower >= upper`, stops the driver at start-up. The six limits are logged
  (`Joint '...' limits: [..., ...] rad.`).
- **What is refused.** `check_command()` (`joint_limit_check.hpp`) goes J1 to J6 and the first
  bad joint wins: a value that is not finite (NaN, inf), below `lower` or above `upper`. Both
  ends are allowed. The count is checked first: anything but six values is refused unread. It
  never clamps: a bad command means a bug upstream, and fixing it silently would hide it.
- **What happens then.** The command is not sent. `write()` asks the backend for a stop
  (`request_halt()`, non-blocking), logs
  `Command refused, arm halted: joint '...' = ... rad (... deg), above upper limit [..., ...] rad.`
  and returns `ERROR`. The controller manager then deactivates the controllers on this
  component, and `on_error()` takes it to UNCONFIGURED (see Lifecycle).
- **Coming back.** `configure` and `activate`, as after any error. `on_activate` refuses an arm
  that already stands outside its limits ("Move the arm inside its limits by hand"), before it
  powers the servos on.
- **Why `<limits enable="false"/>`.** With a command interface and a URDF `<limit>`,
  ros2_control adds its own `JointSaturationLimiter`, which clamps a command to the limit
  before `write()` sees it: the arm would stop at the limit, but nothing would be refused or
  logged. The tag turns that limiter off (the launch log says
  `Joint limits are disabled for joint ...`). It also removes the joint's limits from
  `HardwareInfo::limits`, which is why `load_limits()` parses the URDF itself
  (`info.original_xml`). `on_init` warns if the tag is missing.

## Parameters

All optional. An unknown name (a typo) or a bad value makes `on_init` fail with a message that
says which one.

| Name                    | Default        | Accepted values      | Meaning                          |
|-------------------------|----------------|----------------------|----------------------------------|
| `backend`               | `serial`       | `serial`, `fake`     | Where the angles come from       |
| `port`                  | `/dev/ttyTHS1` | any non-empty path   | Serial device                    |
| `baud`                  | `1000000`      | `115200`, `1000000`  | Baud rate                        |
| `velocity_filter_alpha` | `0.3`          | (0, 1]               | Low-pass weight of the velocity: closer to 1 follows faster but is noisier |
| `moving_period_ms`      | `50`           | 20 to 100            | One serial turn after a `send_angles`: a `get_angles` and the send. Measured: 50 held five 2-minute runs with no stall, 40 stalled once. At most half the 200 ms link timeout (#35), so one lost reply never trips it |
| `send_settle_ms`        | `30`           | 0 to 200, below `moving_period_ms` | Wait after a `send_angles` before the next `get_angles`: the arm ignores the port while it passes the command to the servos. Measured: 30 gave 0 stalls in 4464 requests, 25 gave 43, 20 froze the arm once for 20 s |

The default backend is `serial` on purpose: a URDF that forgets the parameter on the robot fails
loudly if the port cannot be opened, instead of silently showing a fake arm that stands still.

## Lifecycle

The component is not a ROS node: its state changes with
`ros2 control set_hardware_component_state positronic_arm <state>`, not with `ros2 lifecycle`.

| Transition  | What happens                                                                     |
|-------------|----------------------------------------------------------------------------------|
| `on_init`   | Checks the joints, reads the joint limits from the URDF (#34), checks the parameters. |
| `on_configure` | Drops a backend left over from an earlier error, then creates a new one. `serial` opens and locks the port and starts the thread; fails if the port cannot be opened. States are set to NaN ("unknown"). |
| `read()` (inactive or active) | Publishes position and velocity once the arm has answered; until then the states stay NaN. While active, a link silent for over 200 ms latches `driver/fault_code` = 1 and stops the arm (#35). Publishes the latched fault code every cycle. |
| `on_activate` | Waits up to 1 s for the arm's first reply and fails if none comes ("Is it powered and on /dev/ttyTHS1?"). Then refuses an arm that stands outside its joint limits (#34). Then sends `power_on`, so the servos hold their pose, and fails if it did not leave the port within 300 ms. Every command starts at the measured pose: the first thing a controller sees is "stay where you are". Refuses a link that went silent since the first reply (#35). The only place a latched fault is cleared. |
| `write()` (active only) | Checks the six commands against the joint limits (#34); a bad one is not sent, the arm gets a stop and `write()` returns ERROR. Otherwise hands the newest command to the serial thread. Returns at once when the component is not active, so no command reaches the arm while inactive, and also while a fault is latched (#35). |
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

## Link timeout (#35)

No valid reply from the arm for 200 ms while the component is active is a fault. `read()`
latches it and does three things: it sets `driver/fault_code` to 1, it calls `request_halt()`
(every target held, a stop left for the serial thread), and it logs once. The component stays
ACTIVE, so the supervisor (#45) can read the code and decide; it reacts to any value but 0.
Only `on_activate` clears it, and it refuses a link that is still silent.

| `fault_code` | Meaning |
|---|---|
| 0 | No fault |
| 1 | Link timeout: no valid reply for 200 ms while active |
| 2 | Reserved for S3, "the arm does not follow the target". Not raised yet |

New codes are added at the end of `FaultCode` in `positronic_hardware.hpp`; a value is never
reused.

**Why 200 ms.** The port reads return at once (`VTIME = 0`), so a turn without a reply ends after
`REPLY_TIMEOUT` (50 ms). While the arm moves, one lost reply is up to ~120 ms of silence (the
rest of a 50 ms turn, a silent turn, a late 20 ms reply) and two are ~170 ms. 200 ms tolerates
two lost replies and faults on the third. The fault is seen at the next `read()`, so the
supervisor gets it within 200 ms plus one control period (33 ms at 30 Hz).

**What the arm does on silence.** It finishes its last commanded target and holds there: 8 of
8 classifiable trials at our operating point (30 Hz) in #20, the other 2 too close to the target
to tell (`docs/03_verification/serial_measurement/docs/serial_link.md`, section 5.1). So the latch cannot stop a move already sent; it keeps any new target from going out,
and the stop reaches the arm only if the line comes back. **On a dead line no software stops a
move in progress, the supervisor (#45) included: only cutting the servo power does.** The
hardware stop button (#27) was closed without being built in M1, so today that means the power
switch.

**Who reads it.** The safety supervisor (#45), on `/dynamic_joint_states`. `/joint_states`
carries the joints only, but `joint_state_broadcaster` has no `joints` and `interfaces`
parameters in `jetson_controllers.yaml`, so it claims every state interface and publishes them
all on `/dynamic_joint_states` once per update (30 Hz), the `driver` gpio included. No gpio
controller is needed. To see it without the arm:

```bash
ros2 launch positronic_bringup driver.launch.py backend:=fake &
ros2 topic echo /dynamic_joint_states --once    # joint_names has "driver", fault_code 0.0
```

When `read()` or `write()` returns ERROR the controller manager deactivates every controller
of the component, the broadcaster included, and the topic goes silent. So the supervisor also
halts when it gets no valid fault code for 200 ms.

**On the arm.** `hil_link_timeout` (built with the tests, never run by ctest) drives `SerialArm`
on the real port through a transport that drops every reply, mid-motion, and prints one CSV line
per trial: the time from the last bytes delivered to the timeout, the J1 target and where J1
ended. It moves the arm 5 degrees on J1 at 30 % speed: two people, bench clear, a hand on the
power switch, the driver stopped (the port is exclusive).

```bash
make nano-build && make nano-shell
./build/positronic_hardware/hil_link_timeout | tee link_timeout.csv
```

Run on 2026-11-10, 6 trials, J1 ±5° at 30 % speed (raw CSV in
`docs/03_verification/link_timeout/2026-11-10_hil_link_timeout.csv`):

| Measurement | Result |
|---|---|
| Last bytes from the arm to timeout | 194.0 to 201.3 ms, mean 197.2 ms. The watchdog counts from the last complete reply; a few bytes (bus noise, a partial frame) can arrive after it, which is why some trials read just under 200 ms |
| Reported to the supervisor | within that plus one control period: ≤ ~234 ms at 30 Hz |
| J1 at the cut → at the end | It kept moving after the replies stopped: +3.1° to +4.0° (3 trials) and −2.8° to −3.1° (3 trials) |
| J1 at the end vs target | 0.96° to 1.14° short, every trial, in the direction of motion. The same shortfall shows on the normal return moves, so it is the servo, not the silence |

So on silence the arm **finished** its last target, as in #20.

## Tests

```bash
make build
make test
```

No arm is needed:

| Test                       | What it checks                                                      |
|----------------------------|---------------------------------------------------------------------|
| `test_load_robot_system`   | Loads the plugin through a `ResourceManager` like the controller_manager does: joints, interfaces, parameters, lifecycle; a missing port; a silent arm on a pseudo-terminal (NaN states, activation refused, the component configurable again); deactivate holds the commands and activate resumes them from the measured pose; the joint limits (#34): at the limit, limit + 5 deg, NaN, recovery, missing and swapped limits, arm outside its limits at activation; the fault interface (#35): exported, required by `on_init`, 0 while the link is alive, latched to 1 on a silent link with no target sent while latched, cleared only by deactivate and activate, a silent link refused at activation. |
| `test_serial_arm`          | `SerialArm` replaying UART bytes recorded on the real arm: the pose, the send_angles frame, the stop and the power on (the stop goes out and holds every target, a command stored before it never goes out, `power_on()` sends once, the hold stays until the power on is on the wire, `request_halt()` returns at once, a port that refuses writes, a silent arm); the link timeout (#35), raised by silence and cleared when replies resume; `PosixSerialPort` exclusivity on a pseudo-terminal. |
| `test_link_watchdog`       | `LinkWatchdog` alone with made-up time points (#35): no fault right after reset, the 200 ms boundary, a reply restarts the count, no latching. |
| `test_reply_assembler`     | Split replies, servo-bus noise, request echo, newest reply wins, `clear()`. |
| `test_velocity_estimator`  | The velocity filter: first sample, constant rate, gaps, `reset()`.  |
| `test_joint_limit_check`   | The joint limit check (#34) alone: inside, at and past every limit, NaN and infinity, wrong joint count, first bad joint reported, invalid limits. |

Passing tests do not prove the driver is right on the arm. On the robot, check that activation
succeeds, that the first published pose is the real one, and that the velocity looks sane while
the arm is moved by hand (`release_all_servos`, see `positronic_protocol`). For #34, send one
joint to its limit + 5 deg past MoveIt (a `forward_position_controller`) and check that the arm
does not move and the log names the joint.

## Known limits

- The velocity uses the control loop `period` as dt, not the time between two serial samples.
  When the controller runs faster than the arm answers (about every 16 ms), some `read()` calls
  see the same pose twice and the velocity is underestimated.
- A read error and a read timeout look the same to the serial thread: both are "no reply", and
  the link timeout (#35) catches either after 200 ms.
- `halt()` and `power_on()` report true when their frame is on the wire, not when the firmware
  acknowledged it (#103). With a silent arm a serial turn lasts about 50 ms (`VTIME = 0`), so a
  stop reaches the port within ~100 ms; the 300 ms deadline leaves a wide margin.
- The joint limit check refuses, it never corrects (#34). With ros2_control's limiter off, a
  `joint_trajectory_controller` trajectory that crosses a URDF limit, even by 0.001 rad, makes
  `write()` return ERROR in the cycle where the command crosses it: the arm moves towards the
  limit, stops there, the component goes to UNCONFIGURED and its controllers are deactivated.
  What the supervisor sees is that component state plus the `Command refused, arm halted` line
  with the joint and the value. #33 must keep the controller's goals and tolerances inside the
  URDF limits: MoveIt plans inside them, a hand-written trajectory may not.

## Files

- `include/positronic_hardware/positronic_hardware.hpp`, `src/positronic_hardware.cpp`: the plugin.
- `arm_backend.hpp`: the `ArmBackend` interface, `NUM_JOINTS`, `JointPositions`.
- `fake_arm.hpp`: `FakeArm`.
- `joint_limit_check.hpp`: `check_command()`, the joint limit check of a position command (#34).
- `serial_arm.hpp`, `src/serial_arm.cpp`: `SerialArm` (`SerialTiming`, `POLL_PERIOD`, `SEND_SPEED`, `REPLY_TIMEOUT`, `EMPTY_READ_BACKOFF`, `REQUEST_TIMEOUT`) and `PosixSerialPort`.
- `link_watchdog.hpp`: `LinkWatchdog`, the 200 ms link timeout (#35).
- `test/hil_link_timeout.cpp`: the #35 check on the real arm, run by hand.
- `serial_transport.hpp`: the `SerialTransport` interface.
- `reply_assembler.hpp`: `ReplyAssembler`.
- `velocity_estimator.hpp`: `VelocityEstimator`.
- `positronic_hardware.xml`: registers the plugin with pluginlib.