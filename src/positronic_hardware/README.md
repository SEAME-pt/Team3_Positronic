# positronic_hardware

The ros2_control hardware interface of the myCobot 280 (Jetson Nano): the plugin
`positronic_hardware/RobotSystem` (#22).

It reads the six joint angles from the arm over the serial port and publishes, for each joint,
a `position` (rad) and a `velocity` (rad/s) state interface. It takes a `position` command
(rad) per joint and sends the six targets to the arm as one `send_angles` frame at 30 % of the
firmware speed (#32), after checking them against the joint limits of the URDF (#34): a
command past a limit, NaN or with the wrong joint count is refused and the arm halts.
Deactivating the component stops the arm and holds it; activating it powers the servos on and
resumes (#36). A servo at 60 degC stops the arm and powers its servos off (see Servo
over-temperature).

## How it fits together

```
controller_manager ── read() / write() ──> RobotSystem (plugin)
                                              │  read_angles(), write_angles(), has_sample():
                                              │  copy in or out of a buffer, never blocks
                                              ▼
                                           ArmBackend
                                           ├─ FakeArm      no port (tests, CI): at every target at once,
                                           │                or lagging like the arm (fake_mode measured)
                                           └─ SerialArm    own thread: ask, decode, publish, send
                                                 │            two realtime_tools::RealtimeBuffer:
                                                 │            newest pose out, newest command in
                                                 ▼
                                           ReplyAssembler  bytes -> get_angles reply (positronic_protocol)
                                                 │
                                           SerialTransport
                                           └─ PosixSerialPort   /dev/ttyTHS1, raw 8N1, exclusive
```

- One turn of the serial thread: a pending stop or power on first, if any, waiting for its ack;
  then `get_angles`, up to 50 ms for the reply; then, when no send goes out or while the poll
  comes broken, `get_servo_temps` (see Servo over-temperature); then, if the arm answered and
  there is a new target, one
  `send_angles`, and the thread reads the line until the Atom's sync-write to the servos is
  whole (#150). After a send the Atom acknowledges it, polls its six servos and passes the
  targets on with that sync-write, the last frame it puts on the line, and drops a frame that
  comes before its end: 19.2 ms after the send median, 28.0 at p99 (10 minutes on the arm, 19 Nov
  2026). `echo_timeout` (60 ms, in `SerialTiming`, not a parameter: it only counts when the Atom
  is late) bounds the wait when the sync-write never comes. Such a send is counted, and while
  active `read()` warns at most every 5 s with how many there were: the Atom sends no sync-write
  for a target it refuses, such as one outside its own joint limits (J4 below about −129° on
  19 Nov 2026), and every such turn then lasts about 68 ms. A turn lasts 10 ms while the arm
  stands still and `moving_period_ms` after a send, about 27 ms of it busy. The arm drops a
  frame that arrives while it is still answering, so nothing is sent before the reply. With
  `pose_from_poll` (#170, on by default) the controller's `write()` wakes the thread and the
  send goes out at once; the servo status frames read before the sync-write are the pose, so
  no `get_angles` follows a send while they are whole (numbers in
  `docs/03_verification/write_sweep`).
- `read()` copies the newest pose out of the buffer and runs the velocity filter. `write()`
  checks the six commands against the joint limits and copies them into the buffer. Neither
  does serial I/O, allocates or throws; the only log is the one line of a refused command.
  Two short locks sit on the path of `write()`: the command buffer's (`RealtimeBuffer`) and, with
  `pose_from_poll`, `wake_mutex_`, held for one store before the serial thread is woken, and by
  that thread as briefly, never while it reads the line.
- Without the pose from the poll, a target equal to the last one sent is not sent again. With
  it, every write sends even an unchanged target once, so the pose keeps coming from the poll,
  unless the gripper is due (see Gripper) or the last send got no sync-write: then it goes out
  again one second after that send, so a refused target costs one wait a second and one send
  the Atom dropped does not end the repeat at rest (24 Nov 2026). A target with a NaN
  or an angle out of the protocol range is skipped and the previous one stays.

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

## Servo over-temperature

The servos of the 280 should not get hot in normal use, so a hot servo means something is badly
wrong: a joint pushing against an obstacle, a load it cannot hold. The driver does not try to
work around it. It stops the arm and cuts its power.

- **Reading.** While the arm moves, the temperatures come for free: every servo status frame of
  the Atom's poll after a `send_angles` carries its servo's temperature, so each whole poll is a
  reading, 30 a second (#170), as long as every frame is fresh: a joint bridged from an earlier
  poll carries an old temperature, and such a poll counts for the pose but not for the guard.
  When no send goes out (inactive or halted, a controller that stopped writing, `pose_from_poll`
  false), the serial thread sends `get_servo_temps` (`0xE5`) once a second, right after a
  `get_angles` reply (the arm's input is free), never while a stop or power on is pending. While
  sends go out and the poll comes broken (a hot servo corrupts its own frames), a reading older
  than 3 s takes the place of a send instead, which goes out next turn: a motion, or a controller
  that keeps writing, with a broken poll cannot hide a hot servo, and is read every 3 s. One
  request costs about one `get_angles`. A reading with a value outside 1 to 99 degC is ignored.
- **The cut.** A servo at or above `servo_temp_limit_c` (60 degC) for 1 s (`HOT_FOR`): the first
  reading at the limit starts the clock, one at the limit 1 s later trips it (about 30 polls while
  sends go out, two requests 1 s apart when none does, up to 2 x 3 s with a broken poll and a
  controller that writes: about 1 degC at the 0.2 degC/s of a loaded J4). Each servo has its own
  clock. A reading more than 1 degC under the limit stops it, so a false hot reading alone cannot
  cut the arm (J1 gives some: up to 98 degC in 10 min of sends, each one alone between normal
  polls). A reading 1 degC under it changes nothing: a servo crossing a degree reads both in turn
  for seconds (`LIMIT_BAND_C`), so the cut may come up to 1 degC early. Then the serial thread
  itself, without waiting for the control loop, holds every target, sends a `stop` and, after its
  ack (~37 ms; the Atom drops a frame that comes while it handles the stop), a `power_off`. It
  sends the `power_off` again every second from then on (the first repeat at the next turn when
  the cut's own power off got no ack), and `power_on()` is refused for the life of that backend.
  It works with the component inactive too: a halted arm still holds its pose, and holding heats
  the servos as well.
- **The arm goes limp.** `power_off` cuts the torque: J2 and J3 drop under the arm's own weight
  if it is stretched out. Chosen on purpose: at 60 degC holding the pose would keep heating the
  servos. After the cut `get_angles` gets no reply (see `positronic_protocol`), so the link goes
  silent too.
- **What the plugin does.** The next `read()` latches `driver/fault_code` = 3
  (`FaultCode::OVERHEAT`, the interface of the link timeout, so the supervisor (#45) sees it
  the same way), logs `Servo overheated: joint '...' at .. degC (limit 60 degC). The arm was
  stopped and its servos powered off ...` and returns ERROR: the controllers on the component
  are deactivated and `on_error()` takes it to UNCONFIGURED, as after a refused command (#34).
  The code stays on the interface through `on_error` and a new `configure`; only an `activate`
  clears it. `read()` checks the temperatures before the link: the silence that follows the
  power off would otherwise latch fault 1, and only the first fault is kept.
- **Blind guard.** While active, no plausible reading for 10 s (the arm stopped answering
  `0xE5`) makes `read()` halt the arm (no power off: nothing is known to be hot) and return
  ERROR.
- **Coming back.** Let the arm cool, power it on again (with the driver stopped,
  `MyCobot280(...).power_on()`, holding the arm, or a power cycle), then configure and activate.
  `on_activate` waits up to 2 s for a first reading and refuses an arm that never reports its
  temperature, or one with a servo within 5 degC of the limit (55 degC by default).
- **Testing it.** `servo_temp_limit_c` can only be lowered (30 to 60). Launching with a limit
  at least 6 degC above the hottest servo's reading (`on_activate` refuses less) cuts the arm
  1 s (`HOT_FOR`) after a servo warms past it. On 24 Nov 2026 the folded arm with the gripper,
  J4 raised 5 deg so it pushed against the load, took J4 from 39 to 51 degC in 110 s.

## Gripper (#123)

The adaptive gripper is an optional seventh joint, `gripper_controller`, after J6 in the
`<ros2_control>` block, with the same interfaces as the others (`position` and `velocity`
states, one `position` command, `<limits enable="false"/>`). `on_init` accepts six or seven
joints; with seven, the gripper joint needs a revolute `<limit>` with `lower < upper`, or the
driver refuses to start. `positronic_real.urdf.xacro` uses
`mycobot_280_jn_sim/mycobot_280_jn_gripper_camera.urdf`, the same arm as Gazebo and MoveIt, so
the gripper goes through the same `GripperCommand` action (`gripper_action_controller`,
`position_controllers/GripperActionController`, in `jetson_controllers.yaml`).

**Mapping.** Linear between the URDF limits and the firmware value (`0x67` / `0x65`, see
`positronic_protocol`):

| URDF (rad) | Firmware value | Gripper |
|---|---|---|
| `lower` = -0.74 | 0 | closed |
| `upper` = 0.15 | 100 | open |

`value = round((rad - lower) / (upper - lower) * 100)`, and back `rad = lower + value / 100 *
(upper - lower)`, kept inside `[lower, upper]`. One firmware step is 0.0089 rad. The velocity
state is the difference of two positions over the control period.

**Tolerance.** `gripper_action_controller` uses its `goal_tolerance` (0.01 rad by default,
just over one firmware step) and `allow_stalling: true`: an adaptive gripper that closes on an
object stops short of the target, and that is a success, not an error. Measured on the arm, 15
goals (open, closed, half): all reached, largest error 0.0089 rad, one firmware step
(`docs/03_verification/gripper_on_the_arm/README.md`).

**Commands.** `write()` checks the gripper command like a joint (#34): NaN, infinity or a value
outside the URDF limits is refused, the arm halts and `write()` returns ERROR. A valid command is
converted to the firmware value and handed to the serial thread through a buffer.

**The idle turn.** The serial thread serves the gripper only in a turn where the arm answered
`get_angles`, no arm target is sent, no temperature was asked and no stop or power on is
pending. A new gripper value goes out as one `set_gripper_value` at speed 20, followed by the
30 ms settle (`SerialTiming::send_settle`); otherwise, every 100 ms, a `get_gripper_value` waits up to 50 ms for
its reply. So a gripper frame never waits behind an arm command and a turn never adds the
gripper to a temperature request: the link timeout of 200 ms (#35) is never tripped by the
gripper. While halted no gripper value is sent; `power_on()` drops the value held during the
halt, so after a resume the gripper only moves on a new command. With the pose from the poll
(#170) every write is sent, even an unchanged one: at rest that repeat gives way to a gripper
that is due (a new value, or the 100 ms read). While the command changes, the gripper waits.

**Activation.** `on_activate` also waits for a first gripper reading and refuses to activate
("The gripper did not answer within ... ms") if none comes. `FakeArm` starts open (100) and is at
every gripper target at once.

**Tests.** `test_serial_arm` (`SerialArmGripper.*`: reads while idle, one send after a
`get_angles` reply, waits while the arm has targets, halt and power on, no gripper frame without
a gripper, the link never times out while the gripper is served, with the replay transport) and
`test_load_robot_system` (the seventh joint, its limits, mapping at the limits, refused targets,
velocity, activation without a gripper reply). The recorded frames are checked in
`positronic_protocol` (`test_recorded_gripper.cpp`).

## Parameters

All optional. An unknown name (a typo) or a bad value makes `on_init` fail with a message that
says which one.

| Name                    | Default        | Accepted values      | Meaning                          |
|-------------------------|----------------|----------------------|----------------------------------|
| `backend`               | `serial`       | `serial`, `fake`     | Where the angles come from       |
| `fake_mode`             | `instant`      | `instant`, `measured` | How `FakeArm` moves (#122): at every target at once, or like the real arm (see Measured fake arm). Only the fake backend reads it |
| `port`                  | `/dev/ttyTHS1` | any non-empty path   | Serial device                    |
| `baud`                  | `1000000`      | `115200`, `1000000`  | Baud rate                        |
| `velocity_filter_alpha` | `0.3`          | (0, 1]               | Low-pass weight of the velocity: closer to 1 follows faster but is noisier |
| `moving_period_ms`      | `33`           | 20 to 100            | One serial turn after a `send_angles`: a `get_angles` (with `pose_from_poll`, only when the poll gave no pose), the send and the wait for the Atom's sync-write (#150), about 27 ms. 33 is one control period at 30 Hz: 10 minutes on the arm with no stall (`docs/03_verification/write_sweep`). At most half the 200 ms link timeout (#35), so one lost reply never trips it |
| `pose_from_poll`        | `true`         | `true`, `false`      | The pose from the Atom's own poll of the servos, published when its sync-write closes, and the send right after the controller's `write()` (#170); the runs on the arm are in `docs/03_verification/write_sweep`. `false` is the #150 turn: a `get_angles` every turn, the period from the thread's clock |
| `echo_timeout_ms`       | `60`           | 20 to 80             | Longest wait after a `send_angles` for the Atom's sync-write to the servos, which ends the turn's wait (#150). The sync-write ends 19 ms after the send at p50, 28 at p99, once 54. At 60, two lost replies after it come to ~196 ms at worst with their 8 ms drains, under the 200 ms link timeout (#35); at 80 they trip it. Until #150 every send was followed by a fixed wait, `send_settle_ms`; `SerialTiming::send_settle` (30 ms) is now only the wait after the gripper's `set_gripper_value`. |
| `servo_temp_limit_c`    | `60`           | 30 to 60             | A servo at or above this temperature for 1 s stops the arm and powers it off. Only lower, to test the cut |
| `tracking_error_deg`    | `8`            | 1 to 45              | Degrees a joint may be from the last target on the wire before the time counts (#103). Keep it wider than the controller's trajectory tolerance (5 deg), so the controller aborts first |
| `tracking_error_ms`     | `300`          | 50 to 5000           | Milliseconds a joint may stay over `tracking_error_deg`, while active, before fault 2: the arm did not follow the command (#103) |
| `supervisor_timeout_ms` | `500`          | 0 to 5000            | Dead-man check (#136): no heartbeat from the safety supervisor for this long, while active, is fault 4. 0 turns the check off. See Dead-man check |
| `send_speed`            | `30`           | 1 to 100             | Speed byte of every `send_angles`, percent of the firmware maximum (#133). Fixed while the driver runs: another speed is another launch. Before every `power_on` the driver also sends fresh mode 1 (`0x16 01`), so the arm streams whatever mode it woke up in |

The default backend is `serial` on purpose: a URDF that forgets the parameter on the robot fails
loudly if the port cannot be opened, instead of silently showing a fake arm that stands still.

A new parameter, in the driver: a field with its default in `HardwareConfig`, and a row in
`NUMERIC_PARAMETERS` in `parameters.cpp` (name, range, whole or not, the field it fills), or its
special rule in `read_special()` if it is not a number in a range, plus its value in the INFO
line of `describe()`. The list of known names and the "Known: ..." error follow from the tables.
Then its test in `test_parameters`, and in `positronic_real.urdf.xacro` an empty `xacro:arg` and
a `driver_param` line, and in `driver.launch.py` a launch argument with an empty default, as for
every other parameter (#163, #150). The default lives only here: the xacro writes the `<param>`
only when the argument is set, so a default there would win over this one. The one exception is
`supervisor_timeout_ms`, 0 in `driver.launch.py`: that launch alone has no supervisor, and with
the check on the arm would never activate (a mode, not a tuned value). `supervised.launch.py`
sets 500. The default here stays 500, so a URDF that forgets the parameter gets the check.

## Measured fake arm (#122)

`FakeArm` is at every target the moment `write_angles()` gets it. The real arm is not, and a
test against the instant fake cannot see what only shows on the bench: a trajectory too fast for
the servos, a goal tolerance tighter than the rest error. `fake_mode` `measured` makes the fake
arm move like the arm did in #33 (`docs/03_verification/jtc_on_the_arm`, gripper off):

| Behaviour | Model | Numbers | From #33 |
|---|---|---|---|
| Lag | Each joint follows its target as a first-order lag | Time constant 0.22 s, every joint | J1 2.7 deg behind at 10 deg/s, 4.9 at 20: a line of slope 0.22 s through 0.5 deg at 0 deg/s |
| Wrist start-up | A standing J4 or J5 starts late after its target changes. A change while it moves does not delay it again | 200 ms | "a wrist servo takes about 0.2 s to start moving after the command changes" |
| Rest error | The reported pose is a fixed offset below where the joint is, always | J1 0.5, J2 1.0, J3 1.0, J4 0.7, J5 1.4, J6 1.0 deg | Every joint settles 0.5 to 1.5 deg off its command; J4 0.5 to 0.9 and J5 1.4 at rest, never commanded |

The lag and the rest error add on a joint moving up: J1 reads 2.2 + 0.5 = 2.7 deg behind at
10 deg/s and 4.4 + 0.5 = 4.9 at 20 (the unit test reads 2.75 and 5.0 with a target every 10 ms).
Moving down the rest error takes off instead, 1.7 at 10 deg/s. Each time the pose it reads is
commanded back (an activate, a controller holding the current state), the arm settles one more
rest error below it, as J4 did once when never commanded. On the arm a joint with no load stays
where it is when its measured pose is commanded back, so a test that activates or holds many
times sees the fake arm drift further than the real one.

Time is the sum of the `period`s `read()` gets, not the `time` argument, which the resource
manager replaces with its own clock. On the control loop that is the time that passed; a test
passes the periods in and never sleeps. A `halt()` stops every joint where it is and cancels a
wrist that had not started yet.

What the trajectory test in `test_load_robot_system` shows: J1 and the wrist J5 fed a 3-point
trajectory (hold, +10 deg, +20 deg) at the 30 Hz control rate stay within 0.060 rad of the
command at 10 deg/s, inside the controller's 0.087 rad trajectory tolerance, and reach 0.131 rad
at 30 deg/s, past it. That is the arm of #33; the instant fake would pass both.

**Link cut.** `cut_link()` is a test hook, in either mode, for the cable coming off as in #20.
The arm stops answering: `read_angles()` keeps the last pose it reported and `link_timed_out()`
turns true once the silence passes 200 ms (`LinkWatchdog`, the serial backend's rule: 200 ms is
still the link, 201 is not). Nothing reaches the arm any more, so a target, the gripper, the stop
and the power on are lost, and `halt()` and `power_on()` report false. The firmware itself goes on:
it finishes the last target it got and holds there, which `restore_link()` shows. Only
`test_fake_arm` uses it: the plugin tests load the plugin through pluginlib and cannot reach the
`FakeArm`.

What it does not imitate:
- The serial line: bytes, the turn timing, lost replies, the sync-write. `test_serial_arm`
  replays recorded bytes for that.
- The gripper's weight: with it J4 rests 2.3 to 2.7 deg off, outside the 0.5 to 1.5 range.
- A rest error that changes from move to move or depends on the direction the joint came from:
  here it is one fixed offset per joint.
- A top speed or acceleration: on a large step the fake arm starts at step / 0.22 s, faster than
  any servo. The controller sends ramps, not steps, so this only matters to a test that jumps.
- The plugin's checks: the joint limits are refused in `write()` before the fake arm (#34), and a
  NaN or infinite target the fake arm ignores, keeping the joint on its previous target.

## Lifecycle

The component is not a ROS node: its state changes with
`ros2 control set_hardware_component_state positronic_arm <state>`, not with `ros2 lifecycle`.

| Transition  | What happens                                                                     |
|-------------|----------------------------------------------------------------------------------|
| `on_init`   | Checks the joints, reads the joint limits from the URDF (#34), checks the parameters. With `supervisor_timeout_ms` above 0, subscribes to the supervisor's heartbeat, and fails if the component has no ROS node to do it with (#136). |
| `on_configure` | Drops a backend left over from an earlier error, then creates a new one. `serial` opens and locks the port and starts the thread; fails if the port cannot be opened. States are set to NaN ("unknown"). |
| `read()` (inactive or active) | Publishes position and velocity once the arm has answered; until then the states stay NaN. Once the serial thread has cut the arm for an overheat, latches `driver/fault_code` = 3 and returns ERROR (see Servo over-temperature); while active, temperatures stale for 10 s halt the arm and return ERROR. While active, a link silent for over 200 ms latches `driver/fault_code` = 1 and stops the arm (#35). While active, a joint further than `tracking_error_deg` from its target for `tracking_error_ms` latches `driver/fault_code` = 2 and stops the arm (#103). While active, no heartbeat from the supervisor for `supervisor_timeout_ms` latches `driver/fault_code` = 4, stops the arm and returns ERROR (#136). The temperatures are checked first, so an overheat is not reported as the link timeout that its power off causes. Publishes the latched fault code every cycle. |
| `on_activate` | First waits up to `supervisor_timeout_ms` for a heartbeat from the supervisor and fails without one (#136). Then waits up to 1 s for the arm's first reply and fails if none comes ("Is it powered and on /dev/ttyTHS1?"). Then refuses an arm that stands outside its joint limits (#34). Then waits up to 2 s for the first servo temperatures and refuses an arm that does not report them or has a servo within 5 degC of `servo_temp_limit_c`. Then sends `power_on`, so the servos hold their pose, and fails if it did not leave the port within 300 ms. Every command starts at the measured pose: the first thing a controller sees is "stay where you are". Refuses a link that went silent since the first reply (#35). The only place a latched fault is cleared. |
| `write()` (active only) | Checks the six commands against the joint limits (#34); a bad one is not sent, the arm gets a stop and `write()` returns ERROR. Otherwise hands the newest command to the serial thread. Returns at once when the component is not active, so no command reaches the arm while inactive, and also while a fault is latched (#35). |
| `on_deactivate` | Sends a stop: the arm halts where it is, and the serial thread holds every target until the next activate. Returns ERROR if the arm did not acknowledge the stop within 300 ms (#103): the thread keeps resending it meanwhile, but a component that cannot confirm the arm stopped must not stay active. |
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
| 2 | Tracking error: a joint further than `tracking_error_deg` (8) from the last target on the wire for longer than `tracking_error_ms` (300) while active. The arm did not follow the command; targets held like the link timeout (#103, see Unfollowed command) |
| 3 | Overheat: a servo at or above `servo_temp_limit_c` for 1 s. The serial thread already stopped the arm and powered it off; `read()` also returns ERROR (see Servo over-temperature) |
| 4 | Supervisor lost: no heartbeat from the safety supervisor for `supervisor_timeout_ms` (500) while active. Targets held like the link timeout; `read()` also returns ERROR (see Dead-man check) |

New codes are added at the end of `FaultCode` in `positronic_hardware.hpp`; a value is never
reused.

## Dead-man check (#136)

The supervisor (#45) stops the arm by asking for it. If the supervisor dies (`kill -9`, a crash,
a hang), nobody asks, and the arm finished its move in the #45 campaign (test 7). So the driver
checks that the supervisor is there: the supervisor publishes `positronic_interfaces/Heartbeat`
on `/safety_supervisor/heartbeat` every 20 ms while it resumes, monitors or halts (not when
stopped), and with no heartbeat for `supervisor_timeout_ms`
while active, `read()` latches `driver/fault_code` = 4, calls `request_halt()` (every target
held, the stop left for the serial thread: the servos keep torque and the arm holds its pose),
logs once and returns ERROR.

- **Why ERROR, unlike the link timeout.** The link timeout stays ACTIVE so the supervisor can
  decide; here there is no supervisor. ERROR makes the controller manager deactivate the
  controllers and take the component to UNCONFIGURED, as a supervisor halt would. Recovery is one
  step: the restarted supervisor's resume configures and activates the component, which clears
  the fault. Like the overheat, `/joint_states` stops until then.
- **How the driver listens.** ros2_control gives each component a node of its own, spun by the
  controller manager's executor (`get_node()`). The subscription is made once in `on_init`, best
  effort; its callback only stores the arrival time, on this machine's steady clock, in the same
  lock-free `LinkWatchdog` as the link timeout. `read()` adds one atomic load and one clock read.
- **When.** Only while active. `on_activate` waits up to `supervisor_timeout_ms` for a heartbeat
  and refuses without one: the supervisor starts beating at its resume, just before it asks for
  the activate, so an activate it did not ask for (by hand, with the supervisor stopped) is
  refused. A link timeout that came first keeps its code, 1, but `read()` still
  returns ERROR.
- **Timing.** 500 ms is 25 heartbeats, both nodes on the Jetson (ADR 0006), no Wi-Fi in between.
  The stop leaves within the timeout, plus one control period (33 ms), plus the turn the serial
  thread is in: about 27 ms usually, up to ~160 ms at worst (a lost reply, 50 ms, an overdue
  temperature request, 50 ms, and a late sync-write, 60 ms). On the arm: fault 4 509-532 ms after
  the last heartbeat (`docs/03_verification/safety_supervisor/`, test 8, 11-23 and 11-25).
  Stalls of ~350 ms were seen on the Nano when a ros2 CLI starts, so 500 ms leaves ~150 ms: raise
  it if false fault 4s show up.
- **Off.** `supervisor_timeout_ms` = 0, for a run without the supervisor: `driver.launch.py`
  alone (bench, `sweep.sh`, the HIL tests). `on_init` logs a warning saying so. Gazebo runs
  `gz_ros2_control`, not this plugin, so the check does not apply there.

## Unfollowed command (#103)

The ack of a stop or a power on says the Atom took the frame, not that the servos obeyed. For
the targets, nothing is acknowledged at all: a `send_angles` the firmware accepts and does not
carry out (a servo pushing against something, a joint that stalls) is invisible on the line. So
`read()` compares, every cycle while active, the pose with the last target the serial thread
put on the wire (`ArmBackend::last_target()`, nullopt before the first send, after an
acknowledged stop and after the overheat cut). A joint further than `tracking_error_deg` from its target for longer than
`tracking_error_ms` latches `driver/fault_code` = 2, logs once (joint, error, limits), and asks
the backend for a stop: targets held until the next activate, like the link timeout. A joint
that comes back inside the threshold forgets its clock, so the normal lag of a move (2.7 to
3.1 deg at 10 deg/s, 4.5 to 4.9 at 20, measured in `docs/03_verification/jtc_on_the_arm`) never
counts. The
defaults, 8 deg and 300 ms, are wider than the controller's 5 deg trajectory tolerance on
purpose: the controller aborts first, the driver's fault is the second net. The check lives in
`tracking_check.hpp`, pure, with its own test.

Measured on the arm with this check (17 Nov 2026, gripper and camera mounted, serial tap,
`target of the last send_angles minus the pose of the next get_angles reply`): T3 at 10 deg/s,
J1 and J2 20 deg each, three runs, max lag 3.5 deg (J1) and 3.6 deg (J2), never over 8 deg, no
fault; T4 at 20 deg/s, three runs, one aborted by the controller at 5.05 deg of error against
its 5 deg tolerance, two passed, no fault. A T2 at 5 deg/s held by hand mid-move: the lag
reached 4.9 deg (J2), the controller aborted at 5.08 deg, the driver's fault did not fire, as
intended. The fault itself, exercised with `tracking_error_deg:=3` (under the controller's
tolerance) and the same hand: `Fault 2: joint 'joint2_to_joint1' is 4.3 deg from its target for
over 300 ms (limit 3.0 deg)`, the stop on the wire 12 ms later and acknowledged 50 ms later, no
target sent after it, `driver/fault_code` = 2, the component still ACTIVE, the controller's own
abort 133 ms after the fault.

Acknowledgement times of the requests on the bench (16 Nov 2026, pyserial taps, driver down and
driver up, 259 stops and 69 power_on sent), the numbers behind `ACK_TIMEOUT`:

| Request | Arm | Samples | Ack after the frame | Status byte |
|---|---|---|---|---|
| `stop` | not moving, no `send_angles` before | 49 | 37 to 39 ms mean, 40.8 max, all acknowledged | 1 in every ack |
| `stop` | mid-move | 30 | 39 to 43 ms mean, 85.8 once, all acknowledged | 1 in every ack |
| `power_on` | at rest | 69 | 58 to 59 ms mean, 60.1 max, all acknowledged | 1 in every ack |
| `stop` written 0 to 30 ms after a `send_angles` | at rest | 180 | 142 acknowledged, 44.9 ms max: 5 of 20 at 0 ms, 13 at 5, 12 at 10, 15 at 15, 19 at 20, 20 of 20 at 30, and 58 of 60 at 22 to 28. A stop the arm does not take gets no ack at all, and the thread resends it | 1 in every ack |

**Why 200 ms.** The port reads return at once (`VTIME = 0`), so a turn without a reply ends after
`REPLY_TIMEOUT` (50 ms). While the arm moves, one lost reply is up to ~130 ms of silence (a
sync-write as late as the 60 ms `echo_timeout`, the 50 ms of the lost reply, a late 20 ms reply;
a turn without a reply sends nothing, so no wait follows it) and two are ~196 ms at worst, each
request after an unanswered one waiting `LATE_REPLY_DRAIN` (8 ms) first. 200 ms
tolerates two lost replies and faults on the third. A temperature request (8 ms, 50 ms when it
goes unanswered) does not reset this watchdog, only a `get_angles` reply does, so the turn that
asks sends nothing: its request never adds to the silence of a send and its sync-write. The fault is seen at the next `read()`, so the
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

## Requests to the Atom (#149)

A request is data, in `requests.hpp`: the command, the frame, what the arm sends back and how
long to wait for it. The serial thread performs every one of them through `SerialArm::perform()`:
write the frame, then wait for nothing, for the ack of the command (status 1) or for a reply the
caller knows how to decode. Nothing in the loop knows a request by name.

| Request | Frame | Expects | Waits | Who sends it |
|---|---|---|---|---|
| `stop_request()` | `FE FE 02 29 FA` | ack `FE FE 03 29 01 FA` | `ACK_TIMEOUT`, 80 ms | `halt()`, `request_halt()`, the overheat cut |
| `power_on_request()` | `FE FE 02 10 FA` | ack `FE FE 03 10 01 FA` | 80 ms | `power_on()` |
| `power_off_request()` | `FE FE 02 11 FA` | ack `FE FE 03 11 01 FA` (62 ms) | 80 ms | the overheat cut after the stop's ack, then once a period (the first repeat at the next turn when the cut's own got no ack) |
| `get_servo_temps_request()` | `FE FE 02 E5 FA` | the temperature reply | `REQUEST_REPLY_TIMEOUT`, 50 ms | the temperature guard, after a get_angles reply |

A new request in three steps: a factory in `requests.hpp` (command, encoder, what it expects, the
timeout), its test in `test_requests` (frame bytes and the pair command-frame), and the call to
`perform()` where the thread needs it. Lifecycle requests that the plugin waits for also need a
`Pending` value, the one-request-at-a-time mechanism of `halt()` and `power_on()`.

## Tests

```bash
make build
make test
```

No arm is needed:

| Test                       | What it checks                                                      |
|----------------------------|---------------------------------------------------------------------|
| `test_load_robot_system`   | Loads the plugin through a `ResourceManager` like the controller_manager does: joints, interfaces, parameters, lifecycle; a missing port; a silent arm on a pseudo-terminal (NaN states, activation refused, the component configurable again); deactivate holds the commands and activate resumes them from the measured pose; the joint limits (#34): at the limit, limit + 5 deg, NaN, recovery, missing and swapped limits, arm outside its limits at activation; the fault interface (#35): exported, required by `on_init`, 0 while the link is alive, latched to 1 on a silent link with no target sent while latched, cleared only by deactivate and activate, a silent link refused at activation; the servo temperature: `servo_temp_limit_c`, activation refused on a hot arm or one that never reports its temperature, an arm heating while active is cut (stop then power off), latches fault 3 and the component goes to UNCONFIGURED, fault 3 and not 1 when the arm goes silent after the cut, cleared by a cool activate; the acknowledged requests (#103): a deactivate whose stop the arm never acknowledges returns ERROR after 300 ms and leaves the component configurable, an activate whose power_on is never acknowledged returns ERROR with a stop after the last power_on and no target sent, and a 58 ms ack still activates; the unfollowed command (#103): a target 2 deg away is never a fault, 9 deg for 300 ms latches fault 2 with a stop and no further target, 9 deg for 100 ms and back is not, a bad tracking parameter is rejected; `fake_mode` `measured` (#122) lags on the periods `read()` gets, and a 3-point trajectory on J1 and J5 at 30 Hz stays inside the 0.087 rad trajectory tolerance at 10 deg/s and leaves it at 30; the dead-man check (#136), with an executor and a stand-in supervisor: the heartbeat stops while active, `read()` returns ERROR, the stop goes out and fault 4 is latched, an activate with the heartbeat back clears it and targets go out again, activate refused without a heartbeat, nothing at 0, `on_init` refuses the check without a node, a link timeout that came first keeps fault 1. |
| `test_serial_arm`          | `SerialArm` replaying UART bytes recorded on the real arm: the pose, the send_angles frame, the stop and the power on (the stop goes out and holds every target, a command stored before it never goes out, `power_on()` sends once, the hold stays until the power on is on the wire, `request_halt()` returns at once, a port that refuses writes, a silent arm); the acknowledged requests (#103): `halt()` returns only after the ack, fails at the deadline and keeps resending when the arm never acknowledges, a `request_halt()` during the power_on wait puts the hold back, the last target follows the wire and an acknowledged stop clears it; the link timeout (#35), raised by silence and cleared when replies resume; the temperature guard (read while idle, not in turns that send until overdue, then right after a `get_angles`; the cut: stop then power off, `power_on()` refused, the power off repeated, also while halted; implausible readings ignored; stale when `0xE5` never answers); the wait after a send (#150): the next `get_angles` never before the sync-write is whole and long before `echo_timeout`, and an arm that never sync-writes makes the wait last `echo_timeout`, counted in `echo_timeouts()`; the pose from the poll (#170, `SerialArmPollPose.*`): the frames are the pose and no `get_angles` follows the send, a write wakes the thread, a missing frame brings the `get_angles` back, a joint missing from one poll keeps its position for `POLL_BRIDGE_AGE`, the edges of a plausible frame, the temperatures and the cut from the poll, `pose_from_poll` false keeps the #150 turn; the turn's decisions: a write sends the unchanged target again, a write that lands during a `get_angles` counts for its turn, a due gripper takes the turn of that repeat, a halt ends the turn's wait, a poll that is not published keeps its good frames for the bridge, a poll with a bridged frame is no temperature reading, a send without its sync-write goes out again after `REPEAT_RETRY` and a refused target once a second; the cut's power off after the stop's ack; a late answer not taken for the next request's (an arm that drops what comes while it answers); `PosixSerialPort` exclusivity on a pseudo-terminal. |
| `test_fake_arm`            | `FakeArm` with a clock the test sets (#122): instant is at every target at once; measured follows a step with exactly 1 - 1/e after one time constant, lags 2.7 and 4.9 deg on J1 at 10 and 20 deg/s (within 0.5), less the rest error moving down, starts J4 and J5 exactly 200 ms late and not again while they move, nor for a target sent again, rests every joint `REST_ERROR_DEG` off its command (also the pose it read), a halt stops every joint and cancels a waiting wrist, NaN and infinite targets ignored, a clock that stands still or goes back moves nothing; `cut_link()`: timed out at 201 ms and not at 200, a second cut keeps the first one's time, everything sent while cut is lost (`halt()` and `power_on()` false), the arm finishes its last target and holds, a waiting wrist still starts, the same in the instant mode. |
| `test_link_watchdog`       | `LinkWatchdog` alone with made-up time points (#35): no fault right after reset, the 200 ms boundary, a reply restarts the count, no latching. |
| `test_echo_timeout_report` | When `read()` warns about sends without a sync-write (#150): nothing new is never due, the first count at once, nothing more within the 5 s, the sends since the last warning after it, and a reset for a new backend (without it a new backend's small count would stay hidden). |
| `test_reply_assembler`     | Split replies, servo-bus noise, request echo, newest reply wins, `clear()`. |
| `test_velocity_estimator`  | The velocity filter: first sample, constant rate, gaps, `reset()`.  |
| `test_joint_limit_check`   | The joint limit check (#34) alone: inside, at and past every limit, NaN and infinity, wrong joint count, first bad joint reported, invalid limits. |
| `test_servo_temperature`   | The over-temperature logic alone: plausible range, hottest servo, activation cool-down, 1 s of hot readings to trip, a false hot reading alone never trips, the 1 degC band (limit - 1 changes nothing, limit - 2 stops the clock), one clock per servo, J1 named first on a tie, implausible readings, the trip stays, a lowered limit. |
| `test_requests`            | The request table (#149): every factory carries the frame its command encodes, the command sits in byte 3 of the frame, what each one waits for, and `ACK_TIMEOUT` between the arm's 60 ms and the 200 ms watchdog. |
| `test_parameters`          | The hardware parameters alone (#163): the defaults, every parameter reaching its field, every unknown name in one message, in alphabetical order, with the list of known names, both ends of every range accepted and one step past them refused with the range (`send_settle_ms` up to 99), a fraction where a whole number is expected, NaN and infinity, a settle longer than the period accepted since #150, the rules of `backend`, `fake_mode`, `port`, `baud` and `velocity_filter_alpha`, and the INFO line with the fake mode. |
| `test_tracking_check`      | The unfollowed-command logic alone (#103): the defaults, below the threshold never trips, trips once a joint has been over for the time, coming back inside forgets the clock, the joint and the signed error reported, an unknown pose (NaN) does not count, reset forgets every clock. |

Passing tests do not prove the driver is right on the arm. On the robot, check that activation
succeeds, that the first published pose is the real one, and that the velocity looks sane while
the arm is moved by hand (`release_all_servos`, see `positronic_protocol`). For #34, send one
joint to its limit + 5 deg past MoveIt (a `forward_position_controller`) and check that the arm
does not move and the log names the joint.

## Known limits

- The dead-man check (#136) proves that some process publishes on
  `/safety_supervisor/heartbeat`, not which one: a supervisor on another machine in the same ROS
  domain, or a `ros2 topic pub`, keeps it satisfied. ROS domain isolation is assumed.
- The velocity uses the control loop `period` as dt, not the time between two serial samples.
  When the controller runs faster than the arm answers (about every 16 ms), some `read()` calls
  see the same pose twice and the velocity is underestimated.
- A read error and a read timeout look the same to the serial thread: both are "no reply", and
  the link timeout (#35) catches either after 200 ms.
- `halt()` and `power_on()` report true only when the firmware acknowledged the request
  (`FE FE 03 <cmd> 01 FA`), not when the frame left the port (#103). The serial thread writes
  the frame at the start of a turn and reads the line for up to `ACK_TIMEOUT` (80 ms: a stop is
  acknowledged in 37 to 45 ms, a power_on in 58 to 60, see the table under Unfollowed command;
  the one 86 ms stop seen mid-move costs a resend). No ack means the arm did not take the
  request (a busy window after a `send_angles`, a lost byte): the request stays pending and the
  frame goes out again next turn, until the ack comes or the caller's 300 ms pass. A `halt()`
  that returns false therefore means "the arm never confirmed the stop", and the stop is still
  being resent. What the ack does not say: that the servos obeyed: that is the tracking error
  (fault 2, see Unfollowed command).
- The servo temperature format (`0xE5`, one byte per servo) comes from pymycobot 4.0.4
  (`positronic_protocol` has a one-line check for an arm). On a firmware that does not answer
  it, the driver refuses to activate. The power off of the cut is acknowledged
  (`FE FE 03 11 01 FA`, 62 ms after it); after it `get_angles` stops answering.
- The Atom answers no request while it is answering another, and its answers carry no request
  number. A request whose answer comes past its window (the Atom answered 1 to 2 ms past the
  50 ms windows on 24 Nov 2026) could have that answer taken for the next request's, which then
  went out while the Atom was busy and was dropped. Before every request the thread drops what
  is waiting on the line, and after an unanswered one also what comes for 8 ms
  (`LATE_REPLY_DRAIN`, sized to the late answers seen, 2.4 ms and 6 ms past their windows, and
  to the link timeout's budget). An answer
  later than that is still taken for the next one.
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
- `servo_temperature.hpp`: `SERVO_TEMP_LIMIT_C`, `OverheatDetector`, the over-temperature logic.
- `requests.hpp`: `Request`, `Expect`, `ACK_TIMEOUT`, `REQUEST_REPLY_TIMEOUT`, the factories `stop_request()`, `power_on_request()`, `power_off_request()`, `get_servo_temps_request()` (#149).
- `parameters.hpp`, `src/parameters.cpp`: `HardwareConfig`, `Backend`, `parse_hardware_parameters()`, `describe()`, the table of numeric parameters (#163).
- `tracking_check.hpp`: `TRACKING_ERROR_DEG`, `TRACKING_ERROR_MS`, `TrackingCheck`, the unfollowed-command logic (#103).
- `serial_arm.hpp`, `src/serial_arm.cpp`: `SerialArm` (`SerialTiming`, `TemperatureGuard`, `POLL_PERIOD`, `DEFAULT_SEND_SPEED`, `REPLY_TIMEOUT`, `EMPTY_READ_BACKOFF`, `REQUEST_TIMEOUT`) and `PosixSerialPort`.
- `link_watchdog.hpp`: `LinkWatchdog`, the 200 ms link timeout (#35), also the time of the supervisor's last heartbeat (#136).
- `test/hil_link_timeout.cpp`: the #35 check on the real arm, run by hand.
- `serial_transport.hpp`: the `SerialTransport` interface.
- `reply_assembler.hpp`: `ReplyAssembler`.
- `velocity_estimator.hpp`: `VelocityEstimator`.
- `positronic_hardware.xml`: registers the plugin with pluginlib.