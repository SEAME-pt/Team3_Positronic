# positronic_hardware: test review

Reviewed 2026-11-17. Coverage 92.2 % (495 of 537 lines). Tests: `src/positronic_hardware/test/`.
The plugin tests load the real plugin through ros2_control's resource manager, with the fake
backend or a pseudo-terminal standing in for the serial port.

**What the package does:** the ros2_control plugin (`RobotSystem`) that ROS talks to. It checks
every command against the URDF joint limits, hands it to a backend, and reports the joint
positions, velocities and a fault code. The real backend (`SerialArm`) owns the serial port in
its own thread; `FakeArm` stands in without the robot.

## Behaviours

| Behaviour | Normal | Failure path | Edges | Verdict |
|---|:-:|:-:|:-:|---|
| Joint limit check (`check_command`) | ✅ | ✅ NaN, ±inf, wrong count, null pointer | ✅ exactly at each limit, tolerance in/out, limit + 5°, one protocol step over, asymmetric limits | Excellent |
| Limits read from the URDF | ✅ | ✅ joint without limit, swapped limits | ⚠️ URDF that does not parse | Good |
| Parameters (`on_init`) | ✅ | ✅ unknown name, bad values, settle ≥ period | ✅ | Good |
| `write()` refuses a bad command and halts | ✅ | ✅ past limit, below asymmetric limit, NaN | ✅ exactly at the limit | Good, comes back after a refusal too |
| `read()` reports positions and velocity | ✅ | ✅ NaN ("unknown") before the first reply | | Good |
| `read()` trusts every reading | | ❌ no check at all | | See H2 |
| Link watchdog (#35) | ✅ | ✅ arm never answers, silence | ✅ exactly the timeout is not a fault | Very good |
| Fault latch and clear | ✅ | ✅ | | Good |
| Activate | ✅ | ✅ silent arm, arm outside its limits, silent link | ⚠️ power_on that cannot be sent: untested | See H4 |
| Deactivate sends a stop | ✅ | ✅ stop cannot leave → ERROR | | Good |
| Shutdown | ❌ | ❌ | | See H1 |
| Halt / power on hold in `SerialArm` | ✅ | ✅ stop cannot be written, silent arm | ✅ command stored before the stop never goes out | Very good |
| Acks from the arm | | ❌ never read | | See H3 |
| `ReplyAssembler` (bytes → pose) | ✅ | ✅ noise, echo | ✅ split over 3 reads, newest of two wins, buffer cap | Good |
| `VelocityEstimator` | ✅ | ✅ zero period, long gap | ⚠️ negative or NaN period not tested (the code holds the estimate, which is safe) | Good |
| `PosixSerialPort` | ✅ exclusive open | ⚠️ open/termios errors untested | | Acceptable: needs a failing device |

## Findings

Decisions are added below as they are made.

- **H1.** `on_shutdown` sends no stop, and is never tested.
- **H2.** `read()` trusts every reading (confirms protocol F1).
- **H3.** The acks are never read (confirms protocol F2).
- **H4.** Two lifecycle failure paths have no test.
- **H5.** About a dozen tests rely on fixed sleeps.

### H1. Shutdown: the stop depends on ros2_control, and no test proves it

**First worry (wrong):** `on_shutdown` only does `arm_.reset()` (closes the port, stops the
thread) and sends no stop, unlike `on_deactivate`, which calls `arm_->halt()`. The arm keeps
going to its last target when the link goes quiet, so a shutdown while moving would leave it
unwatched.

**What ros2_control really does (Jazzy branch source, checked 2026-11-17):** on a clean exit
(Ctrl+C, launch stopped) the controller manager's pre-shutdown callback calls
`resource_manager_->shutdown_components()`, and going to FINALIZED from ACTIVE runs
`deactivate_hardware(hardware) && shutdown_hardware(hardware)`. So `on_deactivate`, and its stop,
runs first. `on_cleanup` is safe for the same reason: it only runs from INACTIVE, so
`on_deactivate` already sent the stop.

**Every way the plugin changes state:**

| # | Trigger | Who | Path |
|---|---|---|---|
| 1 | Launch | Controller manager, from its config | UNCONFIGURED → INACTIVE → ACTIVE (stops at INACTIVE with `start_inactive:=true`) |
| 2 | Service `set_hardware_component_state` | Safety supervisor, or a person with `ros2 control set_hardware_component_state` | Any requested step, e.g. ACTIVE → INACTIVE on a fault |
| 3 | A transition returns ERROR | Automatic | `on_error` (stop without waiting) → UNCONFIGURED |
| 4 | `write()` returns ERROR (refused command) | Automatic | Controllers stopped → `on_error` → UNCONFIGURED |
| 5 | Clean exit | Automatic, controller manager pre-shutdown callback | From ACTIVE: `on_deactivate` (stop), then `on_shutdown` |
| 6 | Crash (segfault, `kill -9`, Jetson power loss) | Nobody | No callback runs at all |

Not triggers: controllers (they only write commands; `switch_controller` changes controllers,
not the plugin) and a link timeout (the plugin stays ACTIVE and reports a fault code; the
supervisor then uses trigger 2).

**What is left:**
- Our safety on a clean exit depends on ros2_control code we do not own, read on the branch tip,
  not on the version in the container. No test proves it; `on_shutdown` is never run by a test
  (coverage: lines 444–447).
- On a crash no software can send a stop: only the hardware e-stop (cutting servo power) helps.

**Lesson:** check how the framework calls your code before calling something a bug. And when
safety depends on code you don't own, lock it in with a test.

**Decision:**
- Test in `test/test_load_robot_system.cpp`: take the plugin from ACTIVE straight to FINALIZED
  through the resource manager, check that a stop frame reached the pseudo-terminal. The helpers
  already exist (see `DeactivateFailsWhenTheStopCannotLeave…`). **Fix now** (~1 h).
- One line in the safety docs: "a crash sends no stop; only the e-stop helps". **Later.**

### H2. `read()` trusts every reading

Protocol F1 from the other side: a frame with the right shape is accepted, whatever it says.

**Confirmed:** `update_states` publishes whatever `read_angles()` returns, with no range or
jump check. Readings are checked against the limits only once, in `on_activate`.

**What makes it concrete here:**
1. `on_activate` copies the measured pose into the commands ("stay where you are"). A fake
   reading inside the limits at that moment would mean "move to the fake pose": a real motion.
2. Partial protection: a fake reading outside the limits that a controller echoes back as a
   command is refused by `write()`, which halts the arm. Fake readings inside the limits pass.
3. The velocity estimate turns a jump into a large fake velocity, seen by controllers and the
   supervisor.

**Where the check belongs:** in `SerialArm::poll_loop`, before the pose is stored, not in
`read()`. Only the serial thread knows when each reading arrived (it already stamps it for the
watchdog); `read()` runs on the control loop's clock and often sees the same reading several
times. The thread is also not real-time (counting and logging are allowed) and can simply not
store a bad reading, so the control loop keeps the last good pose.

**Design points for the new issue:**
- The thread does not know the joint limits: the plugin passes them to `SerialArm` when it
  creates it, like the timing.
- "N bad readings in a row" for free: if a rejected reading does not count as a reply for the
  watchdog, 200 ms of only bad readings raises the existing `LINK_TIMEOUT` fault and the
  supervisor halts the arm with no change on its side. Downside: the fault names the wrong
  cause. A new `FaultCode` at the end of the enum names it correctly.
- The edge cases listed in protocol F1 (first reading, real fast moves, arm moved by hand, NaN
  time) apply here.

**Decision:** **new issue** (as in protocol F1). Nothing in #46.

### H3. The arm's acks are never read

**What:** the arm answers `send_angles`, `stop` and `power_on` with `FE FE 03 <cmd> 01 FA`.
`SerialArm` counts a command as sent once `transport_->write()` returns true, which only means
the bytes left the Jetson. `ReplyAssembler` looks for angle replies only, so the ack is dropped
as noise. `halt()` returning true means "the stop left the Jetson", not "the arm got it" (the
comment in `on_deactivate` says so honestly).

**Can the arm miss a command?** Yes: it drops a `send_angles` that arrives while it is busy,
which is why `send_settle` (30 ms) exists. 30 ms gave 0 stalls in 4464 requests, but that
measured timing, not "did every command arrive".

**What makes a lost command worse:** after a successful write the target goes into `last_sent`,
and the same target is not sent again. A dropped target is not resent until the target changes.

**Scenario:** the arm moves, the supervisor deactivates the plugin, the stop is written,
`halt()` returns true, but the busy arm dropped it. The arm keeps going to its last target and
nobody notices: the supervisor even logs "the driver sent the stop"
(`positronic_safety/src/safety_supervisor.cpp`). Bounded harm (the last target passed the limit
check), but the stop does not happen now, which is its whole point.

**What would fix it (a feature):**
1. The stop ack first: `halt()` returns true only once the stop ack arrived; no ack within a
   short time, send the stop again.
2. Then the `send_angles` ack: missing ack, do not record the target as `last_sent`, so it goes
   out again next turn.
3. The three `decode_ack` tests of protocol F3 are part of this work.

**Open question:** the status byte was always `01` in the recording; nobody knows if the arm
ever answers `00` or what it would mean. The value of the ack is "did the arm get it", not the
status.

**Decision:** **new issue**, "read the acks, starting with stop", before or together with the H2
issue (a stop that silently did not arrive is the more direct safety problem). Nothing in #46.

### H4. `on_activate` with a `power_on` that cannot be sent has no test

**What:** coverage leaves 17 lines of `src/positronic_hardware.cpp` untested. The one that matters
is lines 491–494: `if (!arm_->power_on()) { ...; return CallbackReturn::ERROR; }`. If it were
broken or deleted, nothing would fail.

**Why it matters:** `halt()` sets `halted_ = true`, and only the serial thread clears it, in the
turn that writes the `power_on` frame. Without the check, a `power_on` that never left would give
an ACTIVE plugin with `halted_` still true: no command goes out, the servos may be unpowered (a
released arm sags), the fault code stays `NONE` so the supervisor sees nothing, and everyone
believes the arm is under control. `TRACKING_ERROR` is reserved but not raised, so nothing
catches it.

**The test (in `test/test_load_robot_system.cpp`, with the pseudo-terminal helpers):** bring the
plugin to INACTIVE with the arm answering, wait for a first reading, close the arm side of the
pseudo-terminal (every write then fails), activate at once; expect ERROR and UNCONFIGURED.

**Trap:** `on_activate` checks the link before `power_on`. A slow test (over 200 ms) would fail at
the link check instead and still pass, for the wrong reason. Also assert that activate took at
least `REQUEST_TIMEOUT` (300 ms, the wait of `power_on()`), as the deactivate test does.

**The other untested lines:**

| Line(s) | What | Verdict |
|---|---|---|
| 567 | `write()` while not active returns OK, sends nothing | Second guard: later |
| 423–426, 354, 206 | Interface missing, URDF does not parse, base `on_init` fails | Unreachable after `on_init`'s checks: accept |
| 189–192 | Log text for a wrong joint count | Unreachable (`on_init` requires 6 joints): accept |
| 579 | A command handle busy | Needs a real race: accept |
| 444–447 | `on_shutdown` | Covered by the H1 test |

**Lesson:** a test that checks only "it failed" can pass for the wrong reason. Check that it
failed where you expected.

**Decision:**
- `power_on` failure test: **fix now** (~1 h). It is a transition that powers the arm on.
- `write()` while inactive: **later**.
- The rest: **accept**, reasons above.

### H5. Tests that wait a fixed time, then check that nothing happened

**What:** about ten places in `test/test_serial_arm.cpp` sleep `3 * moving_period` (150 ms) or
`3 * REPLY_TIMEOUT`, then assert that something did **not** happen: the same command is not
sent twice, a NaN command is not sent, a command during a halt is not sent, `power_on` is not
sent twice. `CLAUDE.md` asks for the margin of a timing-dependent test to be explained; these do
not say why three turns are enough. (`PAST_THE_TIMEOUT`, 300 ms, does explain its margin.)

**Why it matters:** on a slow machine the serial thread may run zero turns during the sleep. The
"nothing happened" assertion then passes without proving anything: a **false green**. Several
of these guard safety behaviour ("a command during a halt never goes out"). The opposite case, a
test waiting for something to happen, fails loudly on a slow machine, which is annoying but
harmless.

**Fix:** wait for proof that the thread ran, not for a fixed time. Every turn writes one
`get_angles` request and the test transport records every frame, so a helper
`wait_for_turns(port, 3)` (about 10 lines, like `wait_for_sends`, with a 2 s deadline) replaces
each sleep. The test then proves "three real turns and still nothing sent", on any machine.

**Lesson:** a test that checks that something did not happen must first prove it gave the code
the chance to do it.

**Decision:** **later**, at the top of the list: do it on Day 2 if time is left after the
"fix now" items (~1 h).

## Summary

| Finding | Decision |
|---|---|
| H1. Stop on shutdown depends on ros2_control, untested | Test "shutdown from ACTIVE sends a stop": **fix now**. Crash note in safety docs: later |
| H2. `read()` trusts every reading | **New issue** (check in the serial thread) |
| H3. Acks never read | ~~New issue~~ **done in `dev`** (#146, #159), see the recheck |
| H4. `power_on` failure on activate untested | Test: ~~fix now~~ **already covered in `dev`** (#146). `write()` while inactive: later. Rest: accept |
| H5. Fixed sleeps in "nothing happened" tests | Later, top of the list |

**Fix now (#46):** two tests in `test/test_load_robot_system.cpp`, about 2 h in total.

## Recheck after merging `dev` (2026-11-19)

The review above was done on an old base (11 commits behind `dev`). After merging `dev` into
`46-coverage-gate-at-80` (merge commit `d9cffaa`), each finding was checked again on the new code.

| Finding | Was | Now |
|---|---|---|
| H1 stop on shutdown | Fix now | **Still needed, done.** `dev` has no ACTIVE → FINALIZED test. `ShutdownFromActiveSendsAStop` kept, rewritten with `dev`'s helpers (`load_on_pseudo_terminal()`, `received_requests(0x29)`). With acks, `halt()` returns only after the arm confirmed the stop, so the test needs no wait. Mutation check on the merged code: `on_deactivate` without `halt()` fails it. |
| H2 readings trusted | New issue | **Still open.** Only temperatures get a plausibility check (`serial_arm.cpp:459`); joint angles are still taken as they come. The tracking fault (#159) catches an arm that does not follow its target, not a wrong reading. New issue stays. |
| H3 acks never read | New issue | **Done in `dev`.** #146 (issue #103) waits for the ack of every stop and power_on and resends until it comes; a `send_angles` the arm does not follow raises `TRACKING_ERROR` after `tracking_error_ms` (#159). No new issue. |
| H4 `power_on` failure untested | Fix now | **Already covered in `dev`.** `ActivateFailsWhenTheArmDoesNotAcknowledgeThePowerOn` (#146) runs the same path (`power_on()` false → ERROR after ≥ `REQUEST_TIMEOUT`, then UNCONFIGURED) and checks more (a stop follows, no target leaves). Our test was dropped in the merge. The H4 remark "`TRACKING_ERROR` is reserved but not raised" is out of date (#159). |
| H5 fixed sleeps | Later (top) | **Still there**, and more: `test_serial_arm.cpp` has about 13 `3 * moving_period` sleeps plus the new temperature tests' sleeps. Stays later, top. |
| Side note: unthrottled logs in `read()`/`write()` | Team question | **Still there** (`RCLCPP_ERROR` in `read()`, lines 671–725). |

**Lesson:** review against the newest `dev`, or merge it first. A review of old code finds problems
that may already be fixed.

## Side notes (not test findings)

- `refuse()` (called from `write()`) and `raise_fault()` (called from `read()`) log with
  `RCLCPP_ERROR`, not a `_THROTTLE` macro, which `CLAUDE.md` asks for in `read()`/`write()`. The
  code logs once per transition, so it does not flood, but it is still a rule exception. A
  question for the team, not for #46.
