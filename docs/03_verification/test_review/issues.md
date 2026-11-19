# Issues from the #46 test review

Five new issues (missing behaviour) and one follow-up issue (the "later" items), ready to paste
into GitHub. The details of each finding are in the package files next to this one.

Not here on purpose: the three `MoveitBackend` bugs (destructor hang, `ready()` true without
`move_group`, crash without a robot description) were sent to the mission owner, who opened
#165. Check that #165 lists all three.

---

## 1. Reject impossible joint readings in the serial thread

**Template:** User story / task. **Labels:** user-story, hardware

### Summary
`SerialArm` takes every decoded `get_angles` reply as the arm's pose. The protocol has no
checksum, so a frame with the right shape but wrong bytes (line noise, a cut and glued reply) is
accepted, and controllers and the supervisor act on it. Add a plausibility check where readings
arrive (`SerialArm::poll_loop`): reject a reading outside the joint range, or too far from the
last one (`max_jump = max_velocity × dt × margin`). Found in the #46 test review (protocol F1,
hardware H2).

### Acceptance criteria
- [ ] A reading outside a joint's URDF range (plus a tolerance) is rejected and not published.
- [ ] A reading that jumps more than `max_velocity × dt × margin` from the last one is rejected.
- [ ] A rejected reading does not feed the link watchdog, so a stream of bad readings ends in
      `LINK_TIMEOUT` (or a new fault code, added at the end of `FaultCode`).
- [ ] `SerialArm` gets the joint limits it needs (passed in from the plugin).
- [ ] The margin and its reason are written next to the constant.

### Safety
Yes: it changes which pose the controllers and the supervisor see. Test on the bench with the
arm at rest first; a wrong margin rejects good readings and ends in a link fault (safe: the arm
stops).

### Test checklist
- [ ] Unit test: in range, at the limit, past the limit, NaN, a jump just under and just over
      `max_jump`, a stream of bad readings ends in the fault
- [ ] Run in simulation
- [ ] Run on the real arm

### Links
Found in: #46 (`docs/03_verification/test_review/positronic_hardware.md`, H2;
`positronic_protocol.md`, F1)

---

## 2. The safety supervisor halts the arm on Ctrl+C

**Template:** User story / task. **Labels:** user-story, safety

### Summary
On Ctrl+C, rclcpp (Jazzy) does not run any lifecycle transition, so the supervisor's
`on_deactivate` and `on_shutdown` never run and no halt is sent: the plugin stays ACTIVE and the
controllers keep commanding, with nobody watching. `main()` (`safety_supervisor_main.cpp`) only
spins and exits. Add a pre-shutdown hook that sends step 1 of the halt ("hardware inactive"),
which is enough: the controller manager then runs the plugin's `on_deactivate`, which sends the
stop. Can be merged into #136 ("supervisor dies") instead. Found in the #46 test review (safety
S1).

### Acceptance criteria
- [ ] Ctrl+C on the supervisor while MONITORING sends `set_hardware_component_state` → inactive
      before the process exits.
- [ ] The request is actually on the wire before exit (wait for it, bounded, e.g. ≤ 200 ms).
- [ ] Ctrl+C while STOPPED sends nothing.
- [ ] A crash (`kill -9`) still sends nothing: documented as a known limit (only the hardware
      e-stop helps).

### Safety
Yes: it is a halt path. Test in simulation first, then on the arm with the e-stop in reach.

### Test checklist
- [ ] Unit test (the hook's effect, with the fake controller manager)
- [ ] Run in simulation: Ctrl+C on the supervisor during a motion
- [ ] Run on the real arm

### Links
Found in: #46 (`positronic_safety.md`, S1). Related: #136

---

## 3. Decide what the mission heartbeat must prove (ADR)

**Template:** User story / task. **Labels:** user-story, safety, architecture

### Summary
The supervisor treats any message on `/mission/heartbeat` as proof the mission is fine: it
ignores the content (`const Heartbeat & /*message*/`). So a mission node in ERROR keeps the arm
enabled while it publishes; a message from any node (`ros2 topic pub` from a laptop) counts; a
late message counts as fresh. The heartbeat is sent by a timer, so it also does not prove the
planning code is alive. Decide, as a team, what the heartbeat must prove, and write it in an
ADR (as `CONTRIBUTING.md` asks for decisions like this). Found in the #46 test review (safety
S4).

| The heartbeat could prove | The supervisor would check |
|---|---|
| Something publishes | Nothing (today) |
| The mission node is alive | `node_name` |
| The mission node is healthy | `state != ERROR` |
| The mission makes progress | A counter updated by the planning code, not a timer |

Each level catches more, and adds ways to halt for no reason.

### Acceptance criteria
- [ ] ADR in `docs/02_architecture/adr/` with the decision and its reasons.
- [ ] The supervisor checks what the ADR says (if anything changes).
- [ ] A node test locks the decision in (today's behaviour if nothing changes: "a heartbeat with
      state ERROR keeps the arm enabled").
- [ ] One sentence in the safety README says what the heartbeat proves.

### Safety
Yes if the supervisor's checks change: each new check is a new way to halt. Run the heartbeat
campaign again (`docs/03_verification/heartbeat_link`) if timing is involved.

### Test checklist
- [ ] Unit test (node test with the fake mission)
- [ ] Run in simulation

### Links
Found in: #46 (`positronic_safety.md`, S4)

---

## 4. bug: a planning scene that fails to load goes unnoticed

**Template:** Bug. **Labels:** bug

## What happened
`planning_scene.py` sends the workcell obstacles (bench, four walls, roof) to `move_group` and
exits. If it fails (no `move_group` within `timeout_sec`, `move_group` refuses, no answer in
5 s), it logs an error and exits with code 1. `move_group.launch.py` does not react to that exit
code, and nothing tells the mission node or the supervisor. MoveIt then plans as if the workcell
were empty, through the bench and the walls.

## What should happen
A scene that did not load stops the system, or at least keeps the mission from moving the arm:
for example the launch shuts down on a non-zero exit of the loader, or the mission node checks
that the obstacles are in the planning scene before it activates.

## How to reproduce
1. `ros2 launch positronic_moveit_config move_group.launch.py` with `move_group` slow to start,
   or run the loader with `timeout_sec:=0.1` before `move_group` is up.
2. The loader logs "/apply_planning_scene not available" and exits 1.
3. Everything else keeps running, and nothing reports the missing obstacles. (Not yet checked:
   that a goal through the bench is then actually planned. That is the expected consequence.)

## Environment
- simulation (also applies to hardware)
- laptop, dev container
- branch `46-coverage-gate-at-80` (loader tests: `test/test_planning_scene_node.py`)

## Logs, screenshots, plots
Found in: #46 (`positronic_moveit_config.md`, C1).

---

## 5. bug: the linters never run on positronic_mission

**Template:** Bug. **Labels:** bug

## What happened
In `src/positronic_mission/CMakeLists.txt` the lint lines are commented out:

```cmake
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()
```

`package.xml` still lists `ament_lint_auto` and `ament_lint_common`, so it looks covered, but
cpplint, cppcheck, uncrustify, xmllint and copyright never run on this package in `make test`.

## What should happen
The two lines are back, like in every other package, and whatever the linters then report is
fixed (or excluded with a reason).

## How to reproduce
1. `make test`
2. `colcon test-result --verbose --test-result-base build/positronic_mission`: no
   `cpplint`, `cppcheck`, `uncrustify` results, unlike `build/positronic_hardware`.

## Environment
- branch `dev`

## Logs, screenshots, plots
Found in: #46 (`positronic_mission.md`, side notes). Check first whether they were switched off
on purpose (a lint error that blocked a merge?).

---

## 6. Follow-up from the #46 test review: tests to improve later

**Template:** User story / task. **Labels:** user-story, test

### Summary
The #46 review sorted every test gap into "fix now" (done in #46), "new issue" (issues above)
and "later". This issue collects the "later" items: useful, not blocking. Each one is small; pick
any. Details and file/line references in `docs/03_verification/test_review/`. Top of the list
first.

### Acceptance criteria
Top of the list:
- [ ] **hardware H5**: replace the fixed `3 * moving_period` sleeps in `test_serial_arm.cpp`
      (about 13, all "nothing happened" checks) with waiting for evidence (`wait_for_turns(port, 3)`):
      today they can pass without proving anything.
- [ ] **protocol F3**: edge tests for `decode_ack` (noise before it, cut at every position,
      wrong command, wrong status). The stop path depends on it since #146.

Hardware and protocol:
- [ ] protocol F1: a test that documents that a well-shaped fake frame is accepted (no checksum).
- [ ] protocol F2: one README sentence: the package translates, it does not protect the arm.
- [ ] protocol F4: remove or strengthen the weak tests in `test_smoke.cpp` and `test_types.cpp`.
- [ ] hardware H4: a test for `write()` while not active (returns OK, sends nothing).
- [ ] hardware H1: one line in the safety docs: a crash sends no stop, only the e-stop helps.

Safety:
- [ ] S5: node tests for the activation refusals: a halt still running, no controller manager
      services, a resume that fails on the spot, bad parameters at configure.
- [ ] README: during RESUMING (up to ~6 s) the heartbeat and fault data are not checked.
- [ ] S1 note: the comment in `on_shutdown` ("no on_deactivate before it") does not match Jazzy,
      which deactivates first; say that its halt is a second guard.

Mission:
- [ ] M4: explain the margins in the heartbeat timing tests (`Heartbeat`, 20–30 messages in
      1 s), or check "no gap above the supervisor's timeout" instead of a count.
- [ ] M5: tests for reset while not active, and cleanup while a goal thread exists.

MoveIt config:
- [ ] C2: tests for a bad scene file (non-box type, missing `size`, wrong number of dimensions).
- [ ] C3: a config test for `joint_limits.yaml` (velocity scaling ≤ 0.2, every `max_velocity` ≤
      the URDF velocity, every name a URDF joint); remove the `gripper_controller` entry, which
      is a controller, not a joint.

Bringup:
- [ ] B1: a `main()` test for `joint_states_delay.py` (exit 0 when within the limit, 1 with
      `--limit-ms 0`), like the rate tool's.
- [ ] B2: check `heartbeat_timeout_ms` against the measured worst Wi-Fi gap (157 ms + the 40 ms
      period), not only "positive".

Question for the team (not a test):
- [ ] `read()`/`write()` log faults with `RCLCPP_ERROR`, not a `_THROTTLE` macro as `CLAUDE.md`
      asks. Once per transition, so no flood: keep as an exception, or change?

### Safety
No for the tests themselves. C3 and B2 lock safety values in.

### Test checklist
- [ ] Unit tests (each item is one)

### Links
Found in: #46 (`docs/03_verification/test_review/README.md`, Status table)
