# positronic_safety

The safety supervisor of the arm (#45): a lifecycle node that halts the arm and keeps it
halted until a person resumes.

- **ACTIVE** means the arm may move and is watched.
- Any trigger runs the one halt path, then the supervisor **deactivates itself**.
- It stays INACTIVE, and the arm stopped, until a person runs
  `ros2 lifecycle set /safety_supervisor activate`, which runs the resume path.

It runs on the Jetson, next to the driver (ADR 0005), started by
`positronic_bringup/launch/supervised.launch.py`.

## Triggers

| Trigger | Reason in the status | How it is seen |
|---|---|---|
| Stop request | `STOP_SERVICE` | `~/stop` (`std_srvs/Trigger`) |
| Mission heartbeat lost | `HEARTBEAT_LOST` | no message on `/mission/heartbeat` for 200 ms |
| Driver fault | `DRIVER_FAULT` | `driver/fault_code` other than 0 on `/dynamic_joint_states` (#35) |
| Fault data lost or bad | `FAULT_DATA_LOST` | no usable fault code for 200 ms: topic silent, gpio missing, NaN |
| Supervisor deactivated | `DEACTIVATED` | a person ran `ros2 lifecycle set ... deactivate` (or shutdown) |
| Resume failed | `RESUME_FAILED` | a resume request failed or was not answered |

"Fault data lost" is there because the controller manager deactivates every controller of the
component, the broadcaster included, when the driver's `read()` or `write()` returns ERROR
(a command past a joint limit does that). A supervisor that only looked at the code would be
blind exactly then.

## The halt and the resume

| Path | Requests, in order | Why this order |
|---|---|---|
| Halt | 1. hardware component `positronic_arm` inactive  2. cancel every goal of `arm_controller`  3. deactivate the command controllers | The driver sends the stop in `on_deactivate`, so the arm stops as early as possible |
| Resume | 1. hardware component active  2. activate the controllers | In the other order a controller writes its old target again as soon as the driver is active, and the arm moves to it |

Rules, each one a unit test of `SupervisorLogic`:

- One halt path for every trigger.
- One request in flight at a time; the next leaves when the answer comes, or after
  `halt_step_timeout_ms`. Every request is asynchronous: no callback waits.
- A halt never stops half way: a failed request does not hold back the next one.
- A failed resume request turns the resume into a halt.
- After a link fault the driver's deactivate fails and the component ends UNCONFIGURED. That
  counts as stopped (it sends nothing to the arm); the next resume configures it again.
- A stop request while already stopped runs the halt again.
- Activation is refused while a halt is running, without a heartbeat, or without the
  controller manager.

## Interfaces

| Name | Type | |
|---|---|---|
| `~/status` | `positronic_interfaces/msg/SafetyStatus`, latched | mode, reason, last fault code, `hardware_stopped`; one message per change of mode |
| `~/stop` | `std_srvs/srv/Trigger` | starts a halt; answers at once, the status tells when it is over |
| `/mission/heartbeat` | `positronic_interfaces/msg/Heartbeat`, best effort | subscribed |
| `/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | subscribed, for `driver/fault_code` |
| `/controller_manager/set_hardware_component_state`, `/controller_manager/switch_controller` | services | called |
| `/arm_controller/follow_joint_trajectory` | action | only its cancel is used |

`hardware_stopped` false after a halt means the supervisor asked but the hardware component
did not confirm that it left ACTIVE: treat the arm as able to move, and cut the power.

## Parameters

The values the arm runs with are in `positronic_bringup/config/safety_supervisor.yaml`, with
the reason for each. An empty name or a timeout that is not positive makes `configure` fail.

| Name | Default |
|---|---|
| `hardware_component` | `positronic_arm` |
| `controller_manager` | `/controller_manager` |
| `trajectory_action` | `/arm_controller/follow_joint_trajectory` |
| `controllers_to_start` | `joint_state_broadcaster`, `arm_controller` |
| `controllers_to_stop` | `arm_controller`, `forward_position_controller` |
| `heartbeat_topic`, `heartbeat_timeout_ms` | `/mission/heartbeat`, 200 |
| `fault_topic`, `fault_gpio`, `fault_interface`, `fault_timeout_ms` | `/dynamic_joint_states`, `driver`, `fault_code`, 200 |
| `halt_step_timeout_ms`, `resume_step_timeout_ms` | 500, 3000 |
| `check_period_ms` | 20 |

`controllers_to_stop` must list every controller that writes a command interface; a test of
`positronic_bringup` checks it against `jetson_controllers.yaml`.

## Run it

Without the arm, see "Real arm under the safety supervisor" in the `positronic_bringup` README
(`backend:=fake`). On the arm: `docs/03_verification/safety_supervisor/`.

## Tests

```bash
make build
make test
```

No arm and no controller manager are needed:

| Test | What it checks |
|---|---|
| `test_supervisor_logic` | The decisions, with no ROS: the order of the halt and of the resume, a halt going on when requests fail, the first reason kept, stay stopped until `resume()`, a failed resume becoming a halt, late answers ignored. |
| `test_deadline_watchdog` | The timeouts, with made-up time points: never fed is expired, the boundary, a feed restarts the count. |
| `test_fault_code` | Finding `driver/fault_code` in a `/dynamic_joint_states` message; missing, NaN and mismatched lists give nothing. |
| `test_safety_supervisor` | The node against a fake controller manager, trajectory controller and mission node in one process: every trigger and its reason, the order of the requests, the active goal cancelled, UNCONFIGURED counted as stopped, a controller manager that does not answer, stay stopped, resume after a driver fault. |

Passing tests do not prove it on the arm: the fault-injection procedure and its results are in
`docs/03_verification/safety_supervisor/`.

## Known limits

- **A dead serial line.** The halt goes through the driver. When its stop cannot reach the
  arm, the arm finishes its last move (see "Link timeout" in the `positronic_hardware` README).
- **The supervisor dying.** Nothing stops the arm for it: the serial line is alive, so the
  driver's link timeout does not fire, and the hardware button (#27) was not built. The fix
  is a dead-man check in the driver; until then, the power switch.
- **The heartbeat margin.** The mission node publishes at 10 Hz and the limit is 200 ms: one
  late heartbeat is tolerated, two are a halt. A halt 2 s after activation was seen with the
  fake backend while a `ros2` tool started (206 ms of silence). To be measured on the arm.
- **`driver.launch.py` alone** starts everything active with no supervisor. The arm is only
  guarded when started with `supervised.launch.py`.
- **The numbers on the arm** (time to halt, time to rest) are not measured yet.

## Files

- `include/positronic_safety/supervisor_logic.hpp`: `SupervisorLogic`, `Mode`, `Reason`, `Step`.
- `include/positronic_safety/deadline_watchdog.hpp`: `DeadlineWatchdog`.
- `include/positronic_safety/fault_code.hpp`: `find_fault_code()`.
- `include/positronic_safety/safety_supervisor.hpp`, `src/safety_supervisor.cpp`: the node.
- `src/safety_supervisor_main.cpp`: `main()`, one single-threaded executor.
