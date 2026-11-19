# positronic_safety

The safety supervisor of the arm (#45): a lifecycle node on the Jetson, next to the driver
(ADR 0006), started by `positronic_bringup/launch/supervised.launch.py`.

- **ACTIVE**: the arm may move and is watched.
- Any trigger runs the one halt path, then the supervisor **deactivates itself**.
- The arm stays stopped until a person runs `ros2 lifecycle set /safety_supervisor activate`.

## Triggers

| Trigger | Reason in the status | How it is seen |
|---|---|---|
| Stop request | `STOP_SERVICE` | `~/stop` (`std_srvs/Trigger`) |
| Mission heartbeat lost | `HEARTBEAT_LOST` | nothing on `/mission/heartbeat` for 300 ms |
| Driver fault | `DRIVER_FAULT` | `driver/fault_code` ≠ 0 on `/dynamic_joint_states` (#35) |
| Fault data lost or bad | `FAULT_DATA_LOST` | no usable fault code for 200 ms |
| Supervisor deactivated | `DEACTIVATED` | `ros2 lifecycle set ... deactivate` |
| Resume failed | `RESUME_FAILED` | a resume request failed or was not answered |

"Fault data lost" exists because the controller manager deactivates the broadcaster too when
the driver's `read()`/`write()` returns ERROR: a supervisor watching only the code would be
blind exactly then.

## Halt and resume

| Path | Order | Why |
|---|---|---|
| Halt | hardware `positronic_arm` inactive → cancel `arm_controller` goals → deactivate the command controllers | The driver sends the stop in `on_deactivate`, so the arm stops first |
| Resume | hardware active → controllers | The other way, a controller writes its old target as soon as the driver is active |

Every request is asynchronous, one at a time. A halt never stops half way; a failed resume
becomes a halt; UNCONFIGURED after a failed deactivate counts as stopped. Activation is refused
during a halt, without a heartbeat, or without the controller manager. Each rule is a unit test
of `SupervisorLogic`.

## Interfaces and parameters

- `~/status` (`positronic_interfaces/SafetyStatus`, latched): mode, reason, last fault code,
  `hardware_stopped`. `hardware_stopped` false after a halt means the hardware did not confirm
  it left ACTIVE: cut the power.
- `~/stop` (`std_srvs/Trigger`): starts a halt and answers at once.
- `~/heartbeat` (`positronic_interfaces/Heartbeat`, best effort): published at every check
  (20 ms) from configure to cleanup, in every mode. The driver's dead-man check (#136) halts the
  arm when it stops while the arm may move. Published from the check timer itself, so a stuck
  executor goes silent too. The driver listens on `/safety_supervisor/heartbeat`: renaming the
  node breaks it, and the driver then refuses to activate.
- **Ctrl+C** on the supervisor alone runs no lifecycle transition, so `on_shutdown` never comes.
  A pre-shutdown hook in `main()` (`halt_before_exit()`) starts the halt while the supervisor is
  ACTIVE and waits until the hardware component is inactive, at most `halt_step_timeout_ms`;
  while STOPPED it sends nothing.
- Parameters, with the reason for each value: `positronic_bringup/config/safety_supervisor.yaml`.
  `controllers_to_stop` must list every controller that writes a command interface; a
  `positronic_bringup` test checks it against `jetson_controllers.yaml`.

## Tests

`make test`; no arm, no controller manager: `test_supervisor_logic` (the decisions),
`test_deadline_watchdog` (timeouts), `test_fault_code` (reading the fault code),
`test_safety_supervisor` (the node against a fake controller manager, trajectory controller
and mission node). Results on the arm: `docs/03_verification/safety_supervisor/`.

## Known limits

- **A dead serial line:** the halt goes through the driver; if its stop cannot reach the arm,
  the arm finishes its last move.
- **The supervisor dying** (`kill -9`, a crash, a hang): the driver stops the arm by itself
  once the heartbeat has been missing for `supervisor_timeout_ms` (500 ms), with fault 4, and
  takes the component out of ACTIVE (#136). Until then the arm keeps moving: 500 ms plus a
  control period and the serial turn, about 0.6 s. Start the supervisor again and activate it to resume: one activate, the driver's fault
  clears when its component activates.
- **The heartbeat crosses the Wi-Fi:** the 300 ms limit holds only with the mission node at
  **25 Hz** and the Jetson's **Wi-Fi power saving off**
  (`docs/03_verification/heartbeat_link`). Detection takes up to 320 ms.
- **`driver.launch.py` alone** starts everything active with no supervisor, and the driver's
  dead-man check off (`supervisor_timeout_ms:=0`).
