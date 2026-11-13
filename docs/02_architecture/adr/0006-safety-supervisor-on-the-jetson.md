# 0006. The safety supervisor runs on the Jetson, next to the driver

2026-11-11, proposed, Felipe Suassuna (issue #45).

## Context

The safety supervisor (#45) halts the arm on a stop request, a lost mission heartbeat or a
driver fault. It halts through the controller manager: it sets the hardware component
inactive, cancels the trajectory goals and deactivates the controllers. The controller
manager and the driver run on the Jetson, in the arm's base. MoveIt runs on a laptop, over
Wi-Fi (ADR 0003), and so can the mission node that commands the arm.

SEA:ME grades the e-stop under fault injection: a node is killed, the link is dropped, bad
data is fed. The stop must not depend on what was just broken.

## Decision

- The supervisor runs **on the Jetson**, in the same container and the same launch as the
  driver (`supervised.launch.py`). `positronic_safety` is in `NANO_PACKAGES`.
- The driver comes up with the hardware component and `arm_controller` **inactive**. Only the
  supervisor makes them active, when a person activates it.
- The mission heartbeat crosses the network to reach it. A dropped link is then the same
  thing as a dead mission node: no heartbeat for 300 ms, halt.

## Alternatives

- **On the laptop, with the mission node:** one machine less to deploy to. But when the link
  drops the supervisor is on the wrong side of it: it sees the fault and cannot reach the
  controller manager to halt. That is exactly the "drop the link" test.
- **Inside the mission node:** the node that commands would also be the one that stops, so
  killing it kills the stop.
- **Inside the driver plugin:** it has no ROS services, runs on the control loop and must not
  block. It keeps what only it can do, the link timeout and the joint limits; the decisions
  across nodes (heartbeat, stop request, stay stopped) stay outside.
- **As a ros2_control controller:** it would run in the control loop, but a controller cannot
  change the state of a hardware component or cancel another controller's goals.

## Consequences

- A halt is two service calls and one action call on the same machine: no Wi-Fi in the stop
  path. With the fake backend the hardware-inactive request was answered within 1 to 2 ms,
  and the whole halt took 42 to 67 ms (`docs/03_verification/safety_supervisor/`).
- The supervisor compares the stamps of the fault messages with its own clock. Both are the
  Jetson's clock, so no clock synchronisation is needed.
- The Jetson builds and runs one more node. Its cost there (CPU, memory) is not measured yet.
- **If the supervisor's process dies, nothing stops the arm.** The serial line is still
  alive, so the driver's link timeout does not fire, and the hardware button (#27) was not
  built. Until then: the power switch.
- A lost heartbeat cannot be told from a slow network: the timeout decides. It is 300 ms,
  chosen from measurements on the lab Wi-Fi (`docs/03_verification/heartbeat_link`): longest
  gap 157 ms in the campaign, and one of at least 204 ms with the real mission node. It assumes
  the mission node at 25 Hz and the Jetson's Wi-Fi power saving off; with power saving on, gaps
  of 0.2 to 0.4 s came every few seconds and the supervisor halted by itself.

## Revisit when

- The driver gets a dead-man check of its own (it stops when the supervisor goes silent):
  then the supervisor dying is covered, and this ADR needs a line saying so.
- The measurements on the arm (`docs/03_verification/safety_supervisor/`) show false halts or
  a halt slower than the criteria of #45.
- Another machine joins the cell (M3, the car): who may stop whom has to be decided again.
