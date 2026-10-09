# Sprint 1

19 Oct to 30 Oct 2026. Review on Friday 30 Oct.

Sprint goal: the real arm is visible in ROS 2 from any laptop, and every PR
is built and tested by CI.

| Role | Who |
|---|---|
| Scrum Master | |
| A, motion and safety | |
| B, planning and simulation | |
| C, quality and measurement | |

## Goals

| Goal | Done when | Issues |
|---|---|---|
| G1 Driver publishes joint states from the Jetson container | /joint_states at 50 Hz or more for 10 min, no period above 40 ms, measured with ros2 topic hz and a bag | #19 #21 #22 #23 |
| G2 Real arm mirrored in RViz on a laptop | Arm moved by hand, RViz follows with less than 100 ms delay, no mirrored joint | #24 |
| G3 CI in the team image on every PR | Build, 1 GoogleTest, 1 pytest, format check, ruff, all green on a PR to dev | #28 |
| G4 Simulation baseline | URDF in RViz with sliders, MoveIt demo planning and executing with mock hardware, Gazebo running a 3-point trajectory | #25 #26 |
| G5 Architecture document v1 | Layers, node graph with topic names, message types, draft timing budget, reviewed by all three pairs | #29 |
| G6 E-stop button read by a script | Wired to the GPIO, pressed reads 1, wire cut reads 1, released reads 0 | #27 |
| G7 Serial link measured | Rate and jitter of get_angles over 10 min, plot, update rate proposed | #20 |

Numbers are initial targets, confirmed with the first measurements (#20).

Not in this sprint: commands to the arm from ROS 2 (sprint 2), MoveIt on
hardware (sprint 3), HIL runner (sprint 3).

## Sprint backlog

A
- #19 Jetson driver image (arm64) starts on boot
- #21 Serial protocol library in C++
- #22 Hardware interface plugin: init and read with a fake backend
- #23 /joint_states at 50 Hz from the Jetson container

B
- #24 Real arm mirrored in RViz on a laptop
- #25 mycobot description and MoveIt config building on Jazzy
- #26 Arm in Gazebo with ros2_control
- #27 E-stop button wired to the Jetson GPIO and read by a script

C
- #20 Measure the serial link: get_angles rate and jitter
- #28 CI in the team Docker image on every PR
- #29 Architecture document v1
- #31 Sprint 01 file: goals, daily log, review

Order inside the sprint: #20 first (two days, fixes the update rate for #22
and #23), #19 by Wednesday of week 1 so #23 can run on the Jetson in week 2,
#24 after #23.

## Definition of done

- PR to dev, 1 review, CI green
- Tests added or updated, or the PR says why not
- Docs updated if an interface or behaviour changed
- Numbers in the PR evidence section when the story has one

## Bench calendar

| | Mon | Tue | Wed | Thu | Fri |
|---|---|---|---|---|---|
| Morning | C (#20) | A | A | B (#24) | demo prep |
| Afternoon | A | B | C | A | review |

Outside the slot: simulation or fake backend.

## Daily log

One file per day in docs/standups/:
- [2026-10-19](../standups/2026-10-19.md)
- [2026-10-20](../standups/2026-10-20.md)
- [2026-10-21](../standups/2026-10-21.md)
- [2026-10-22](../standups/2026-10-22.md)
- [2026-10-23](../standups/2026-10-23.md)
- [2026-10-26](../standups/2026-10-26.md)
- [2026-10-27](../standups/2026-10-27.md)
- [2026-10-28](../standups/2026-10-28.md)
- [2026-10-29](../standups/2026-10-29.md)
- [2026-10-30](../standups/2026-10-30.md)

## Notes for the review

Add a line here during the sprint: anything measured, broken, learned or
worth showing on Friday.

-

## Review (30 Oct)

| Goal | Done | Evidence |
|---|---|---|
| G1 | | |
| G2 | | |
| G3 | | |
| G4 | | |
| G5 | | |
| G6 | | |
| G7 | | |

Demo: arm moved by hand, RViz follows. CI green on a PR. Gazebo trajectory.

Measured this sprint: joint state rate, mirror delay, serial link jitter.

Challenges:

Areas to improve:

## Sprint 2 goals, draft

- Hardware interface write() and joint_trajectory_controller on the real arm
- Joint limit check and communication timeout in the driver
- Same 3-point trajectory in Gazebo and on hardware, difference plotted
- Real-time profile v1
