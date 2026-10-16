# M1 issues: epics and user stories

Issues in the team repository. Sprint in brackets, "opt" is optional work
after the basics.

## EPIC 1 (#12): Arm visible in ROS 2 (driver, read only)

- #19 Jetson driver image (arm64) starts on boot [S1]
- #20 Measure the serial link: read rate, write streaming, silence [S1]
- #21 Serial protocol library in C++ [S1]
- #22 Hardware interface plugin: init and read with a fake backend [S1]
- #23 /joint_states at the measured rate from the Jetson container [S1]
- #24 Real arm mirrored in RViz on a laptop [S1]

## EPIC 2 (#13): Arm executes trajectories (driver, write)

- #32 Hardware interface write() with position commands [S2]
- #33 joint_trajectory_controller on the real arm [S2]
- #34 Joint limit check in the plugin [S2]
- #35 Serial link timeout and firmware silence behaviour [S2]

## EPIC 3 (#14): Simulation with the same controllers

- #25 mycobot description and MoveIt config building on Jazzy [S1]
- #26 Arm in Gazebo with ros2_control [S1]
- #37 Sim-to-real script and report v1 [S2]
- #53 Sim-to-real report final [S4]

## EPIC 4 (#15): Cartesian goal pose through MoveIt 2

- #41 MoveIt 2 on the real arm, Plan and Execute from RViz [S3]
- #42 Planning scene: table, base and workspace limits [S1]
- #43 Mission node: go-to-pose action and state machine [S3]
- #44 Pose error measurement: 20 goals on hardware [S3]

## EPIC 5 (#16): Emergency stop and lifecycle

- #27 E-stop button wired to the Jetson GPIO and read by a script [opt]
- #36 Lifecycle in the driver [S3]
- #45 Safety supervisor node: e-stop halt logic [S3]
- #52 E-stop demo with timestamps and video [S4]

## EPIC 6 (#17): CI and quality

- #28 CI in the team Docker image on every PR [S1]
- #38 clang-tidy and cppcheck in CI [S2]
- #46 Coverage gate at 80 % [S3]
- #47 Self-hosted HIL runner on the Jetson [S3]

## EPIC 7 (#18): Measurements and documents

- #29 Architecture document v1 [S1]
- #31 Sprint 01 file: goals, daily log, review [S1]
- #39 Real-time profile script [S2]
- #40 Sprint 02 file: goals, daily log, review [S2]
- #48 Sprint 03 file: goals, daily log, review [S3]
- #54 Architecture document final [S4]
- #56 Sprint 04 file: goals, daily log, review [S4]

## Count

7 epics, 33 stories, one optional.

| Sprint | Stories |
|---|---|
| S1 | 12 |
| S2 | 8 |
| S3 | 8 |
| S4 | 4 |

## Possible split by pairs

Three pairs, one per block. One owner per story inside the pair. Sprint files
(#31, #40, #48, #56) are the Scrum Master's, the role rotates. The e-stop demo
(#52) is proposed for B: the pair that tests is not the pair that wrote the
supervisor.

- A, motion and safety (C++): driver, ros2_control, lifecycle, safety
  supervisor, timeouts
- B, planning and simulation: URDF, MoveIt, Gazebo, sim-to-real, mission
  node, RViz, joint limits, fault injection demo
- C, quality and measurement: CI, HIL runner, coverage, measurements,
  documents, Jetson image

### Sprint 1

**A** (3)
- #21 Serial protocol library in C++
- #22 Hardware interface plugin: init and read with a fake backend
- #23 /joint_states at the measured rate from the Jetson container

**B** (4)
- #25 mycobot description and MoveIt config building on Jazzy
- #26 Arm in Gazebo with ros2_control
- #24 Real arm mirrored in RViz on a laptop
- #42 Planning scene: table, base and workspace limits

**C** (4)
- #20 Measure the serial link: read rate, write streaming, silence
- #19 Jetson driver image (arm64) starts on boot
- #28 CI in the team Docker image on every PR
- #29 Architecture document v1

### Sprint 2

**A** (3)
- #32 Hardware interface write() with position commands
- #33 joint_trajectory_controller on the real arm
- #35 Serial link timeout and firmware silence behaviour

**B** (2)
- #34 Joint limit check in the plugin
- #37 Sim-to-real script and report v1

**C** (2)
- #38 clang-tidy and cppcheck in CI
- #39 Real-time profile script

### Sprint 3

**A** (2)
- #36 Lifecycle in the driver
- #45 Safety supervisor node: e-stop halt logic

**B** (2)
- #43 Mission node: go-to-pose action and state machine
- #41 MoveIt 2 on the real arm, Plan and Execute from RViz

**C** (3)
- #46 Coverage gate at 80 %
- #47 Self-hosted HIL runner on the Jetson
- #44 Pose error measurement: 20 goals on hardware

### Sprint 4

**A** (0)

**B** (2)
- #53 Sim-to-real report final
- #52 E-stop demo with timestamps and video

**C** (1)
- #54 Architecture document final

| Sprint | A | B | C |
|---|---|---|---|
| S1 | 3 | 4 | 4 |
| S2 | 3 | 2 | 2 |
| S3 | 2 | 2 | 3 |
| S4 | 0 | 2 | 1 |

Optional, after the basics: #27 physical e-stop button on the Jetson GPIO (B).
