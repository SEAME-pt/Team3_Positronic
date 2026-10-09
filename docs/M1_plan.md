# M1 issues: epics and user stories

Issues opened on 9 Oct 2026. Sprint in brackets.

## EPIC 1 (#12): Arm visible in ROS 2 (driver, read only)

- #19 Jetson driver image (arm64) starts on boot [S1]
- #20 Measure the serial link: get_angles rate and jitter [S1]
- #21 Serial protocol library in C++ [S1]
- #22 Hardware interface plugin: init and read with a fake backend [S1]
- #23 /joint_states at 50 Hz from the Jetson container [S1]
- #24 Real arm mirrored in RViz on a laptop [S1]

## EPIC 2 (#13): Arm executes trajectories (driver, write)

- #32 Hardware interface write() with position commands [S2]
- #33 joint_trajectory_controller on the real arm [S2]
- #34 Joint limit check in the plugin [S2]
- #35 Communication timeout watchdog (200 ms) [S2]

## EPIC 3 (#14): Simulation with the same controllers

- #25 mycobot description and MoveIt config building on Jazzy [S1]
- #26 Arm in Gazebo with ros2_control [S1]
- #37 Sim-to-real script and report v1 [S2]
- #53 Sim-to-real report final [S4]

## EPIC 4 (#15): Cartesian goal pose through MoveIt 2

- #41 MoveIt 2 on the real arm, Plan and Execute from RViz [S3]
- #42 Planning scene: table, base and workspace limits [S3]
- #43 Mission node: go-to-pose action and state machine [S3]
- #44 Pose error measurement: 20 goals on hardware [S3]

## EPIC 5 (#16): Emergency stop and lifecycle

- #27 E-stop button wired to the Jetson GPIO and read by a script [S1]
- #36 Lifecycle in the driver [S3]
- #45 E-stop halt logic in the driver [S3]
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

7 epics, 33 stories.

| Sprint | Stories |
|---|---|
| S1 | 12 |
| S2 | 8 |
| S3 | 9 |
| S4 | 4 |

## Possible split by pairs

Three pairs, one per block. One owner per story inside the pair.

- A, motion and safety (C++): driver, ros2_control, lifecycle, e-stop,
  limits, timeout, fault injection
- B, planning and simulation: URDF, MoveIt, Gazebo, sim-to-real, mission
  node, RViz
- C, quality and measurement: CI, HIL runner, coverage, measurements,
  documents, sprint files

### Sprint 1

**A** (4)
- #19 Jetson driver image (arm64) starts on boot
- #21 Serial protocol library in C++
- #22 Hardware interface plugin: init and read with a fake backend
- #23 /joint_states at 50 Hz from the Jetson container

**B** (4)
- #24 Real arm mirrored in RViz on a laptop
- #25 mycobot description and MoveIt config building on Jazzy
- #26 Arm in Gazebo with ros2_control
- #27 E-stop button wired to the Jetson GPIO and read by a script

**C** (4)
- #20 Measure the serial link: get_angles rate and jitter
- #28 CI in the team Docker image on every PR
- #29 Architecture document v1
- #31 Sprint 01 file: goals, daily log, review

### Sprint 2

**A** (4)
- #32 Hardware interface write() with position commands
- #33 joint_trajectory_controller on the real arm
- #34 Joint limit check in the plugin
- #35 Communication timeout watchdog (200 ms)

**B** (1)
- #37 Sim-to-real script and report v1

**C** (3)
- #38 clang-tidy and cppcheck in CI
- #39 Real-time profile script
- #40 Sprint 02 file: goals, daily log, review

### Sprint 3

**A** (2)
- #36 Lifecycle in the driver
- #45 E-stop halt logic in the driver

**B** (3)
- #41 MoveIt 2 on the real arm, Plan and Execute from RViz
- #42 Planning scene: table, base and workspace limits
- #43 Mission node: go-to-pose action and state machine

**C** (4)
- #44 Pose error measurement: 20 goals on hardware
- #46 Coverage gate at 80 %
- #47 Self-hosted HIL runner on the Jetson
- #48 Sprint 03 file: goals, daily log, review

### Sprint 4

**A** (1)
- #52 E-stop demo with timestamps and video

**B** (1)
- #53 Sim-to-real report final

**C** (2)
- #54 Architecture document final
- #56 Sprint 04 file: goals, daily log, review

| Sprint | A | B | C |
|---|---|---|---|
| S1 | 4 | 4 | 4 |
| S2 | 4 | 1 | 3 |
| S3 | 2 | 3 | 4 |
| S4 | 1 | 1 | 2 |
