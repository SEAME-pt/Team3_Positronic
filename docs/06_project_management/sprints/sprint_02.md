# Sprint 2

2 Nov to 13 Nov 2026 (the sprint window runs to Sat 14 Nov, but there is no
daily on the weekend). Planning on Mon 2 Nov; review and retrospective on
Fri 13 Nov. Module: M1.

Planning deck: `../../07_presentations/sprint_02/planning`.
Review deck: `../../07_presentations/sprint_02/review`.

Sprint goal: the real arm executes joint trajectories from ROS 2 with position
commands, stops safely on a joint limit or a lost link, and the same motion is
compared between simulation and hardware.

Issue: #40 Sprint 02 file: goals, daily log, review.

| Role | Who |
|---|---|
| Scrum Master | Leandro Gertrudes |
| A, motion and safety | Felipe, Pedro, Rui |
| B, planning and simulation | Leandro, Vinicius, Wallace |

Team: Felipe, Leandro, Pedro, Rui, Vinicius, Wallace.

## Goals

Numbers are proposals, confirmed at planning and against the sprint 1
measurements (#20, #23).

| Goal | Done when | Issues |
|---|---|---|
| G1 Hardware interface sends position commands | write() sends joint targets over serial at the rate fixed in sprint 1, the arm moves to a commanded pose, no command sent after a timeout | #32 |
| G2 Joint trajectory controller on the real arm | joint_trajectory_controller executes a 3-point trajectory on hardware at speed ≤ 30, final joint error reported | #33 |
| G3 Driver is safe on limits and silence | A command beyond a joint limit is rejected with a log line; a link timeout and firmware silence are detected and the driver halts, both shown with a test | #34 #35 |
| G4 Sim-to-real report v1 | Same 3-point trajectory in Gazebo and on hardware, difference plotted per joint, script reruns from one command | #37 |
| G5 Static analysis in CI | clang-tidy and cppcheck run on every PR to dev, warnings visible in the PR | #38 |
| G6 Real-time profile v1 | Script measures loop period and jitter of the driver, plot of period over time and a histogram | #39 |

Not in this sprint: lifecycle and safety supervisor, MoveIt on hardware,
coverage gate and HIL runner. (Several of these were pulled in during the
sprint and delivered anyway — see the review.)

## Sprint backlog

Scrum Master: #40 Sprint 02 file.

A
- #32 Hardware interface write() with position commands
- #33 joint_trajectory_controller on the real arm
- #35 Serial link timeout and firmware silence behaviour

B
- #34 Joint limit check in the plugin
- #37 Sim-to-real script and report v1

C (decided at planning)
- #38 clang-tidy and cppcheck in CI
- #39 Real-time profile script

Order inside the sprint: #32 first, because #33 and #34 depend on write().
#35 can start in parallel on the timeout side. #37 needs #33 for the hardware
run, so prepare the Gazebo side and the plot script in week 1.

## Definition of done

- PR to `dev`, 2 approvals (pair partner + one from another pair), CI green, `make format-check` clean.
- Tests added or updated, or the PR says why not.
- Docs updated if an interface or behaviour changed.
- Numbers in the PR evidence section when the story has one.
- Safety: new motion code starts at speed ≤ 30, and the emergency stop always wins.

## Bench calendar

| | Mon | Tue | Wed | Thu | Fri |
|---|---|---|---|---|---|
| Week 1 morning | planning | <...> | <...> | <...> | <...> |
| Week 1 afternoon | <...> | <...> | <...> | <...> | <...> |
| Week 2 morning | <...> | <...> | <...> | <...> | demo prep |
| Week 2 afternoon | <...> | <...> | <...> | <...> | review |

Outside the slot: simulation or fake backend. Only one process can use the
serial port at a time.

## Risks

- First write() on hardware: the arm can move fast. Start at speed ≤ 30, keep a hand near the stop.
- Only one process on the serial port: do not run the driver and a measurement script together.
- Bench time is shared by the pairs.

## Daily log

Daily at 09:30, 15 min, filled at the end of each daily. The weekly PR
(`docs(standups): sprint 02 week W`) goes on Fri 6 Nov and Fri 13 Nov. One
file per day in `../standups/sprint_02/`:

- [2026-11-02](../standups/sprint_02/2026-11-02.md)
- [2026-11-03](../standups/sprint_02/2026-11-03.md)
- [2026-11-04](../standups/sprint_02/2026-11-04.md)
- [2026-11-05](../standups/sprint_02/2026-11-05.md)
- [2026-11-06](../standups/sprint_02/2026-11-06.md)
- [2026-11-09](../standups/sprint_02/2026-11-09.md)
- [2026-11-10](../standups/sprint_02/2026-11-10.md)
- [2026-11-11](../standups/sprint_02/2026-11-11.md)
- [2026-11-12](../standups/sprint_02/2026-11-12.md)
- [2026-11-13](../standups/sprint_02/2026-11-13.md)

## Notes for the review

- Also delivered beyond the backlog: safety supervisor #45 (every stop halted 51–106 ms), safe stop frame #36 (6–16 ms), mission node #43 (GoToPose and state machine through MoveIt, demoed into Gazebo), servo over-temperature cut #115 (60 °C), coverage gate #46 (3 of 5 packages ≥ 80 %), HIL runner #47 (self-hosted Jetson runner, read-only), joint 6 zero offset #94, and a code-review agent in CI #118.
- <...>

## Review & retrospective (13 Nov)

| Goal | Achieved | Evidence |
|---|---|---|
| G1 Hardware interface position commands (#32) | Yes | write() sends joint targets every 33 ms, one serial turn ~50 ms, newest target wins; one joint moves to a commanded pose. |
| G2 Trajectory controller on the real arm (#33) | Yes | 3-point trajectory on hardware; largest joint error 3.1°–4.9° across 10–20°/s, inside the 5° trajectory tolerance; above 20°/s aborts. |
| G3 Driver safe on limits and silence (#34 #35) | Yes | #34: CommandFault checked in write() before the servos (WRONG_COUNT, NOT_FINITE, BELOW_LOWER, ABOVE_UPPER), refused → ERROR → halt. #35: no valid reply for 200 ms stops the writes, measured 194–201 ms over 6 trials, fault published as `driver/fault_code`. |
| G4 Sim-to-real report v1 (#37) | Yes | Same trajectory, simulation vs 3 real runs; worst peak 3.57° (joint6_to_joint5), RMS ≤ 1.7°. |
| G5 Static analysis in CI (#38) | Yes | clang-tidy and cppcheck run on every PR; warnings fail the PR. |
| G6 Real-time profile v1 (#39) | Yes | Jetson loop period and jitter profiled over 10 min (idle and write() runs), with a histogram. |

Demo: the arm executes a 3-point trajectory, a limit rejection and a timeout
halt are shown, and a Gazebo-vs-hardware plot; plus GoToPose → MoveIt → Gazebo.

Measured this sprint: command write timing, final joint error 3.1°–4.9°,
dead-link stop 194–201 ms, supervisor halt 51–106 ms, safe stop frame 6–16 ms,
sim-to-real worst peak 3.57°.

### Went well

- All six goals were delivered, and each one has a measurement behind it. The raw data of
  every run is under `docs/03_verification/`, one folder per issue.
- The interface between #35 (link timeout), #36 (lifecycle) and #45 (supervisor) was agreed
  in the first week, before the code. That is what let the supervisor land inside the sprint.
- The first motion of the arm from ROS 2 (#32) came on day 4, early enough for #33, #34 and
  #37 to run on hardware in the second week.

### Went badly

- The dependencies between issues were not studied at planning. #34 waited for #32, #35 for
  #36, #37 for #33, and #33 for the first motion, so the second week carried most of the
  hardware work and the pairs waited for each other.

### Actions

- At planning, draw the dependencies between the issues before assigning them: which issue
  needs which, which files they share, and the order of the PRs. Sprint 3 planning (16 Nov)
  already lists the shared files and "one PR at a time" on each issue.

## Sprint 3 goals, draft

- Lifecycle in the driver and safety supervisor with e-stop halt logic
- MoveIt 2 on the real arm, mission node with go-to-pose action
- Coverage gate at 80 % and self-hosted HIL runner on the Jetson
- Pose error measurement: 20 goals on hardware
