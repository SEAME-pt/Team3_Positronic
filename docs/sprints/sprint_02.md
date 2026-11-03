# Sprint 2

2 Nov to 13 Nov 2026 (the sprint window runs to Sat 14 Nov, but there is no daily on the weekend). Planning on Mon 2 Nov; review and retrospective on Fri 13 Nov.

Sprint goal: the real arm executes joint trajectories from ROS 2 with position commands, stops safely on a joint limit or a lost link, and the same motion is compared between simulation and hardware.

Issue: #40 Sprint 02 file: goals, daily log, review.

| Role | Who |
|---|---|
| Scrum Master | |
| A, motion and safety | Felipe, Pedro, Rui |
| B, planning and simulation | Leandro, Vinicius, Wallace |
| C, quality and measurement | nobody assigned yet |

Team: Felipe, Leandro, Pedro, Rui, Vinicius, Wallace.

Module: **M1**

## Goals

Numbers are proposals. Confirm them at planning, and with the measurements from sprint 1 (#20, #23).

| Goal | Done when | Issues |
|---|---|---|
| G1 Hardware interface sends position commands | write() sends joint targets over serial at the rate fixed in sprint 1, arm moves to a commanded pose, no command sent after a timeout | #32 |
| G2 Joint trajectory controller on the real arm | joint_trajectory_controller executes a 3-point trajectory on hardware at speed <= 30, final joint error reported | #33 |
| G3 Driver is safe on limits and silence | A command beyond a joint limit is rejected with a log line; link timeout and firmware silence are detected and the driver halts, both shown with a test | #34 #35 |
| G4 Sim-to-real report v1 | Same 3-point trajectory in Gazebo and on hardware, difference plotted per joint, script reruns from one command | #37 |
| G5 Static analysis in CI | clang-tidy and cppcheck run on every PR to dev, warnings visible in the PR | #38 |
| G6 Real-time profile v1 | Script measures loop period and jitter of the driver, plot of period over time and a histogram | #39 |

Not in this sprint: lifecycle and safety supervisor (sprint 3), MoveIt on hardware (sprint 3), coverage gate and HIL runner (sprint 3).

## Sprint backlog

A
- #32 Hardware interface write() with position commands
- #33 joint_trajectory_controller on the real arm
- #35 Serial link timeout and firmware silence behaviour

B
- #34 Joint limit check in the plugin
- #37 Sim-to-real script and report v1

C (nobody assigned yet, decide at planning who takes these)
- #38 clang-tidy and cppcheck in CI
- #39 Real-time profile script

Carry-over from sprint 1 (fill at planning):

-

Order inside the sprint: #32 first, because #33 and #34 depend on write(). #35 can start in parallel on the timeout side. #37 needs #33 for the hardware run, so prepare the Gazebo side and the plot script in week 1.

## Definition of done

- PR to dev, 2 approvals (pair partner + one from another pair), CI green, `make format-check` clean
- Tests added or updated, or the PR says why not
- Docs updated if an interface or behaviour changed
- Numbers in the PR evidence section when the story has one
- Safety: new motion code starts at speed <= 30, and the emergency stop always wins

## Bench calendar

| | Mon | Tue | Wed | Thu | Fri |
|---|---|---|---|---|---|
| Week 1 morning | planning | | | | |
| Week 1 afternoon | | | | | |
| Week 2 morning | | | | | demo prep |
| Week 2 afternoon | | | | | review |

Outside the slot: simulation or fake backend. Only one process can use the serial port at a time.

## Risks

- First write() on hardware: arm can move fast. Start at speed <= 30, keep a hand near the stop.
- Only one process on the serial port: do not run the driver and a measurement script together.
- Bench time is shared by three pairs.

## Daily log

Daily at 09:30, 15 min. Fill at the end of each daily. The weekly PR (`docs(standups): sprint 02 week W`) goes on Fri 6 Nov and Fri 13 Nov.
Each daily below can also be split into `docs/standups/YYYY-MM-DD.md` if we keep one file per day.

### 2026-11-02 (Mon) · day 1 of 10

Scrum Master: 
Meetings today: Sprint 2 planning

**Felipe**, block A
- Done: terminou os slides da apresentação
- Today: Preparação e planejamento para a proxima sprint iniciando pela issue #45
- Blocked: nothing

**Leandro**, block B
- Done: Terminou os slides da apresentação de feedback e apresentou
- Today: Fazer os slides do planning e apresenta-los
- Blocked: nothing.

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: finish measuring the latency and prepare sprint  01 revuew to presented the controllers and mirror slides, pick video and linve RViz mirror of the real arm
- Today: help review the sprint 02 planning presentation. start #32 branch created from dev, read the current driver to see where write() fits, design how the serial thread alternates get_angles and send_angles at 30gz withouth stalls 
- Blocked: nothing.

**Vinicius**, block B
- Done: Apresentou o feedback
- Today: Finalizar o lifeCycle node
- Blocked: entender action

**Wallace**, block B
- Done: Estudou o que foi feito na sprint fez uma animação para a apresentação e realizou a apresentação.
- Today: Escolher as issues que quer trabalhar e vai planeja-las e estuda-las.
- Blocked: Nada

Challenges:
- 

Decisions:
- 

---

### 2026-11-03 (Tue) · day 2 of 10

Scrum Master: 
Meetings today: none

**Felipe**, block A
- Done: 
- Today: 
- Blocked: 

**Leandro**, block B
- Done: 
- Today: 
- Blocked: 

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: 
- Today: 
- Blocked: 

**Vinicius**, block B
- Done: 
- Today: 
- Blocked: 

**Wallace**, block B
- Done: 
- Today: 
- Blocked: 

Challenges:
- 

Decisions:
- 

---

### 2026-11-04 (Wed) · day 3 of 10

Scrum Master: 
Meetings today: none

**Felipe**, block A
- Done: 
- Today: 
- Blocked: 

**Leandro**, block B
- Done: 
- Today: 
- Blocked: 

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: 
- Today: 
- Blocked: 

**Vinicius**, block B
- Done: 
- Today: 
- Blocked: 

**Wallace**, block B
- Done: 
- Today: 
- Blocked: 

Challenges:
- 

Decisions:
- 

---

### 2026-11-05 (Thu) · day 4 of 10

Scrum Master: 
Meetings today: none

**Felipe**, block A
- Done: 
- Today: 
- Blocked: 

**Leandro**, block B
- Done: 
- Today: 
- Blocked: 

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: 
- Today: 
- Blocked: 

**Vinicius**, block B
- Done: 
- Today: 
- Blocked: 

**Wallace**, block B
- Done: 
- Today: 
- Blocked: 

Challenges:
- 

Decisions:
- 

---

### 2026-11-06 (Fri) · day 5 of 10

Scrum Master: 
Meetings today: none

Reminder: open the PR `docs(standups): sprint 02 week 1` today.

**Felipe**, block A
- Done: 
- Today: 
- Blocked: 

**Leandro**, block B
- Done: 
- Today: 
- Blocked: 

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: 
- Today: 
- Blocked: 

**Vinicius**, block B
- Done: 
- Today: 
- Blocked: 

**Wallace**, block B
- Done: 
- Today: 
- Blocked: 

Challenges:
- 

Decisions:
- 

---

### 2026-11-09 (Mon) · day 6 of 10

Scrum Master: 
Meetings today: none

**Felipe**, block A
- Done: 
- Today: 
- Blocked: 

**Leandro**, block B
- Done: 
- Today: 
- Blocked: 

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: 
- Today: 
- Blocked: 

**Vinicius**, block B
- Done: 
- Today: 
- Blocked: 

**Wallace**, block B
- Done: 
- Today: 
- Blocked: 

Challenges:
- 

Decisions:
- 

---

### 2026-11-10 (Tue) · day 7 of 10

Scrum Master: 
Meetings today: none

**Felipe**, block A
- Done: 
- Today: 
- Blocked: 

**Leandro**, block B
- Done: 
- Today: 
- Blocked: 

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: 
- Today: 
- Blocked: 

**Vinicius**, block B
- Done: 
- Today: 
- Blocked: 

**Wallace**, block B
- Done: 
- Today: 
- Blocked: 

Challenges:
- 

Decisions:
- 

---

### 2026-11-11 (Wed) · day 8 of 10

Scrum Master: 
Meetings today: none

**Felipe**, block A
- Done: 
- Today: 
- Blocked: 

**Leandro**, block B
- Done: 
- Today: 
- Blocked: 

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: 
- Today: 
- Blocked: 

**Vinicius**, block B
- Done: 
- Today: 
- Blocked: 

**Wallace**, block B
- Done: 
- Today: 
- Blocked: 

Challenges:
- 

Decisions:
- 

---

### 2026-11-12 (Thu) · day 9 of 10

Scrum Master: 
Meetings today: none

**Felipe**, block A
- Done: 
- Today: 
- Blocked: 

**Leandro**, block B
- Done: 
- Today: 
- Blocked: 

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: 
- Today: 
- Blocked: 

**Vinicius**, block B
- Done: 
- Today: 
- Blocked: 

**Wallace**, block B
- Done: 
- Today: 
- Blocked: 

Challenges:
- 

Decisions:
- 

---

### 2026-11-13 (Fri) · day 10 of 10

Scrum Master: 
Meetings today: Sprint 2 review & retrospective

Reminder: open the PR `docs(standups): sprint 02 week 2` today, then the `dev` to `main` release PR after the review.

**Felipe**, block A
- Done: 
- Today: 
- Blocked: 

**Leandro**, block B
- Done: 
- Today: 
- Blocked: 

**Pedro**, block A
- Done: 
- Today: 
- Blocked: 

**Rui**, block A
- Done: 
- Today: 
- Blocked: 

**Vinicius**, block B
- Done: 
- Today: 
- Blocked: 

**Wallace**, block B
- Done: 
- Today: 
- Blocked: 

Challenges:
- 

Decisions:
- 

---

## Notes for the review

Add a line here during the sprint: anything measured, broken, learned or worth showing on Friday.

-

## Review & retrospective (13 Nov)

| Goal | Done | Evidence |
|---|---|---|
| G1 | | |
| G2 | | |
| G3 | | |
| G4 | | |
| G5 | | |
| G6 | | |

Demo: arm executes a 3-point trajectory, limit rejection and timeout halt shown, Gazebo vs hardware plot.

Measured this sprint: command write rate, final joint error, sim-to-real difference per joint, loop jitter.

Went well:

Went badly:

Actions:

## Sprint 3 goals, draft

- Lifecycle in the driver and safety supervisor with e-stop halt logic
- MoveIt 2 on the real arm, mission node with go-to-pose action
- Coverage gate at 80 % and self-hosted HIL runner on the Jetson
- Pose error measurement: 20 goals on hardware
