# Sprint 3

16 Nov to 27 Nov 2026. Planning on Monday 16 Nov; review and retrospective
on Friday 27 Nov.

Planning presentation: `07_presentations/sprint_03/planning`.

Sprint goal: the first functional version: Cartesian moves, gripper and
camera on the real arm.

| Role                                                       | Who                                                                    |
| ---------------------------------------------------------- | ---------------------------------------------------------------------- |
| Scrum Master (chair, scribe, weekly with the facilitators) | Wallace Lucas (@wlucas-f)                                              |
| Owners                                                     | One per issue on the board, and one person per outcome. No fixed teams |

Module: **M1**

## Outcomes

| Outcome | Done when | Issues | Owner |
|---|---|---|---|
| O1 Cartesian moves on arm | The same 5 Cartesian goals planned and executed in Gazebo, then from RViz on the real arm, MoveIt success and no JTC abort; GoToPose to 3 poses on the arm, and the mission node killed mid-trajectory halts the arm within 300 ms, 3 runs; launch test with `backend:=fake` in CI | #41 #104 | Vinícius |
| O2 Camera on the arm | Camera mount in the model matches the real arm; the wrist camera publishes images and `camera_info` from the Jetson in the URDF camera frame, calibrated (reprojection error in the README), `/joint_states` keeps 30 Hz with the camera on; age of the pose sample exported and recorded for 60 s | #97 #138 #120 | Leandro |
| O3 Gripper on the arm | Open, close and half through the gripper action, 5 runs each on the arm, within the recorded tolerance; MoveIt opens and closes it from RViz | #123 | Pedro |
| O4 Driver checks the arm | halt() and power_on() return only after the firmware ack; a command the arm does not follow raises TRACKING_ERROR; each fault's reason on `/diagnostics`; the 0.3 s loop pauses reproduced and explained in `pauses.md` | #103 #121 #124 | Rui |
| O5 Speed and dead-man | Firmware speed 30, 50 and 70 swept, with a table that keeps or lowers the 15 deg/s limit of #41; the driver stops on its own when the supervisor goes silent (SUPERVISOR_LOST), tested on the arm | #133 #136 | Felipe |
| O6 CI tests like the arm | FakeArm with the firmware behaviour measured in #33 (lag, wrist start-up, rest error, cut link) and a trajectory test in CI; coverage gate blocks below 80 % from the last day | #122 #46 | Wallace |

Also this sprint, outside the outcomes: `make format` idempotent and checked
in CI (#119, Felipe), architecture diagrams in the repo (#125, Felipe), the
sprint file (#48, Wallace).

Not in this sprint: servo temperature fault (moved to sprint 2 as the full
implementation of #115; #126 closed as not planned), pose error over 20 goals
(#44), e-stop demo (#52), final sim-to-real report (#53), final architecture
document (#54). Sprint 02 issues still open on 16 Nov are decided at the
planning.

## Sprint backlog

| Issue                                | Owner    | Week 1                                                              | Week 2                               | Days      | Depends on                                                |
| ------------------------------------ | -------- | ------------------------------------------------------------------- | ------------------------------------ | --------- | --------------------------------------------------------- |
| #41 MoveIt 2 on the real arm         | Vinícius | Part 1: real model with gripper and camera, yaml, 5 goals in Gazebo | Part 2: 5 goals on the arm           | 4         | #97; #124 for the limit (starts at 15 deg/s, scaling 0.3) |
| #104 Mission chain on the real arm   | Vinícius | Launch test in simulation                                           | 3 poses and node kill on the arm     | 2         | #41 part 1 (sim), part 2 (arm)                            |
| #97 Camera mount in the model        | Leandro  | Mount decided, URDF fixed (day 1)                                   | –                                    | 1         | –                                                         |
| #138 Wrist camera on the real arm    | Leandro  | Camera identified, driver in the Jetson image, launch               | Calibration, verification on the arm | 2–3 (USB) | #97                                                       |
| #120 Age of the pose sample          | Leandro  | –                                                                   | Interface, 60 s on the arm           | 1         | #103 (serial queue)                                       |
| #123 Gripper on the real arm         | Pedro    | Part 1: bytes recorded, frames                                      | Part 2: interface, controller, runs  | 5         | #41 part 1, serial queue, #122                            |
| #103 Stop confirmed by the firmware  | Rui      | Part 1: ack (by day 2); part 2: tracking fault (by day 5)           | –                                    | 3         | –                                                         |
| #124 Control-loop pauses             | Rui      | Runs alone on days 1–3, vmstat                                      | `pauses.md`, limit recommendation    | 2         | –                                                         |
| #121 Driver diagnostics              | Rui      | –                                                                   | Diagnostics node                     | 3         | #103                                                      |
| #119 make format idempotent in CI    | Felipe   | Format PR and CI step (day 1)                                       | –                                    | 0.5       | After PR 117, before any serial branch                    |
| #133 Firmware speed spike            | Felipe   | Parameter; one arm day, sweep 30/50/70                              | Table and `joint_limits.yaml` advice | 2         | #115 temperature request                                  |
| #136 Dead-man check                  | Felipe   | –                                                                   | Dead-man, last in the serial queue   | 2         | #45 merged, #35 FaultCode                                 |
| #125 Architecture diagrams           | Felipe   | –                                                                   | Six diagrams                         | 2         | #45 on dev                                                |
| #122 FakeArm with firmware behaviour | Wallace  | Lag, wrist, rest error, link cut                                    | –                                    | 4         | –                                                         |
| #46 Coverage gate at 80 %            | Wallace  | Missing tests                                                       | Gate blocks on the last day          | 1         | –                                                         |
| #48 Sprint 03 file                   | Wallace  | Goals, weekly notes                                                 | Review and retro                     | 1         | –                                                         |

Days are estimates from the Sprint 03 proposal and the issues. #138 assumes
a USB camera; a CSI camera means 4 to 5 days and is re-estimated first.
Whoever finishes early pulls one of the 1-day sprint 4 issues: #127, #128 or
#134.

Order inside the sprint:
- Day 1: #119 (`make format` on dev, CI fails when it changes anything,
  before any serial branch) and #97 (camera fixed before MoveIt moves the
  arm, because of the cable at +90 deg on joint 6).
- #97 before #41 and #138: the camera frame must be in the model first.
- #124 runs alone on days 1–3; its result keeps 15 deg/s or lowers it before
  `joint_limits.yaml` is closed. #133 gets its arm day before #41's.
- Serial queue, one PR at a time on `serial_arm.cpp` and the plugin: #103
  part 1 (day 2), #103 part 2 (day 5), #120, #123, #133, then #136 in week 2.
- FaultCode values are never renumbered: #103 uses TRACKING_ERROR = 2, #115
  appends its value, #136 appends SUPERVISOR_LOST.
- #122 closes in week 1, before #123 adds the gripper command to FakeArm.
- PRs in two parts: #103, #123, #133, #41. The others are one PR each.
- If #41 part 2 slips, the arm runs of #104 and the RViz part of #123 move to
  sprint 4; the rest of the sprint stands.

## How we work (new this sprint)

- **Weekly on Wednesday** (18 and 25 Nov), the whole team: how satisfied
  the group is, which the quick, technical dailies miss, and an agreed status
  the Scrum Master takes to the facilitators' weekly.
- **Blockers go to the Scrum Master**, the official channel: raised the day
  they appear, logged in the standup file.
- **Dependencies through the Scrum Master**: when an issue needs another
  first, or two issues touch the same file, the order is agreed through the
  SM. One PR at a time on a shared file.
- **AI review**: Claude reviews our PRs as a GitHub Action, on trial this
  sprint. Its comments are one more input; the two approvals and green CI
  still decide the merge. At the review we report what it caught.
- **Single point of follow-up: the SM.** Delays and dependencies are followed
  up by the Scrum Master with each owner, not chased informally between
  teammates, so the follow-up stays professional and in one place.

## Definition of done

- PR to dev, 2 approvals, CI green, `make format-check` clean
- Tests added or updated, or the PR says why not
- Docs updated if an interface or behaviour changed
- Numbers in the PR evidence section when the story has one

## Bench sessions

Every session needs the real arm: bench clear, hand on the plug, two people,
one person moving the arm.

| Issue | Session |
|---|---|
| #97 | Joint 6 to +90 deg once, photo next to Gazebo (day 1) |
| #124 | Days 1–3, JTC goals of #33, alone |
| #133 | One day, two people: 30 → 50 → 70, never 100 |
| #41 | 5 goals, after #97 fixed the camera |
| #138 | Joint 6 to +90 deg at speed 30 for the comparison image; 2 min of `rt_record.py` with the camera on |
| #103 | Ack delay, 20 samples; tracking fault |
| #121 + #120 | Fault injected; 60 s of pose age |
| #136 | Supervisor stopped, arm halts (week 2) |
| #104 | 3 poses, node kill, stop from the mission node |
| #123 | Twice: record the bytes; runs at the end |

Slots are booked through the Scrum Master.

## Daily log

One file per day in `../standups/sprint_03/`.

## Notes for the review

-

## Review & retrospective (27 Nov 2026)

| Outcome | Achieved | Evidence / reason |
|---|---|---|
| O1 | | |
| O2 | | |
| O3 | | |
| O4 | | |
| O5 | | |
| O6 | | |

Demo: Deck: `07_presentations/sprint_03/review`.

Measured this sprint:

Went well: Went badly: Actions:

## Sprint 4 goals, draft

From the Sprint 03/04 proposal:
- E-stop demo with video (#52), final sim-to-real report (#53), final
  architecture document (#54), pose error over 20 goals (#44)
- End-to-end latency budget, rosbag of every arm session
