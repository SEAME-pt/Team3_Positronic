# Test review (#46)

What the unit tests of each package really check, what they miss, and what we decided to do
about each gap. Coverage says which lines ran; this review says whether the behaviour was
checked.

## How a package is reviewed

1. One row per behaviour: is the normal case tested, the failure path (bad input, timeout, lost
   link), the edges (exactly at a limit, just past it, NaN, ±inf, empty, oversized)?
2. Look at the tests themselves: does each one assert something that matters?
3. Each finding gets a decision:
   - **fix now**: a safety path or a failure path with no test (an `issue` in `CLAUDE.md`
     terms). Done in #46.
   - **later**: useful, not blocking. Listed in a follow-up issue.
   - **new issue**: not a missing test but missing behaviour (a feature).

## Coverage at the start (2026-11-17, `make coverage`)

| Package | Coverage | Gate |
|---|---:|:---:|
| positronic_protocol | 100 % | PASS |
| positronic_hardware | 92.2 % | PASS |
| positronic_safety | 86.0 % | PASS |
| positronic_bringup | 82.4 % | PASS |
| positronic_mission | 75.3 % | FAIL |
| positronic_moveit_config | 56.2 % | FAIL |

`positronic`, `positronic_interfaces`, `positronic_description` and `positronic_hil` have no
code that coverage counts.

## Packages

- [positronic_protocol](positronic_protocol.md)
- [positronic_hardware](positronic_hardware.md)
- [positronic_safety](positronic_safety.md)
- [positronic_mission](positronic_mission.md)
- [positronic_moveit_config](positronic_moveit_config.md)
- [positronic_bringup](positronic_bringup.md)

## Carried to other packages

Questions a review raised that belong to another package's review.

| From | To | Question | Answer |
|---|---|---|---|
| protocol F1 | hardware | Does `read()` reject a reading that is impossible (outside the joint range, or too far from the last one)? | No: hardware H2, new issue |
| protocol F2 | hardware | `decode_ack` is never called outside the protocol tests: the acks are not read. Is a refused command noticed? | No: hardware H3, new issue |

## Status

| Package | Review | Fix now | Later | New issue |
|---|---|---|---|---|
| positronic_protocol | Done 2026-11-17 | – | F1 test, F2 README note, F3 (with the acks issue), F4 cleanup | – |
| positronic_hardware | Done 2026-11-17 | H1 shutdown test, H4 `power_on` failure test | H5 (top), H1 crash note, H4 `write()` while inactive | H2 reading plausibility, H3 read the acks |
| positronic_safety | Done 2026-11-18 | S1 shutdown test, S2 stale fault code test, S3 controller manager gone test | S4 locking test + README sentence, S5 four refusal tests, RESUMING README note | S1 `main()` Ctrl+C halt (or #136), S4 heartbeat ADR |
| positronic_mission | Done 2026-11-19 | M1 `MoveitBackend` tests (gate), M2 MoveIt not ready, M3 pose reaches the backend, M6 flaky feedback check | M4 heartbeat timing margins, M5 small gaps | Linters off in `CMakeLists.txt` |
| positronic_moveit_config | Done 2026-11-19 | C1 `apply()`/`main()` tests (gate) | C2 bad scene files, C3 joint limits config test | C1 a failed scene load must stop the system |
| positronic_bringup | Done 2026-11-19 | – | B1 delay tool `main()`, B2 heartbeat timeout vs measured gap | – |

## Phase 2 work list (fix now)

| # | Package | Test | Estimate |
|---|---|---|---|
| 1 | mission | M1 `test_moveit_backend.cpp` (stop before planning, `reset_stop`, not connected, `ready()` false) | 1 h |
| 2 | moveit_config | C1 `apply()` with a fake `/apply_planning_scene`: success, refused, no service; `main()` exit code | 1 h |
| 3 | mission | M2 activation refused when MoveIt is not ready; M3 the goal's pose reaches the backend | 35 min |
| 4 | hardware | H1 ACTIVE → FINALIZED sends a stop on the pseudo-terminal | 1 h |
| 5 | hardware | H4 `on_activate` when `power_on` cannot be sent | 1 h |
| 6 | safety | S1 `shutdown()` while MONITORING sends "hardware inactive" | 30 min |
| 7 | safety | S2 stale fault code with a stamp from before the resume (+ opposite case, mutation check) | 45 min |
| 8 | safety | S3 halt with the controller manager gone | 45 min |

Items 1–2 make the gate pass; do them first. Then Phase 3: remove `continue-on-error: true` from
the coverage step in `.github/workflows/ci.yml`.

### Progress

| # | Result |
|---|---|
| 1 | **Blocked.** `test/test_moveit_backend.cpp` written; it found two bugs in `MoveitBackend` (see mission M1, "What the test found"). Sent to the package owner 2026-11-19. The test hangs until bug 1 is fixed, so it is not registered in `CMakeLists.txt` yet. **Mission stays below 80 % (about 76 %) until it runs: Phase 3 waits for it.** |
| 2 | **Done.** `test/test_planning_scene_node.py`, 6 tests, 1.9 s. moveit_config 56 % → 98 % (left: line 38, C2; line 133, `__main__`). Mutation check: dropping `not result.success` fails `test_apply_is_false_when_move_group_refuses`. |
| 3 | **Done.** M2 `ActivationRefusedWhenBackendNotReady` (`test_lifecycle.cpp`), M3 the pose check in `GoalSucceedsWhenIdle` + `EachGoalSendsItsOwnPose` (`test_goals.cpp`); `MockBackend` got `ready_result` and `last_pose()`. Mutation checks: a default pose sent to the backend fails both pose tests; deleting the ready check fails the M2 test. |
| 4 | **Done.** H1 `ShutdownFromActiveSendsAStop` (`test_load_robot_system.cpp`): ACTIVE → FINALIZED through the resource manager, the stop frame `FE FE 02 29 FA` reaches the pseudo-terminal. Mutation: `on_deactivate` without `halt()` fails it, which also confirms that the installed ros2_control deactivates before shutdown. |
| 5 | **Done.** H4 `ActivateFailsWhenPowerOnCannotBeSent`: the arm answers, then goes away; activate gives ERROR after ≥ `REQUEST_TIMEOUT`, then UNCONFIGURED. Mutation: ignoring the `power_on` result fails it. |
| 6 | **Done.** S1 `ShutdownWhileMonitoringSendsHardwareInactive`. Mutation found that the installed rclcpp_lifecycle runs `on_deactivate` before `on_shutdown` on a shutdown from ACTIVE: `on_shutdown`'s halt is a second guard never reached here (see safety S1). The test checks the outcome. |
| 7 | **Done.** S2 `StaleFaultCodeAfterAResumeDoesNotHaltButANewOneDoes`: new fake knob `stale_fault_after_activation` (every other message for 500 ms carries code 1 stamped before the activation; alternating because the supervisor keeps only the newest message). A fresh code 1 afterwards still halts. Mutation: removing the filter fails it. |
| 8 | **Done.** S3 `HaltWithTheControllerManagerGoneEndsAtOnceAndSaysNotStopped`: the fake drops its services; the halt ends STOPPED in < 200 ms with `hardware_stopped == false`. Mutation: waiting for a missing service instead of failing at once fails it. |
| + | **Done.** Mission M6 (from #161): the flaky feedback check in `GoalSucceedsWhenIdle` removed; `test_goals` 10/10 under parallel load. `TEST_ALONE` for mission (#161) can go after this merges. |

## Issues to open

- **New issues:** reading plausibility in `SerialArm` (H2, F1); read the acks, starting with stop
  (H3, F3); supervisor `main()` halts on Ctrl+C (S1, or add to #136); what must the heartbeat
  prove, with an ADR (S4); a failed planning scene load must stop the system (C1); linters off in
  `positronic_mission` (side note).
- **One follow-up issue** with every "later" item in the Status table.
