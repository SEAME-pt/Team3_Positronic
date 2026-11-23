# positronic_mission: test review

Reviewed 2026-11-18. Coverage 75.3 % (198 of 263 lines): **below the 80 % gate**. Tests:
`src/positronic_mission/test/`. The node is tested against `MockBackend` (a fake arm that waits
`duration_ms`, then returns `outcome`); `pose_validation` is pure and tested directly.

**What the package does:** the mission node, a lifecycle node. It takes a `GoToPose` goal
(an action), checks the pose, asks the motion backend (MoveIt) to plan and move, and reports
its state (IDLE, INACTIVE, MOVING, STOPPED, ERROR) in a heartbeat every 40 ms. It also has a
stop service and a reset service.

| File | Coverage | Lines never run |
|---|---:|---|
| `src/mission_node.cpp` | 178 / 190 | 51–53 reset while not active, 124 default backend, 150 cleanup join, 163–164 MoveIt not ready, 302–303 shutdown during a goal, 349–351 "should not be reachable" |
| `src/pose_validation.cpp` | 15 / 15 | – |
| `src/moveit_backend.cpp` | **0 / 53** | all |

## Behaviours

| Behaviour | Normal | Failure path | Edges | Verdict |
|---|:-:|:-:|:-:|---|
| Pose check (frame, NaN/inf, quaternion norm) | ✅ | ✅ | ✅ norm just inside / just outside 1e-3, `w = -1` | Very good |
| Goal: accepted when IDLE, result OK | ✅ | | | Good, but see M3 |
| Goal rejected: inactive, bad frame, moving, STOPPED, ERROR | ✅ | ✅ | | Very good |
| Cancel, stop service, stop right after accept | ✅ | ✅ | ✅ stop before the motion starts | Very good |
| Planning failed → IDLE; execution failed → ERROR until reset | ✅ | ✅ | | Very good |
| Deactivate, shutdown, destroy during a goal: stop quickly | ✅ | | | Very good |
| Lifecycle cycles keep the state (IDLE, STOPPED, ERROR) | ✅ | | | Good |
| Activation refused when MoveIt is not ready | | ❌ mock always ready | | See M2 |
| Reset refused while not active | | ❌ | | Later |
| Heartbeat rate, name, stamps | ✅ | | ⚠️ wall-clock window | See M4 |
| `MoveitBackend` (the real one) | ❌ 0 % | ❌ | ❌ | See M1 |

## Findings

Decisions are added below as they are made.

- **M1.** `MoveitBackend`, the code that really moves the arm, never runs in a test (0 %).
- **M2.** "MoveIt not ready → stay inactive" is untested: the mock is always ready.
- **M3.** The mock ignores the pose: no test checks the goal's pose reaches the backend.
- **M4.** Timing tests: the heartbeat rate is checked with a 1 s sleep and a 20–30 window, no
  margin explained.
- **M5.** Small gaps: reset while not active (lines 51–53), cleanup while a goal thread exists
  (line 150).
- **M6.** `GoalSucceedsWhenIdle` was flaky: it required the one feedback message the node sends
  when a goal starts (found in #161, added 2026-11-19).

### M1. `MoveitBackend` never runs in a test

**What:** every node test calls `node_->set_backend(mock_)`, so `src/moveit_backend.cpp` (the
code that talks to MoveIt on the real arm) has 0 of 53 lines run. It is the main reason the
package is below the gate.

**The trap:** `MockBackend` has its own copy of the stop check (`if (stop_requested_) return
PREEMPTED;`). Delete that check from the real backend and every test stays green: a mock tests
the **node**, not the code it replaces. `EStopRightAfterAcceptNeverRunsFullMotion` proves the
node stops the mock, not that the real backend refuses to plan after a stop.

**Why it matters:** that check is what makes a stop that lands before planning work on the real
arm. Without it, the real backend would plan and move anyway.

**The test (`test/test_moveit_backend.cpp`, no MoveIt running):** a real `MoveitBackend("arm")`:
1. `stop()` then `plan_and_execute()` → PREEMPTED.
2. `stop()`, `reset_stop()`, `plan_and_execute()` → PLANNING_FAILED (no `move_group`; the stop
   was cleared).
3. Never connected → PLANNING_FAILED.
4. `ready()` with no MoveIt → false, within a bounded time (to check: how long the
   `MoveGroupInterface` constructor waits; the code gives it 2 s).

About 20–25 of the 53 lines: mission goes to about 84 %. Planning and executing need a running
MoveIt: evidence from sim or HIL, not unit tests.

**Lesson:** a mock tests the code that **uses** it. The code it replaces needs its own tests.

**Decision:** **fix now** (~1 h). A safety path (a stop before the motion) on the real arm with
no test, and needed for the gate.

**What the test found (2026-11-19):**
1. **The destructor can hang forever.** `~MoveitBackend()` calls `executor_->cancel()`; if that
   runs before the spin thread has entered `spin()`, the cancel is lost (`spin()` sets its
   "spinning" flag again) and `join()` never returns. A backend built and destroyed at once hung
   every time; with a 200 ms pause after construction the tests passed. Possible fix: the spin
   thread runs `spin_until_future_complete` on a promise the destructor sets.
2. **`ready()` is true with no `move_group`.** With the robot description present (as
   `mission.launch.py` gives it) and no `move_group`, `MoveGroupInterface` only logs "action
   client/server not ready" and does not throw. The node goes ACTIVE, its heartbeat says IDLE,
   and every goal fails at planning: the M2 guard does not hold on the real system.
3. **`ready()` crashes with no robot description** (e.g. `ros2 run`): "terminate called without
   an active exception" instead of returning false; the `try/catch` cannot help (probably a
   thread started inside MoveIt's constructor before it throws).

Also: MoveIt waits 10 s for the URDF and 10 s for the SRDF by default; the test needs
`robot_description_timeout` and `robot_description_semantic_timeout` set short.

**Status:** sent to the package owner 2026-11-19, who opened #165; all three fixed in #168
(merged 2026-11-23). Bug 1: the constructor waits until the executor is spinning, so the
destructor's cancel can't be lost. Bug 2: `ready()` first waits up to 2 s for move_group's
`move_action` server. Bug 3: `ready()` returns false when `robot_description` or
`robot_description_semantic` is not a parameter, before building `MoveGroupInterface`. The test is
registered and passes; mission is at 86.2 %.

### M2. "MoveIt not ready → stay inactive" is untested

**What:** `on_activate` (`src/mission_node.cpp:162`) returns FAILURE when `backend_->ready()` is
false. `MockBackend::ready()` always returns true, so lines 163–164 never run.

**Why it matters:** while the node is INACTIVE it sends no heartbeat, so the safety supervisor
refuses to activate: the whole system stays off. If the check were deleted, the node would go
ACTIVE with no MoveIt, publish IDLE, the supervisor would enable the arm, and every goal would
end PLANNING_FAILED (no `move_group`). Not dangerous, but the heartbeat would claim a healthy
mission that cannot move (see safety S4).

**The test:** `MockBackend` gets a knob `std::atomic<bool> ready_result{true}`. Test: set it
false, configure, activate → the transition fails, the node stays INACTIVE, no heartbeat.

**Decision:** **fix now** (~20 min). A failure path with no test.

### M3. The mock ignores the pose

**What:** `MockBackend::plan_and_execute(const PoseStamped & /* pose */)` throws the pose away.
If `execute()` sent a default pose, or the previous goal's pose, to the backend, every test
would still pass.

**Why it matters:** the pose is where the arm goes. The path "goal pose → backend" is the one
thing the node exists for.

**The test:** the mock stores the last pose it received; `GoalSucceedsWhenIdle` checks it equals
the goal's pose (frame and position). A few lines.

**Decision:** **fix now** (~15 min). Where the arm goes is a safety path, and the check is cheap.

### M4. Wall-clock heartbeat test

**What:** `Heartbeat` (`test/test_lifecycle.cpp:148`) sleeps 1 s and expects 20–30 messages
(25 at 40 ms). `deactivate_activate_cycle()` sleeps 100 ms + 300 ms to check that no heartbeat
comes while inactive. `CONTRIBUTING` asks for an explained margin on timing tests; a slow CI
runner can make the 20–30 window flaky. The "nothing arrives" sleeps are fine (absence can only
be checked by waiting), but need a comment on why the time is long enough.

**Decision:** **later**: explain the margins in comments, widen the lower bound or derive it from
the supervisor's 300 ms timeout (what matters is "no gap above the timeout", not the count).

### M5. Small gaps

Reset refused while the node is not active (lines 51–53); `on_cleanup` joining a goal thread
(line 150). **Later.**

### M6. A feedback check that depends on timing

**What:** `GoalSucceedsWhenIdle` (`test/test_goals.cpp`) checked `feedback_count_ > 0`. The node
publishes exactly one feedback message, when the goal starts. If the client's feedback
subscription is not matched yet (a busy machine, tests in parallel), that message is lost, and
no amount of waiting brings it back. Seen once in CI (run 44391150973) and in 2 of 11 local
parallel runs, while making tests run in parallel for #144 (PR #161). The workaround there runs
mission's tests alone (`TEST_ALONE` in the Makefile and `coverage.sh`, about 14 s per CI run).

**Why the review missed it:** the assert used `wait_until`, which looks safe ("wait for
evidence"). But waiting only helps if the evidence can still come; a single message that was
lost never comes. Lesson: before trusting a wait, ask whether the thing waited for can be lost.

**Fix (2026-11-19):** the feedback check is removed, with a comment why: feedback is information
for the client, not a safety behaviour, and waiting for the subscription to match is not easy
from the action client's public API. The unused `feedback_count_` and its callback are removed
from the fixture. Check: `test_goals` passed 10 of 10 runs with the hardware and safety tests
running at the same time.

**Decision:** **fixed in #46.** Tell #161: `TEST_ALONE` for mission can go once this is merged.

## Summary

| Decision | Items |
|---|---|
| Fix now (#46) | M1 `test_moveit_backend.cpp`: stop before planning, reset_stop, not connected, `ready()` false (~1 h, also lifts the gate to ~84 %); M2 activation refused when MoveIt not ready (~20 min); M3 the pose reaches the backend (~15 min) |
| Fixed in #46 | M6 flaky feedback check removed |
| Later (follow-up issue) | M4 timing margins in the heartbeat tests; M5 reset while inactive, cleanup join |
| New issue | Linters switched off in `CMakeLists.txt` (side note) |

## Side notes

- `CMakeLists.txt` has the linters switched off (`ament_lint_auto_find_test_dependencies()` is
  commented out), though `package.xml` lists them. So cpplint, cppcheck, uncrustify and
  copyright never run on this package in `make test`. A question for the team, not a test gap.
- Lines 349–351 ("should not be reachable") and 302–303 (ROS shut down during a goal) are fine
  untested.
