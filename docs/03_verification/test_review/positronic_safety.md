# positronic_safety: test review

Reviewed 2026-11-18. Coverage 86.0 % (374 of 435 lines). Tests: `src/positronic_safety/test/`.
`SupervisorLogic` (the decisions) and `DeadlineWatchdog` (the timeouts) are pure and tested with
made-up times; the node is tested against a fake controller manager, trajectory controller and
mission node.

**What the package does:** the safety supervisor, a lifecycle node. While ACTIVE it watches four
triggers (stop service, mission heartbeat, driver fault code, fault data present). Any trigger
runs one halt path: hardware inactive (the driver sends the stop), cancel trajectory goals,
controllers off. Then it deactivates itself and stays stopped until a person activates it.

## Behaviours

| Behaviour | Normal | Failure path | Edges | Verdict |
|---|:-:|:-:|:-:|---|
| Halt order and "one halt path" (logic) | ✅ | ✅ every step fails, second trigger ignored | ✅ late answers, tickets never repeat | Excellent |
| Resume order, failed resume → halt (logic) | ✅ | ✅ | ✅ halt during a resume | Excellent |
| Deadline watchdog | ✅ | ✅ never fed | ✅ exactly the timeout | Very good |
| Reading the fault code | ✅ | ✅ missing gpio/interface, NaN, empty | ✅ lists of different length | Very good |
| Node: each trigger halts | ✅ stop, heartbeat, fault, fault data, NaN code, deactivate | | | Good |
| Node: halt with a controller manager that does not answer | ✅ | | | Good |
| Node: halt with a controller manager that is gone (service not available) | | ❌ | | See S3 |
| Node: stale fault code ignored after a resume | ⚠️ test exists, filter never runs | | | See S2 |
| Node: activation refused | ✅ no heartbeat | ❌ halt running, no services, resume fails on the spot | | See S5 |
| Node: parameters | | ❌ bad values untested | | Later |
| Node: shutdown, cleanup | ❌ | | | See S1 |
| Heartbeat content | | ❌ not read at all | | See S4 |
| `safety_supervisor_main.cpp` | ❌ 0 % | | | See S1 |

## Findings

Decisions are added below as they are made.

- **S1.** The supervisor's `on_shutdown` never runs on Ctrl+C.
- **S2.** The test for "a stale fault code does not halt again" never runs the code it is about.
- **S3.** A halt while the controller manager is gone is not tested at node level.
- **S4.** Any message on the heartbeat topic counts, whatever it says.
- **S5.** Three activation refusals are untested at node level (each has a second, tested guard).

### S1. The supervisor's `on_shutdown` never runs on Ctrl+C

**What:** `on_shutdown` halts the arm if it could still move, and its comment says "a shutdown
can come straight from ACTIVE". But in rclcpp (Jazzy branch, checked 2026-11-18) a lifecycle
node does not run `on_shutdown` by itself when the process gets Ctrl+C: someone must request the
shutdown transition. `main()` (`src/safety_supervisor_main.cpp`) only spins and exits. (Rolling
only adds a warning in the destructor.) Unlike the controller manager (hardware H1), the
supervisor has no pre-shutdown hook.

**What it means:**

| Situation | Result |
|---|---|
| Ctrl+C on the whole launch | The controller manager deactivates the plugin, so the stop is sent (hardware H1). Safe. |
| Ctrl+C on the supervisor only (its terminal, a restart) | No halt: the plugin stays ACTIVE, the controllers keep running, nobody watches. |

The second row is the "supervisor dies" known limit (#136): a clean exit is treated like a
crash, though it does not have to be. Coverage: `on_shutdown` (lines 270–278) and all of `main()`
never run.

**Which halt step matters on exit:** the halt is asynchronous; each step is sent only after the
previous one is answered. On exit only step 1 would leave. That is enough: step 1 ("hardware
inactive") makes the controller manager run the plugin's `on_deactivate`, which sends the stop,
whether or not the supervisor is still alive to read the answer. With the plugin INACTIVE the
controllers cannot move the arm, so steps 2 and 3 are only cleanup. Condition: the request must
actually leave before the process exits (send it from a pre-shutdown hook, or pause a few ms).

**Lesson:** in a halt, find the one step that stops the motion and make sure it goes out first.
The supervisor's design already puts it first.

**Decision:**
- Node test: `supervisor_->shutdown()` while MONITORING sends "hardware inactive" to the fake.
  **Fix now** (~30 min).
- `main()` runs the halt on Ctrl+C (pre-shutdown hook): **new issue**, or added to #136.

### S2. The "stale fault code" test never runs the filter

**The rule:** after a driver fault and a resume, the driver clears its code to 0, but messages
sent before the resume may still arrive carrying the old code. The filter at
`src/safety_supervisor.cpp:288` drops messages stamped before the last activation, so they do not
halt the arm again. `ResumesAfterADriverFaultOnceTheDriverClearedIt` says it checks this.

**What:** coverage shows the filter's `return` (line 289) never runs. The fake controller manager
sets `fault_code = 0.0` while it handles the activation request and stamps every message with the
current time, so after the resume every message is new and carries 0. The test would also pass
with the filter deleted.

**The test that matters:** a message with `fault_code = 1` and a stamp from **before** the
resume.

**Fix:**
1. The fake gets an option: right after answering the activation, publish one message with
   code 1 and a stamp from before the activation. The test checks the supervisor stays in
   MONITORING.
2. The opposite case, so the filter is not too wide: code 1 with a stamp after the activation
   still halts.
3. Prove it: delete the filter on purpose, run the test, watch it fail, put the filter back
   (**mutation testing**).

**Lesson:** "if the code broke, would this test fail?" Break the code on purpose to find out.

**Decision:** **fix now** (~45 min). A safety rule (no false halt after a resume, a real halt
still works) that is effectively untested.

### S3. A halt while the controller manager is gone is not tested

**What:** if the controller manager crashed, each halt step fails at once (service not
available), the halt goes on (cancel goals: no action server, counts as OK), and ends STOPPED with
`hardware_stopped = false` and the log "THE ARM MAY STILL MOVE. Cut the servo power." Correct, but
no node test runs it. `HaltGoesOnWhenTheControllerManagerDoesNotAnswer` covers a manager that does
not answer (each step waits for its timeout); a manager that is gone goes through another path,
the loop in `run()` that sends the next step at once (line 357, never run).

**Why it matters:** the plugin runs inside the controller manager's process. If it crashed, the
port closed, no stop was sent, and the arm finishes its move. The supervisor cannot stop it; its
only useful job is to tell the truth (`hardware_stopped = false`, "cut the servo power") and not
hang half way. The test protects that honest report.

**The test (`test/test_safety_supervisor.cpp`):** start monitoring, remove the fake's services as
if it crashed, call the stop service, expect STOPPED with `hardware_stopped == false` quickly
(no waiting for the 500 ms step timeouts).

**Lesson:** when software cannot fix a failure, test that it reports it honestly.

**Decision:** **fix now** (~45 min).

### S4. Any message on the heartbeat topic counts

**What:** the mission node publishes a stamp, its node name and its state (IDLE, INACTIVE,
MOVING, STOPPED, ERROR) from a timer (`positronic_mission/src/mission_node.cpp`, `heart_beat()`).
The supervisor's subscription ignores the content (`const Heartbeat & /*message*/`) and only
feeds the watchdog. So a mission node in ERROR keeps the arm enabled while it publishes; a
message from any other node (`ros2 topic pub /mission/heartbeat ...` on a laptop) counts; a late
message counts as fresh.

**What the heartbeat proves:** only that something publishes on the topic. The timer keeps
firing even if the planning code is stuck, so it does not prove the mission works.

| The heartbeat could prove | The supervisor would check |
|---|---|
| Something publishes | Nothing (today) |
| The mission node is alive | `node_name` |
| The mission node is healthy | `state != ERROR` |
| The mission makes progress | A counter updated by the planning code, not a timer |

Each level catches more, and adds ways to halt for no reason: a team decision, which
`CONTRIBUTING.md` says goes in an ADR.

**Lesson:** before testing a safety signal, ask what it actually proves. Often less than its
name suggests.

**Decision:**
- "What must the heartbeat prove?": **new issue**, decided by the team with an ADR.
- One node test that locks the decision in (if nothing changes: "a heartbeat with state ERROR
  keeps the arm enabled"), and one sentence in the safety README: **later**.
- Nothing in #46.

### S5. Activation refusals untested at node level

**A correction first:** the overview said `ActivationIsRefusedWithoutAHeartbeat` "can pass for the
wrong reason". It does not: `on_activate` checks the heartbeat **before** the controller manager's
services, so with no heartbeat the refusal always comes from the heartbeat check. It would only
pass for the wrong reason if the heartbeat check were deleted and the services were slow to be
found.

**What:** `on_activate` (`src/safety_supervisor.cpp:206`) refuses in four cases. Only "no
heartbeat" has a node test. Never run:

| Refusal | Lines | Second guard (if this check were deleted) |
|---|---|---|
| A halt is still running | 211–212 | `SupervisorLogic::resume()` refuses unless STOPPED (logic test `ResumeIsRefusedUnlessStopped`) |
| Controller manager services not available | 221–224 | Step 1 of the resume fails at once → halt → STOPPED → refused at line 233 |
| The resume failed on the spot | 233 | – (it is the last check) |
| Bad parameters at configure | 155 | – (the node stays UNCONFIGURED) |

**Defence in depth:** two guards for the same danger, so one mistake does not break safety.
Without the "halt still running" check the arm still does not move (`resume()` returns an empty
order), but `on_activate` would return SUCCESS while the mode is HALTING: the node would report
ACTIVE while halting. The first guard keeps the reported state honest.

**Lesson:** when a check looks untested, look for a second guard before calling it a safety gap,
and say what the first guard still adds.

**Decision:** **later** (not a safety path with *no* test). Four node tests in the follow-up
issue: halt running, no services, resume fails on the spot, bad parameters.

## Summary

| Decision | Items |
|---|---|
| Fix now (#46) | S1 node test: `shutdown()` while MONITORING sends "hardware inactive" (~30 min); S2 stale fault code test with a stamp from before the resume, plus the opposite case and a mutation check (~45 min); S3 halt with the controller manager gone (~45 min) |
| Later (follow-up issue) | S4 locking test and README sentence; S5 four node tests; README sentence on RESUMING (side note) |
| New issue | S1 `main()` halts on Ctrl+C with a pre-shutdown hook (or add to #136); S4 "what must the heartbeat prove?" with an ADR |

## Side notes

- During RESUMING (up to ~6 s: two steps of up to 3 s) the heartbeat and fault data are not
  checked; they are checked as soon as the mode is MONITORING (the heartbeat watchdog is not
  reset, so a heartbeat lost during the resume halts at the first check). The stop service works
  in every mode. Worth one sentence in the README.
