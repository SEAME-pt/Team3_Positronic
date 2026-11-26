# positronic_bringup: test review

Reviewed 2026-11-19. Coverage 82.4 % (136 of 165 lines, scripts only; `launch/` is not counted).
Tests: `src/positronic_bringup/test/`. Decided without the guided questions (the user asked to
speed up).

**What the package does:** the launch files, the configuration (controllers, safety supervisor)
and two measurement tools: `joint_states_rate.py` (how often new poses arrive) and
`joint_states_delay.py` (how old each `/joint_states` message is, pass/fail against 100 ms, #24).

| File | Coverage | Lines never run |
|---|---:|---|
| `scripts/joint_states_rate.py` | 80 / 84 | 149, 154–155, 169 (`main` branches) |
| `scripts/joint_states_delay.py` | 56 / 81 | 122–155 (`main`) |

## Behaviours

| Behaviour | Normal | Failure path | Edges | Verdict |
|---|:-:|:-:|:-:|---|
| Robot description: original arm kept, our plugin, real arm by default, arguments reach the plugin | ✅ | | ✅ both backends | Very good |
| Joints: URDF revolute joints in order, position command, limiter off, every joint has a URDF limit | ✅ | | | Very good |
| Fault code reported, supervisor reads this description's names | ✅ | | | Very good |
| Controllers: broadcaster, every state interface, a halt deactivates every command controller, a resume activates the declared ones | ✅ | | | Very good: catches a controller added without updating the supervisor |
| Supervisor timeouts | ✅ positive only | | ❌ not checked against each other | See B2 |
| Launch files build, driver active unless asked, supervised launch | ✅ | | | Good |
| Rate tool: summary, files, node, `main` | ✅ | | | Good |
| Delay tool: delay, summary, pass/fail, files, node | ✅ | ✅ late messages, no messages | | Good |
| Delay tool `main` (the exit code a campaign relies on) | ❌ | ❌ | | See B1 |

## Findings

### B1. The delay tool's `main()` is untested

**What:** `joint_states_delay.py` decides pass/fail for the #24 limit, and `main()` turns it into
the exit code (`return 0 if passed else 1`). `final_summary` is tested, but `main()` (arguments,
`--limit-ms`, `--out`, the exit code) never runs. The rate tool has this test
(`test_joint_states_rate_main.py`); the delay tool does not.

**Decision:** **later**. A measurement tool, not a path that moves the arm; the decision logic
itself is tested. Copy the rate tool's main test: one run that passes (exit 0), one with
`--limit-ms 0` that fails (exit 1).

### B2. The supervisor timeouts are only checked to be positive

**What:** `test_supervisor_timeouts_are_positive` checks each value is above 0. The real rule is
a relation: `heartbeat_timeout_ms` (300) must stay above the mission's heartbeat period (40 ms)
plus the worst gap measured on the lab Wi-Fi (157 ms, `docs/03_verification/heartbeat_link`).
Set it to 50 and every test passes, but the arm halts on every Wi-Fi hiccup.

**Decision:** **later**. A test that reads the YAML and checks `heartbeat_timeout_ms > 157 + 40`
with a comment pointing at the campaign.

## Summary

| Decision | Items |
|---|---|
| Fix now (#46) | – |
| Later (follow-up issue) | B1 delay tool `main()` test; B2 heartbeat timeout checked against the measured gap |
| New issue | – |
