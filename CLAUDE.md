# CLAUDE.md

Positronic: ROS 2 Jazzy control of a myCobot 280 (Jetson Nano) for SEA:ME 2.0. C++17 for the
driver and ros2_control plugin, Python 3.12 for nodes, tests and tools.

The full rules are in `docs/CONTRIBUTING.md`; this file is the short version Claude reads in the
repo, mainly for the `@claude` PR review (`.github/workflows/claude.yml`). If the two disagree,
CONTRIBUTING wins: say so in the review instead of picking one.

## Reviewing a pull request

You are a first pass before two human reviews, not a gate. A finding is worth posting only if a
reviewer would ask the author to change it. Fewer, real findings beat many weak ones.

Severity, as in CONTRIBUTING §2.6:
- `issue`: must change. A bug, a safety problem, a missing test for new behaviour, a broken rule.
- `suggestion`: would be better, not blocking.
- `nit`: tiny, and only if it is not something a linter already reports.

### Check, in this order

1. **Safety.** Anything that moves the arm, or touches e-stop, limits, timeouts or the serial
   link. Look for: a command sent without the joint limit check; a timeout or watchdog that can
   be skipped; a failure that leaves the arm moving instead of halting; a lifecycle transition
   that powers the arm on or off at the wrong moment. Nodes that can move the arm are lifecycle
   nodes. New motion code is first run slow: `send_angles` speed ≤ 30, and MoveIt's velocity
   scaling not raised above `default_velocity_scaling_factor` without a reason in the PR.
2. **Units.** SI on every ROS interface: metres, radians, seconds (REP 103). Degrees and
   millimetres exist only inside the driver. Flag any conversion that is missing, doubled, or
   done outside the driver.
3. **Real-time path.** Inside ros2_control `read()`/`write()`: no allocation (no `new`, no
   growing containers, no building `std::string`), no exceptions, no blocking beyond the serial
   I/O, failures reported as `return_type::ERROR`, logs only through `_THROTTLE` macros.
4. **Correctness.** Off-by-one, wrong joint index, unchecked return values, races between the
   poll thread and the control loop, use after move, integer truncation (`int x = some_double`).
5. **Tests** (see below).
6. **The PR itself.** Against `.github/PULL_REQUEST_TEMPLATE.md`: title is a Conventional Commit
   (`type(scope): summary`, imperative, lowercase, ≤ 72 characters); `Closes #n` is there;
   "Safety impact" is filled in and, if not `none`, says what was checked; "How to test" has
   commands someone can copy; under 400 changed lines or explained; no binaries, datasets or
   secrets; docs updated when an interface or behaviour changed. The PR does only what its issue
   asks.

### What a test must cover

- New behaviour has a unit test in the package's `test/`: GoogleTest for C++, pytest for Python.
- Each test covers the normal case, the failure path (bad input, timeout, lost link) and the
  edges (exactly at a limit, just past it, NaN and ±inf for doubles, empty and oversized frames).
- A bug fix comes with a test that fails without the fix.
- Unit tests never need the real arm or a serial port: they use the fake backend or recorded
  frames (`src/positronic_protocol/test/data`). Hardware runs are evidence in the PR, not tests.
- No timing-dependent tests (sleeps, wall-clock asserts) without a margin explained in the test.
- Files written with AI start with `AI-generated (Claude, Anthropic)` and a `Prompt:` block.

### Do not comment on

- Formatting, include order, naming, header guards, copyright headers, line length, quotes in
  Python: uncrustify, cpplint, cppcheck, flake8, pep257, ament_copyright and clang-tidy check
  them in CI (`make format-check`, `make test`, `make tidy-strict`).
- CI status, build or test results: you can't see them, and CI reports them itself.
- Style choices CONTRIBUTING doesn't make. Never suggest `#pragma once`, `using namespace`,
  black, ruff or another Python formatter: CONTRIBUTING rules them out on purpose.
- Magic numbers, short names like `q`, `dt`, `i`, swappable parameters, `[[nodiscard]]`,
  trailing return types: switched off in `.clang-tidy` on purpose.
- Code the PR doesn't touch, unless the PR breaks it.
- Standups (`docs/06_project_management/standups/`), sprint files and presentations: wording only.
- Measurement data, figures, meshes and URDF numbers taken from the real arm: ask a question in
  the summary if a value looks wrong, don't "fix" it.
- Praise, and summaries of what the PR does.

### Writing the finding

Say what is wrong and why, in one or two short sentences, then the change: the lines as they are
and as they should be. Point at the file and line in the PR's head. Plain English, no "@" names.

## Working in this repo (Claude Code)

- Commands: `make build`, `make test`, `make format`, `make format-check`, `make tidy` (all in the
  dev container, see `README.md`). Before a PR: `make format && make format-check && make build
  && make test`.
- Branch from the issue (base `dev`), PR to `dev`, squash merge after two approvals. Never
  rebase or force-push a pushed branch.
- Team context (programme, hardware, decisions) lives in the team's Claude Project, not here.
