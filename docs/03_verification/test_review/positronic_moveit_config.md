# positronic_moveit_config: test review

Reviewed 2026-11-19. Coverage 56.2 % (50 of 89 lines): **below the 80 % gate**. Tests:
`src/positronic_moveit_config/test/test_planning_scene.py`. Decided without the guided questions
(the user asked to speed up).

**What the package does:** MoveIt's configuration (SRDF, kinematics, joint limits, OMPL,
controllers) and `scripts/planning_scene.py`, a one-shot node started by `move_group.launch.py`
that sends the workcell obstacles (bench, four walls, roof) to `move_group` and exits.

| Part | Coverage | Lines never run |
|---|---:|---|
| `_pose_from_dict`, `build_collision_objects`, `load_scene_file` | ✅ | 38 (non-box type error) |
| `PlanningSceneLoader` (`__init__`, `apply`), `main` | **0 %** | 74–133 |

## Behaviours

| Behaviour | Normal | Failure path | Edges | Verdict |
|---|:-:|:-:|:-:|---|
| YAML → collision objects (frame, one box, one pose) | ✅ | | | Very good |
| Workcell geometry: bench top at z ≈ 0, walls just past the 0.28 m reach, roof present | ✅ | | | Good: locks the measured numbers |
| `remove=true` → every operation REMOVE | ✅ | | | Good |
| Unsupported shape type, missing `size`, wrong number of dimensions | | ❌ | | See C2 |
| Send to `move_group`: success | ❌ | | | See C1 |
| `move_group` not there, refuses, no answer in 5 s | | ❌ | | See C1 |
| What happens when the scene is not loaded | | ❌ nothing notices | | See C1 |
| `joint_limits.yaml` (scaling 0.2, max velocity 1.0 rad/s) | ❌ no test | | | See C3 |

## Findings

### C1. Sending the scene to `move_group` is untested, and a failure goes unnoticed

**What:** `PlanningSceneLoader.apply()` and `main()` never run in a test. They have three
failure paths: the service is not there within `timeout_sec`, `move_group` answers
`success = false`, no answer within 5 s (`future.result()` is `None`). Each returns `False` and
`main()` exits 1.

**Why it matters:** the obstacles are what stops MoveIt from planning through the bench or the
walls. If they are not loaded, MoveIt plans as if the workcell were empty. Today the only sign is
an error log and exit code 1 of a node that was going to exit anyway: `move_group.launch.py`
does not react to it, and nothing tells the mission node or the supervisor.

**The tests (no MoveIt needed):** in `test_planning_scene.py`, a test node offers a fake
`/apply_planning_scene` service:
1. Fake answers success → `apply()` is True, the request is a diff with every object.
2. Fake answers `success = false` → `apply()` is False.
3. No service, `timeout_sec = 0.1` → `apply()` is False quickly.
4. `main()` returns 1 when `apply()` fails.

This brings the package to about 90 %.

**Decision:** tests **fix now** (~1 h: failure paths with no test, and needed for the gate).
"A scene that failed to load must stop the system, or at least the mission" (e.g. the launch
shuts down on a non-zero exit, or the mission checks the scene before activating): **new issue**.

### C2. Bad scene files are untested

A non-box object raises `ValueError` (line 38, never run); a missing `size` raises `KeyError`; a
`size` with 2 numbers builds a broken box that MoveIt may reject. **Later**: one test for the
non-box error, one for a wrong `size`.

### C3. Nothing locks the MoveIt speed limits

`CLAUDE.md` says MoveIt's velocity scaling must not go above `default_velocity_scaling_factor`
without a reason, and new motion code runs slow. `joint_limits.yaml` sets the scaling to 0.2 and
`max_velocity: 1.0` rad/s on every joint (the URDF allows 2.618). No test reads this file, so a
change to it passes CI silently. Also, the file lists `gripper_controller` as a joint, which is a
controller name, not a joint.

**Later:** a config test like bringup's `test_controllers_config.py`: scaling ≤ 0.2, every
`max_velocity` ≤ the URDF velocity, every name in `joint_limits` is a URDF joint.

## Summary

| Decision | Items |
|---|---|
| Fix now (#46) | C1 four tests for `apply()` and `main()` (~1 h, gate to ~90 %) |
| Later (follow-up issue) | C2 bad scene file tests; C3 joint limits config test, `gripper_controller` entry |
| New issue | C1 a failed scene load must stop the system or the mission |
