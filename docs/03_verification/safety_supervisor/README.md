# Safety supervisor on the arm: fault injection (#45)

How to show, with numbers, that the safety supervisor halts the real arm. What the supervisor
is and how it decides: `src/positronic_safety/README.md`.

**Status: not run yet.** The supervisor passed its tests without the arm (unit tests, the node
against a fake controller manager, the launch with `backend:=fake`). Everything below still
has to be done on the arm, and the results written in the last section.

## Safety

These tests move the real arm and stop it on purpose.

- Two people. One runs the commands; the other stands at the arm with a **hand on the power
  switch** and does nothing else.
- Bench clear, nobody inside the arm's reach, one person commands the arm at a time.
- Small moves at 15 deg/s or less (the working limit of `jtc_on_the_arm`), inside the joint
  limits, starting from the pose of `T0_to_start.yaml`.
- Safe state: supervisor INACTIVE, hardware component inactive, the arm holding its pose.
  To stop at any moment: `ros2 service call /safety_supervisor/stop std_srvs/srv/Trigger`.
- **Software cannot stop an arm whose serial line is dead**, and nothing stops the arm if the
  supervisor's own process dies (test 7 shows it). In both cases: the power switch.
- Activating the supervisor powers the servos on. They hold their pose; nothing moves until a
  goal is sent.

## Setup

On the Jetson (from a laptop, with the `nano` alias of `docs/build.md`):

```bash
make nano-build && make nano-shell
ros2 launch positronic_bringup supervised.launch.py
```

A second shell in the same container (`ssh nano`, then `docker exec -it positronic-arm bash`),
left running for the whole session:

```bash
source install/setup.bash
python3 docs/03_verification/safety_supervisor/tools/halt_latency.py | tee halts.csv
```

On the laptop that commands the arm: the mission node, configured and activated (its
heartbeat is what the supervisor waits for), then, with the bench clear:

```bash
ros2 lifecycle set /safety_supervisor activate
ros2 topic echo /safety_supervisor/status --once      # mode: 2
```

The trajectory for every test is `T1.yaml` of `docs/03_verification/jtc_on_the_arm/`, sent as
that folder's README says. After each halt: check the arm stands still, then activate the
supervisor again.

Try the whole setup once with `backend:=fake` in the dev container before going to the arm:
`halt_latency.py` has unit tests for its maths, but has not run against ROS yet.

## Tests

| # | What to do, mid-trajectory unless said | Expected | Criterion of #45 |
|---|---|---|---|
| 1 | Nothing: let the arm stand, ACTIVE, for 5 minutes, while someone runs `ros2 topic list`, `ros2 node list` and `ros2 topic echo` now and then | No halt. Count the HEARTBEAT_LOST halts | none; decides the heartbeat margin (see below) |
| 2 | `ros2 service call /safety_supervisor/stop std_srvs/srv/Trigger`, 5 times | STOP_SERVICE, `hardware_stopped` 1 | `rest_ms` under 100 |
| 3 | `kill -9` the mission node, 5 times | HEARTBEAT_LOST | `heartbeat_to_halt_ms + rest_ms` under 300 |
| 4 | Take the laptop off the network (Wi-Fi off) | HEARTBEAT_LOST, same as test 3 | same as test 3 |
| 5 | Arm standing: `ros2 control switch_controllers --deactivate joint_state_broadcaster` | FAULT_DATA_LOST: the supervisor lost sight of the driver | halts |
| 6 | After a halt, wait one minute with the mission node alive | Still stopped; only `ros2 lifecycle set /safety_supervisor activate` resumes | stays stopped |
| 7 | `kill -9` the supervisor, during a small move | **The arm finishes its move.** This is the known limit, shown on purpose | written limit |

`halt_latency.py` prints one line per halt; its columns are explained at the top of the file.
`rest_ms` is the time from the trigger until every joint is slower than 0.02 rad/s.

Not on this list: a **serial link fault** (`driver/fault_code` = 1) cannot be injected into the
running driver yet. It is covered without the arm (`test_load_robot_system` latches the code,
`test_safety_supervisor` halts on it), and `hil_link_timeout` shows what the arm does on a
silent line. Injecting it on the arm needs a hook in the driver: a follow-up issue.

## What we expect to learn

- **Test 2 may miss 100 ms.** The lifecycle measurements of the driver say a move at 30 % speed
  stops about 8 degrees after the request. If `rest_ms` is over 100 with `halt_ms` of a few
  ms, the time is the arm braking, not the supervisor, and the criterion has to be rewritten
  with the measured number.
- **Test 1 may show false halts.** On 2026-11-11, with the fake backend in a VM, the
  supervisor halted 2 s after activation: `No mission heartbeat for 206 ms (limit 200 ms)`,
  while a `ros2` command-line tool was starting. The mission node publishes at 10 Hz, so
  200 ms tolerates one late heartbeat. If test 1 halts, the fix to propose is a faster
  heartbeat (25 Hz gives five periods inside 200 ms), not a longer timeout, which would cost
  the 300 ms of test 3.

## Results

Not run yet. Put the raw `halts.csv` in this folder, named `YYYY-MM-DD_halts.csv`, and a table
here: per test, the number of trials, min / mean / max of `halt_ms` and `rest_ms`, and pass or
fail against the criterion.
