# Serial-thread timing sweep on the real arm (#32)

What the driver can sustain when it writes commands and reads poses on the same serial line.
The thread polls `get_angles` every turn and sends one `send_angles` when the command changed.
Until #150 it then waited a fixed `send_settle_ms` for the arm to pass the command to the
servos; since #150 it waits for the Atom's sync-write to the servos instead (see the last
section). The period is a launch argument of `driver.launch.py` (`moving_period_ms`), so a
sweep needs no rebuild.

## Offline measurement (pymycobot, arm standing still)

`measure_serial.py --gap-ms` from `../serial_measurement/tools`: `send_angles` of the current
pose, a sleep of `gap` ms, then `get_angles`. A stall is a call above 450 ms.

| gap ms | pairs | stalls | round trip p50 ms | max ms |
|---|---|---|---|---|
| 0 | 200 | 113 | 548 | 1567 |
| 10 | 200 | 25 | 37 | 1559 |
| 20 | 200 | 18 | 43 | 570 |
| 23 | 300 | 1 | 44 | 558 |
| 24 | 300 | 0 | 45 | 49 |
| 25 | 1000 | 3 | 47 | 568 |
| 30 | 1000 | 0 | 52 | 59 |
| 35 | 1000 | 0 | 59 | 67 |

The arm ignores the port for about 25 to 30 ms after accepting a `send_angles`. With a 30 ms
gap a write plus a read cost 52 ms here, so the first design used 66 ms turns while the arm
moves and 10 ms turns while it stands still. The runs on the arm below showed that 52 ms was
pymycobot overhead and set the turn at 50 ms.

## On the arm: `tools/sweep.sh`

Inside the Jetson container (`make nano-shell`), arm powered on, bench clear:

```
bash docs/03_verification/write_sweep/tools/sweep.sh
```

For every point it starts the driver with that period, switches the joints from
`arm_controller` to `forward_position_controller`, streams a command at the control rate (30 Hz,
`UPDATE_RATE` to change both) for 120 s (joint 1 swinging 10 deg, 4 s per cycle) while
`joint_states_rate.py` records the interval between new poses on `/joint_states`, then stops
the driver and cools down for 2 min. Results: `log/sweep_<date>/<period>/summary.txt`. A point
`P:S` also sets `send_settle_ms`, as the sweeps of #32 did.

`/joint_states` shows the rate the rest of ROS sees, not stalls: the arm stands still at the
ends of the swing and now and then answers with the position it already had, and both look
like a long interval. Stalls (a `get_angles` with no reply) are counted on the serial line:

```
python3 docs/03_verification/write_sweep/tools/proxy_tap.py /ws/log/tap_33.csv   # terminal 1
PORT=$(cat /tmp/proxy_port) POINTS="33" bash docs/03_verification/write_sweep/tools/sweep.sh
```

`proxy_tap.py` opens the real port, offers a pty to the driver and writes every chunk of both
directions with its time to the csv given. Stop it with Ctrl+C after the run and start it again
with a new name for the next. From a script, start it in the background, wait until
`/tmp/proxy_port` exists before starting the driver, and stop it with `kill -TERM`: a background
job of a script ignores SIGINT, and every row is already flushed. On the PC, `python3 tools/tap_stalls.py tap_33.csv` prints
requests, replies, stalls, repeated poses and the round trip, and `python3 tools/tap_turns.py
tap_33.csv` the turn after each `send_angles`: when the next `get_angles` left, when the Atom's
sync-write ended, and every sync-write later than 30 ms.

## Results (joint 1 swinging 10 deg at speed 30)

| period | settle | runs | new poses per 118 s | interval median ms | requests | replies | stalls | round trip p50 / max ms |
|---|---|---|---|---|---|---|---|---|
| 66 | 30 | 5 (4 tapped) | 1145 to 1215 | 66.7 | 13799 | 13799 | 0 | 7.7 / 18.2 |
| 66 | 25 | 1 | 1235 | 66.7 | 3205 | 3205 | 0 | 7.7 / 15.5 |
| 66 | 20 | 2 (1 tapped) | 1135 to 1173 | 66.7 | 3212 | 3212 | 0 | 7.7 / 21.5 |
| 50 | 30 | 5 | 1435 to 1449 | 66.5 | 19041 | 19041 | 0 | 7.7 / 19.8 |
| 40 | 30 | 5 | 1716 to 1770 | 33.4 | 23172 | 23171 | 1 | 7.7 / 26.4 |
| 35 | 25 | 1 | 1901 | 33.4 | 5125 | 5083 | 43 | 7.7 / 54.4 |
| 30 | 20 | 1 | 963 in 98 s | 33.4 | 5664 | 5561 | 110 | 7.7 / 56.6 |

The 66/25 and 66/20 runs did not exercise the settle: with a 66 ms turn the thread sleeps to
the end of the period anyway, and the next `get_angles` came 58 ms after every `send_angles`
(measured in the taps, same in all three settles). The settle only acts when the period is
shorter than round trip plus settle. With the driver the round trip is 8 ms and a `send_angles`
is written in under 1 ms, so the 52 ms per pair of the offline table was pymycobot overhead,
not the arm: a turn can be close to settle plus 10 ms. The last three rows test the settle for
real, each with the period just above it: 30 ms holds (1 stall in 23172 requests over five
runs, 24 sends/s, new poses at 30 Hz), 25 ms stalls 43 times in 2 min, 20 ms stalls 110 times
and the arm froze 20 s once. 50/30 (20 Hz, the next `get_angles` 42 ms after a send) gave
zero stalls over five runs too, with 11 ms of margin over the one stall seen at 40/30, and is
the default (`SerialTiming`). 40/30 is the point for when one stall per 10 minutes is
acceptable.
Servo temperatures after the 35/25 run (pymycobot `get_servo_temps`): 40 C on joint 1, the
one swinging, 32 to 35 C on the others.

At 66/30 the arm answered every request within 19 ms. 15 to 18 % of the replies while moving
repeat the previous position: the arm reads its servos at its own pace, in steps of 0.09 deg,
and the servo stands still 0.8 to 0.9 s at each end of the swing while the command comes back
through its lag (0.3 deg median, 2.1 deg max).

## #150: the turn waits for the sync-write

After a `send_angles` the Atom acknowledges it, polls its six servos and passes the targets on
with one sync-write to the servos (`ff ff fe 34 83 29 07`, 56 bytes), the last frame it puts on
the line, and drops a frame that comes before that. The tap of 13 Nov put the last byte
of the sync-write at 20.4 ms after the send median, 26.7 at p99 and 31.0 at most: the fixed
30 ms settle sat at the edge of the worst case and every turn paid it in full. Since #150 the
thread reads the line until the sync-write is whole, and sends the next `get_angles` at once.
`echo_timeout` (60 ms) bounds the wait when the sync-write never comes; `send_settle_ms` is left
only for the overheat cut, between its stop and its power off.

Arm alone (no gripper, no camera) unless the row says otherwise, joint 1 swinging 10 deg, 19 Nov
2026, every point tapped.
The four 2-minute points ran with the wait bounded at 30 ms, the 10-minute run with the 60 ms
of `echo_timeout`. Sends per second over the stream; times in ms after the `send_angles`.

| period | wait bound | sends/s | requests | replies | stalls | next `get_angles` p50 / max | sync-write end p50 / p99 / max |
|---|---|---|---|---|---|---|---|
| 50 | 30 | 19.8 | 4031 | 4031 | 0 | 41.1 / 46.0 | 20.3 / 26.2 / 29.1 |
| 40 | 30 | 24.5 | 4725 | 4725 | 0 | 31.2 / 38.9 | 18.6 / 27.2 / 30.5 |
| 35 | 30 | 27.6 | 5131 | 5130 | 2 | 26.3 / 32.0 | 19.3 / 27.5 / 32.7 |
| 33 | 30 | 28.8 | 5517 | 5517 | 0 | 24.4 / 32.5 | 19.3 / 27.8 / 29.2 |
| **33, 10 min, this branch** | **60** | **28.9** | **19804** | **19804** | **0** | **24.4 / 35.5** | **19.2 / 28.0 / 35.3** |
| 33, gripper and camera | 60 | 29.2 | 5542 | 5542 | 0 | 24.5 / 41.2 | 18.4 / 26.8 / 31.8 |

- Both stalls at 35 ms came from a late sync-write: one ended 32.7 ms after its send, the other
  about 54 ms, and the `get_angles` sent at the 30 ms bound was lost both times. Hence the 60 ms
  of `echo_timeout`, which only counts when the Atom is late.
- In the 10 minutes six sync-writes ended past 30 ms (30.1 to 35.3 ms). All were waited for,
  the `get_angles` following 0.2 to 2.5 ms later, and the driver never warned of a send without
  a sync-write. No reply was lost. An earlier 10-minute run of the same turn, before the review
  fixes, gave the same picture (19874 replies, 0 stalls, sync-write 19.3 / 28.0 ms); its one
  sync-write at 92.4 ms was the tap proxy stalling about 90 ms, not the arm.
- No `get_angles` left before the end of a sync-write that came within the bound, in any run.
- With the gripper and the camera mounted and the gripper on the servo bus (the Atom pings
  it, id 7, after every send), the sync-write stays the six-servo frame: all 3386 sends of that
  run had it. Two ended past 30 ms (31.8 and 30.7 ms) and were waited for.
- 33 ms is the default since #150: one control period at 30 Hz, so every update can reach the
  arm. The round trip of a `get_angles` (7.8 ms p50, 9.4 p99) plus the wait for the sync-write
  is about 27 ms; a shorter period gains nothing while the controller runs at 30 Hz.
- `/joint_states`, 10 minutes at 33: new poses every 33.3 ms median, 34.0 at p90 (66.5 median at
  50 ms before #150). The longest gap, 1.8 s at the start, is the arm at rest before the stream.
- The servo bus was clean in every run: no bad status frame from any servo. With the gripper
  and the camera mounted and the arm held at 0 deg, servo 4 heats past about 45 degC and the
  frames of servos 4 and 5 go bad; that is a separate problem, not a timing one, still under
  test.
