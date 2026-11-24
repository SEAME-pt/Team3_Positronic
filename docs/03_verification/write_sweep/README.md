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
`arm_controller` to `forward_position_controller`, streams a command at 30 Hz for 120 s (joint 1
swinging 10 deg, 4 s per cycle) while `joint_states_rate.py` records the interval between new
poses on `/joint_states`, then stops the driver and cools down for 2 min. Results: `log/sweep_<date>/<period>/summary.txt`. A point
`P:E` also sets `echo_timeout_ms`, the longest wait for the sync-write (the sweeps of #32 set
`send_settle_ms` the same way; it is no parameter since #150).

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
job of a script ignores SIGINT, and every row is already flushed. On the PC,
`python3 tools/tap_stalls.py tap_33.csv` prints requests, replies, stalls, repeated poses and
the round trip, and `python3 tools/tap_turns.py tap_33.csv` the turn after each `send_angles`:
when the Atom's sync-write ended, every sync-write later than 30 ms, the sends followed by a
`get_angles` before the next send (#170) and the intervals between sends.

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
`echo_timeout_ms` (60 ms, 20 to 80) bounds the wait when the sync-write never comes and is the
parameter in place of the settle, which is left as a constant after the gripper's
`set_gripper_value` only (the overheat cut waits for the stop's ack since the third part of #170:
its power off, 30 ms after the stop, had been dropped in all three recorded cuts).

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
  about 54 ms (the Atom stalls like this about once in 10 minutes: 54.7 ms on 19 Nov, 70.1 on
  20 Nov, see #170 below), and
  the `get_angles` sent at the 30 ms bound was lost both times. Hence the 60 ms of
  `echo_timeout`, which only counts when the Atom is late: sync-writes as late as 46.1 ms were
  waited for without a loss.
- In the 10 minutes six sync-writes ended past 30 ms (30.1 to 35.3 ms). All were waited for,
  the `get_angles` following 0.2 to 2.5 ms later, and the driver never warned of a send without
  a sync-write. No reply was lost. An earlier 10-minute run of the same turn, before the review
  fixes, gave the same picture (19874 replies, 0 stalls, sync-write 19.3 / 28.0 ms).
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

## #170: the pose from the Atom's poll, the send on the controller's write

After a `send_angles` the Atom reads its six servos (a SYNC READ of 15 bytes at register 56; the
gripper, id 7, answers too when plugged) before the sync-write. Since #170 the thread decodes
those status frames, checksum included, and publishes the pose when the sync-write closes: no
`get_angles` follows a send. The thread is woken by the controller's `write()` and sends then,
so the stream follows the controller's cadence. A `get_angles` goes out only when a turn has no
pose from a poll: the first turn, a lost or corrupt poll, a sync-write past `echo_timeout_ms`.
At rest the controller still writes every cycle, and each write sends the unchanged target
again, so the pose keeps coming from the poll (6 to 22 sends/s at rest in the final run of this
part, between `get_angles`: a write that landed during a `get_angles` was lost, fixed in the third
part). The exceptions to that repeat are in the driver's README.

Three runs of 10 minutes at 33 ms, 30 Hz, arm alone (no gripper, no camera), joint 1 swinging
10 deg, all tapped. The #150 column is the 10-minute run of the table above; the prototype is
the first cut of #170 on 19 Nov; the last column is this branch on 20 Nov.

| | #150 | #170 prototype, 19 Nov | #170, 20 Nov |
|---|---|---|---|
| sends/s in the stream | 28.9 | 29.8 | 30.0 |
| `get_angles` in the stream | 17214 (one per turn) | 2 | 9 |
| sync-write end p50 / p99 / max ms | 19.2 / 28.0 / 35.3 | 18.8 / 27.8 / 54.7 | 18.7 / 27.8 / 34.8 |
| send interval p50 / p90 / p99 ms | 33.4 / 36.9 / 58.0 | 33.3 / 35.4 / 42.7 | 33.3 / 35.5 / 41.7 |
| send intervals over 40 ms | 4.2 % | 2.0 % | 1.3 % |
| polls broken / stalls / bad frames | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| `/joint_states` new poses median / p90 ms | 33.3 / 34.0 | 33.3 / 34.0 | 33.3 / 34.0 |

- `get_angles` in 10 minutes: 19804 with #150, 672 here (9 turns fell back to one in the
  stream, the rest at rest before and after it), 97 % fewer. The pose of every send comes from
  the servos' own answer to the Atom, 19 ms after the send.
- Two earlier 10-minute runs of the day shaped the code. The first had 199 `get_angles` in the
  stream: since #169 the turn that asks the temperatures sends nothing, so the next turn had no
  pose from a poll and asked one, every 3 s, with 52 ms between the sends around it. The reply
  to the temperatures now counts as the arm's reply for the next turn, which sends at once. The
  second run, with that fix, had 4 `get_angles` but 2.1 % of the send intervals over 40 ms:
  191 around a temperature request (43 ms, the request's own 8 ms) and 182 of 55 to 66 ms, where
  the controller's `write()` had landed while the thread was still reading the sync-write, and
  the wait then cleared the wake flag and slept until its two-period deadline. The flag is now
  cleared when the turn reads its command, so such a write is sent right after the sync-write:
  those 182 became 49, the 1.3 % of the table. The 192 around a temperature request remain
  (199 requests went out in the stream, 192 of them inside a slow interval): that is the request
  itself, which the status frames can replace (the next part of #170).
- The first run also had the Atom stall once: acknowledged at 2.2 ms, pinged id 7 at 5.1 ms,
  then nothing on the line for 65 ms, the SYNC READ only at 70.1 ms. The driver waited the 60 ms
  of `echo_timeout_ms`, sent one `get_angles`, answered, and logged one warning. One such stall
  per 10 minutes on 19 Nov too (54.7 ms); the timeout is what covers it. The run in the table
  had none (34.8 ms at most, all waited for).
- The joints that did not move, J2 to J6, stayed within one step (0.09 deg) on `/joint_states`
  over the 10 minutes, poses from the poll and from `get_angles` mixed: no offset between the two
  readings.
- Refused target, 30 s: the rest pose streamed with J4 at -131.8 deg, which the Atom refuses
  whole (nothing moves; the URDF allows -150). Three sends got no sync-write and were counted
  (that code sent the unchanged target again on each write; it now goes out once)
  and warned about, the pose kept coming by `get_angles` (2610 asked, 2610 answered, 0 stalls),
  and the tracking check raised its fault after 300 ms and held the targets. The arm stayed
  within one step of where it was.
- Gripper and camera mounted, 2 min at 33 ms: the Atom polls seven servos and the frame of id 7
  is skipped. J4, holding the gripper, went from 37 to 50 degC: 57 % of the frames recognised as
  its came with a bad checksum, and 2694 of the 6861 polls had none of it (the other six servos:
  2 bad in about 41000, and J5 missing from 132 polls), so `get_angles` followed 73 % of the
  sends. Still 29.2 sends/s, 2875 requests and 2875 replies, 0 stalls, 0 warnings: the fallback
  fits in the period. The send interval went to 61 ms at p99 (7.9 % over 40 ms), the price of
  the 8 ms of each `get_angles`.
- That run also showed the hole of an 8-bit checksum: of J4's 2369 corrupt frames about one in
  256 passed it, and four times `/joint_states` carried J4 at 177.6 or 180.0 deg (0 steps) for
  two cycles, while the arm held it at -2.4. The Atom's own `get_angles` replies never did: it
  checks the frame too and keeps a servo's last good position when its frame fails (1123 bad J4
  frames, 1123 sane replies). The thread now does the same: a frame whose position is more than
  30 deg from the last pose, or whose voltage or temperature cannot be a servo's, is left out,
  and a joint missing from a poll keeps its last good position for 100 ms before the
  `get_angles` comes back. Replayed on the frames of that run, the bridge brings the sends that
  fall back to `get_angles` from 63 % to 55 % (2163 of 3918): it helps a little, and it has not
  yet run on a hot servo on the arm.
- Control loop at 32 and 33 Hz (periods 31 and 30 ms), 2 min each, arm alone: 31.5 and 32.5
  sends/s, 3 and 5 `get_angles` in the stream, sync-write at most 29.9 and 33.8 ms, 0 stalls,
  0 echo timeouts, 2.5 % and 2.2 % of the send intervals over 40 ms. Then 10 min at 33 Hz with
  the final code: 20236 sends, 33.0/s, 15 `get_angles` in the stream, sync-write p99 27.1 and
  max 42.2 ms (six past the 30 ms period in 20236, two past 33), send intervals p99 38.5 ms
  and 0.7 % over 40 (132 of the 139 are the temperature turns), 0 stalls, 0 echo timeouts,
  0 warnings. Nothing lost up to 33 Hz; the Atom's sync-write p99 leaves 2.5 ms at 33 Hz and
  nothing at 35. 30 stays the default; the choice is #134's, and this is its evidence.
- `/joint_states` repeats the previous pose in 42 to 50 % of its messages at 30, 32 and 33 Hz
  alike (41.6 to 46.5 at 30 Hz, 48.8 at 32, 50.1 at 33), and 8 to 11 % of the intervals between
  new poses span two cycles; #150 had 48 % repeated poses in its 10 minutes. It is how the
  servos' position reaches the line, not the turn or the rate: the `joint_states_rate.py` p90
  crosses from one cycle to two with that share (62 ms at 32 Hz, 34 at 30 and 33). Not measured
  further here.

How the numbers were made: `python3 tools/tap_turns.py tap.csv` gives the sends per second of
the stream, the send intervals with the share over 40 ms, the sends followed by a `get_angles`
before the next send, and the sync-write end; `python3 tools/tap_stalls.py tap.csv` the
requests, replies and stalls.

## #170, third part: the temperatures from the poll frames

Since this part a poll whose six frames are all fresh is a temperature reading for the guard, so
`get_servo_temps` is only asked when no send goes out, or every 3 s while the poll comes broken,
and the cut is by time:
see "Servo over-temperature" in [the driver's README](../../../src/positronic_hardware/README.md).
The reason for the time: J1 gives false hot readings in frames with a good checksum, one poll at
a time. In the run below, at a median of 42 degC, 12 polls were 10 degC or more above it, up to
98, 2 of them at 60 or more, each one alone between normal polls. Two readings in a row, the
rule before, are 33 ms apart at 30 Hz.

10 minutes at 33 ms, arm alone, 20 Nov 2026, against the run of the second part above:

| | second part | **third part** |
|---|---|---|
| sends/s in the stream | 30.0 | 30.0 |
| temperature requests in the stream | 199 (192 of them a slow interval) | **0** (10 at rest) |
| turns that fell back to `get_angles` in the stream | 9 | 2 |
| send interval p50 / p90 / p99 ms | 33.3 / 35.5 / 41.7 | 33.3 / 35.0 / 38.2 |
| send intervals over 40 ms | 1.3 % | **0.3 %** (61 of 63 are Linux jitter, 1 the Atom at 44.6 ms, 1 a fallback) |
| stalls / warnings | 0 / 0 | 0 / 0 |

The cut from the frames, gripper mounted, `servo_temp_limit_c` 50, joint 1 swinging, 20 Nov
(the code of then: two readings in a row, a bridged poll counted). J4 read 45 degC in its frames
from 25 s and sent 1114 corrupt frames over the run, and `/joint_states` never left -2.37 deg:
the bridge and the jump check of the second part, at work on a hot servo. Three corrupt frames
in the polls of sends passed the checksum (75, 73 and 62 degC, all with 0.0 V, at 32, 35 and
56 s) and were rejected by the plausibility rule. Times from the first send: the first
plausible J4 frame at 50 in the poll of a send came at 66.44 s, and the stop 87 ms after it,
then the power off, no send after the stop, the power off repeated every second. That cut came
from one frame: its poll was not published (J5 missing and not bridgeable), the next two polls
had no J4 frame, the bridge carried the 50 into both, and the count took them as two readings.
With this part none of the three is a reading, and that run would have waited for
`get_servo_temps`, whose last answer, 1.8 s before the stop, was 49.

The repeat of 21 Nov (cut50c, J4 from 40 degC under the gripper, forearm horizontal) shows that
path: J4's frames stopped passing the checksum at 46 degC and never came back, the guard fell
back to `get_servo_temps`, which read 46, 47, 48, 49, 50, each two or three times, and the stop
and power off came 1.8 s after the first reading at 50. The cut by time, replayed on that tap's
readings, trips at the same reading as the rule of then (the request 53.20 s after the first
send, the stop 9 ms after it). The replays feed each tap's readings (the whole polls between a
send and its sync-write, and the `get_servo_temps` replies) through `OverheatDetector`, with a
script that is not in the repo; the rule itself is pinned by unit tests:
`HotPollsCutTheArmOnceTheyLastHotFor`, `PollsWithABridgedFrameLeaveTheAsksGoing` and the
`OverheatDetector` tests. The Atom's own
reading kept following the servo while no frame of it passed: whether it can go stale is not
shown by any tap.
