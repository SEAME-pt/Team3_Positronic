# Serial-thread timing sweep on the real arm (#32)

What the driver can sustain when it writes commands and reads poses on the same serial line.
The thread sends one `send_angles` per turn when the command changed, waits `send_settle_ms`
for the arm to pass it to the servos, and polls `get_angles` every turn. Both timings are
launch arguments of `driver.launch.py` (`moving_period_ms`, `send_settle_ms`), so a sweep
needs no rebuild.

## Offline measurement (2 Nov 2026, pymycobot, arm standing still)

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

For every point it starts the driver with those timings, activates
`forward_position_controller`, streams a command at 30 Hz for 120 s (joint 1 swinging 10 deg,
4 s per cycle) while `joint_states_rate.py` records the interval between new poses on
`/joint_states`, then stops the driver and cools down for 2 min. Results:
`log/sweep_<date>/<period>_<settle>/summary.txt`.

`/joint_states` shows the rate the rest of ROS sees, not stalls: the arm stands still at the
ends of the swing and now and then answers with the position it already had, and both look
like a long interval. Stalls (a `get_angles` with no reply) are counted on the serial line:

```
python3 docs/03_verification/write_sweep/tools/proxy_tap.py /ws/log/tap_66_30.csv   # terminal 1
PORT=$(cat /tmp/proxy_port) POINTS="66:30" bash docs/03_verification/write_sweep/tools/sweep.sh
```

`proxy_tap.py` opens the real port, offers a pty to the driver and writes every chunk of both
directions with its time to the csv given. Stop it with Ctrl+C after the run and start it again
with a new name for the next. On the PC, `python3 tools/tap_stalls.py tap_66_30.csv` prints
requests, replies, stalls, repeated poses and the round trip.

## Results (3 Nov 2026, joint 1 swinging 10 deg at speed 30)

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
