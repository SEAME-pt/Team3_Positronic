# Serial captures of the myCobot 280

This folder records the raw bytes that `pymycobot` exchanges with the arm's controller over
`/dev/ttyTHS1` (1 000 000 baud). The captures are the reference for our C++ protocol library
(issue #21): its encoder must produce the same `TX` bytes and its decoder must read the same
`RX` bytes.

| File | What it is |
|---|---|
| `capture_frames.py` | The capture tool. It **moves the arm** (see Safety). |
| `frames_20261012_172133.txt` | First capture: power on/off, `get_angles`, `send_angles` to the current pose, stop. No timestamps. |
| `frames_20261013_161443.txt` | Read-only part: firmware version, 10 s of idle line, 20 × `get_angles`, `get_angles` in power_off. |
| `frames_20261013_163056.txt` | Motion part (`--motion-only`): J1 30° out and back, STOP during a motion, `release_all_servos`. |

## Safety

There is no e-stop in the cell yet, and this script moves the arm.

- One person keeps a hand on the power switch the whole time.
- Only J1 moves: 30° towards the centre of its range and back, at speed 20. Check that side is
  clear at the height the arm is at, and that the arm isn't resting on anything.
- Every motion waits for Enter. `q`, Ctrl+C or any error sends STOP before exiting.
- `power_on` locks the servos where they are, and `power_off` / `release_all_servos` make the arm
  limp (J2 and J3 drop). Hold the arm before confirming any of them.

## How to run

On the Jetson **host** (not in the container), so it uses the system pymycobot 4.0.4b5 that
recorded the reference captures:

```bash
# from your laptop
scp scripts/capture_frames/capture_frames.py er@nano:~/captures/
ssh nano

# on the Jetson: nobody else may have the port
docker ps --filter name=mimesis-arm   # must list nothing
fuser /dev/ttyTHS1                    # must print nothing
cd ~/captures
python3 capture_frames.py --operator <name> --arm <label>                # full run
python3 capture_frames.py --operator <name> --arm <label> --motion-only  # motion part only

# back on your laptop
scp nano:captures/frames_*.txt scripts/capture_frames/
```

Run it from `~/captures`, not from `~/mimesis/<you>`: `make nano-*` mirrors that folder with
`--delete`, so a capture that only exists there is lost.

## Capture format

```
# pymycobot 4.0.4b5                       <- header: versions, date, operator, arm, port
## get_angles_00                          <- one section per step
TX fe fe 02 20 fa                         <- every byte we sent
RX ff ff 07 02 ... fe fe 0e 20 ... fa     <- every byte we received, servo-bus traffic included
RESULT [142.03, 0.35, 0.79, -39.11, -14.85, -2.37]   <- what pymycobot returned
TXT +0.000 fe fe 02 20 fa                 <- bursts with time in ms after the first TX
RXT +3.426 ff ff 07 02 ...
```

Parsers must ignore `#` lines and keywords they don't know (`TXT`, `RXT`, `NOTE`), so new fields
can be added without breaking the tests.

## What the captures showed (13 Oct 2026, firmware 7.3)

- **The `send_angles` ack is immediate** (2–3 ms). It means "received", not "arrived".
- **The line is silent when idle, but not during a motion**: the firmware's servo commands
  (56 bytes every ~10 ms) show up on our RX. The reader must drain the port continuously.
- **`get_angles` gets no reply in `power_off`**, but does after `release_all_servos`.
- **STOP works**: the servo traffic stops at once, and the ack comes ~32 ms later.
- **Frames arrive split** across reads (e.g. `fe` + `fe 03 29 01 fa`), and the noise before a
  reply changes length (a `get_angles` RX ranged from 185 to 192 bytes).
- **Accuracy is about 1°**, not the 0.01° the protocol carries.
- `get_angles` takes ~25 ms through pymycobot, but the first reply byte arrives at ~3 ms. Most of
  the time is likely Python reading one byte per call (to be confirmed in #20).

Full results and open questions are in the issue #21 comments.

## Before committing changes to the script

```bash
black --check scripts/ && ruff check scripts/   # what CI runs
chmod +x scripts/capture_frames/capture_frames.py
```
