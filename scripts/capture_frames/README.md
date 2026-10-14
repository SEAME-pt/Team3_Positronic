# Serial captures of the myCobot 280

This folder records the raw bytes that `pymycobot` exchanges with the arm's controller over
`/dev/ttyTHS1` (1 000 000 baud). The captures are the reference for our C++ protocol library
(issue #21): its encoder must produce the same `TX` bytes and its decoder must read the same
`RX` bytes.

| File | What it is |
|---|---|
| `capture_frames.py` | The capture tool. It **moves the arm** (see Safety). |
| `frames_<date>_<time>.txt` | The capture: one complete run of the script, used by the C++ tests. |

One run of the script produces one file, and the file header records the script's SHA-256, so a
capture can always be traced to the exact script version that made it:

```bash
grep '^# script' frames_*.txt          # the hash recorded in the capture
sha256sum capture_frames.py             # must print the same hash
```

If the script changes, run it again and replace the capture, so the two always match.

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
python3 capture_frames.py --operator <name> --arm <label>

# back on your laptop
scp nano:captures/frames_*.txt scripts/capture_frames/
```

Run it from `~/captures`, not from `~/mimesis/<you>`: `make nano-*` mirrors that folder with
`--delete`, so a capture that only exists there is lost.

## What one run records

| Step | What happens | What it answers |
|---|---|---|
| S1 | `power_on` (the servos lock where they are) | Ack of power_on |
| S2 | Firmware version, refresh mode, sync mode | Which firmware the capture belongs to |
| S3 | 10 s listening, nothing sent | Does the arm talk on its own when idle? |
| S4 | 20 × `get_angles` | Echo, latency, start pose |
| S5 | J1 30° towards the centre of its range and back | `send_angles` ack timing, traffic during a motion |
| S6 | STOP 0.5 s into a slow motion | Is STOP obeyed, and when is it acked? |
| S7 | `release_all_servos`, then J1 and J6 moved by hand | Does `get_angles` answer, and follow the arm, while limp? |
| S8 | `power_off` | Does `get_angles` answer after power_off? |

The script stops for Enter before every step that depends on a person, and takes about 5 minutes.

## How it works

```
pymycobot  --->  serial.Serial.write / read  --->  /dev/ttyTHS1  <--->  arm controller
                       (patched: every chunk is
                        logged with a timestamp)
                               |
                               v
                 _events: [("TX" | "RX", t_ns, bytes), ...]
                               |
                               v  one section per step
                 frames_<date>_<time>.txt
```

### 1. Recording every byte

Before anything else, the script replaces `write` and `read` on the `serial.Serial` **class**, so
every byte that goes through any serial port is appended to `_events` with a monotonic timestamp:

```python
_events = []  # (direction, t_ns, bytes)
_orig_write = serial.Serial.write
_orig_read = serial.Serial.read


def logged_write(self, data):
    _events.append(("TX", time.monotonic_ns(), bytes(data)))
    return _orig_write(self, data)


def logged_read(self, size=1):
    data = _orig_read(self, size)
    if data:
        _events.append(("RX", time.monotonic_ns(), bytes(data)))
    return data


serial.Serial.write = logged_write
serial.Serial.read = logged_read
```

- **Why the class and not one object:** pymycobot creates its own `Serial` object inside
  `MyCobot280`. Patching the class catches it without touching pymycobot, and also records the bytes
  pymycobot reads and throws away while it searches for `FE FE`. That's where the servo-bus traffic
  shows up.
- **Why `time.monotonic_ns()`:** it never goes backwards, even if NTP corrects the clock during a
  capture. `time.time()` can.

### 2. Two ways to run a step

**Mode A, `step()`: one pymycobot call.** pymycobot sends the request and reads the reply as usual;
the patch records both. Used for `get_angles`, `power_on`, versions and so on.

```python
def step(log, name, fn):
    _events.clear()
    t_start = time.monotonic_ns()
    try:
        result = fn()
    except (MyCobot280DataException, serial.SerialException, IndexError, ValueError) as exc:
        # Recorded instead of aborting: a failed read is an answer too (e.g. Q7).
        result = f"ERROR {exc!r}"
    last_rx_ms = write_section(log, name, result, t_start)
    ...
```

**Mode B, `window()`: read the port raw for a fixed time.** Used for the idle line, the motions
and the STOP test. Here pymycobot can't be trusted to read, because it:

1. flushes the input buffer before every command, so bytes that arrive between commands are lost;
2. stops reading at the footer of the reply it expects, so anything after it is never read;
3. resends the command if no reply comes within 0.5 s, so a late reply would show up as repeated TX.

So the window sends with `_async=True` (write only, no read) and reads the port itself:

```python
def window(log, name, mc, seconds, actions=(), result="None (async)"):
    port = mc._serial_port
    port.reset_input_buffer()  # start the window clean
    _events.clear()
    t_start = time.monotonic_ns()
    pending = sorted(actions, key=lambda action: action[0])
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        while pending and pending[0][0] <= time.monotonic() - t0:
            pending.pop(0)[1]()
        port.read(port.in_waiting or 1)  # blocks up to port.timeout (0.1 s)
    write_section(log, name, result, t_start)
```

`actions` are `(delay_s, function)` pairs run once each when the window reaches `delay_s`. The STOP
test uses two: `send_angles` at 0 s and the raw STOP frame at 0.5 s.

### 3. From events to a section

`write_section()` turns the events of one step into the lines of the capture file:

- **`TX` / `RX`:** every byte sent / received, concatenated. This is what the C++ tests use.
- **`TXT` / `RXT`:** the same bytes grouped into **bursts** (chunks less than 1 ms apart), each with
  its time in ms after the first TX of the section. The `+` only marks a relative time:
  `RXT +2.537 fe fe 03 22 01 fa` means "this arrived 2.537 ms after we sent".
- **`NOTE`:** written if the first TX burst appears inside the RX, which would mean the arm echoes
  what we send.

### 4. Safety in the code

J1 always moves **towards the centre of its range**, so the target can't pass a joint limit wherever
J1 starts. The other joints get their current angles as targets, so they stay where they are:

```python
direction = -1.0 if start_j1 >= 0 else 1.0
moved_j1 = start_j1 + direction * J1_OFFSET_DEG
```

Every motion waits for Enter in `confirm()`, and whatever ends the run early (`q`, Ctrl+C or an
exception) goes through one place that sends STOP before exiting:

```python
try:
    latencies = run(log, mc)
except BaseException:
    emergency_stop(mc)  # writes FE FE 02 29 FA
    raise
```

### 5. Tracing a capture to the script

The header records the SHA-256 of the script file itself, so a capture and the script that produced
it can always be matched:

```python
with open(os.path.abspath(__file__), "rb") as script:
    script_sha = hashlib.sha256(script.read()).hexdigest()
```

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

## What the captures showed (first runs, 13 Oct 2026, firmware 7.3)

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

These come from the first two runs, recorded with earlier versions of the script and kept
in the git history. Full results and open questions are in the issue #21 comments.

## Before committing changes to the script

```bash
black --check scripts/ && ruff check scripts/   # what CI runs
chmod +x scripts/capture_frames/capture_frames.py
```
