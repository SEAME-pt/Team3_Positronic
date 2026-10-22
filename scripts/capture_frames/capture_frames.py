#!/usr/bin/env python3
# Copyright 2026 Positronic
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""Record the raw serial bytes exchanged with the myCobot 280, one section per step.

One run of this script produces one file, frames_<timestamp>.txt. That file is
the "judge" for the C++ protocol tests of issue #21: the C++ encoder and decoder
must reproduce its bytes. The file header records the SHA-256 of this script, so
anyone can check which version produced it (sha256sum capture_frames.py).

Run it on the Jetson host (system pymycobot 4.0.4b5), from a folder outside
~/mimesis/<you>, because nano-sync deletes files that only exist there:

    scp scripts/capture_frames/capture_frames.py er@nano:~/captures/
    ssh nano
    cd ~/captures && python3 capture_frames.py --operator leandro --arm A1

Sequence (each step is one or more sections in the file):
  S1  power_on                     the servos lock where they are (hold the arm)
  S2  firmware version and modes   read only
  S3  idle line for 10 s           nothing sent: does the arm talk on its own?
  S4  20 x get_angles              echo check, latency, start pose
  S5  J1 out and back              30 deg towards the centre of its range, speed 20
  S6  STOP during a slow motion    is the stop obeyed, and when is it acked?
  S7  release_all_servos           arm limp but powered: does get_angles answer,
                                   and does it follow the arm moved by hand?
  S8  power_off                    arm limp and disconnected: does get_angles answer?

Two capture modes:
  A. pymycobot call: the TX, RX and RESULT of one transaction.
  B. raw window: send without reading (or send nothing), then read the port for a
     fixed time. pymycobot can't be used here because it flushes the input before
     each command, stops reading at the footer and resends after 0.5 s without a
     reply, which would hide when a late reply really arrives.

Safety (there is no e-stop in the cell yet):
  - one person keeps a hand on the power switch the whole time;
  - only J1 moves by command: 30 deg towards the centre of its range and back;
  - every motion waits for Enter, and q, Ctrl+C or any error sends STOP;
  - power_on, release_all_servos and power_off change the torque: hold the arm.
"""

import argparse
import hashlib
import os
import platform
import statistics
import time

import pymycobot
import serial
from pymycobot import MyCobot280
from pymycobot.error import MyCobot280DataException

PORT = "/dev/ttyTHS1"
BAUD = 1000000
SPEED = 20  # new motion code starts at speed <= 30
STOP_TEST_SPEED = 10
J1_OFFSET_DEG = 30.0
J1_SAFE_LIMIT_DEG = 160.0  # pymycobot limit is +/-168; keep a margin
MOVE_WINDOW_S = (
    12.0  # long enough to see an ack that only comes at the end of the motion
)
IDLE_WINDOW_S = 10.0
STOP_TEST_WINDOW_S = 5.0
STOP_TEST_DELAY_S = 0.5
STOP_FRAME = bytes([0xFE, 0xFE, 0x02, 0x29, 0xFA])
BURST_GAP_NS = 1000000  # bytes less than 1 ms apart belong to the same burst

# Every byte that goes through any serial.Serial, with a monotonic timestamp.
# Patching the class (not one object) also records the bytes pymycobot reads and
# throws away while it searches for FE FE, which is where the servo-bus noise shows up.
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


def bursts(events, direction):
    """Group consecutive events of one direction that are less than 1 ms apart."""
    groups = []  # [t_first_ns, bytearray]
    last_t = None
    for kind, t, data in events:
        if kind != direction:
            continue
        if groups and t - last_t < BURST_GAP_NS:
            groups[-1][1].extend(data)
        else:
            groups.append([t, bytearray(data)])
        last_t = t
    return groups


def ms(delta_ns):
    return f"{delta_ns / 1e6:+.3f}"


def write_section(log, name, result, t_start_ns):
    """Write one section and return the time of its last RX byte, in ms after the first TX."""
    events = list(_events)
    tx = b"".join(data for kind, _, data in events if kind == "TX")
    rx = b"".join(data for kind, _, data in events if kind == "RX")
    tx_bursts = bursts(events, "TX")
    rx_bursts = bursts(events, "RX")
    t_ref = tx_bursts[0][0] if tx_bursts else t_start_ns

    lines = [
        f"## {name}",
        f"TX {tx.hex(' ')}".rstrip(),
        f"RX {rx.hex(' ')}".rstrip(),
        f"RESULT {result}",
    ]
    lines += [f"TXT {ms(t - t_ref)} {data.hex(' ')}" for t, data in tx_bursts]
    lines += [f"RXT {ms(t - t_ref)} {data.hex(' ')}" for t, data in rx_bursts]
    if tx_bursts and bytes(tx_bursts[0][1]) in rx:
        lines.append("NOTE first TX burst found inside RX (echo?)")
    log.write("\n".join(lines) + "\n\n")
    log.flush()

    last_rx_ms = None
    if rx_bursts:
        last_t = max(t for kind, t, _ in events if kind == "RX")
        last_rx_ms = (last_t - t_ref) / 1e6
    print(f"{name:<28} TX {len(tx):3d} B  RX {len(rx):4d} B  -> {result}")
    return last_rx_ms


def step(log, name, fn):
    """Mode A: run one pymycobot call and record what it sent and received."""
    _events.clear()
    t_start = time.monotonic_ns()
    try:
        result = fn()
    except (
        MyCobot280DataException,
        serial.SerialException,
        IndexError,
        ValueError,
    ) as exc:
        # Recorded instead of aborting: a failed read is an answer too (e.g. Q7).
        result = f"ERROR {exc!r}"
    last_rx_ms = write_section(log, name, result, t_start)
    time.sleep(
        0.2
    )  # only spaces the commands: late bytes are not recorded (mode B does that)
    return result, last_rx_ms


def window(log, name, mc, seconds, actions=(), result="None (async)"):
    """Mode B: run timed actions while reading the port raw for a fixed time.

    actions: (delay_s, fn) pairs, run once each when the window reaches delay_s.
    """
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


def confirm(prompt):
    answer = input(f"\n>>> {prompt}\n    [Enter] go   [q] abort: ")
    if answer.strip().lower() == "q":
        raise SystemExit("Aborted by the operator.")


def ask_yes(prompt):
    return input(f"\n>>> {prompt} [y/N]: ").strip().lower() == "y"


def valid_angles(result):
    return isinstance(result, list) and len(result) == 6


def emergency_stop(mc):
    try:
        mc._serial_port.write(STOP_FRAME)
        print("\nSTOP sent. If the arm is still moving, cut the power.")
    except (serial.SerialException, OSError) as exc:
        print(f"\nCould not send STOP ({exc!r}). Cut the power if the arm is moving.")


def move_j1(log, mc, name, base, j1_deg):
    """Mode B: move only J1 (other joints keep their current angles), then read back."""
    target = list(base)
    target[0] = j1_deg
    confirm(
        f"{name}: J1 moves to {j1_deg:+.1f} deg at speed {SPEED}, the other joints stay. "
        "Workspace clear and a hand on the power switch?"
    )
    window(
        log,
        name,
        mc,
        MOVE_WINDOW_S,
        actions=[(0.0, lambda: mc.send_angles(target, SPEED, _async=True))],
    )


def read_angles(log, mc, prefix, count):
    results = []
    for i in range(count):
        result, _ = step(log, f"{prefix}_{i}", mc.get_angles)
        results.append(result)
    return results


def move_j1_out_and_back(log, mc, base):
    """S5 and S6: J1 towards the centre of its range, STOP test, J1 back."""
    start_j1 = base[0]
    # Towards the centre, so the target can never pass a joint limit.
    direction = -1.0 if start_j1 >= 0 else 1.0
    moved_j1 = start_j1 + direction * J1_OFFSET_DEG
    if abs(moved_j1) > J1_SAFE_LIMIT_DEG:
        print(
            f"\nJ1 target {moved_j1:+.1f} deg is outside the safe range: skipping S5-S6."
        )
        return
    print(f"\nStart pose (deg): {base}")

    # S5: out.
    move_j1(log, mc, "send_angles_j1_out", base, moved_j1)
    confirm("Has the arm stopped moving?")
    read_angles(log, mc, "get_angles_j1_out", 3)

    # S6: start back towards the start at low speed, STOP after a short delay.
    target = list(base)
    target[0] = start_j1
    confirm(
        f"S6 stop test: J1 starts back towards {start_j1:+.1f} deg at speed "
        f"{STOP_TEST_SPEED}, STOP after {STOP_TEST_DELAY_S} s. "
        "Workspace clear and a hand on the power switch?"
    )
    window(
        log,
        "stop_during_motion",
        mc,
        STOP_TEST_WINDOW_S,
        actions=[
            (0.0, lambda: mc.send_angles(target, STOP_TEST_SPEED, _async=True)),
            (STOP_TEST_DELAY_S, lambda: mc._serial_port.write(STOP_FRAME)),
        ],
    )
    confirm("Has the arm stopped moving?")
    read_angles(log, mc, "get_angles_after_stop", 1)

    # S5 (cont.): back to where J1 started.
    move_j1(log, mc, "send_angles_j1_back", base, start_j1)
    confirm("Has the arm stopped moving?")
    read_angles(log, mc, "get_angles_j1_back", 1)


def run(log, mc):
    """Run S1-S8 and return the get_angles latencies of S4 (ms)."""
    # S1: power_on locks the servos where they are; a limp arm must be held first.
    confirm(
        "S1 power_on: the servos lock where they are now. If the arm is limp, "
        "hold it in a stable pose that touches nothing. Ready?"
    )
    step(log, "power_on", mc.power_on)
    time.sleep(1.0)

    # S2: firmware and modes, read only.
    step(log, "get_system_version", mc.get_system_version)
    step(log, "get_modify_version", mc.get_modify_version)
    step(log, "get_fresh_mode", mc.get_fresh_mode)
    step(log, "check_async_or_sync", mc.check_async_or_sync)

    # S3: idle line, nothing sent.
    print(f"\nS3: listening to an idle line for {IDLE_WINDOW_S:.0f} s...")
    window(log, "idle_10s", mc, IDLE_WINDOW_S, result="None (no tx)")

    # S4: 20 reads in a row (echo check, latency per transaction, start pose).
    latencies = []
    base = None
    for i in range(20):
        result, last_rx_ms = step(log, f"get_angles_{i:02d}", mc.get_angles)
        if valid_angles(result):
            base = result
            if last_rx_ms is not None:
                latencies.append(last_rx_ms)

    # S5 + S6
    if base is None:
        print("\nget_angles never returned 6 angles: skipping every motion (S5-S6).")
    else:
        move_j1_out_and_back(log, mc, base)

    # S7: limp but powered. Read while still, then after moving J1 and J6 by hand.
    confirm(
        "S7 HOLD THE ARM: release_all_servos makes it limp and J2/J3 will drop. "
        "Holding it?"
    )
    step(log, "release_all_servos", mc.release_all_servos)
    read_angles(log, mc, "get_angles_released", 3)
    confirm(
        "Keep supporting the arm. By hand, turn ONLY J1 by about 45 deg "
        "and J6 by about 90 deg. Done?"
    )
    read_angles(log, mc, "get_angles_released_moved", 3)

    # S8: limp and disconnected (pymycobot retries each read for ~1.5 s).
    confirm("S8 HOLD THE ARM: power_off. Holding it?")
    step(log, "power_off", mc.power_off)
    read_angles(log, mc, "get_angles_power_off", 1)

    return latencies


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--operator", required=True, help="who is running the capture")
    parser.add_argument("--arm", required=True, help="label of the arm being recorded")
    parser.add_argument(
        "--out-dir", default=".", help="where to write the capture file"
    )
    parser.add_argument("--port", default=PORT)
    args = parser.parse_args()

    out_dir = os.path.abspath(args.out_dir)
    if f"{os.sep}mimesis{os.sep}" in out_dir + os.sep:
        print(
            f"WARNING: {out_dir} is inside the folder nano-sync mirrors with --delete."
        )
        confirm("The capture would be lost on the next make nano-*. Continue anyway?")
    out = os.path.join(out_dir, f"frames_{time.strftime('%Y%m%d_%H%M%S')}.txt")

    print("Before starting:")
    print(
        "  - nobody else has the arm (docker ps --filter name=mimesis-arm, fuser /dev/ttyTHS1)"
    )
    print("  - one person keeps a hand on the power switch the whole time")
    print("  - about 30 cm clear around the arm, J1 will turn 30 deg and back")
    print("  - someone ready to hold the arm when it goes limp (S7, S8)")
    confirm("Ready?")

    mc = MyCobot280(args.port, BAUD)
    time.sleep(0.5)

    with open(out, "w") as log:
        date = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        with open(os.path.abspath(__file__), "rb") as script:
            script_sha = hashlib.sha256(script.read()).hexdigest()
        header = [
            f"# pymycobot {pymycobot.__version__}",
            f"# date: {date}  operator: {args.operator}  arm: {args.arm}",
            f"# port: {args.port}  baud: {BAUD}  python: {platform.python_version()}",
            f"# script: scripts/capture_frames/capture_frames.py  sha256: {script_sha}",
            f"# speed: {SPEED}  j1_offset_deg: {J1_OFFSET_DEG}",
            "# TXT/RXT: bursts of bytes, ms after the first TX of the section",
            "# (or after the section start when nothing was sent)",
        ]
        log.write("\n".join(header) + "\n\n")
        try:
            latencies = run(log, mc)
        except BaseException:
            emergency_stop(mc)
            raise

    if latencies:
        print(
            f"\nget_angles latency (TX to last RX byte), n={len(latencies)}: "
            f"median {statistics.median(latencies):.2f} ms, max {max(latencies):.2f} ms"
        )
    print(f"\nSaved to {out}")
    print("The arm is in power_off: leave it resting on a stable support.")


if __name__ == "__main__":
    main()
