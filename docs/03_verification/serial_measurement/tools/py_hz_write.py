"""Write-only fixed-rate streaming test (v2).

usage: python py_hz_write.py <rate_hz> <step_deg> [fresh] [--offset-ms N] [--duration S] [--settle S]
  fresh      0 or 1. If given, set_fresh_mode(fresh) is called explicitly. If omitted, the mode is
             NOT touched (whatever the arm was left in). Always pass it explicitly for comparisons.
  --offset-ms  delay before the first scheduled write, to shift the phase of the stream.

Rows (one JSON per line): id, sched, start, duration, late, missed.
A sidecar <name>.meta.json records the arguments, library version and fresh-mode read-back.
"""

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass

from pymycobot import MyCobot280

ap = argparse.ArgumentParser()
ap.add_argument("rate", type=float)
ap.add_argument("step", type=float)
ap.add_argument("fresh", type=int, nargs="?", choices=[0, 1], default=None)
ap.add_argument("--offset-ms", type=float, default=0.0)
ap.add_argument("--duration", type=float, default=120)
ap.add_argument("--settle", type=float, default=15)
args = ap.parse_args()

RANGE_LOW, RANGE_HIGH, SPEED = -30, 30, 30
PERIOD = 1 / args.rate


def unique_name():
    base = f"py_write_{args.rate:g}hz_{int(args.step)}deg"
    if args.fresh is not None:
        base += f"_fresh{args.fresh}"
    if args.offset_ms:
        base += f"_off{args.offset_ms:g}ms"
    name, n = base + ".txt", 2
    while os.path.exists(name):  # never overwrite an earlier run
        name, n = f"{base}_{n}.txt", n + 1
    return name


OUTNAME = unique_name()


@dataclass
class Row:
    id: int
    sched: float
    start: float
    duration: float
    late: float
    missed: int


def wait_until(t):
    """Sleep coarsely, then spin for the last ~1 ms to cut sleep jitter."""
    while True:
        remaining = t - time.perf_counter()
        if remaining <= 0:
            return
        time.sleep(remaining - 0.001 if remaining > 0.002 else 0)


mc = MyCobot280("/dev/ttyTHS1", 1000000)
mc.power_on()

meta = {"args": vars(args), "outfile": OUTNAME}
if args.fresh is not None:
    mc.set_fresh_mode(args.fresh)
try:
    meta["fresh_mode_readback"] = mc.get_fresh_mode()
except (AttributeError, OSError) as e:
    meta["fresh_mode_readback"] = f"unavailable ({type(e).__name__})"

# Read the pose once (outside the timed loop) so the other joints stay put.
base = None
for _ in range(10):
    a = mc.get_angles()
    if isinstance(a, list) and len(a) == 6:
        base = a
        break
    time.sleep(0.1)
if base is None:
    sys.exit("could not read starting pose")

rows = []
try:
    mc.send_angles(base[:5] + [RANGE_LOW], 20)
    time.sleep(args.settle)

    target, direction, i = RANGE_LOW, 1, 0
    t_start = time.perf_counter() + args.offset_ms / 1000
    next_t = t_start
    end = t_start + args.duration

    while next_t < end:
        wait_until(next_t)
        t0 = time.perf_counter()
        mc.send_angles(base[:5] + [target], SPEED)
        t1 = time.perf_counter()

        sched = next_t
        missed = 0
        next_t += PERIOD
        if next_t < t1:  # overran: skip ahead and count the missed ticks
            missed = int((t1 - next_t) / PERIOD) + 1
            next_t += missed * PERIOD
        rows.append(Row(i, sched, t0, t1 - t0, t0 - sched, missed))
        i += 1

        target += direction * args.step
        if target >= RANGE_HIGH or target <= RANGE_LOW:
            target = min(max(target, RANGE_LOW), RANGE_HIGH)
            direction = -direction
except KeyboardInterrupt:
    print("\nInterrupted, saving what was collected")
finally:
    try:
        mc.stop()
    except OSError as e:
        print(f"stop failed: {e}")
    with open(OUTNAME, "w") as f:
        f.writelines(json.dumps(asdict(r)) + "\n" for r in rows)
    meta["servo_temps_end"] = mc.get_servo_temps()
    with open(OUTNAME.replace(".txt", ".meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print(f"saved {len(rows)} rows to {OUTNAME}  temps={meta.get('servo_temps_end')}")
