"""Silence test: stream a ramp, stop sending mid-motion WITHOUT calling stop(),
then poll the arm and classify what the firmware did.

usage: python py_silence.py [--rate 50] [--step 2.0] [--trials 10] [--fresh 0|1] [--poll 6]

Classes:  finished  = arm ends at the last commanded target
          held      = arm stops about where it was when the stream stopped
          kept going= arm ends beyond the last commanded target
          partway   = none of the above (look at the trace)
          ambiguous = arm had already caught up with the target at the cut
          discarded = a write stalled (>450 ms) so the cut time is unreliable
"""

import argparse
import json
import random
import statistics
import time

from pymycobot import MyCobot280

ap = argparse.ArgumentParser()
ap.add_argument("--rate", type=float, default=50)
ap.add_argument(
    "--step", type=float, default=2.0, help="deg per write; higher = arm lags more"
)
ap.add_argument("--trials", type=int, default=10)
ap.add_argument("--fresh", type=int, choices=[0, 1], default=None)
ap.add_argument(
    "--poll", type=float, default=6.0, help="seconds to watch after the cut"
)
args = ap.parse_args()

LOW, HIGH, SPEED, TOL = -30.0, 30.0, 30, 2.0
PERIOD = 1 / args.rate
OUT = (
    f"py_silence_{args.rate:g}hz_step{args.step:g}"
    + (f"_fresh{args.fresh}" if args.fresh is not None else "")
    + ".json"
)


def wait_until(t):
    while True:
        r = t - time.perf_counter()
        if r <= 0:
            return
        time.sleep(r - 0.001 if r > 0.002 else 0)


def read_j6(mc):
    a = mc.get_angles()
    ok = isinstance(a, list) and len(a) == 6 and all(isinstance(v, float) for v in a)
    return a[5] if ok else None


def go_to(mc, base, angle, timeout=25):
    """Move slowly to `angle` and wait until three reads agree within 0.3 deg."""
    mc.send_angles(base[:5] + [angle], 20)
    recent, t_end = [], time.time() + timeout
    while time.time() < t_end:
        v = read_j6(mc)
        if v is None:
            continue
        recent = (recent + [v])[-3:]
        if (
            len(recent) == 3
            and max(recent) - min(recent) < 0.3
            and abs(v - angle) < 1.5
        ):
            return True
    return False


mc = MyCobot280("/dev/ttyTHS1", 1000000)
mc.power_on()
if args.fresh is not None:
    mc.set_fresh_mode(args.fresh)

base = None
for _ in range(10):
    a = mc.get_angles()
    if isinstance(a, list) and len(a) == 6:
        base = a
        break
    time.sleep(0.1)
if base is None:
    raise SystemExit("could not read starting pose")

n_max = int(55 / args.step)
k_lo, k_hi = max(3, int(0.2 * n_max)), max(4, int(0.8 * n_max))
results = []
try:
    for trial in range(args.trials):
        start = LOW if trial % 2 == 0 else HIGH  # alternate direction
        d = 1 if start == LOW else -1
        if not go_to(mc, base, start):
            print(f"trial {trial}: could not settle at start, skipping")
            continue
        k = random.randint(k_lo, k_hi)  # writes before the cut
        durs, target = [], start
        next_t = time.perf_counter()
        for i in range(k):
            wait_until(next_t)
            target = start + d * args.step * (i + 1)
            t0 = time.perf_counter()
            mc.send_angles(base[:5] + [target], SPEED)
            durs.append(time.perf_counter() - t0)
            next_t += PERIOD
        t_cut = time.perf_counter()  # last write returned: silence starts here

        trace = []
        while time.perf_counter() - t_cut < args.poll:
            v = read_j6(mc)
            if v is not None:
                trace.append((round(time.perf_counter() - t_cut, 3), v))
        if len(trace) < 6:
            print(f"trial {trial}: too few valid reads, skipping")
            continue

        a0 = trace[0][1]
        final = statistics.median(v for _, v in trace[-5:])
        lag = d * (target - a0)  # >0: arm still behind the target at the cut
        moved = d * (final - a0)
        t_settle = next((t for t, v in trace if abs(v - final) < 0.5), None)
        if max(durs) > 0.45:
            cls = "discarded"
        elif lag < 2 * TOL:
            cls = "ambiguous"
        elif d * (final - target) > TOL:
            cls = "kept going"
        elif abs(final - target) <= TOL:
            cls = "finished"
        elif abs(final - a0) <= TOL:
            cls = "held"
        else:
            cls = "partway"
        r = {
            "trial": trial,
            "direction": d,
            "writes": k,
            "last_target": target,
            "angle_at_cut": a0,
            "final": final,
            "lag_at_cut": round(lag, 1),
            "moved_after_cut": round(moved, 1),
            "settle_s": t_settle,
            "max_write_ms": round(max(durs) * 1000, 1),
            "cls": cls,
            "trace": trace,
        }
        results.append(r)
        print(
            f"trial {trial:2d} dir {d:+d} writes {k:2d} target {target:6.1f} at-cut {a0:6.1f} "
            f"final {final:6.1f} lag {lag:5.1f} -> {cls}"
        )
finally:
    try:
        mc.stop()  # safety only, AFTER the measurement
    except OSError as e:
        print(f"stop failed: {e}")
    with open(OUT, "w") as f:
        json.dump({"args": vars(args), "results": results}, f)
    counts = {}
    for r in results:
        counts[r["cls"]] = counts.get(r["cls"], 0) + 1
    print("\nSummary:", counts, "->", OUT)
