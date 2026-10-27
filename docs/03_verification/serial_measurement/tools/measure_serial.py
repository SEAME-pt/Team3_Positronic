"""Read-side serial measurement: get_angles() back to back at the maximum rate, one CSV row per read.

usage: python measure_serial.py [--duration 600] [--fresh 0|1] [--rtt 200] [--out PREFIX]

Phase 1 (reads): get_angles() in a tight loop for --duration seconds. No sleep, so the loop runs as fast
  as the port allows. Period = time between the starts of consecutive reads.
Phase 2 (round trip, --rtt N, 0 to skip): N pairs of send_angles(current pose) + get_angles(). The write
  commands the pose the arm is already at, so it does not move.

Files written (PREFIX defaults to measure_serial_fresh<mode>):
  PREFIX_reads.csv      id, t (s since first read), duration (s), valid (0 when get_angles() was not a 6-list)
  PREFIX_rtt.csv        id, t, write_s, read_s, rtt_s, valid
  PREFIX_summary.json   arguments, library version, fresh-mode read-back, statistics
  PREFIX_hist.png       read period histogram (skipped with a note if matplotlib is missing)

A read slower than 450 ms is counted as a stall, the same threshold py_hz_write.py uses.
"""

import argparse
import csv
import json
import os
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from pymycobot import MyCobot280

STALL_S = 0.45
SPEED = 30


def stats_block(x):
    x = np.asarray(x)
    return {
        "n": len(x),
        "mean": float(x.mean()),
        "p50": float(np.percentile(x, 50)),
        "p99": float(np.percentile(x, 99)),
        "max": float(x.max()),
    }


def analyse_reads(t, dur, valid):
    t, dur, valid = np.asarray(t), np.asarray(dur), np.asarray(valid)
    period = np.diff(t)
    span = t[-1] - t[0]
    return {
        "reads": len(t),
        "invalid": int((valid == 0).sum()),
        "span_s": float(span),
        "mean_rate_hz": float((len(t) - 1) / span),
        "period_ms": {
            k: v * 1000 if k != "n" else v for k, v in stats_block(period).items()
        },
        "read_call_ms": {
            k: v * 1000 if k != "n" else v for k, v in stats_block(dur).items()
        },
        "stalls": int((dur > STALL_S).sum()),
        "stall_fraction": float((dur > STALL_S).mean()),
    }


def plot_hist(period_s, summary, path):
    p = np.asarray(period_s) * 1000
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4))
    a.hist(p, bins=np.linspace(0, np.percentile(p, 99.5), 80), color="tab:blue")
    a.set_xlabel("read period (ms)")
    a.set_ylabel("reads")
    a.set_title("Zoom: up to p99.5")
    b.hist(
        p,
        bins=np.logspace(np.log10(max(p.min(), 0.1)), np.log10(p.max()), 80),
        color="tab:blue",
    )
    b.set_xscale("log")
    b.set_yscale("log")
    b.set_xlabel("read period (ms, log)")
    b.set_title("Full range: log-log")
    pm = summary["period_ms"]
    fig.suptitle(
        f"get_angles period: mean rate {summary['mean_rate_hz']:.1f} Hz, "
        f"p99 {pm['p99']:.1f} ms, max {pm['max']:.0f} ms, n={summary['reads']}"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    return True


def report(summary):
    pm, cm = summary["period_ms"], summary["read_call_ms"]
    print(
        f"\nreads {summary['reads']} over {summary['span_s']:.1f} s, invalid {summary['invalid']}, "
        f"stalls (>{STALL_S * 1000:.0f} ms) {summary['stalls']}"
    )
    print(
        f"mean rate {summary['mean_rate_hz']:.2f} Hz | period mean {pm['mean']:.1f} ms, p50 {pm['p50']:.1f}, "
        f"p99 {pm['p99']:.1f}, max {pm['max']:.0f} ms"
    )
    print(f"read call p50 {cm['p50']:.1f} ms, p99 {cm['p99']:.1f} ms")
    if "rtt_ms" in summary:
        r = summary["rtt_ms"]
        print(
            f"round trip (write + read) n={r['n']}: mean {r['mean']:.1f} ms, p50 {r['p50']:.1f}, "
            f"p99 {r['p99']:.1f}, max {r['max']:.0f} ms"
        )


ap = argparse.ArgumentParser()
ap.add_argument("--duration", type=float, default=600)
ap.add_argument(
    "--fresh",
    type=int,
    choices=[0, 1],
    default=None,
    help="calls set_fresh_mode(); always pass it explicitly. If omitted the mode is not touched.",
)
ap.add_argument(
    "--rtt",
    type=int,
    default=200,
    help="write+read pairs for the round-trip phase (0 skips it)",
)
ap.add_argument("--out", default=None)
args = ap.parse_args()

prefix = args.out or "measure_serial" + (
    f"_fresh{args.fresh}" if args.fresh is not None else ""
)
if os.path.exists(prefix + "_reads.csv"):
    sys.exit(
        f"{prefix}_reads.csv exists, pass --out to avoid overwriting an earlier run"
    )

mc = MyCobot280("/dev/ttyTHS1", 1000000)
mc.power_on()

meta = {"args": vars(args)}
if args.fresh is not None:
    mc.set_fresh_mode(args.fresh)
try:
    meta["fresh_mode_readback"] = mc.get_fresh_mode()
except (AttributeError, OSError) as e:
    meta["fresh_mode_readback"] = f"unavailable ({type(e).__name__})"
print("fresh mode read-back:", meta["fresh_mode_readback"])


def valid_angles(a):
    return (
        isinstance(a, list)
        and len(a) == 6
        and all(isinstance(v, (int, float)) for v in a)
    )


base = None
for _ in range(10):
    a = mc.get_angles()
    if valid_angles(a):
        base = a
        break
    time.sleep(0.1)
if base is None:
    sys.exit("could not read starting pose")

t, dur, valid = [], [], []
try:
    print(f"reading for {args.duration:.0f} s, Ctrl+C stops early and still saves")
    with open(prefix + "_reads.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "t", "duration", "valid"])
        t_first = time.perf_counter()
        t_end = t_first + args.duration
        i = 0
        while True:
            t0 = time.perf_counter()
            if t0 >= t_end:
                break
            a = mc.get_angles()
            d = time.perf_counter() - t0
            ok = int(valid_angles(a))
            t.append(t0 - t_first)
            dur.append(d)
            valid.append(ok)
            w.writerow([i, f"{t0 - t_first:.6f}", f"{d:.6f}", ok])
            i += 1
            if i % 2000 == 0:
                f.flush()
                print(f"  {t[-1]:.0f} s, {i} reads", end="\r")
except KeyboardInterrupt:
    print("\nInterrupted, analysing what was collected")

if len(t) < 3:
    sys.exit("too few reads to analyse")
summary = analyse_reads(t, dur, valid)

rtt_rows = []
if args.rtt > 0:
    print(f"\nround trip: {args.rtt} x (send_angles(current pose) + get_angles)")
    try:
        t_first = time.perf_counter()
        for i in range(args.rtt):
            t0 = time.perf_counter()
            mc.send_angles(base, SPEED)
            t1 = time.perf_counter()
            a = mc.get_angles()
            t2 = time.perf_counter()
            rtt_rows.append(
                (i, t0 - t_first, t1 - t0, t2 - t1, t2 - t0, int(valid_angles(a)))
            )
    except KeyboardInterrupt:
        print("\nInterrupted round-trip phase")
    if len(rtt_rows) >= 3:
        with open(prefix + "_rtt.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id", "t", "write_s", "read_s", "rtt_s", "valid"])
            w.writerows(
                [
                    (
                        r[0],
                        f"{r[1]:.6f}",
                        f"{r[2]:.6f}",
                        f"{r[3]:.6f}",
                        f"{r[4]:.6f}",
                        r[5],
                    )
                    for r in rtt_rows
                ]
            )
        summary["rtt_ms"] = {
            k: v * 1000 if k != "n" else v
            for k, v in stats_block([r[4] for r in rtt_rows]).items()
        }
        summary["rtt_write_p50_ms"] = float(
            np.percentile([r[2] for r in rtt_rows], 50) * 1000
        )
        summary["rtt_read_p50_ms"] = float(
            np.percentile([r[3] for r in rtt_rows], 50) * 1000
        )
        summary["rtt_stalls"] = int(sum(r[4] > STALL_S for r in rtt_rows))

meta["servo_temps_end"] = mc.get_servo_temps()
summary["meta"] = meta
with open(prefix + "_summary.json", "w") as f:
    json.dump(summary, f, indent=1)
report(summary)
if plot_hist(np.diff(t), summary, prefix + "_hist.png"):
    print(f"plot saved to {prefix}_hist.png")
print(
    f"saved {prefix}_reads.csv, {prefix}_summary.json"
    + (f", {prefix}_rtt.csv" if rtt_rows else "")
)
