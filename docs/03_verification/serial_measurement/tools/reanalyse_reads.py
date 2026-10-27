"""Re-analyse a measure_serial reads CSV: print the statistics, redraw the histogram.

usage: python reanalyse_reads.py PREFIX_reads.csv

Writes PREFIX_hist.png next to the CSV. A read slower than 450 ms is a stall.
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

STALL_S = 0.45


def load_reads(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t"]) for r in rows])
    dur = np.array([float(r["duration"]) for r in rows])
    valid = np.array([int(r["valid"]) for r in rows])
    return t, dur, valid


def stats_ms(x_s):
    x = np.asarray(x_s) * 1000
    return {
        "mean": x.mean(),
        "p50": np.percentile(x, 50),
        "p95": np.percentile(x, 95),
        "p99": np.percentile(x, 99),
        "p99.9": np.percentile(x, 99.9),
        "max": x.max(),
    }


def plot_hist(period_s, path, title):
    p = np.asarray(period_s) * 1000
    fig, (zoom, full) = plt.subplots(1, 2, figsize=(11, 4))
    zoom.hist(p, bins=np.linspace(0, np.percentile(p, 99.5), 80), color="tab:blue")
    zoom.set_xlabel("read period (ms)")
    zoom.set_ylabel("reads")
    zoom.set_title("Zoom: up to the 99.5th percentile")
    bins = np.logspace(np.log10(max(p.min(), 0.1)), np.log10(p.max()), 80)
    full.hist(p, bins=bins, color="tab:blue")
    full.set_xscale("log")
    full.set_yscale("log")
    full.set_xlabel("read period (ms, log)")
    full.set_title("Full range: log-log axes")
    fig.suptitle(title)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=130)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv_path", type=Path)
    args = ap.parse_args()

    t, dur, valid = load_reads(args.csv_path)
    if len(t) < 3:
        raise SystemExit("too few reads to analyse")
    period = np.diff(t)
    rate = (len(t) - 1) / (t[-1] - t[0])
    stalls = int((dur > STALL_S).sum())

    print(f"reads {len(t)}, invalid {int((valid == 0).sum())}, stalls {stalls}")
    print(f"mean rate {rate:.2f} Hz over {t[-1] - t[0]:.1f} s")
    for name, x in (("period", period), ("call", dur)):
        stats = ", ".join(f"{k} {v:.2f}" for k, v in stats_ms(x).items())
        print(f"{name} (ms): {stats}")

    out = Path(str(args.csv_path).removesuffix("_reads.csv") + "_hist.png")
    pm = stats_ms(period)
    title = (
        f"get_angles period: mean rate {rate:.1f} Hz, p99 {pm['p99']:.1f} ms, "
        f"max {pm['max']:.0f} ms, n={len(t)}"
    )
    plot_hist(period, out, title)
    print(f"plot saved to {out}")


if __name__ == "__main__":
    main()
