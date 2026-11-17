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

# AI-generated (Claude, Anthropic).
# Prompt:
#   Write the analysis for #124: for each run folder of bench/run124.sh read cycles.csv, print
#   the percentiles of the cycle stamp period, of the arrival period and of the latency, find
#   every gap above 100 ms, and for each one print the rows around it, the time since the last
#   goal or throwaway participant and the vmstat row of that second. With --figure draw one
#   panel per run, stamp period against time with the goals as vertical marks. Standard library
#   only, matplotlib only for the figure.

"""#124: the pauses of /joint_states in run folders written by bench/run124.sh.

    python3 pauses_124.py <run folder> [<run folder> ...] [--figure out.png]

Per folder: period of the cycle stamps and of the arrival times of /joint_states (cycles.csv of
rt_record.py), the latency arrival minus stamp, and for every gap above 100 ms: the rows
around it, the time since the last goal or participant (goals.txt or churn.txt) and the vmstat
row of that second. With --figure: one panel per folder, cycle stamp period against time, with
the goals or participants as vertical marks.
"""
import csv
import os
import sys

GAP_S = 0.1


def load_cycles(folder):
    """Return the arrival times and the cycle stamps of cycles.csv, in seconds."""
    rows = list(csv.DictReader(open(os.path.join(folder, 'cycles.csv'))))
    rx = [int(r['receive_ns']) * 1e-9 for r in rows]
    hd = [int(r['header_ns']) * 1e-9 for r in rows]
    return rx, hd


def load_marks(folder):
    """Return the wall time of every goal (goals.txt) or throwaway participant (churn.txt)."""
    marks = []
    for name in ('goals.txt', 'churn.txt'):
        path = os.path.join(folder, name)
        if os.path.exists(path):
            for line in open(path):
                parts = line.split()
                is_goal = parts[:1] == ['goal']
                is_start = parts[:3:2] == ['participant', 'start']
                if len(parts) >= 4 and (is_goal or is_start):
                    marks.append(float(parts[3]))
    return marks


def pct(values, f):
    """Return the value at fraction f of the sorted list."""
    return sorted(values)[min(len(values) - 1, int(len(values) * f))]


def analyse(folder):
    """Print the periods of one run and what happened around each pause."""
    rx, hd = load_cycles(folder)
    ph = [b - a for a, b in zip(hd, hd[1:])]
    pr = [b - a for a, b in zip(rx, rx[1:])]
    lat = [r - h for r, h in zip(rx, hd)]
    print(f'\n=== {folder}: {len(rx)} cycles, {rx[-1] - rx[0]:.0f} s')

    def stats(name, values):
        return (f'{name} p50 {pct(values, .5) * 1e3:.1f} p99 {pct(values, .99) * 1e3:.1f} '
                f'max {max(values) * 1e3:.1f} ms')
    print(f'{stats("stamp period", ph)} | {stats("arrival", pr)} | {stats("latency", lat)}')
    marks = load_marks(folder)
    vm = []
    if os.path.exists(os.path.join(folder, 'vmstat.txt')):
        vm = [line for line in open(os.path.join(folder, 'vmstat.txt'))
              if line.strip() and line.strip()[0].isdigit()]
    start_path = os.path.join(folder, 'start.txt')
    t_start = float(open(start_path).readline()) if os.path.exists(start_path) else rx[0]
    # A pause is a stamp gap, as in pauses.md: cycles the loop produced and nobody published.
    # The late message that arrives just before it also shows as an arrival gap, so that gap is
    # part of the same pause. An arrival gap with the stamps intact is a late delivery, not a
    # pause, and is listed apart.
    pauses = [i for i, p in enumerate(ph) if p > GAP_S]
    late = [i for i, p in enumerate(ph)
            if pr[i] > GAP_S and p <= GAP_S and (i + 1 == len(ph) or ph[i + 1] <= GAP_S)]
    print(f'pauses (stamp gap > {GAP_S * 1e3:.0f} ms): {len(pauses)}, '
          f'arrival-only gaps: {len(late)}, marks: {len(marks)}')
    for i in sorted(pauses + late):
        t = hd[i + 1]
        since = min([t - m for m in marks if m <= t], default=float('nan'))
        kind = 'pause' if i in pauses else 'arrival-only gap'
        print(f'  {kind} at +{t - rx[0]:.2f} s: stamp gap {ph[i] * 1e3:.0f} ms, '
              f'arrival gap {pr[i] * 1e3:.0f} ms, '
              f'latency of the late message {lat[i + 1] * 1e3:.0f} ms, {since:.1f} s after a mark')
        for k in range(max(0, i - 3), min(len(ph), i + 4)):
            print(f'      stamp {ph[k] * 1e3:6.1f}  arrival {pr[k] * 1e3:6.1f}  '
                  f'latency {lat[k + 1] * 1e3:6.1f}')
        sec = int(t - t_start)
        if 0 <= sec < len(vm):
            print('      vmstat (r b swpd free buff cache si so bi bo in cs us sy id wa st):')
            for k in range(max(0, sec - 2), min(len(vm), sec + 2)):
                print('      ' + ('>> ' if k == sec else '   ') + vm[k].strip())


def figure(folders, out):
    """Draw one panel per run, stamp period against time, goals as vertical marks."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(folders), 1, figsize=(11, 2.1 * len(folders)), sharex=True)
    axes = [axes] if len(folders) == 1 else list(axes)
    for ax, folder in zip(axes, folders):
        rx, hd = load_cycles(folder)
        t0 = rx[0]
        ax.plot([r - t0 for r in rx[1:]], [(b - a) * 1e3 for a, b in zip(hd, hd[1:])],
                lw=0.6, color='#1f4e79')
        marks = load_marks(folder)
        for m in marks:
            ax.axvline(m - t0, color='#c55a11', lw=0.5, alpha=0.6)
        pauses = sum(1 for a, b in zip(hd, hd[1:]) if b - a > GAP_S)  # stamp gaps, as analyse()
        ax.set_ylim(0, 400)
        ax.set_ylabel('ms')
        label = f'{os.path.basename(folder.rstrip("/"))}: {pauses} pauses, {len(marks)} marks'
        ax.text(0.01, 0.80, label, transform=ax.transAxes, fontsize=9)
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel('s since the first message')
    fig.suptitle('/joint_states stamp period. Orange: a goal (or a throwaway participant) starts')
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print('wrote', out)


def main(argv):
    """Analyse every folder given, draw the figure when --figure is given."""
    out = None
    if '--figure' in argv:
        k = argv.index('--figure')
        out = argv[k + 1]
        argv = argv[:k] + argv[k + 2:]
    for folder in argv:
        analyse(folder)
    if out:
        figure(argv, out)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
