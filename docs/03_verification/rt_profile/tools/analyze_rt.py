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
#   Write the analysis half of #39, with no ROS 2 import so it can be unit-tested and run on a
#   laptop. Read one or more cycles.csv written by rt_record.py and, for each, report the loop
#   period measured on the controller manager cycle stamps and on the arrival times: count,
#   duration, mean rate, mean, p50, p99 and max period, how many periods passed 1.5x and 2x
#   the nominal period and when the worst one happened, plus an ASCII histogram that works
#   without matplotlib and a png of the same histogram when matplotlib is there. When a
#   steps.csv sits next to a cycles.csv, also measure the command to motion latency of each
#   step: take the first cycle after the publish as the write() that carried the command, then
#   the first later cycle whose stepped joint moved more than a threshold, and report mean and
#   max over the samples, counting the steps that never moved instead of silently dropping
#   them. Print a Markdown comparison table when several runs are given, so the idle run and
#   the saturated run sit side by side, and exit non-zero when a run has too few cycles.

"""Period, jitter and command-to-motion latency of the control loop (#39)."""

# Input: the cycles.csv (and optional steps.csv) of one or more runs of rt_record.py.
#
#   python3 docs/03_verification/rt_profile/tools/analyze_rt.py \
#       /ws/log/rt/idle/cycles.csv /ws/log/rt/load/cycles.csv --rate 30 \
#       --figure-dir docs/03_verification/rt_profile/figures --report report.md
#
# A period is the interval between two consecutive cycles. Two clocks are reported:
#   cycle   header.stamp, set by the controller manager at the top of the cycle. This is the
#           loop itself, and so the interval between two write() calls.
#   arrival when the recorder received the message: the loop plus DDS and the recorder's own
#           scheduling. Always the noisier of the two.

import argparse
import csv
import dataclasses
import math
import os
import sys

NS_PER_S = 10 ** 9
NS_PER_MS = 10 ** 6
# A cycle this much longer than nominal is reported. 2.0 is the #23 criterion, 1.5 shows the
# smaller misses that a 2x limit hides.
LATE_FACTORS = (1.5, 2.0)
# The arm reports angles in steps of 0.09 deg (0.0016 rad), see docs/03_verification/write_sweep.
# Two of those steps is the smallest move that cannot be quantisation alone.
DEFAULT_MOTION_THRESHOLD_RAD = 0.0035
HISTOGRAM_BINS = 12


@dataclasses.dataclass
class Cycle:
    """One control loop cycle, as the recorder saw it."""

    receive_ns: int
    header_ns: int
    positions: tuple


@dataclasses.dataclass
class Step:
    """One command published by the recorder."""

    index: int
    publish_ns: int
    joint: int
    target_rad: float
    baseline_rad: float


@dataclasses.dataclass
class Stats:
    """Summary of a list of periods, in milliseconds."""

    count: int
    mean_ms: float
    p50_ms: float
    p99_ms: float
    max_ms: float
    max_at_s: float


@dataclasses.dataclass
class Latency:
    """Command to motion latency of one step, in milliseconds."""

    index: int
    write_ms: float  # from the write() that carried the command to the first joint state change
    publish_ms: float  # from the publish, which adds up to one cycle of waiting
    moved_rad: float


def percentile(sorted_values, percent):
    """Return the nearest-rank percentile of an already sorted list."""
    rank = max(1, math.ceil(percent / 100.0 * len(sorted_values)))
    return sorted_values[rank - 1]


def periods_ms(stamps_ns):
    """Return the interval in ms between each pair of consecutive stamps."""
    return [(after - before) / NS_PER_MS for before, after in zip(stamps_ns, stamps_ns[1:])]


def stats(periods, start_ns=None, stamps_ns=None):
    """
    Return the Stats of a list of periods in ms.

    With `stamps_ns` and `start_ns`, max_at_s says how far into the run the longest period
    ended; without them it is 0.
    """
    if not periods:
        raise ValueError('need at least two cycles to measure a period')
    worst = max(range(len(periods)), key=periods.__getitem__)
    at_s = 0.0
    if stamps_ns is not None and start_ns is not None:
        at_s = (stamps_ns[worst + 1] - start_ns) / NS_PER_S
    ordered = sorted(periods)
    return Stats(
        count=len(periods),
        mean_ms=sum(periods) / len(periods),
        p50_ms=percentile(ordered, 50),
        p99_ms=percentile(ordered, 99),
        max_ms=periods[worst],
        max_at_s=at_s)


def late_periods(periods, limit_ms):
    """Return (index, period in ms) of every period longer than limit_ms."""
    return [(index, value) for index, value in enumerate(periods) if value > limit_ms]


def histogram(periods, bins=HISTOGRAM_BINS, upper_ms=None):
    """
    Return (edges, counts) of a histogram of `periods`, plus the overflow as the last count.

    The bins span from the shortest period to `upper_ms` (p99 by default), so the shape near
    nominal stays readable; everything above lands in the overflow bin.
    """
    ordered = sorted(periods)
    low = ordered[0]
    high = upper_ms if upper_ms is not None else percentile(ordered, 99)
    if high <= low:
        high = low + 1.0
    width = (high - low) / bins
    edges = [low + width * index for index in range(bins + 1)]
    counts = [0] * (bins + 1)  # the extra one is the overflow
    for value in ordered:
        if value > high:
            counts[bins] += 1
        else:
            counts[min(bins - 1, int((value - low) / width))] += 1
    return edges, counts


def ascii_histogram(periods, width=46):
    """Return the histogram as text lines, for the evidence document."""
    edges, counts = histogram(periods)
    peak = max(counts) or 1
    lines = []
    for index in range(len(edges) - 1):
        bar = '#' * int(round(width * counts[index] / peak))
        label = f'{edges[index]:7.1f} to {edges[index + 1]:6.1f} ms'
        lines.append(f'{label:<21} | {bar:<{width}} {counts[index]}')
    overflow = '#' * int(round(width * counts[-1] / peak))
    label = f'{edges[-1]:7.1f} ms and above'
    lines.append(f'{label:<21} | {overflow:<{width}} {counts[-1]}')
    return lines


def first_index_after(stamps_ns, when_ns):
    """Return the index of the first stamp strictly after `when_ns`, or None."""
    for index, stamp in enumerate(stamps_ns):
        if stamp > when_ns:
            return index
    return None


def latency_samples(cycles, steps, threshold_rad=DEFAULT_MOTION_THRESHOLD_RAD, timeout_s=3.0):
    """
    Return (samples, skipped) for the commanded steps.

    For one step: the write() that carried it is the first cycle after the publish, and the
    motion is the first cycle after that whose stepped joint moved further than
    `threshold_rad` from where it was at that write(). A step whose target is inside the
    threshold, or that produced no motion within `timeout_s`, is counted in `skipped` with the
    reason instead of entering the statistics.
    """
    stamps = [cycle.header_ns for cycle in cycles]
    samples = []
    skipped = []
    for step in steps:
        write_index = first_index_after(stamps, step.publish_ns)
        if write_index is None:
            skipped.append((step.index, 'no cycle recorded after the publish'))
            continue
        start_rad = cycles[write_index].positions[step.joint]
        if abs(step.target_rad - start_rad) <= threshold_rad:
            skipped.append((step.index, 'target within the motion threshold of the pose'))
            continue
        deadline_ns = stamps[write_index] + int(timeout_s * NS_PER_S)
        moved = None
        for cycle in cycles[write_index + 1:]:
            if cycle.header_ns > deadline_ns:
                break
            if abs(cycle.positions[step.joint] - start_rad) > threshold_rad:
                moved = cycle
                break
        if moved is None:
            skipped.append((step.index, f'no motion within {timeout_s:.0f} s'))
            continue
        samples.append(Latency(
            index=step.index,
            write_ms=(moved.header_ns - stamps[write_index]) / NS_PER_MS,
            publish_ms=(moved.header_ns - step.publish_ns) / NS_PER_MS,
            moved_rad=moved.positions[step.joint] - start_rad))
    return samples, skipped


def read_cycles(path):
    """Return the Cycle rows of a cycles.csv."""
    with open(path, newline='', encoding='utf-8') as handle:
        return [
            Cycle(receive_ns=int(row['receive_ns']), header_ns=int(row['header_ns']),
                  positions=tuple(float(row[name]) for name in sorted(row) if
                                  name.startswith('pos')))
            for row in csv.DictReader(handle)]


def read_steps(path):
    """Return the Step rows of a steps.csv."""
    with open(path, newline='', encoding='utf-8') as handle:
        return [
            Step(index=int(row['index']), publish_ns=int(row['publish_ns']),
                 joint=int(row['joint']), target_rad=float(row['target_rad']),
                 baseline_rad=float(row['baseline_rad']))
            for row in csv.DictReader(handle)]


def run_lines(label, cycles, rate_hz, threshold_rad, steps=None):
    """Return the report lines of one run, and the cycle Stats it is summarised by."""
    nominal_ms = 1000.0 / rate_hz
    header_stamps = [cycle.header_ns for cycle in cycles]
    arrival_stamps = [cycle.receive_ns for cycle in cycles]
    cycle_stats = stats(periods_ms(header_stamps), header_stamps[0], header_stamps)
    arrival_stats = stats(periods_ms(arrival_stamps), arrival_stamps[0], arrival_stamps)
    duration_s = (header_stamps[-1] - header_stamps[0]) / NS_PER_S
    lines = [
        f'=== {label}: {len(cycles)} cycles over {duration_s:.1f} s, '
        f'nominal {nominal_ms:.1f} ms ({rate_hz:g} Hz)',
        f'  mean rate  {(len(cycles) - 1) / duration_s:.2f} Hz',
        f'  cycle      mean {cycle_stats.mean_ms:.1f}  p50 {cycle_stats.p50_ms:.1f}  '
        f'p99 {cycle_stats.p99_ms:.1f}  max {cycle_stats.max_ms:.1f} ms '
        f'at {cycle_stats.max_at_s:.1f} s',
        f'  arrival    mean {arrival_stats.mean_ms:.1f}  p50 {arrival_stats.p50_ms:.1f}  '
        f'p99 {arrival_stats.p99_ms:.1f}  max {arrival_stats.max_ms:.1f} ms',
    ]
    periods = periods_ms(header_stamps)
    for factor in LATE_FACTORS:
        late = late_periods(periods, factor * nominal_ms)
        name = f'late {factor:g}x'
        detail = f'  {name:<10} {len(late)} periods above {factor * nominal_ms:.1f} ms'
        if late:
            worst = sorted(late, key=lambda item: -item[1])[:5]
            detail += '; worst: ' + ', '.join(f'{value:.1f} ms' for _, value in worst)
        lines.append(detail)
    lines.append('  histogram of the cycle period:')
    lines += [f'    {line}' for line in ascii_histogram(periods)]
    if steps:
        lines += latency_lines(cycles, steps, threshold_rad)
    return lines, cycle_stats


def latency_lines(cycles, steps, threshold_rad):
    """Return the command to motion latency lines of one run."""
    samples, skipped = latency_samples(cycles, steps, threshold_rad)
    if not samples:
        return [f'  latency    no usable sample out of {len(steps)} steps '
                f'({len(skipped)} skipped)']
    write = sorted(sample.write_ms for sample in samples)
    publish = sorted(sample.publish_ms for sample in samples)
    lines = [
        f'  latency    {len(samples)} samples of {len(steps)} steps, motion threshold '
        f'{threshold_rad:.4f} rad',
        f'    write() to first joint state change   mean {sum(write) / len(write):.1f}  '
        f'min {write[0]:.1f}  max {write[-1]:.1f} ms',
        f'    publish to first joint state change   mean {sum(publish) / len(publish):.1f}  '
        f'min {publish[0]:.1f}  max {publish[-1]:.1f} ms',
    ]
    if skipped:
        lines.append('    skipped: ' + ', '.join(f'#{index} ({why})' for index, why in skipped))
    return lines


def comparison_table(rows, rate_hz):
    """Return the Markdown table of several runs, to paste into the evidence document."""
    nominal_ms = 1000.0 / rate_hz
    lines = [
        f'| Run | cycles | duration s | mean ms | p50 ms | p99 ms | max ms | '
        f'> {1.5 * nominal_ms:.1f} ms | > {2 * nominal_ms:.1f} ms |',
        '|---|---|---|---|---|---|---|---|---|',
    ]
    for label, cycle_stats, duration_s, late15, late20 in rows:
        lines.append(
            f'| {label} | {cycle_stats.count + 1} | {duration_s:.1f} | '
            f'{cycle_stats.mean_ms:.1f} | {cycle_stats.p50_ms:.1f} | {cycle_stats.p99_ms:.1f} | '
            f'{cycle_stats.max_ms:.1f} | {late15} | {late20} |')
    return '\n'.join(lines) + '\n'


def zoom_upper_ms(ordered, nominal_ms):
    """
    Return the upper edge of the zoomed histogram, in ms.

    p99 keeps the shape around nominal readable, but on a run where more than 1 % of the
    cycles are late p99 sits in the tail and the zoom stops being a zoom. The cap of 1.5x
    nominal is the smaller of the two late limits reported: everything up to the first late
    cycle is drawn, the rest is counted in the axis label and shown in the full-range panel.
    """
    return max(min(percentile(ordered, 99), 1.5 * nominal_ms), ordered[0] + 1.0)


def zoom_lower_ms(ordered, nominal_ms):
    """
    Return the lower edge of the zoomed histogram, in ms.

    Same idea as zoom_upper_ms at the other end: a handful of short cycles (a late cycle is
    always followed by a short one, so the pair sums to two nominal periods) would otherwise
    drag the left edge far below the peak and squash it against the right. The floor is p1, no
    lower than 0.5x nominal, so the window stays centred on the peak and the few short cycles
    are counted in the label, not drawn.
    """
    return min(max(percentile(ordered, 1), 0.5 * nominal_ms), nominal_ms - 1.0)


def plot_histogram(periods, label, rate_hz, path):
    """Save the histogram as a png. False when matplotlib is not installed."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    ordered = sorted(periods)
    nominal_ms = 1000.0 / rate_hz
    figure, (zoom, full) = plt.subplots(1, 2, figsize=(11, 4))
    upper = zoom_upper_ms(ordered, nominal_ms)
    lower = zoom_lower_ms(ordered, nominal_ms)
    above = sum(1 for value in ordered if value > upper)
    below = sum(1 for value in ordered if value < lower)
    zoom.hist(periods, bins=60, range=(lower, upper), color='tab:blue')
    zoom.axvline(nominal_ms, color='tab:red', linewidth=1)
    zoom.set_xlabel(f'cycle period (ms), {lower:.1f} to {upper:.1f} '
                    f'({below} shorter, {above} longer, not drawn)')
    zoom.set_ylabel('cycles')
    zoom.set_title('Zoom, red line = nominal')
    full.hist(periods, bins=60, color='tab:blue')
    full.set_yscale('log')
    full.set_xlabel('cycle period (ms), full range')
    full.set_title('Full range, log counts')
    figure.suptitle(f'{label}: control loop period, nominal {nominal_ms:.1f} ms, '
                    f'p99 {percentile(ordered, 99):.1f} ms, max {ordered[-1]:.1f} ms, '
                    f'n={len(periods)}')
    figure.tight_layout()
    figure.savefig(path, dpi=130)
    plt.close(figure)
    return True


def parse_arguments(argv):
    """Return the parsed command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cycles', nargs='+', help='cycles.csv of each run, in report order')
    parser.add_argument('--rate', type=float, required=True,
                        help='nominal update_rate of the controller manager, in Hz')
    parser.add_argument('--label', action='append', default=[],
                        help='name of each run (default: the name of its folder)')
    parser.add_argument('--motion-threshold-rad', type=float,
                        default=DEFAULT_MOTION_THRESHOLD_RAD,
                        help='smallest move counted as motion, in rad '
                             f'(default {DEFAULT_MOTION_THRESHOLD_RAD})')
    parser.add_argument('--figure-dir', help='folder for period_hist_<label>.png')
    parser.add_argument('--report', help='also write the comparison table as Markdown here')
    return parser.parse_args(argv)


def label_for(path, labels, index):
    """Return the label of a run: the one given, else the name of the folder it sits in."""
    if index < len(labels):
        return labels[index]
    return os.path.basename(os.path.dirname(os.path.abspath(path))) or f'run{index}'


def main(argv=None):  # pragma: no cover
    """Analyse every run given, print the report and return the exit code."""
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    nominal_ms = 1000.0 / args.rate
    table_rows = []
    for index, path in enumerate(args.cycles):
        label = label_for(path, args.label, index)
        cycles = read_cycles(path)
        if len(cycles) < 2:
            print(f'{path}: {len(cycles)} cycles, nothing to measure.', file=sys.stderr)
            return 1
        steps_path = os.path.join(os.path.dirname(os.path.abspath(path)), 'steps.csv')
        steps = read_steps(steps_path) if os.path.exists(steps_path) else None
        lines, cycle_stats = run_lines(label, cycles, args.rate, args.motion_threshold_rad,
                                       steps)
        print('\n'.join(lines))
        print()
        periods = periods_ms([cycle.header_ns for cycle in cycles])
        table_rows.append((
            label, cycle_stats,
            (cycles[-1].header_ns - cycles[0].header_ns) / NS_PER_S,
            len(late_periods(periods, 1.5 * nominal_ms)),
            len(late_periods(periods, 2.0 * nominal_ms))))
        if args.figure_dir:
            os.makedirs(args.figure_dir, exist_ok=True)
            figure = os.path.join(args.figure_dir, f'period_hist_{label}.png')
            if plot_histogram(periods, label, args.rate, figure):
                print(f'plot saved to {figure}\n')
            else:
                print('matplotlib is not installed: no png written, '
                      'the ASCII histogram above is the same data\n')
    table = comparison_table(table_rows, args.rate)
    print(table)
    if args.report:
        with open(args.report, 'w', encoding='utf-8') as handle:
            handle.write(table)
    return 0


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
