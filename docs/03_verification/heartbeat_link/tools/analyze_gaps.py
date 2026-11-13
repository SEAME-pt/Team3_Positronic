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
#   #45: write the analysis of the heartbeat link campaign. Read the CSV of record_gaps.py
#   (phase, channel, t_ns). Per phase: the gaps between consecutive heartbeats of that phase
#   only, a heartbeat gap that overlaps a /joint_states gap over 100 ms counted as a Jetson
#   stall rather than a link gap, the rate, median, p99, p99.9 and max gap, and for each
#   candidate timeout the number of link gaps longer than it (a halt the supervisor would have
#   made). Then apply the decision rule agreed before the runs: per rate, timeout = the larger
#   of 1.5 x the longest link gap and 4 heartbeat periods, rounded up to 50 ms; choose the rate
#   with the smallest timeout, the lower rate on a tie. Also list the supervisor halts recorded
#   during the run. Pure functions, unit-tested without ROS.
# Follow-up prompt:
#   The first campaign compared 50 Hz quiet with 25 Hz busy and recommended 50 Hz / 100 ms. A
#   stall of the link delays every heartbeat in flight whatever the rate, so the longest link gap
#   of any phase must bound the timeout of every rate. Apply it that way, keep each rate's own
#   longest gap in the table, and say in the report which gap was applied.
"""Analyse the heartbeat link campaign and recommend a rate and a timeout (#45)."""

#     python3 analyze_gaps.py heartbeat_2026-11-13_1000.csv

import csv
import math
import sys

TIMEOUTS_MS = (200, 250, 300, 400, 500)
# A /joint_states gap over this is the Jetson (or the recorder) stalling: it comes at 30 Hz.
LOCAL_STALL_NS = 100_000_000
# The decision rule, fixed before the runs (README).
MARGIN = 1.5
MIN_PERIODS = 4
ROUND_MS = 50
CHECK_PERIOD_MS = 20


def read_run(path):
    """Return {'beats': {phase: [ns]}, 'local': [ns], 'halts': {phase: count}} from a file."""
    run = {'beats': {}, 'local': [], 'halts': {}}
    with open(path) as file:
        for row in csv.DictReader(file):
            t_ns = int(row['t_ns'])
            if row['channel'] == 'heartbeat' and row['phase'] != 'done':
                run['beats'].setdefault(row['phase'], []).append(t_ns)
            elif row['channel'] == 'joint_states':
                run['local'].append(t_ns)
            elif row['channel'] == 'halt':
                run['halts'][row['phase']] = run['halts'].get(row['phase'], 0) + 1
    for values in run['beats'].values():
        values.sort()
    run['local'].sort()
    return run


def gaps(times):
    """Return the gaps between consecutive arrivals as (start ns, length ns)."""
    return [(a, b - a) for a, b in zip(times, times[1:])]


def percentile(values, fraction):
    """Return the value at `fraction` (0..1) of the sorted values, nearest rank."""
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(1, math.ceil(fraction * len(ordered))) - 1]


def overlaps(gap, stalls):
    """Return True when `gap` overlaps one of `stalls`, all given as (start, length)."""
    start, length = gap
    return any(s < start + length and start < s + n for s, n in stalls)


def rate_of(phase):
    """Return the rate in Hz at the start of a phase name ('25hz_busy' -> 25), else None."""
    head = phase.lower().split('hz')[0]
    return int(head) if head.isdigit() else None


def summarize(beats, local, timeouts_ms=TIMEOUTS_MS):
    """Return the figures of one phase as a dict; durations in ms."""
    all_gaps = gaps(beats)
    stalls = [g for g in gaps(local) if g[1] > LOCAL_STALL_NS]
    link = [g for g in all_gaps if not overlaps(g, stalls)]
    lengths = [n for _, n in all_gaps]
    span_s = (beats[-1] - beats[0]) / 1e9 if len(beats) > 1 else 0.0

    def ms(value):
        return None if value is None else value / 1e6

    return {
        'messages': len(beats),
        'minutes': span_s / 60,
        'rate_hz': len(all_gaps) / span_s if span_s else 0.0,
        'p50_ms': ms(percentile(lengths, 0.5)),
        'p99_ms': ms(percentile(lengths, 0.99)),
        'p999_ms': ms(percentile(lengths, 0.999)),
        'max_ms': ms(max(lengths, default=None)),
        'link_max_ms': ms(max((n for _, n in link), default=None)),
        'jetson_stalls': len(all_gaps) - len(link),
        'halts': {t: sum(n > t * 1_000_000 for _, n in link) for t in timeouts_ms},
    }


def round_up(value_ms):
    """Return value_ms rounded up to the next multiple of ROUND_MS."""
    return int(math.ceil(value_ms / ROUND_MS) * ROUND_MS)


def recommend(summaries):
    """
    Apply the decision rule to {phase: summary}.

    A stall of the link delays every heartbeat in flight, whatever the rate, so the longest link
    gap of any phase bounds the timeout of every rate (a busy phase run at one rate only still
    counts for all of them).

    Return (choices, chosen, applied): choices is {rate: (its own longest gap ms, timeout ms)},
    chosen the rate to use or None when no phase had data, applied the gap the rule used (ms).
    """
    own = {}
    for phase, s in summaries.items():
        rate = rate_of(phase)
        if s['link_max_ms'] is None or rate is None:
            continue
        own[rate] = max(own.get(rate, 0.0), s['link_max_ms'])
    if not own:
        return {}, None, None
    applied = max(own.values())
    choices = {
        rate: (gap, round_up(max(MARGIN * applied, MIN_PERIODS * 1000.0 / rate)))
        for rate, gap in own.items()
    }
    chosen = min(choices, key=lambda rate: (choices[rate][1], rate))
    return choices, chosen, applied


def phase_table(name, s, halts):
    """Return the Markdown section of one phase."""
    def ms(value):
        return '-' if value is None else f'{value:.0f}'
    hours = s['minutes'] / 60
    lines = [
        f'### {name}',
        '',
        f'{s["messages"]} heartbeats in {s["minutes"]:.1f} min ({s["rate_hz"]:.2f} Hz). '
        f'Gap: median {ms(s["p50_ms"])} ms, p99 {ms(s["p99_ms"])} ms, '
        f'p99.9 {ms(s["p999_ms"])} ms, max {ms(s["max_ms"])} ms. '
        f'Longest link gap {ms(s["link_max_ms"])} ms; {s["jetson_stalls"]} gap(s) were the '
        f'Jetson stalling. Supervisor halts recorded: {halts}.',
        '',
        '| Timeout (ms) | Would-be false halts | Per hour |',
        '|---|---|---|',
    ]
    for timeout, count in s['halts'].items():
        per_hour = f'{count / hours:.1f}' if hours else '-'
        lines.append(f'| {timeout} | {count} | {per_hour} |')
    return '\n'.join(lines) + '\n'


def report(run):
    """Return the full Markdown report of a run: one section per phase, then the decision."""
    summaries = {phase: summarize(times, run['local']) for phase, times in run['beats'].items()}
    order = sorted(summaries, key=lambda phase: (rate_of(phase) or 0, phase))
    parts = [phase_table(phase, summaries[phase], run['halts'].get(phase, 0)) for phase in order]
    choices, chosen, applied = recommend(summaries)
    parts.append('### Decision rule')
    parts.append('')
    if applied is not None:
        parts.append(f'Longest link gap of any phase: {applied:.0f} ms, applied to every rate.')
        parts.append('')
    parts.append('| Rate (Hz) | Its own longest link gap (ms) | Timeout (ms) |')
    parts.append('|---|---|---|')
    for rate in sorted(choices):
        gap, timeout = choices[rate]
        parts.append(f'| {rate} | {gap:.0f} | {timeout} |')
    parts.append('')
    if chosen is None:
        parts.append('No heartbeat data: no recommendation.')
    else:
        timeout = choices[chosen][1]
        parts.append(
            f'Recommended: heartbeat at {chosen} Hz, heartbeat_timeout_ms {timeout}. '
            f'A dead commanding node is detected within {timeout + CHECK_PERIOD_MS} ms '
            f'(timeout + check period). '
            f'Confirm with the supervisor active before adopting it (README, rule 4).')
    return '\n'.join(parts) + '\n'


def main(argv=None):
    """Print the report of the file given."""
    paths = sys.argv[1:] if argv is None else argv
    if len(paths) != 1:
        print('usage: analyze_gaps.py RUN.csv', file=sys.stderr)
        return 2
    print(report(read_run(paths[0])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
