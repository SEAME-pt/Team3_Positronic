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
#   Write a pytest suite for analyze_rt.py that needs no ROS 2 and no arm: build cycles and
#   steps in memory and test the pure functions. Cover: periods of a steady stream; nearest
#   rank percentiles; mean, p50, p99, max and where the worst period happened; the 1.5x and 2x
#   late counts on a stream with one injected stall; the histogram counts add up to the number
#   of periods and an outlier lands in the overflow bin; the ASCII histogram has one line per
#   bin plus the overflow; the write() that carried a step is the first cycle after the
#   publish; the latency is measured to the first cycle that moved more than the threshold, so
#   quantisation dither does not count as motion; a step that commands less than the threshold
#   and a step that never moves are both skipped with a reason instead of entering the
#   statistics; cycles.csv and steps.csv survive a write and read round trip; and the
#   comparison table has one row per run.

"""Unit tests for the real-time profile analysis of issue #39 (no ROS 2 needed)."""

import csv
import os

import analyze_rt as rt
import pytest

RATE_HZ = 30.0
PERIOD_NS = int(rt.NS_PER_S / RATE_HZ)
NOMINAL_MS = 1000.0 / RATE_HZ
POSE = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6)
START_NS = 1_700_000_000 * rt.NS_PER_S


def cycles(count=100, period_ns=PERIOD_NS, positions=POSE, start_ns=START_NS):
    """Return `count` cycles at a steady period, all at the same pose."""
    return [
        rt.Cycle(receive_ns=start_ns + index * period_ns + 200_000,
                 header_ns=start_ns + index * period_ns,
                 positions=tuple(positions))
        for index in range(count)]


def test_periods_of_a_steady_stream():
    """A steady stream has count - 1 periods, all equal to the nominal one."""
    periods = rt.periods_ms([cycle.header_ns for cycle in cycles(10)])
    assert len(periods) == 9
    assert periods == pytest.approx([NOMINAL_MS] * 9)


def test_percentile_is_nearest_rank():
    """The percentile is the value at the nearest rank, never interpolated."""
    values = [1.0, 2.0, 3.0, 4.0, 5.0]
    assert rt.percentile(values, 50) == 3.0
    assert rt.percentile(values, 99) == 5.0
    assert rt.percentile(values, 1) == 1.0


def test_stats_locate_the_worst_period():
    """mean, p50 and max come out, and max_at_s says when the long cycle ended."""
    stamps = [cycle.header_ns for cycle in cycles(11)]
    stamps = stamps[:5] + [stamp + 200 * rt.NS_PER_MS for stamp in stamps[5:]]
    result = rt.stats(rt.periods_ms(stamps), stamps[0], stamps)
    assert result.count == 10
    assert result.max_ms == pytest.approx(NOMINAL_MS + 200.0)
    assert result.p50_ms == pytest.approx(NOMINAL_MS)
    # The long period ends on the sixth cycle: five nominal periods plus the stall.
    assert result.max_at_s == pytest.approx(5 * NOMINAL_MS / 1000.0 + 0.2)


def test_stats_need_two_cycles():
    """One cycle is not a period."""
    with pytest.raises(ValueError):
        rt.stats([])


def test_late_counts_at_both_factors():
    """A 60 ms cycle passes 1.5x nominal but not 2x, so it is counted once."""
    periods = [NOMINAL_MS] * 20 + [60.0]
    assert len(rt.late_periods(periods, 1.5 * NOMINAL_MS)) == 1
    assert rt.late_periods(periods, 2.0 * NOMINAL_MS) == []


def test_histogram_counts_every_period_and_overflows():
    """Every period is counted once, and one far outlier lands in the overflow bin."""
    periods = [NOMINAL_MS + index * 0.01 for index in range(99)] + [600.0]
    edges, counts = rt.histogram(periods)
    assert len(edges) == rt.HISTOGRAM_BINS + 1
    assert sum(counts) == len(periods)
    assert counts[-1] == 1


def test_ascii_histogram_has_a_line_per_bin():
    """One line per bin plus the overflow line, each ending with its own count."""
    lines = rt.ascii_histogram([NOMINAL_MS + index * 0.1 for index in range(50)])
    assert len(lines) == rt.HISTOGRAM_BINS + 1
    assert lines[-1].startswith(' ')
    assert sum(int(line.rsplit(' ', 1)[1]) for line in lines) == 50


def test_zoom_lower_stays_a_zoom_when_a_few_cycles_are_short():
    """The zoomed histogram never stretches below 0.5x nominal, however short the outliers."""
    quiet = sorted([NOMINAL_MS] * 100)
    assert rt.zoom_lower_ms(quiet, NOMINAL_MS) == pytest.approx(NOMINAL_MS - 1.0)
    short = sorted([NOMINAL_MS] * 97 + [5.0, 6.0, 7.0])
    assert rt.zoom_lower_ms(short, NOMINAL_MS) == pytest.approx(0.5 * NOMINAL_MS)


def test_zoom_upper_stays_a_zoom_when_many_cycles_are_late():
    """The zoomed histogram never stretches past 1.5x nominal, however long the tail is."""
    quiet = sorted([NOMINAL_MS] * 100)
    assert rt.zoom_upper_ms(quiet, NOMINAL_MS) == pytest.approx(NOMINAL_MS + 1.0)
    late = sorted([NOMINAL_MS] * 90 + [400.0] * 10)
    assert rt.zoom_upper_ms(late, NOMINAL_MS) == pytest.approx(1.5 * NOMINAL_MS)


def test_first_index_after_is_strict():
    """The write() of a step is the first cycle after it, never the one it landed on."""
    stamps = [10, 20, 30]
    assert rt.first_index_after(stamps, 20) == 2
    assert rt.first_index_after(stamps, 19) == 1
    assert rt.first_index_after(stamps, 30) is None


def moving_run(delay_cycles=4, moved_rad=0.02, dither_rad=0.0):
    """
    Return (cycles, steps) of one step that moves after `delay_cycles` cycles.

    `dither_rad` is added to the pose before the motion, to stand for the quantisation of the
    arm's own position reports.
    """
    rows = cycles(40)
    publish_ns = rows[9].header_ns + 1_000  # between cycle 9 and cycle 10
    for index in range(10, len(rows)):
        offset = dither_rad if index < 10 + delay_cycles else moved_rad
        rows[index] = rt.Cycle(
            receive_ns=rows[index].receive_ns, header_ns=rows[index].header_ns,
            positions=(POSE[0] + offset,) + POSE[1:])
    step = rt.Step(index=0, publish_ns=publish_ns, joint=0,
                   target_rad=POSE[0] + 0.0873, baseline_rad=POSE[0])
    return rows, [step]


def test_latency_is_measured_from_the_write_that_carried_the_step():
    """The latency runs from the first cycle after the publish to the first moved cycle."""
    rows, steps = moving_run(delay_cycles=4)
    samples, skipped = rt.latency_samples(rows, steps)
    assert skipped == []
    assert len(samples) == 1
    assert samples[0].write_ms == pytest.approx(4 * NOMINAL_MS)
    # The publish happened just after a cycle, so it waits almost a whole cycle more.
    assert samples[0].publish_ms > samples[0].write_ms


def test_dither_below_the_threshold_is_not_motion():
    """A move of one quantisation step does not start the clock; the real move does."""
    rows, steps = moving_run(delay_cycles=6, dither_rad=0.0016)
    samples, _ = rt.latency_samples(rows, steps)
    assert samples[0].write_ms == pytest.approx(6 * NOMINAL_MS)


def test_step_that_commands_no_motion_is_skipped():
    """A target inside the threshold is reported, not measured."""
    rows = cycles(40)
    step = rt.Step(index=3, publish_ns=rows[9].header_ns + 1_000, joint=0,
                   target_rad=POSE[0] + 0.001, baseline_rad=POSE[0])
    samples, skipped = rt.latency_samples(rows, [step])
    assert samples == []
    assert skipped[0][0] == 3
    assert 'threshold' in skipped[0][1]


def test_step_that_never_moves_is_skipped():
    """A commanded step the arm never answered is counted, with the reason."""
    rows = cycles(40)
    step = rt.Step(index=1, publish_ns=rows[9].header_ns + 1_000, joint=0,
                   target_rad=POSE[0] + 0.0873, baseline_rad=POSE[0])
    samples, skipped = rt.latency_samples(rows, [step], timeout_s=1.0)
    assert samples == []
    assert 'no motion' in skipped[0][1]


def test_csv_round_trip(tmp_path):
    """What rt_record.py writes is what analyze_rt.py reads back."""
    rows = cycles(5)
    path = tmp_path / 'cycles.csv'
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['receive_ns', 'header_ns'] + [f'pos{i}' for i in range(6)])
        for cycle in rows:
            writer.writerow([cycle.receive_ns, cycle.header_ns]
                            + [f'{value:.6f}' for value in cycle.positions])
    read_back = rt.read_cycles(str(path))
    assert len(read_back) == 5
    assert read_back[0].header_ns == rows[0].header_ns
    assert read_back[0].positions == pytest.approx(POSE)

    steps_path = tmp_path / 'steps.csv'
    with open(steps_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['index', 'publish_ns', 'joint', 'target_rad', 'baseline_rad'])
        writer.writerow([0, 12345, 0, '0.187300', '0.100000'])
    steps = rt.read_steps(str(steps_path))
    assert steps[0].publish_ns == 12345
    assert steps[0].target_rad == pytest.approx(0.1873)


def test_run_lines_and_comparison_table():
    """One block per run, and one table row per run."""
    rows, steps = moving_run()
    lines, cycle_stats = rt.run_lines('idle', rows, RATE_HZ, rt.DEFAULT_MOTION_THRESHOLD_RAD,
                                      steps)
    assert lines[0].startswith('=== idle: 40 cycles')
    assert any('latency' in line for line in lines)
    table = rt.comparison_table([('idle', cycle_stats, 1.3, 0, 0)], RATE_HZ)
    assert table.count('\n') == 3
    assert '| idle |' in table


def test_label_falls_back_to_the_folder_name():
    """Without --label a run is named after the folder its cycles.csv sits in."""
    assert rt.label_for(os.path.join('log', 'load', 'cycles.csv'), [], 0) == 'load'
    assert rt.label_for(os.path.join('log', 'load', 'cycles.csv'), ['idle'], 0) == 'idle'
