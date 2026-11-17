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
#   Unit tests for analyze_gaps.py on arrival times written by hand: the gaps, the nearest-rank
#   percentile, a gap longer than a timeout counted as a halt and one exactly at it not, a
#   heartbeat gap during a /joint_states stall counted as a Jetson stall, the rate read from a
#   phase name, the decision rule (1.5 x the longest gap, at least 4 periods, rounded up to 50,
#   the smallest timeout wins, the lower rate on a tie), reading a recorder file with its
#   phases and halts, and the report naming the recommendation.
# Follow-up prompt:
#   The longest link gap of any phase applies to every rate: test the first campaign's case,
#   a 170 ms stall at 25 Hz busy and 63 ms at 50 Hz quiet, which must give 25 Hz and 300 ms.
"""Tests for analyze_gaps.py, no ROS needed (python3 -m pytest on this folder)."""

import analyze_gaps as ag

MS = 1_000_000


def beats(*ms):
    """Return arrival times in ns from times in ms."""
    return [t * MS for t in ms]


def test_gaps_between_consecutive_arrivals():
    """Check the gaps as (start, length)."""
    assert ag.gaps(beats(0, 40, 100)) == [(0, 40 * MS), (40 * MS, 60 * MS)]
    assert ag.gaps(beats(0)) == []


def test_percentile_nearest_rank():
    """Check the nearest-rank percentile."""
    values = list(range(1, 101))
    assert ag.percentile(values, 0.5) == 50
    assert ag.percentile(values, 0.99) == 99
    assert ag.percentile([], 0.5) is None


def test_a_gap_over_the_timeout_is_a_halt():
    """Check that only gaps strictly longer than the timeout count, like the supervisor."""
    s = ag.summarize(beats(0, 40, 240, 280, 481), [], timeouts_ms=(200, 250))
    assert s['halts'] == {200: 1, 250: 0}  # 201 ms counts, exactly 200 does not


def test_a_gap_during_a_jetson_stall_is_not_a_link_gap():
    """Check that a heartbeat gap overlapping a /joint_states stall is set apart."""
    s = ag.summarize(beats(0, 300, 340), beats(0, 33, 66, 266, 299), timeouts_ms=(250,))
    assert s['halts'] == {250: 0}
    assert s['jetson_stalls'] == 1
    assert s['max_ms'] == 300 and s['link_max_ms'] == 40


def test_a_jetson_stall_elsewhere_does_not_excuse_a_gap():
    """Check that a stall at another moment does not hide a link gap."""
    s = ag.summarize(beats(0, 300, 340, 380), beats(400, 600, 633), timeouts_ms=(250,))
    assert s['halts'] == {250: 1}


def test_rate_of_phase():
    """Check the rate read from a phase name."""
    assert ag.rate_of('25hz_busy') == 25
    assert ag.rate_of('10hz_quiet') == 10
    assert ag.rate_of('50Hz_confirm') == 50
    assert ag.rate_of('mission_node') is None  # a stray publisher's name


def summary(link_max_ms):
    """Return the part of a summary the decision rule reads."""
    return {'link_max_ms': link_max_ms}


def test_rule_takes_one_and_a_half_times_the_longest_gap():
    """Check 1.5 x the worst gap of all the phases of a rate, rounded up to 50 ms."""
    choices, chosen, applied = ag.recommend({'25hz_quiet': summary(130),
                                             '25hz_busy': summary(205)})
    assert choices == {25: (205, 350)}  # 1.5 x 205 = 307.5 -> 350
    assert chosen == 25 and applied == 205


def test_rule_keeps_at_least_four_periods():
    """Check that a slow heartbeat never gets a timeout under 4 periods."""
    choices, _, _ = ag.recommend({'10hz_quiet': summary(110)})
    assert choices == {10: (110, 400)}  # 1.5 x 110 = 165, but 4 x 100 ms = 400


def test_rule_picks_the_smallest_timeout_then_the_lower_rate():
    """Check the choice between rates."""
    choices, chosen, _ = ag.recommend({'10hz_quiet': summary(110), '50hz_quiet': summary(70),
                                       '25hz_quiet': summary(90)})
    # 1.5 x 110 = 165 -> 200 for all; 4 periods: 400, 160, 80 ms.
    assert {rate: timeout for rate, (_, timeout) in choices.items()} == {10: 400, 25: 200, 50: 200}
    assert chosen == 25  # 25 and 50 Hz tie at 200 ms: the lower rate


def test_a_stall_seen_at_one_rate_counts_for_every_rate():
    """Check the first campaign: 170 ms at 25 Hz busy bounds 50 Hz too, tested quiet only."""
    choices, chosen, applied = ag.recommend({
        '10hz_quiet': summary(157), '25hz_quiet': summary(88), '25hz_busy': summary(170),
        '50hz_quiet': summary(63)})
    assert applied == 170
    assert choices == {10: (157, 400), 25: (170, 300), 50: (63, 300)}
    assert chosen == 25


def test_no_data_no_recommendation():
    """Check that an empty run recommends nothing."""
    assert ag.recommend({}) == ({}, None, None)


def test_read_run_and_report(tmp_path):
    """Check reading a recorder file and that the report gives the recommendation."""
    lines = ['phase,channel,t_ns']
    lines += [f'25hz_quiet,heartbeat,{t * 40 * MS}' for t in range(100)]
    lines += [f',joint_states,{t * 33 * MS}' for t in range(120)]
    lines += ['25hz_quiet,halt,5', 'done,heartbeat,0']
    path = tmp_path / 'run.csv'
    path.write_text('\n'.join(lines) + '\n')
    run = ag.read_run(str(path))
    assert len(run['beats']['25hz_quiet']) == 100
    assert run['halts'] == {'25hz_quiet': 1}
    text = ag.report(run)
    # A steady 40 ms gap: 1.5 x 40 = 60, 4 x 40 = 160 -> 200 ms.
    assert 'heartbeat at 25 Hz, heartbeat_timeout_ms 200' in text
    assert 'Supervisor halts recorded: 1' in text
