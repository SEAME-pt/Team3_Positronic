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
#   Unit tests for halt_latency.py on /joint_states samples written by hand: at rest and not
#   (NaN included), the moment of rest as the first of N slow samples in a row after the
#   trigger, a slow sample followed by a fast one not counting, the speed just before the
#   trigger, the travel of the joint that moved most, and the CSV line with and without a
#   rest and a heartbeat.
"""Tests for halt_latency.py, no ROS needed (python3 -m pytest on this folder)."""

import math

import halt_latency as hl

MS = 1_000_000


def sample(t_ms, position, velocity):
    """Return a one-joint /joint_states sample at t_ms."""
    return (t_ms * MS, (position,), (velocity,))


# A joint moving at 0.3 rad/s that is told to halt at 100 ms and is at rest from 233 ms on.
SAMPLES = [
    sample(0, 0.000, 0.30),
    sample(33, 0.010, 0.30),
    sample(67, 0.020, 0.30),
    sample(100, 0.030, 0.30),
    sample(133, 0.038, 0.20),
    sample(167, 0.043, 0.10),
    sample(200, 0.045, 0.05),
    sample(233, 0.046, 0.01),
    sample(267, 0.046, 0.00),
    sample(300, 0.046, 0.00),
]


def test_at_rest():
    """Check the rest test on one set of velocities."""
    assert hl.at_rest((0.0, 0.01, -0.02), 0.02)
    assert not hl.at_rest((0.0, 0.03), 0.02)
    assert not hl.at_rest((0.0, math.nan), 0.02)  # unknown is not "at rest"
    assert not hl.at_rest((), 0.02)


def test_rest_is_the_first_of_the_slow_samples_in_a_row():
    """Check the moment of rest after the trigger."""
    assert hl.rest_time_ns(SAMPLES, 100 * MS, 0.02, 3) == 233 * MS


def test_a_slow_sample_followed_by_a_fast_one_does_not_count():
    """Check that the run of slow samples starts again after a fast one."""
    samples = [sample(100, 0.0, 0.01), sample(133, 0.0, 0.30),
               sample(167, 0.0, 0.01), sample(200, 0.0, 0.01), sample(233, 0.0, 0.01)]
    assert hl.rest_time_ns(samples, 100 * MS, 0.02, 3) == 167 * MS


def test_samples_before_the_trigger_do_not_count():
    """Check that an arm standing still before the trigger is not 'already at rest'."""
    samples = [sample(0, 0.0, 0.0), sample(33, 0.0, 0.0), sample(67, 0.0, 0.0),
               sample(100, 0.0, 0.3), sample(133, 0.0, 0.3)]
    assert hl.rest_time_ns(samples, 90 * MS, 0.02, 3) is None


def test_no_rest_without_enough_samples():
    """Check that fewer than the needed samples give no rest."""
    assert hl.rest_time_ns(SAMPLES, 280 * MS, 0.02, 3) is None


def test_speed_just_before_the_trigger():
    """Check the speed of the last sample at or before the trigger."""
    assert hl.speed_before(SAMPLES, 100 * MS) == 0.30
    assert hl.speed_before(SAMPLES, 150 * MS) == 0.20
    assert hl.speed_before(SAMPLES, -1) is None


def test_travel_after_the_trigger():
    """Check how far the joint went between the trigger and the rest."""
    travel = hl.travel_after(SAMPLES, 100 * MS, 233 * MS)
    assert math.isclose(travel, 0.016)


def test_travel_is_the_joint_that_moved_most():
    """Check that the largest change among the joints is reported."""
    samples = [(0, (0.0, 1.0), (0.0, 0.0)), (100 * MS, (0.1, 0.7), (0.0, 0.0))]
    assert math.isclose(hl.travel_after(samples, 0, 100 * MS), 0.3)


def test_csv_line():
    """Check one full line: 50 ms of requests, at rest 133 ms after the trigger."""
    line = hl.csv_line(1, 2, True, 100 * MS, 150 * MS, SAMPLES, 206 * MS, 0.02, 3)
    assert line == '1,HEARTBEAT_LOST,1,50.0,133.0,17.19,0.92,206.0'


def test_csv_line_without_rest_or_heartbeat():
    """Check that what was not measured is left empty, not printed as zero."""
    line = hl.csv_line(3, 1, False, 280 * MS, None, SAMPLES, None, 0.02, 3)
    assert line == '3,STOP_SERVICE,0,,,0.00,,'


def test_header_has_one_name_per_column():
    """Check that the header and a line have the same number of columns."""
    line = hl.csv_line(1, 2, True, 100 * MS, 150 * MS, SAMPLES, 206 * MS, 0.02, 3)
    assert len(hl.HEADER.split(',')) == len(line.split(','))
