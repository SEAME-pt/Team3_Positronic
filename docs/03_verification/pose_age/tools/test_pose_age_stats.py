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
#   Unit tests for pose_age_stats.py (#120), no ROS needed: find_age() finds driver/pose_age_ms
#   among other entries and interfaces and returns None when the gpio or the interface is
#   missing; summarize() gives count, NaN count, min, mean, p50, p99 and max, leaves NaN out of
#   the statistics, and gives None for every statistic when no age is finite; percentile() at
#   its edges; markdown() writes '-' for a missing value.
# Follow-up prompt:
#   The interface is now driver/pose_age_s, in seconds; find_age() returns ms. Check the
#   conversion, and that NaN stays NaN.
"""Unit tests of pose_age_stats.py."""

import math
from types import SimpleNamespace

import pose_age_stats
import pytest


def entry(names, values):
    """Return one InterfaceValue-like entry of a DynamicJointState."""
    return SimpleNamespace(interface_names=names, values=values)


def test_finds_the_age_among_other_entries():
    """Check that the driver entry is found after the joints, among other interfaces."""
    names = ['joint2_to_joint1', 'driver']
    values = [
        entry(['position', 'velocity'], [0.1, 0.0]),
        entry(['fault_code', 'pose_age_s'], [0.0, 0.0125]),
    ]
    assert pose_age_stats.find_age(names, values) == pytest.approx(12.5)


def test_no_driver_or_no_age_is_none():
    """Check that a message without the gpio, or a gpio without the interface, gives None."""
    assert pose_age_stats.find_age(['joint2_to_joint1'], [entry(['position'], [0.1])]) is None
    assert pose_age_stats.find_age(['driver'], [entry(['fault_code'], [0.0])]) is None
    assert pose_age_stats.find_age([], []) is None


def test_nan_is_returned_as_nan():
    """Check that NaN, the age before the first pose, comes through as NaN, not None."""
    age = pose_age_stats.find_age(['driver'], [entry(['pose_age_s'], [math.nan])])
    assert math.isnan(age)


def test_summary_of_known_ages():
    """Check every statistic on a known set, with NaN counted apart."""
    ages = [math.nan] + [float(value) for value in range(1, 101)]
    summary = pose_age_stats.summarize(ages)
    assert summary['count'] == 101
    assert summary['nan'] == 1
    assert summary['min'] == 1.0
    assert summary['mean'] == 50.5
    assert summary['p50'] == 50.0
    assert summary['p99'] == 99.0
    assert summary['max'] == 100.0


def test_summary_without_a_finite_age():
    """Check that only NaN, or nothing at all, gives no statistics."""
    for ages in ([], [math.nan, math.nan]):
        summary = pose_age_stats.summarize(ages)
        assert summary['count'] == len(ages)
        assert summary['nan'] == len(ages)
        for key in ('min', 'mean', 'p50', 'p99', 'max'):
            assert summary[key] is None


def test_infinite_ages_are_left_out():
    """Check that an infinite age is not finite and stays out of the statistics."""
    summary = pose_age_stats.summarize([math.inf, 5.0])
    assert summary['max'] == 5.0
    assert summary['nan'] == 0


def test_percentile_edges():
    """Check the nearest rank at both ends and with one value."""
    assert pose_age_stats.percentile([7.0], 0.99) == 7.0
    assert pose_age_stats.percentile([1.0, 2.0], 0.0) == 1.0
    assert pose_age_stats.percentile([1.0, 2.0], 1.0) == 2.0


def test_markdown_table():
    """Check the table: one decimal, '-' for a missing value."""
    table = pose_age_stats.markdown(pose_age_stats.summarize([2.0, 4.0]), 60)
    assert '| 2 in 60 s | 0 | 2.0 | 3.0 | 2.0 | 4.0 | 4.0 |' in table
    empty = pose_age_stats.markdown(pose_age_stats.summarize([]), 60)
    assert '| 0 in 60 s | 0 | - | - | - | - | - |' in empty
