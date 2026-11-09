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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Check the maths of joint_states_rate.py: repeats are ignored, intervals, the summary."""

import importlib.util
import os

from builtin_interfaces.msg import Time

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'scripts', 'joint_states_rate.py')


def load_script():
    """Import the script as a module, without running its main()."""
    spec = importlib.util.spec_from_file_location('joint_states_rate', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stamp_ns():
    """Check that a stamp of 1 s and 500 000 ns is 1 000 500 000 ns."""
    rate = load_script()
    assert rate.stamp_ns(Time(sec=1, nanosec=500_000)) == 1_000_500_000


def test_summary_line_counts_new_poses_and_the_longest_gap():
    """Check the line printed every second."""
    rate = load_script()
    assert rate.summary([10.0, 66.0, 12.5]) == '3 new poses  max gap 66.0 ms'
    assert rate.summary([]) is None


def test_final_summary_lists_count_min_median_p90_and_max():
    """Check the end-of-run summary."""
    rate = load_script()
    samples = [(0.1, 10.0), (0.2, 66.0), (0.3, 200.0)]
    lines = rate.final_summary(samples)
    assert lines[0] == '3 new poses in 0 s'
    assert lines[1] == ('interval min 10.0 ms  median 66.0 ms  p90 200.0 ms'
                        '  max 200.0 ms at 0.3 s')


def test_final_summary_says_so_without_samples():
    """Check that measuring nothing is reported, not hidden."""
    rate = load_script()
    assert rate.final_summary([]) == ['no new poses received']
