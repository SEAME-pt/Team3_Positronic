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

"""Check the maths of joint_states_delay.py: one message, the line per second, the summary."""

import importlib.util
import os

from builtin_interfaces.msg import Time

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'scripts', 'joint_states_delay.py')


def load_script():
    """Import the script as a module, without running its main()."""
    spec = importlib.util.spec_from_file_location('joint_states_delay', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_delay_is_now_minus_stamp_in_ms():
    """A stamp 5 ms before now gives 5.0 ms, across a second boundary."""
    module = load_script()
    stamp = Time(sec=99, nanosec=998_000_000)
    now_ns = 100 * 10**9 + 3_000_000
    assert module.delay_ms(stamp, now_ns) == 5.0


def test_summary_counts_averages_and_takes_the_worst():
    """Three delays give the count, the mean and the maximum, one decimal each."""
    module = load_script()
    assert module.summary([4.0, 5.0, 9.0]) == '3 msgs  avg 6.0 ms  max 9.0 ms'


def test_summary_of_nothing_is_none():
    """No message in the second: nothing to average."""
    module = load_script()
    assert module.summary([]) is None


def test_final_summary_passes_when_every_delay_is_within_the_limit():
    """Three messages under 100 ms: PASS, zero above the limit, the worst named with its time."""
    module = load_script()
    lines, passed = module.final_summary([(0.0, 4.0), (1.0, 30.0), (2.0, 5.0)], 100.0)
    assert passed
    assert lines[0] == '3 msgs in 2 s'
    assert lines[1] == 'avg 13.0 ms  max 30.0 ms at 1.0 s'
    assert lines[2] == '0 msgs above 100 ms'
    assert lines[-1] == 'PASS: limit 100 ms'


def test_final_summary_fails_and_lists_the_late_messages():
    """Two messages above the limit: FAIL, counted and listed with their time."""
    module = load_script()
    samples = [(0.0, 5.0), (15.2, 103.0), (16.1, 117.9), (20.0, 6.0)]
    lines, passed = module.final_summary(samples, 100.0)
    assert not passed
    assert lines[2] == '2 msgs above 100 ms'
    assert lines[3] == 'first: 103.0 ms at 15.2 s, 117.9 ms at 16.1 s'
    assert lines[-1] == 'FAIL: limit 100 ms'


def test_final_summary_of_nothing_fails():
    """No message at all is not a pass: nothing was measured."""
    module = load_script()
    lines, passed = module.final_summary([], 100.0)
    assert not passed
    assert lines == ['no messages received']


def test_write_results_makes_the_csv_and_the_summary(tmp_path):
    """delays.csv has a header and one line per message, summary.txt the lines given."""
    module = load_script()
    module.write_results(str(tmp_path), [(0.0, 4.0), (0.02, 5.5)], ['3 msgs', 'PASS'])
    assert (tmp_path / 'delays.csv').read_text() == 'seconds,delay_ms\n0.000,4.000\n0.020,5.500\n'
    assert (tmp_path / 'summary.txt').read_text() == '3 msgs\nPASS\n'
