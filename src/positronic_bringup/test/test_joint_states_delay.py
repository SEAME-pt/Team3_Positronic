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

"""Check the maths of joint_states_delay.py: the delay of one message and the line per second."""

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
