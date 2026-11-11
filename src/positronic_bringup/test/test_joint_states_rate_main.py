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

"""
Check main() of joint_states_rate.py end to end, in its own process.

main() calls rclpy.init() and shuts the context down at the end, and rclpy cannot be initialised
twice in one process, so this test lives in its own file: ament runs each test file as its own
pytest process. It runs main() for a short --duration with nobody publishing, which exercises the
arguments, the spin loop, the shutdown, the summary and the two files, and is the one path the
node tests of test_joint_states_rate.py cannot reach.
"""

import importlib.util
import os
import sys

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'scripts', 'joint_states_rate.py')


def load_script():
    """Import the script as a module, without running its main()."""
    spec = importlib.util.spec_from_file_location('joint_states_rate', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_main_runs_for_the_duration_and_writes_the_files(tmp_path, monkeypatch, capsys):
    """Check main() with --duration and --out: returns 0, reports no poses, writes both files."""
    rate = load_script()
    out = tmp_path / 'run'
    monkeypatch.setattr(sys, 'argv', ['joint_states_rate.py', '--duration', '0.3',
                                      '--out', str(out)])

    assert rate.main() == 0

    printed = capsys.readouterr().out
    assert 'no new poses received' in printed
    assert f'written to {out}/intervals.csv and summary.txt' in printed
    assert (out / 'summary.txt').read_text() == 'no new poses received\n'
    assert (out / 'intervals.csv').read_text() == 'seconds,interval_ms\n'
