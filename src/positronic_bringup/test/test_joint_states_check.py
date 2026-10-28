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

"""Check the maths of joint_states_check.py: parsing the reference and comparing in degrees."""

import importlib.util
import math
import os

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'scripts', 'joint_states_check.py')


def load_script():
    """Import the script as a module, without running its main()."""
    spec = importlib.util.spec_from_file_location('joint_states_check', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_reference_accepts_the_pymycobot_print():
    """The list as pymycobot prints it, brackets included, gives six floats."""
    module = load_script()
    assert module.parse_reference('[16.87, -16.17, 25.48, -10.45, 1.23, 0.26]') == \
        [16.87, -16.17, 25.48, -10.45, 1.23, 0.26]


def test_parse_reference_refuses_five_angles():
    """Five values is a copy error, not a reference."""
    module = load_script()
    with pytest.raises(ValueError):
        module.parse_reference('1,2,3,4,5')


def test_compare_converts_radians_to_degrees_in_joint_order():
    """Positions come by name, not by position in the message."""
    module = load_script()
    names = list(reversed(module.JOINTS))  # the message may list them in any order
    positions = [math.radians(d) for d in reversed([10.0, -20.0, 30.0, -40.0, 50.0, -60.0])]
    rows = module.compare(names, positions, [10.0, -20.0, 30.0, -40.5, 50.0, -60.0])
    assert [row[0] for row in rows] == module.JOINTS
    assert rows[0][1] == pytest.approx(10.0)
    assert rows[3][3] == pytest.approx(0.5)


def test_report_ends_with_the_verdict():
    """PASS when the worst difference is within the tolerance, FAIL above it."""
    module = load_script()
    rows = [('joint2_to_joint1', 1.0, 1.2, 0.2), ('joint3_to_joint2', 0.0, 0.0, 0.0)]
    assert module.report(rows, 0.5)[-1].startswith('PASS')
    assert module.report(rows, 0.1)[-1].startswith('FAIL')
