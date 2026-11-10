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
#   Write a pytest suite for coverage_gate.py that needs no ROS 2 and no real build: write small
#   lcov tracefiles in memory. Cover: SF/LF/LH parsed per record, absolute (C++) and relative
#   (Python) paths both grouped by the package after src/, the total is a sum and not an average
#   of percentages, the worst files are lowest first and skip files at 100 %, the threshold
#   passes exactly at 80 % and fails just below it, .py files under test/ and launch/ are not
#   reported as unmeasured but others are, and main() exits 0, 1 or 2 and writes the report to
#   $GITHUB_STEP_SUMMARY.

"""Unit tests for the coverage gate of issue #109 (no ROS 2 needed)."""

import coverage_gate as gate


def record(path, found, hit):
    return f'TN:\nSF:{path}\nDA:1,1\nLF:{found}\nLH:{hit}\nend_of_record\n'


TRACEFILE = (
    record('/ws/src/positronic_hardware/src/serial_arm.cpp', 100, 90)
    + record('/ws/src/positronic_hardware/include/positronic_hardware/fake_arm.hpp', 20, 10)
    + record('/ws/src/positronic_mission/src/mission_node.cpp', 40, 0)
    + record('src/positronic_bringup/scripts/joint_states_delay.py', 40, 20)
)


def test_parse_reads_each_record():
    files = gate.parse_tracefile(TRACEFILE)
    assert [(f.path, f.found, f.hit) for f in files] == [
        ('src/positronic_hardware/src/serial_arm.cpp', 100, 90),
        ('src/positronic_hardware/include/positronic_hardware/fake_arm.hpp', 20, 10),
        ('src/positronic_mission/src/mission_node.cpp', 40, 0),
        ('src/positronic_bringup/scripts/joint_states_delay.py', 40, 20),
    ]


def test_package_is_the_folder_after_the_first_src():
    assert gate.package_of('/ws/src/positronic_hardware/src/serial_arm.cpp') == \
        'positronic_hardware'
    assert gate.package_of('src/positronic_bringup/scripts/x.py') == 'positronic_bringup'
    assert gate.package_of('/usr/include/c++/13/vector') == '(outside src)'


def test_by_package_sums_the_files_of_a_package():
    packages = gate.by_package(gate.parse_tracefile(TRACEFILE))
    assert packages == {
        'positronic_hardware': (120, 100),
        'positronic_mission': (40, 0),
        'positronic_bringup': (40, 20),
    }


def test_total_is_a_sum_not_an_average():
    files = gate.parse_tracefile(
        record('src/a/big.cpp', 90, 90) + record('src/a/small.cpp', 10, 0))
    found, hit = gate.total(files)
    assert (found, hit) == (100, 90)
    assert gate.percent(hit, found) == 90.0  # the average of 100 % and 0 % would be 50 %


def test_worst_files_lowest_first_and_full_files_left_out():
    files = gate.parse_tracefile(TRACEFILE + record('src/positronic_protocol/src/units.cpp', 5, 5))
    worst = gate.worst_files(files, 10)
    assert [f.path for f in worst] == [
        'src/positronic_mission/src/mission_node.cpp',
        'src/positronic_bringup/scripts/joint_states_delay.py',  # 50 %, 20 missed
        'src/positronic_hardware/include/positronic_hardware/fake_arm.hpp',  # 50 %, 10 missed
        'src/positronic_hardware/src/serial_arm.cpp',
    ]
    assert len(gate.worst_files(files, 2)) == 2


def test_threshold_passes_exactly_at_80_and_fails_below():
    assert gate.passes(100, 80, 80)
    assert not gate.passes(100, 79, 80)
    assert not gate.passes(743, 582, 80)  # the first measurement, 78.3 %


def test_python_files_skip_test_and_launch(tmp_path):
    for rel in ('src/pkg/scripts/node.py', 'src/pkg/test/test_node.py',
                'src/pkg/launch/sim.launch.py', 'src/pkg/pkg/helper.py'):
        (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / rel).write_text('')
    found = gate.python_files(str(tmp_path / 'src'))
    assert found == ['src/pkg/pkg/helper.py', 'src/pkg/scripts/node.py']

    files = gate.parse_tracefile(record('src/pkg/scripts/node.py', 10, 5))
    assert gate.unmeasured(files, found) == ['src/pkg/pkg/helper.py']


def test_report_lists_packages_total_and_verdict():
    text, passed = gate.report(gate.parse_tracefile(TRACEFILE), 80, 10, ['src/pkg/x.py'])
    assert not passed
    assert '| positronic_hardware | 100 / 120 | 83.3 % |' in text
    assert '| **Total** | **120 / 200** | **60.0 %** |' in text
    assert 'WARNING' in text and '- src/pkg/x.py' in text
    assert text.rstrip().endswith('FAIL: 60.0 % < 80 %')


def test_main_exit_codes_and_step_summary(tmp_path, monkeypatch):
    summary = tmp_path / 'summary.md'
    monkeypatch.setenv('GITHUB_STEP_SUMMARY', str(summary))
    good = tmp_path / 'good.info'
    good.write_text(record('src/a/a.cpp', 10, 9))
    bad = tmp_path / 'bad.info'
    bad.write_text(TRACEFILE)
    empty = tmp_path / 'empty.info'
    empty.write_text('')
    no_src = ['--src', str(tmp_path / 'nothing')]

    assert gate.main([str(good)] + no_src) == 0
    assert 'PASS: 90.0 % >= 80 %' in summary.read_text()
    assert gate.main([str(bad)] + no_src) == 1
    assert gate.main([str(empty)] + no_src) == 2
    assert gate.main([str(tmp_path / 'missing.info')] + no_src) == 2
