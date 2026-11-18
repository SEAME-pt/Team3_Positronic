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
#   Write a pytest suite for summarize_tests.py that needs no ROS 2 and no real build: write small
#   JUnit and CTest XML files in memory or in tmp_path, shaped like the files gtest, pytest and
#   the ament linters write. Cover: passed, failed, errored and skipped cases (gtest's notrun
#   too); readable names for gtest, pytest and linter cases; a failure with only a message
#   attribute; invalid XML; a CTest failure with no failing result (crash, timeout) reported once
#   and a reported one not repeated; long messages cut at the limit and not below it; the
#   verdict when all pass, when a test fails and when only colcon test failed; colour only on a
#   terminal, NO_COLOR and FORCE_COLOR; main() exits 0, 1 and 2 and writes $GITHUB_STEP_SUMMARY.

"""Unit tests for the make test summary of issue #145 (no ROS 2 needed)."""

import summarize_tests as ts

GTEST = """<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="4" failures="1" disabled="1" errors="0" name="AllTests">
  <testsuite name="SerialArm" tests="4">
    <testcase name="ReadsFrame" status="run" result="completed"
              classname="positronic_hardware.SerialArm" />
    <testcase name="RefusesShortFrame" status="run" result="completed"
              classname="positronic_hardware.SerialArm">
      <failure message="short">test_serial_arm.cpp:42
Expected equality of these values:
  frame.size()
    Which is: 3
  8</failure>
    </testcase>
    <testcase name="DISABLED_Slow" status="notrun" result="suppressed"
              classname="positronic_hardware.SerialArm" />
    <testcase name="Timeout" status="run" result="completed"
              classname="positronic_hardware.SerialArm" />
  </testsuite>
</testsuites>
"""

PYTEST = """<?xml version="1.0" encoding="utf-8"?><testsuites><testsuite name="pytest">
<testcase classname="positronic_bringup.src.positronic_bringup.test.test_launch" name="test_a"/>
<testcase classname="positronic_bringup.src.positronic_bringup.test.test_launch" name="test_b">
<skipped message="no display"/></testcase>
<testcase classname="positronic_bringup.src.positronic_bringup.test.test_launch" name="test_c">
<error message="ImportError: no module named rclpy"/></testcase>
</testsuite></testsuites>
"""

FLAKE8 = """<?xml version="1.0" encoding="UTF-8"?><testsuite name="positronic_bringup.flake8">
<testcase name="E501 (./test/test_launch.py:7:100)" classname="positronic_bringup.flake8">
<failure message="line too long (103 &gt; 99 characters)"/></testcase>
</testsuite>
"""

CTEST = """<?xml version="1.0" encoding="UTF-8"?><Site><Testing>
<Test Status="passed"><Name>flake8</Name></Test>
<Test Status="failed"><Name>flake8</Name></Test>
<Test Status="failed"><Name>test_crash</Name>
<Results><Measurement><Value>[ RUN      ] Arm.Moves
Segmentation fault</Value></Measurement></Results></Test>
</Testing></Site>
"""

PASSING = """<testsuite><testcase classname="pkg.Suite" name="Ok"/></testsuite>"""


def test_gtest_counts_passed_failed_and_disabled_as_skipped():
    passed, skipped, failures = ts.parse_results(GTEST, 'positronic_hardware', 'test_serial_arm')
    assert (passed, skipped) == (2, 1)
    assert [f.name for f in failures] == ['SerialArm.RefusesShortFrame']
    assert failures[0].suite == 'test_serial_arm'
    assert 'Which is: 3' in failures[0].message


def test_pytest_skipped_and_error_counted():
    passed, skipped, failures = ts.parse_results(PYTEST, 'positronic_bringup', 'test_launch')
    assert (passed, skipped) == (1, 1)
    assert [(f.name, f.message) for f in failures] == [
        ('test_c', 'ImportError: no module named rclpy')]


def test_failure_with_only_a_message_attribute():
    _, _, failures = ts.parse_results(FLAKE8, 'positronic_bringup', 'flake8')
    assert failures[0].name == 'E501 (./test/test_launch.py:7:100)'
    assert failures[0].message == 'line too long (103 > 99 characters)'


def test_case_names():
    assert ts.case_name('pkg', 'test_x', 'pkg.Suite', 'Case') == 'Suite.Case'
    assert ts.case_name('pkg', 'test_x', 'pkg.src.pkg.test.test_x', 'test_y') == 'test_y'
    assert ts.case_name('pkg', 'flake8', 'pkg.flake8', 'E501 (a.py:1:1)') == 'E501 (a.py:1:1)'
    assert ts.case_name('pkg', 'test_x', '', 'Case') == 'Case'
    assert ts.case_name('pkg', 'test_x', 'Param/Suite', 'Case/0') == 'Param/Suite.Case/0'


def test_suite_of():
    assert ts.suite_of('a/b/test_serial_arm.gtest.xml') == 'test_serial_arm'
    assert ts.suite_of('a/b/flake8.xunit.xml') == 'flake8'
    assert ts.suite_of('a/b/other.xml') == 'other'


def test_invalid_xml_raises():
    try:
        ts.parse_results('<testsuite><testcase', 'pkg', 'broken')
    except ValueError as error:
        assert 'pkg/broken' in str(error)
    else:
        raise AssertionError('no ValueError')


def test_empty_result_file_has_no_cases():
    assert ts.parse_results('<testsuites/>', 'pkg', 'empty') == (0, 0, [])


def test_ctest_failures_and_bad_record():
    failed = ts.ctest_failures(CTEST)
    assert sorted(failed) == ['flake8', 'test_crash']
    assert failed['test_crash'].endswith('Segmentation fault')
    assert ts.ctest_failures('not xml') == {}


def test_crash_without_result_is_reported_once():
    result = ts.PackageResult('pkg', failures=[ts.Failure('pkg', 'flake8', 'E501', 'long')])
    extra = ts.unreported('pkg', ts.ctest_failures(CTEST), result)
    assert [(f.suite, f.name) for f in extra] == [('test_crash', '(no result)')]
    assert 'crashed or timed out' in extra[0].message
    assert 'Segmentation fault' in extra[0].message


MISSING = """<testsuite><testcase classname="positronic_safety.test_watchdog.gtest"
name="positronic_safety.test_watchdog.gtest.missing_result">
<failure message="The test did not generate a result file:">The test did not generate a result file:
Running main() from gtest_main.cc
[==========] Running 10 tests from 2 test suites.
[ RUN      ] Watchdog.One
[       OK ] Watchdog.One (0 ms)
[ RUN      ] Watchdog.Two
[       OK ] Watchdog.Two (0 ms)
[ RUN      ] Watchdog.Three
[       OK ] Watchdog.Three (0 ms)
[ RUN      ] TmpCheck.Crashes
-- run_test.py: return code -6</failure></testcase></testsuite>
"""


def test_crash_shows_the_end_of_the_output():
    _, _, failures = ts.parse_results(MISSING, 'positronic_safety', 'test_watchdog')
    assert [f.name for f in failures] == ['(no result)']
    lines = failures[0].message.splitlines()
    assert lines[0].startswith('No result file: the test crashed or timed out')
    assert lines[-2:] == ['[ RUN      ] TmpCheck.Crashes', '-- run_test.py: return code -6']
    assert len(lines) == ts.MAX_MESSAGE_LINES
    # Its suite has a failure now, so CTest's record of the same crash is not added again.
    result = ts.PackageResult('positronic_safety', failures=failures)
    assert ts.unreported('positronic_safety', {'test_watchdog': 'x'}, result) == []


def test_container_paths_are_written_from_src():
    xml = ('<testsuite><testcase classname="pkg.cpplint" name="(/ws/src/pkg/a.cpp:3)">'
           '<failure message="/ws/src/pkg/a.cpp:3: bad"/></testcase></testsuite>')
    _, _, failures = ts.parse_results(xml, 'pkg', 'cpplint')
    assert failures[0].name == '(src/pkg/a.cpp:3)'
    assert failures[0].message == 'src/pkg/a.cpp:3: bad'


def test_short_cuts_long_messages_only():
    exactly = '\n'.join(f'line {i}' for i in range(ts.MAX_MESSAGE_LINES))
    assert ts.short(exactly) == (exactly.splitlines(), 0)
    shown, left = ts.short(exactly + '\nline 6\nline 7')
    assert (len(shown), left) == (ts.MAX_MESSAGE_LINES, 2)
    assert ts.short('') == ([], 0)
    assert ts.short('a\n\n\nb') == (['a', 'b'], 0)


def results(failed=0, skipped=0):
    good = ts.PackageResult('pkg_a', passed=5)
    failures = [ts.Failure('pkg_b', 's', f'c{i}', 'm') for i in range(failed)]
    bad = ts.PackageResult('pkg_b', passed=3, skipped=skipped, failures=failures)
    return [good, bad]


def test_verdict():
    assert ts.verdict(results(), 0) == (True, 'all 8 tests passed in 2 packages')
    assert ts.verdict(results(skipped=1), 0) == (True,
                                                 'all 9 tests passed in 2 packages (1 skipped)')
    passed, why = ts.verdict(results(failed=2), 0)
    assert not passed and why == '2 of 10 tests failed, in pkg_b'
    passed, why = ts.verdict(results(), 2)
    assert not passed and 'colcon test itself exited with 2' in why


def test_terminal_report_ends_with_the_verdict():
    text, passed = ts.terminal_report(results(failed=1), 0, ts.Style(False))
    assert not passed
    assert '✘  pkg_b' in text and '✔  pkg_a' in text
    assert 'pkg_b › s › c0' in text
    assert text.rstrip().splitlines()[-1] == '✘ FAILED: 1 of 9 tests failed, in pkg_b'
    text, passed = ts.terminal_report(results(), 0, ts.Style(False))
    assert passed and text.rstrip().splitlines()[-1].startswith('✔ PASSED')
    assert 'Failed' not in text


def test_markdown_report():
    text = ts.markdown_report(results(failed=1), 0)
    assert '| ❌ | pkg_b | 3 | 1 | 0 |' in text
    assert '<code>pkg_b › s › c0</code>' in text
    assert text.rstrip().endswith('**FAILED**: 1 of 9 tests failed, in pkg_b')


class Tty:
    def __init__(self, tty):
        self.tty = tty

    def isatty(self):
        return self.tty


def test_colour_only_on_a_terminal(monkeypatch):
    monkeypatch.delenv('NO_COLOR', raising=False)
    monkeypatch.delenv('FORCE_COLOR', raising=False)
    assert ts.use_colour(Tty(True)) and not ts.use_colour(Tty(False))
    monkeypatch.setenv('FORCE_COLOR', '1')
    assert ts.use_colour(Tty(False))
    monkeypatch.setenv('NO_COLOR', '1')
    assert not ts.use_colour(Tty(True))
    assert ts.Style(True).red('x') == '\033[31mx\033[0m'
    assert ts.Style(False).red('x') == 'x'


def write_package(build, name, results, ctest=None):
    folder = build / name / 'test_results' / name
    folder.mkdir(parents=True)
    for file_name, text in results.items():
        (folder / file_name).write_text(text)
    if ctest is not None:
        (build / name / 'Testing' / 'stamp').mkdir(parents=True)
        (build / name / 'Testing' / 'TAG').write_text('stamp\nExperimental\n')
        (build / name / 'Testing' / 'stamp' / 'Test.xml').write_text(ctest)


def test_read_build_skips_packages_without_results(tmp_path):
    write_package(tmp_path, 'positronic_hardware', {'test_serial_arm.gtest.xml': GTEST})
    write_package(tmp_path, 'positronic_bringup', {'flake8.xunit.xml': FLAKE8}, ctest=CTEST)
    (tmp_path / 'positronic_interfaces').mkdir()
    found = ts.read_build(str(tmp_path))
    assert [r.name for r in found] == ['positronic_bringup', 'positronic_hardware']
    assert [f.suite for f in found[0].failures] == ['flake8', 'test_crash']
    assert ts.read_build(str(tmp_path / 'missing')) == []


def test_main_exit_codes_and_step_summary(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv('NO_COLOR', '1')
    summary = tmp_path / 'out' / 'summary.md'
    monkeypatch.setenv('GITHUB_STEP_SUMMARY', str(summary))

    good = tmp_path / 'good'
    write_package(good, 'pkg', {'test_ok.gtest.xml': PASSING})
    assert ts.main(['--build', str(good)]) == 0
    assert capsys.readouterr().out.rstrip().endswith('all 1 tests passed in 1 packages')
    assert '| ✅ | pkg | 1 | 0 | 0 |' in summary.read_text()

    assert ts.main(['--build', str(good), '--colcon-rc', '1']) == 1

    bad = tmp_path / 'bad'
    write_package(bad, 'positronic_hardware', {'test_serial_arm.gtest.xml': GTEST})
    assert ts.main(['--build', str(bad)]) == 1

    assert ts.main(['--build', str(tmp_path / 'empty')]) == 2
    broken = tmp_path / 'broken'
    write_package(broken, 'pkg', {'x.xml': '<testsuite'})
    assert ts.main(['--build', str(broken)]) == 2
