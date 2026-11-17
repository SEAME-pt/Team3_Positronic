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
#   Write the end summary of make test for #145. colcon test-result --verbose prints each failure
#   twice (once from CTest's wrapper with the whole tool output, once from the tool's own result
#   file) and has no per package view, so a failing run is hard to read. Read the result files
#   colcon leaves in build/<package>/test_results/ (gtest, pytest and the ament linters all write
#   JUnit XML) with pure functions, so it is unit-tested without ROS. Print one line per package
#   (passed, failed, skipped), then each failure with a short message, then one verdict line, so
#   the last thing on screen says if everything passed. Colour only on a terminal. A CTest test
#   that failed without a failing result file (crash, timeout) must still show up as a failure.
#   Take colcon test's own exit code, so a colcon error fails make test even if every result
#   passed. Append the same report in Markdown to $GITHUB_STEP_SUMMARY when it is set.
"""Summary of colcon test results: per package, then each failure (issue #145)."""

# Reads, for each package in build/:
#   build/<pkg>/test_results/<pkg>/*.xml   one file per test executable or linter (JUnit XML)
#   build/<pkg>/Testing/<TAG>/Test.xml     CTest's own record, only to catch a test that failed
#                                          without writing a result file (crash, timeout)
# CTest's record is not counted otherwise: it repeats every result file, which is why
# colcon test-result shows each failure twice.
#
#   python3 scripts/summarize_tests.py                  build/
#   python3 scripts/summarize_tests.py --colcon-rc 1    colcon test's exit code, fails the run too
# Exit code 0 when everything passed, 1 when a test failed or colcon test failed, 2 when there
# are no results at all (nothing built, or colcon test never ran).

import argparse
import dataclasses
import glob
import os
import sys
import xml.etree.ElementTree as ET

MAX_MESSAGE_LINES = 6
RESULT_SUFFIXES = ('.gtest.xml', '.xunit.xml', '.xml')
MISSING_RESULT = 'missing_result'  # the case ament writes when a test left no result file
NO_RESULT = '(no result)'
CRASHED = 'No result file: the test crashed or timed out. Last lines of its output:'


@dataclasses.dataclass
class Failure:
    """One failed test case: where it is and what it said."""

    package: str
    suite: str  # the result file without its suffix: the test executable or the linter
    name: str
    message: str


@dataclasses.dataclass
class PackageResult:
    """Counts and failures of one package."""

    name: str
    passed: int = 0
    skipped: int = 0
    failures: list = dataclasses.field(default_factory=list)

    @property
    def failed(self):
        return len(self.failures)

    @property
    def total(self):
        return self.passed + self.skipped + self.failed


def suite_of(path):
    """Return the result file name without its suffix (test_x.gtest.xml -> test_x)."""
    name = os.path.basename(path)
    for suffix in RESULT_SUFFIXES:
        if name.endswith(suffix):
            return name[:-len(suffix)]
    return name


def case_name(package, suite, classname, name):
    """Return a readable test name.

    gtest classnames are <package>.<Suite>, so the name becomes Suite.Test. pytest and linter
    classnames repeat the package and the file, which the suite already says: only the name stays.
    """
    if classname.startswith(package + '.'):
        classname = classname[len(package) + 1:]
    if classname and '.' not in classname and classname != suite:
        return f'{classname}.{name}'
    return name


def message_of(element):
    """Return the text of a failure or error element, its message attribute when it has none."""
    text = (element.text or '').strip()
    return text or (element.get('message') or '').strip()


def clean(text):
    """Write the container's paths from src/ on, like the rest of the repo (/ws/src/a -> src/a)."""
    return text.replace('/ws/src/', 'src/')


def parse_results(xml_text, package, suite):
    """Read one JUnit XML file: (passed, skipped, [Failure]).

    Raises ValueError when the text is not XML.
    """
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError as error:
        raise ValueError(f'{package}/{suite}: not valid XML ({error})') from error
    passed, skipped, failures = 0, 0, []
    for case in root.iter('testcase'):
        name = case_name(package, suite, case.get('classname', ''), case.get('name', '?'))
        bad = case.find('failure')
        if bad is None:
            bad = case.find('error')
        if bad is not None:
            message = message_of(bad)
            if case.get('name', '').endswith(MISSING_RESULT):
                # ament's run_test.py writes this case when a test left no result file. The test
                # running when it died is at the end of the output, not the start.
                name = NO_RESULT
                message = CRASHED + '\n' + tail(message, MAX_MESSAGE_LINES - 1)
            failures.append(Failure(package, suite, clean(name), clean(message)))
        elif case.find('skipped') is not None or case.get('status') == 'notrun':
            skipped += 1
        else:
            passed += 1
    return passed, skipped, failures


def ctest_failures(xml_text):
    """Return {test name: output} for the failed tests of a CTest Test.xml."""
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError:
        return {}
    failed = {}
    for test in root.iter('Test'):
        if test.get('Status') == 'failed':
            output = test.findtext('./Results/Measurement/Value') or ''
            failed[test.findtext('Name', '?')] = output.strip()
    return failed


def unreported(package, ctest_failed, result):
    """Return a Failure for each CTest failure with no failing test case in its result file.

    A test that crashes or times out writes no result file (or one that says nothing failed), so
    it would otherwise be missing from the summary.
    """
    reported = {f.suite for f in result.failures}
    return [Failure(package, name, NO_RESULT,
                    clean(CRASHED + '\n' + tail(output, MAX_MESSAGE_LINES - 1)))
            for name, output in sorted(ctest_failed.items()) if name not in reported]


def tail(text, count):
    """Return the last count lines of text."""
    return '\n'.join(text.splitlines()[-count:])


def short(message, count=MAX_MESSAGE_LINES):
    """Return the first count non-empty lines of message, and how many were left out."""
    lines = [line.rstrip() for line in message.splitlines() if line.strip()]
    return lines[:count], max(0, len(lines) - count)


def read_package(build_dir, package):
    """Read every result of one package into a PackageResult (None when it has no results)."""
    files = sorted(glob.glob(os.path.join(build_dir, package, 'test_results', '**', '*.xml'),
                             recursive=True))
    if not files:
        return None
    result = PackageResult(package)
    for path in files:
        with open(path, encoding='utf-8') as handle:
            passed, skipped, failures = parse_results(handle.read(), package, suite_of(path))
        result.passed += passed
        result.skipped += skipped
        result.failures += failures
    result.failures += unreported(package, read_ctest(build_dir, package), result)
    return result


def read_ctest(build_dir, package):
    """Return the CTest failures of the last run of a package ({} when there is no record)."""
    testing = os.path.join(build_dir, package, 'Testing')
    try:
        with open(os.path.join(testing, 'TAG'), encoding='utf-8') as handle:
            stamp = handle.readline().strip()
        with open(os.path.join(testing, stamp, 'Test.xml'), encoding='utf-8') as handle:
            return ctest_failures(handle.read())
    except OSError:
        return {}


def read_build(build_dir):
    """Return a PackageResult for each package of build_dir that has results, by name."""
    if not os.path.isdir(build_dir):
        return []
    results = [read_package(build_dir, name) for name in sorted(os.listdir(build_dir))]
    return [r for r in results if r is not None]


class Style:
    """ANSI colours, or nothing when the output is not a terminal."""

    def __init__(self, enabled):
        self.enabled = enabled

    def _wrap(self, code, text):
        return f'\033[{code}m{text}\033[0m' if self.enabled else text

    def green(self, text):
        return self._wrap('32', text)

    def red(self, text):
        return self._wrap('31', text)

    def yellow(self, text):
        return self._wrap('33', text)

    def bold(self, text):
        return self._wrap('1', text)

    def dim(self, text):
        return self._wrap('2', text)


def use_colour(stream):
    """Colour on a terminal, unless NO_COLOR is set; FORCE_COLOR turns it on anywhere."""
    if os.environ.get('NO_COLOR'):
        return False
    return bool(os.environ.get('FORCE_COLOR')) or stream.isatty()


def verdict(results, colcon_rc):
    """Return (passed, one line saying why)."""
    failed = sum(r.failed for r in results)
    total = sum(r.total for r in results)
    skipped = sum(r.skipped for r in results)
    if failed:
        where = ', '.join(r.name for r in results if r.failed)
        return False, f'{failed} of {total} tests failed, in {where}'
    if colcon_rc:
        return False, (f'every test passed, but colcon test itself exited with {colcon_rc}: '
                       'run make test V=1 to see why')
    extra = f' ({skipped} skipped)' if skipped else ''
    return True, f'all {total} tests passed in {len(results)} packages{extra}'


def terminal_report(results, colcon_rc, style):
    """Build the report for a terminal: (text, passed)."""
    width = max(len(r.name) for r in results)
    lines = ['', style.bold('Test summary'), '']
    for r in results:
        mark = style.red('✘') if r.failed else style.green('✔')
        counts = [f'{r.passed:4d} passed']
        if r.failed:
            counts.append(style.red(f'{r.failed:3d} failed'))
        if r.skipped:
            counts.append(style.yellow(f'{r.skipped:3d} skipped'))
        lines.append(f'  {mark}  {r.name:<{width}}  ' + '   '.join(counts))

    failures = [f for r in results for f in r.failures]
    if failures:
        lines += ['', style.bold(style.red(f'Failed ({len(failures)})')), '']
        for f in failures:
            lines.append(f'  {style.red("✘")} {f.package} {style.dim("›")} {f.suite} '
                         f'{style.dim("›")} {style.bold(f.name)}')
            shown, left = short(f.message)
            lines += [style.dim(f'      {line}') for line in shown]
            if left:
                lines.append(style.dim(f'      … {left} more lines (make test V=1)'))
            lines.append('')

    else:
        lines.append('')

    passed, why = verdict(results, colcon_rc)
    if passed:
        lines.append(style.bold(style.green(f'✔ PASSED: {why}')))
    else:
        lines.append(style.bold(style.red(f'✘ FAILED: {why}')))
    return '\n'.join(lines) + '\n', passed


def markdown_report(results, colcon_rc):
    """Build the same report in Markdown, for the GitHub job summary."""
    lines = ['## Tests', '', '| | Package | Passed | Failed | Skipped |', '|:-:|---|--:|--:|--:|']
    for r in results:
        mark = '❌' if r.failed else '✅'
        lines.append(f'| {mark} | {r.name} | {r.passed} | {r.failed} | {r.skipped} |')
    failures = [f for r in results for f in r.failures]
    if failures:
        lines += ['', f'### Failed ({len(failures)})', '']
        for f in failures:
            shown, left = short(f.message, 20)
            more = f'\n… {left} more lines' if left else ''
            body = '\n'.join(shown) + more
            lines += [f'<details><summary><code>{f.package} › {f.suite} › {f.name}</code>'
                      '</summary>', '', '```', body, '```', '</details>', '']
    passed, why = verdict(results, colcon_rc)
    lines += ['', f'**{"PASSED" if passed else "FAILED"}**: {why}']
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', default='build', help='colcon build base (default: build)')
    parser.add_argument('--colcon-rc', type=int, default=0, help='exit code of colcon test')
    args = parser.parse_args(argv)

    try:
        results = read_build(args.build)
    except (OSError, ValueError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 2
    if not results:
        print(f'ERROR: no test results in {args.build}/: run make build, then make test',
              file=sys.stderr)
        return 2

    text, passed = terminal_report(results, args.colcon_rc, Style(use_colour(sys.stdout)))
    print(text, end='')

    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        os.makedirs(os.path.dirname(os.path.abspath(summary)), exist_ok=True)
        with open(summary, 'a', encoding='utf-8') as handle:
            handle.write(markdown_report(results, args.colcon_rc))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
