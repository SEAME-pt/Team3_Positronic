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
#   Write the coverage gate for #109: read the lcov tracefile made by scripts/coverage.sh and
#   judge it with pure functions, so it can be unit-tested without ROS. Take the LF/LH lines of
#   each record; the total is the sum of lines, not the average of the files. Group by package,
#   the folder right after src/ (C++ paths are absolute, Python paths are relative). Print a
#   Markdown table per package, the worst files, the total and one PASS/FAIL line, and append the
#   same report to $GITHUB_STEP_SUMMARY when it is set. Warn about .py files under src/ that are
#   neither measured nor excluded (test/, launch/), since coverage.py does not see files nothing
#   imports. Exit 1 below the threshold (80 % by default).
#   Then: judge each package against the threshold instead of the total, so a well-tested
#   package cannot hide one with no tests. The total stays in the report, for information.
#   Then: count the .py files coverage.py did not see at 0 % in their package, with the number of
#   statements coverage.py's own parser finds, so an untested Python package fails like an
#   untested C++ one. Skip setup.py. A file that cannot be parsed is an error (exit 2).
"""Line coverage gate: every package of an lcov tracefile against a threshold (issue #109)."""

# Each package must reach the threshold on its own. The total is for information: a big
# package at 100 % would hide a small one at 0 %.
# Package and total are lines hit / lines found, summed over the files. A small file at 0 %
# weighs less than a big one, unlike an average of percentages.
#
#   python3 scripts/coverage_gate.py                       coverage/total.info, 80 %
#   python3 scripts/coverage_gate.py <tracefile> --threshold 80
# Exit code 0 when every package reaches the threshold, 1 when at least one is below it,
# 2 when the tracefile is missing or empty.

import argparse
import dataclasses
import os
import sys

EXCLUDED_DIRS = ('test', 'launch', '__pycache__')  # keep in line with .coveragerc
EXCLUDED_FILES = ('setup.py',)  # packaging, not robot code; also in .coveragerc
# coverage.py's default exclusion, so a file counted here matches what coverage.py would count.
NO_COVER = r'#\s*(pragma|PRAGMA)[:\s]?\s*(no|NO)\s*(cover|COVER)'


@dataclasses.dataclass
class FileCoverage:
    """Lines found and lines hit for one source file."""

    path: str  # from src/ on, e.g. src/positronic_hardware/src/serial_arm.cpp
    found: int
    hit: int


def percent(hit, found):
    """Return hit/found in %, or 100 when there is nothing to cover."""
    return 100.0 * hit / found if found else 100.0


def repo_path(path):
    """Cut a tracefile path to start at the first src/ folder (/ws/src/a/b -> src/a/b)."""
    parts = path.replace('\\', '/').split('/')
    if 'src' not in parts:
        return path
    return '/'.join(parts[parts.index('src'):])


def package_of(path):
    """Return the package of a src/ path: the folder right after the first src/."""
    parts = repo_path(path).split('/')
    return parts[1] if len(parts) > 2 and parts[0] == 'src' else '(outside src)'


def parse_tracefile(text):
    """Read the SF/LF/LH lines of an lcov tracefile into a list of FileCoverage."""
    files = []
    path, found, hit = None, 0, 0
    for line in text.splitlines():
        line = line.strip()
        if line.startswith('SF:'):
            path, found, hit = line[3:], 0, 0
        elif line.startswith('LF:'):
            found = int(line[3:])
        elif line.startswith('LH:'):
            hit = int(line[3:])
        elif line == 'end_of_record' and path is not None:
            files.append(FileCoverage(repo_path(path), found, hit))
            path = None
    return files


def by_package(files):
    """Sum lines found and hit per package: {package: (found, hit)}."""
    packages = {}
    for f in files:
        found, hit = packages.get(package_of(f.path), (0, 0))
        packages[package_of(f.path)] = (found + f.found, hit + f.hit)
    return packages


def total(files):
    """Sum lines found and hit over all files: (found, hit)."""
    return sum(f.found for f in files), sum(f.hit for f in files)


def worst_files(files, count):
    """Return up to count files below 100 %, lowest first (more missed lines first on a tie)."""
    below = [f for f in files if f.hit < f.found]
    below.sort(key=lambda f: (percent(f.hit, f.found), -(f.found - f.hit), f.path))
    return below[:count]


def passes(found, hit, threshold):
    """Return True when hit/found reaches the threshold (in %). Integer math, no rounding."""
    return hit * 100 >= threshold * found


def failing_packages(files, threshold):
    """Return the packages below the threshold, sorted by name."""
    return sorted(name for name, (found, hit) in by_package(files).items()
                  if not passes(found, hit, threshold))


def python_files(src_dir):
    """List the .py files under src_dir that the gate should see, as src/... paths."""
    result = []
    for root, dirs, names in os.walk(src_dir):
        dirs[:] = sorted(d for d in dirs if d not in EXCLUDED_DIRS)
        for name in sorted(names):
            if name.endswith('.py') and name not in EXCLUDED_FILES:
                full = os.path.join(root, name)
                rel = os.path.relpath(full, os.path.dirname(os.path.abspath(src_dir)))
                result.append(rel.replace(os.sep, '/'))
    return result


def unmeasured(files, python_paths):
    """Return the .py paths that are not in the tracefile."""
    measured = {f.path for f in files}
    return [p for p in python_paths if p not in measured]


def statement_count(full_path):
    """Count the statements of a .py file the way coverage.py does (docstrings left out).

    Raises ValueError when the file cannot be parsed.
    """
    from coverage.exceptions import NotPython
    from coverage.parser import PythonParser
    try:
        parser = PythonParser(filename=full_path, exclude=NO_COVER)
        parser.parse_source()
    except (NotPython, SyntaxError, OSError) as error:
        raise ValueError(f'cannot count the statements of {full_path}: {error}') from error
    return len(parser.statements)


def at_zero(paths, repo_dir):
    """Return a FileCoverage at 0 % for each path that has statements (an empty file has none)."""
    result = []
    for path in paths:
        found = statement_count(os.path.join(repo_dir, path))
        if found:
            result.append(FileCoverage(path, found, 0))
    return result


def report(files, threshold, worst_count, missing):
    """Build the Markdown report and the verdict: (text, passed).

    missing: the .py paths coverage.py did not see, already in files at 0 %.
    """
    found, hit = total(files)
    failing = failing_packages(files, threshold)
    lines = ['## Line coverage', '', '| Package | Lines | Coverage | Gate |',
             '|---|---:|---:|:---:|']
    for name, (p_found, p_hit) in sorted(by_package(files).items()):
        gate = 'FAIL' if name in failing else 'PASS'
        lines.append(
            f'| {name} | {p_hit} / {p_found} | {percent(p_hit, p_found):.1f} % | {gate} |')
    lines.append(
        f'| **Total** (information) | **{hit} / {found}** | **{percent(hit, found):.1f} %** | |')

    worst = worst_files(files, worst_count)
    if worst:
        lines += ['', f'Worst {len(worst)} files:', '', '| File | Lines | Coverage |',
                  '|---|---:|---:|']
        for f in worst:
            lines.append(f'| {f.path} | {f.hit} / {f.found} | {percent(f.hit, f.found):.1f} % |')

    if missing:
        lines += ['', 'Counted at 0 % (no test imports them):']
        lines += [f'- {p}' for p in missing]

    if failing:
        lines += ['', f'FAIL: {len(failing)} of {len(by_package(files))} packages below '
                  f'{threshold:g} %: {", ".join(failing)}']
    else:
        lines += ['', f'PASS: every package at or above {threshold:g} %']
    return '\n'.join(lines) + '\n', not failing


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tracefile', nargs='?', default='coverage/total.info')
    parser.add_argument('--threshold', type=float, default=80.0, help='minimum per package, in %%')
    parser.add_argument('--worst', type=int, default=10, help='how many files to list')
    parser.add_argument('--src', default='src', help='source folder, to find unmeasured .py')
    args = parser.parse_args(argv)

    try:
        with open(args.tracefile, encoding='utf-8') as handle:
            files = parse_tracefile(handle.read())
    except OSError as error:
        print(f'ERROR: cannot read {args.tracefile}: {error}', file=sys.stderr)
        return 2
    if total(files)[0] == 0:
        print(f'ERROR: no lines in {args.tracefile}', file=sys.stderr)
        return 2

    missing = unmeasured(files, python_files(args.src)) if os.path.isdir(args.src) else []
    repo_dir = os.path.dirname(os.path.abspath(args.src))
    try:
        counted = at_zero(missing, repo_dir)
    except ValueError as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 2
    files += counted
    text, passed = report(files, args.threshold, args.worst, [f.path for f in counted])
    print(text, end='')

    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        with open(summary, 'a', encoding='utf-8') as handle:
            handle.write(text)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
