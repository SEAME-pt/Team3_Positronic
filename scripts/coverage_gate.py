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
"""Line coverage gate: total of an lcov tracefile against a threshold (issue #109)."""

# The total is lines hit / lines found, summed over all files. A small file at 0 % weighs
# less than a big one, unlike an average of percentages.
#
#   python3 scripts/coverage_gate.py                       coverage/total.info, 80 %
#   python3 scripts/coverage_gate.py <tracefile> --threshold 80
# Exit code 0 at or above the threshold, 1 below it, 2 when the tracefile is missing or empty.

import argparse
import dataclasses
import os
import sys

EXCLUDED_DIRS = ('test', 'launch', '__pycache__')  # keep in line with .coveragerc


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


def python_files(src_dir):
    """List the .py files under src_dir that the gate should see, as src/... paths."""
    result = []
    for root, dirs, names in os.walk(src_dir):
        dirs[:] = sorted(d for d in dirs if d not in EXCLUDED_DIRS)
        for name in sorted(names):
            if name.endswith('.py'):
                full = os.path.join(root, name)
                rel = os.path.relpath(full, os.path.dirname(os.path.abspath(src_dir)))
                result.append(rel.replace(os.sep, '/'))
    return result


def unmeasured(files, python_paths):
    """Return the .py paths that are not in the tracefile."""
    measured = {f.path for f in files}
    return [p for p in python_paths if p not in measured]


def report(files, threshold, worst_count, missing):
    """Build the Markdown report and the verdict: (text, passed)."""
    found, hit = total(files)
    passed = passes(found, hit, threshold)
    lines = ['## Line coverage', '', '| Package | Lines | Coverage |', '|---|---:|---:|']
    for name, (p_found, p_hit) in sorted(by_package(files).items()):
        lines.append(f'| {name} | {p_hit} / {p_found} | {percent(p_hit, p_found):.1f} % |')
    lines.append(f'| **Total** | **{hit} / {found}** | **{percent(hit, found):.1f} %** |')

    worst = worst_files(files, worst_count)
    if worst:
        lines += ['', f'Worst {len(worst)} files:', '', '| File | Lines | Coverage |',
                  '|---|---:|---:|']
        for f in worst:
            lines.append(f'| {f.path} | {f.hit} / {f.found} | {percent(f.hit, f.found):.1f} % |')

    if missing:
        lines += ['', 'WARNING: Python files neither measured nor excluded (not in the total):']
        lines += [f'- {p}' for p in missing]

    verdict = 'PASS' if passed else 'FAIL'
    sign = '>=' if passed else '<'
    lines += ['', f'{verdict}: {percent(hit, found):.1f} % {sign} {threshold:g} %']
    return '\n'.join(lines) + '\n', passed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tracefile', nargs='?', default='coverage/total.info')
    parser.add_argument('--threshold', type=float, default=80.0, help='minimum total, in %%')
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
    text, passed = report(files, args.threshold, args.worst, missing)
    print(text, end='')

    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        with open(summary, 'a', encoding='utf-8') as handle:
            handle.write(text)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
