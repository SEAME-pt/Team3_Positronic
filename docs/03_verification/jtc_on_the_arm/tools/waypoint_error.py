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
#   Write a script that reads the terminal output of `ros2 action send_goal --feedback` on the
#   joint_trajectory_controller (#33), saved with tee, and reports the tracking error per joint
#   in degrees at every waypoint of the goal, taking the feedback closest in time, plus the error
#   of the last feedback and the largest error of the run. Pure functions, unit-tested on a
#   hand-written recording, Markdown table out, several files in one table.
"""Tracking error per waypoint from a recorded `ros2 action send_goal --feedback` (#33)."""

# The file is what the command printed: the goal as sent, then one `Feedback:` block per
# action monitor cycle (20 Hz by default) with the desired, actual and error positions, then
# the result. For every waypoint of the goal the feedback nearest in time gives the error per
# joint. `last` is the error of the final feedback and `max abs` the largest error seen at any
# cycle of the run, per joint.
#
#     python3 waypoint_error.py jtc_T1_r1.txt jtc_T1_r2.txt

import math
import re
import sys
from dataclasses import dataclass

JOINTS = 6

# Six "- value" lines, indented by `indent` spaces, after a "positions:" line.
_POSITIONS = r'positions:\n((?:{indent}- -?[0-9.e+-]+\n){{6}})'
_GOAL_POINT = re.compile(
    r'- ' + _POSITIONS.format(indent=' ' * 4)
    + r'.*?time_from_start:\n\s+sec: (\d+)\n\s+nanosec: (\d+)', re.S)
_SECTION = {
    name: re.compile(name + r':\n  ' + _POSITIONS.format(indent='  '))
    for name in ('desired', 'actual', 'error')
}
_DESIRED_TIME = re.compile(
    r'desired:.*?time_from_start:\n\s+sec: (\d+)\n\s+nanosec: (\d+)', re.S)


@dataclass(frozen=True)
class Waypoint:
    time_s: float
    positions: list


@dataclass(frozen=True)
class Feedback:
    time_s: float
    desired: list
    actual: list
    error: list


@dataclass(frozen=True)
class Run:
    waypoints: list
    feedbacks: list
    status: str


def _floats(block):
    return [float(line.split('-', 1)[1]) for line in block.strip().splitlines()]


def parse(text):
    """The goal, every feedback block and the final status of one recorded run."""
    head, *blocks = text.split('Feedback:\n')
    waypoints = [
        Waypoint(int(sec) + int(nsec) * 1e-9, _floats(positions))
        for positions, sec, nsec in _GOAL_POINT.findall(head)
    ]
    feedbacks = []
    for block in blocks:
        sections = {name: rx.search(block) for name, rx in _SECTION.items()}
        time = _DESIRED_TIME.search(block)
        if not all(sections.values()) or time is None:
            continue  # the result, or a block cut by the end of the file
        feedbacks.append(Feedback(
            int(time.group(1)) + int(time.group(2)) * 1e-9,
            *(_floats(sections[name].group(1)) for name in ('desired', 'actual', 'error'))))
    status = re.search(r'Goal finished with status: (\w+)', text)
    return Run(waypoints, feedbacks, status.group(1) if status else 'UNKNOWN')


def closest(feedbacks, time_s):
    """The feedback whose desired time is nearest to `time_s`."""
    return min(feedbacks, key=lambda f: abs(f.time_s - time_s))


def degrees(radians):
    return [math.degrees(r) for r in radians]


def max_abs_error(feedbacks):
    """Per joint, the largest absolute error over the whole run, in radians."""
    return [max(abs(f.error[j]) for f in feedbacks) for j in range(JOINTS)]


def table(runs):
    """Markdown rows: file, status, waypoint, error per joint in degrees, the largest of the six."""
    lines = [
        '| file | status | waypoint | j1 | j2 | j3 | j4 | j5 | j6 | max |',
        '|---|---|---|---|---|---|---|---|---|---|',
    ]
    for name, run in runs:
        rows = [(f'{w.time_s:.0f} s', closest(run.feedbacks, w.time_s).error)
                for w in run.waypoints]
        rows.append(('last', run.feedbacks[-1].error))
        rows.append(('max abs', max_abs_error(run.feedbacks)))
        for label, error in rows:
            deg = degrees(error)
            cells = ' | '.join(f'{d:+.2f}' for d in deg)
            worst = max(abs(d) for d in deg)
            lines.append(f'| {name} | {run.status} | {label} | {cells} | {worst:.2f} |')
    return '\n'.join(lines)


def main(argv):
    runs = []
    for path in argv[1:]:
        with open(path, encoding='utf-8') as f:
            runs.append((path.rsplit('/', 1)[-1], parse(f.read())))
    print(table(runs))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
