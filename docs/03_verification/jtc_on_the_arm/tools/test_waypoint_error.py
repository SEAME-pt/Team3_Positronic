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
#   Unit tests for waypoint_error.py on a two-feedback recording written by hand: the goal
#   waypoints and their times, the feedback times and positions, the status, the nearest
#   feedback per waypoint, the per-joint maximum, the Markdown rows in degrees, and a block
#   without error positions being skipped.
"""Tests for waypoint_error.py, no ROS needed (python3 -m pytest on this folder)."""

import math

import waypoint_error as we

RECORDING = """Sending goal:
     trajectory:
  points:
  - positions:
    - 0.1
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    velocities: []
    time_from_start:
      sec: 2
      nanosec: 0
  - positions:
    - 0.2
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    velocities: []
    time_from_start:
      sec: 4
      nanosec: 500000000
Goal accepted with ID: abc

Feedback:
    header:
  stamp:
    sec: 1
    nanosec: 0
desired:
  positions:
  - 0.1
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  velocities:
  - 0.0
  time_from_start:
    sec: 1
    nanosec: 900000000
actual:
  positions:
  - 0.08
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  time_from_start:
    sec: 1
    nanosec: 900000000
error:
  positions:
  - 0.02
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0

Feedback:
    header:
  stamp:
    sec: 1
    nanosec: 0
desired:
  positions:
  - 0.2
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  velocities:
  - 0.0
  time_from_start:
    sec: 4
    nanosec: 400000000
actual:
  positions:
  - 0.2
  - 0.0
  - 0.0
  - -0.05
  - 0.0
  - 0.0
  time_from_start:
    sec: 4
    nanosec: 400000000
error:
  positions:
  - 0.0
  - 0.0
  - 0.0
  - 0.05
  - 0.0
  - 0.0

Result:
    error_code: 0

Goal finished with status: SUCCEEDED
"""


def test_parse_reads_the_waypoints_the_feedbacks_and_the_status():
    run = we.parse(RECORDING)
    assert [w.time_s for w in run.waypoints] == [2.0, 4.5]
    assert run.waypoints[1].positions[0] == 0.2
    assert len(run.feedbacks) == 2
    assert run.feedbacks[0].time_s == 1.9
    assert run.feedbacks[1].actual[3] == -0.05
    assert run.status == 'SUCCEEDED'


def test_closest_picks_the_feedback_nearest_to_the_waypoint_time():
    run = we.parse(RECORDING)
    assert we.closest(run.feedbacks, 2.0) is run.feedbacks[0]
    assert we.closest(run.feedbacks, 4.5) is run.feedbacks[1]


def test_max_abs_error_is_per_joint_over_the_run():
    run = we.parse(RECORDING)
    assert we.max_abs_error(run.feedbacks) == [0.02, 0.0, 0.0, 0.05, 0.0, 0.0]


def test_table_reports_degrees_per_waypoint_last_and_max():
    run = we.parse(RECORDING)
    rows = we.table([('r1.txt', run)]).splitlines()
    assert rows[2].startswith('| r1.txt | SUCCEEDED | 2 s |')
    assert f'{math.degrees(0.02):+.2f}' in rows[2]
    assert rows[3].startswith('| r1.txt | SUCCEEDED | 4 s |')  # 4.5 s, printed without decimals
    assert '| last |' in rows[4]
    assert '| max abs |' in rows[5]
    assert rows[5].endswith(f'| {math.degrees(0.05):.2f} |')


def test_a_block_without_error_positions_is_skipped():
    cut = RECORDING.replace('error:\n  positions:\n  - 0.0\n', 'error:\n  positions: []\n', 1)
    assert len(we.parse(cut).feedbacks) == 1
