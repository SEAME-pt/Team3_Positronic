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
#   Write a pytest suite for run_trajectory.py that needs no ROS 2: test the pure checks on the
#   real trajectory.yaml and the real mycobot_280_jn.urdf, and on broken copies built in memory.
#   Cover: the URDF limits include the velocity; the committed trajectory passes and its highest
#   peak is 0.249 rad/s; a position outside the limits or inside the 10 deg margin is refused,
#   home included; a segment too fast is refused and dt is the difference of time_from_start;
#   the first segment starts at home; joint names in another order pass, but a typo, a missing
#   or a duplicated name is refused; wrong list lengths, NaN and times that do not increase are
#   refused; the home move is sized to 30 % with a 4 s minimum; seconds split into sec and
#   nanosec; and main --check-only exits 0 on the real file and 1 on a refused one.

"""Unit tests for the trajectory checks of issue #37 (no ROS 2 needed)."""

import copy
import math

import pytest
import run_trajectory as run

JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']


def limits():
    """Return the joint limits of the real URDF."""
    with open(run.DEFAULT_URDF, encoding='utf-8') as urdf:
        return run.urdf_joint_limits(urdf.read())


def committed():
    """Return the committed trajectory.yaml, loaded."""
    with open(run.DEFAULT_TRAJECTORY, encoding='utf-8') as trajectory:
        return run.load_trajectory(trajectory.read())


def simple(positions, seconds=4.0):
    """Return a one-point trajectory from home (all zeros) to positions."""
    return {'joint_names': list(JOINTS), 'home': [0.0] * 6, 'points': [(positions, seconds)]}


def test_urdf_limits_include_the_velocity():
    """Check the six revolute joints are read with lower, upper and velocity."""
    joint_limits = limits()
    assert list(joint_limits) == JOINTS
    assert joint_limits['joint3_to_joint2'] == (-2.4434, 2.4434, 2.618)


def test_committed_trajectory_passes():
    """Check the file both worlds run passes every check, with its peak far below 30 %."""
    trajectory = committed()
    assert run.check_trajectory(trajectory, limits()) == []
    label, name, peak = max(run.segment_peaks(trajectory), key=lambda entry: entry[2])
    assert peak == pytest.approx(1.9 * 0.5236 / 4.0)
    assert peak < 0.30 * 2.618


def test_load_turns_points_into_pairs():
    """Check every point becomes (positions, seconds), as floats."""
    trajectory = run.load_trajectory(
        'joint_names: [a]\nhome: [0]\npoints:\n  - positions: [1]\n    time_from_start: 4\n')
    assert trajectory == {'joint_names': ['a'], 'home': [0.0], 'points': [([1.0], 4.0)]}


def test_position_outside_the_limits_is_refused():
    """Check a typo like 4.363 instead of 0.4363 is caught, with the joint named."""
    problems = run.check_positions(simple([4.363, 0, 0, 0, 0, 0]), limits())
    assert len(problems) == 1
    assert 'point 1 joint2_to_joint1' in problems[0]


def test_position_inside_the_margin_is_refused():
    """Check the 10 deg margin: just inside it passes, just past it fails."""
    upper = limits()['joint3_to_joint2'][1]
    edge = upper - math.radians(10.0)
    assert run.check_positions(simple([0, edge - 1e-6, 0, 0, 0, 0]), limits()) == []
    assert run.check_positions(simple([0, edge + 1e-6, 0, 0, 0, 0]), limits()) != []


def test_home_is_checked_too():
    """Check a home pose outside the limits is refused, not only the points."""
    trajectory = simple([0.0] * 6)
    trajectory['home'][0] = 3.0
    problems = run.check_positions(trajectory, limits())
    assert len(problems) == 1
    assert problems[0].startswith('home joint2_to_joint1')


def test_too_fast_segment_is_refused():
    """Check 30 deg in 0.5 s (peak 1.99 rad/s) is refused, with the segment named."""
    problems = run.check_speeds(simple([0.5236, 0, 0, 0, 0, 0], seconds=0.5), limits())
    assert len(problems) == 1
    assert problems[0].startswith('home -> point 1 joint2_to_joint1')


def test_dt_is_the_difference_of_the_times():
    """Check segment 2 of the committed file uses 8 - 4 = 4 s, not 8 s."""
    peaks = {(label, name): peak for label, name, peak in run.segment_peaks(committed())}
    assert peaks[('point 1 -> point 2', 'joint4_to_joint3')] == pytest.approx(
        1.9 * 0.4363 / 4.0)


def test_first_segment_starts_at_home():
    """Check segment 1 is measured from home, not from zero."""
    trajectory = simple([0.3, 0, 0, 0, 0, 0])
    trajectory['home'][0] = 0.1
    peaks = run.segment_peaks(trajectory)
    assert peaks[0] == ('home -> point 1', 'joint2_to_joint1', pytest.approx(1.9 * 0.2 / 4.0))


def test_names_in_another_order_pass():
    """Check the order does not matter: the controller matches positions by name."""
    assert run.check_names(list(reversed(JOINTS)), limits()) == []


def test_unknown_name_is_refused():
    """Check a typo is reported both as unknown and as a missing joint."""
    names = JOINTS[:-1] + ['joint6output_to_joint7']
    assert run.check_names(names, limits()) == [
        'not in the URDF: joint6output_to_joint7', 'missing: joint6output_to_joint6']


def test_duplicated_name_is_refused():
    """Check a name listed twice is caught, which a set comparison alone would miss."""
    names = JOINTS + ['joint2_to_joint1']
    assert set(names) == set(limits())
    assert run.check_names(names, limits()) == ['listed twice: joint2_to_joint1']


def test_wrong_number_of_positions_is_refused():
    """Check a point with five positions for six names is caught."""
    problems = run.check_shape(simple([0.1] * 5))
    assert problems == ['point 1: 5 positions for 6 joint names']


def test_nan_is_refused():
    """Check a position that is not a number is caught."""
    assert run.check_shape(simple([math.nan] + [0.0] * 5)) == [
        'point 1: a position is not a finite number']


def test_times_must_increase():
    """Check a point that is not later than the one before is refused."""
    trajectory = copy.deepcopy(committed())
    positions, _ = trajectory['points'][2]
    trajectory['points'][2] = (positions, 6.0)  # after 8.0
    assert run.check_shape(trajectory) == ['point 3: time_from_start 6.0 s is not after 8.0 s']


def test_shape_problems_stop_the_other_checks():
    """Check a broken shape is reported alone, before checks that would crash on it."""
    trajectory = simple([0.1] * 5)
    trajectory['joint_names'][0] = 'typo'
    assert run.check_trajectory(trajectory, limits()) == [
        'point 1: 5 positions for 6 joint names']


def test_home_duration_has_a_minimum():
    """Check a small move to home still takes 4 s."""
    assert run.home_duration([0.1] * 6, [0.0] * 6, JOINTS, limits()) == 4.0


def test_home_duration_respects_30_percent():
    """Check a long move to home is slowed so its peak stays at or below 30 %."""
    seconds = run.home_duration([2.5, 0, 0, 0, 0, 0], [0.0] * 6, JOINTS, limits())
    assert seconds > 4.0
    assert 1.9 * 2.5 / seconds <= 0.30 * 2.618


@pytest.mark.parametrize('seconds, expected', [
    (4.0, (4, 0)),
    (2.5, (2, 500_000_000)),
    (0.1, (0, 100_000_000)),
])
def test_split_seconds(seconds, expected):
    """Check seconds become the (sec, nanosec) pair of a ROS Duration."""
    assert run.split_seconds(seconds) == expected


def test_main_check_only_exit_codes(tmp_path, capsys):
    """Check --check-only exits 0 on the committed file and 1 on a refused copy."""
    assert run.main(['--check-only']) == 0
    with open(run.DEFAULT_TRAJECTORY, encoding='utf-8') as trajectory:
        text = trajectory.read()
    broken = tmp_path / 'trajectory.yaml'
    broken.write_text(text.replace('[0.4363, 0.3491, 0.0,', '[4.363, 0.3491, 0.0,', 1))
    assert run.main(['--check-only', '--trajectory', str(broken)]) == 1
    assert 'REFUSED' in capsys.readouterr().out


def test_build_goal_puts_every_point_in_one_goal():
    """Check the goal carries all the points and the times as sec + nanosec (needs ROS)."""
    pytest.importorskip('control_msgs')
    trajectory = committed()
    goal = run.build_goal(trajectory['joint_names'], trajectory['points'])
    assert list(goal.trajectory.joint_names) == JOINTS
    assert len(goal.trajectory.points) == 5
    assert goal.trajectory.points[-1].time_from_start.sec == 20
