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
#   Write a pytest suite for compare_runs.py that needs no ROS 2 and no bags: build fake runs
#   with numpy. Cover the Sprint 02 guide's cases: the same motion recorded at 100 Hz and at
#   30 Hz gives a gap near zero; a constant 0.1 rad offset gives signed mean and max 5.73 deg;
#   a run whose reference starts 0.5 s later is lined up by find_start; a reference that never
#   moves raises a clear error. Also: the move to home before the trajectory is skipped, the
#   crossing is interpolated between samples, the grid ends at the shorter run, joints listed in
#   another order are matched by name, repeated stamps are dropped, tracking error is zero when
#   feedback follows the reference, the table prints degrees, the run_info warnings, and main()
#   end to end with read_bag replaced.

"""Unit tests for the sim-to-real comparison of issue #37 (no ROS 2 needed)."""

import math

import compare_runs as compare
import numpy as np
import pytest

JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
HOME = [0.0] * 6


def reference_at(seconds, start_s):
    """Return the fake reference: home, then 0.35 rad on every joint over 4 s, then hold."""
    progress = np.clip((seconds - start_s) / 4.0, 0.0, 1.0)
    return np.outer(0.35 * progress, np.ones(6))


def fake_run(name, rate_hz=100.0, start_s=2.0, offset=0.0, length_s=10.0, clock_s=1000.0,
             joints=JOINTS, before_home=None):
    """Return a Run: the reference above, feedback = reference + offset, stamps from clock_s.

    before_home: a pose the reference starts in and leaves for home in the first second,
    like the move to home that run_trajectory.py sends first.
    """
    seconds = np.arange(0.0, length_s, 1.0 / rate_hz)
    reference = reference_at(seconds, start_s)
    if before_home is not None:
        first = seconds < 1.0
        reference[first] = np.outer(1.0 - seconds[first], before_home)
    stamps = ((clock_s + seconds) * compare.NS_PER_S).astype(np.int64)
    columns = [JOINTS.index(name) for name in joints]
    return compare.Run(name, list(joints), stamps, reference[:, columns],
                       reference[:, columns] + offset)


def test_same_motion_at_100_and_30_hz_gives_no_gap():
    """Resampling: the Gazebo rate and the Jetson rate of one motion compare to about zero."""
    _, _, per_joint = compare.compare(fake_run('sim'), fake_run('real', rate_hz=30.0), HOME)
    for values in per_joint.values():
        assert values['max abs'] < math.radians(0.05)


def test_constant_offset_gives_5_73_degrees():
    """A 0.1 rad offset shows as signed mean and max 5.73 deg."""
    _, _, per_joint = compare.compare(fake_run('a'), fake_run('b', offset=0.1), HOME)
    values = per_joint['joint2_to_joint1']
    assert math.degrees(values['signed mean']) == pytest.approx(5.73, abs=0.01)
    assert math.degrees(values['max abs']) == pytest.approx(5.73, abs=0.01)
    assert values['RMS'] == pytest.approx(0.1)
    assert values['final abs'] == pytest.approx(0.1)


def test_negative_offset_is_negative_signed_mean():
    """The sign says which way: B below A is negative."""
    _, _, per_joint = compare.compare(fake_run('a'), fake_run('b', offset=-0.1), HOME)
    assert per_joint['joint2_to_joint1']['signed mean'] == pytest.approx(-0.1)
    assert per_joint['joint2_to_joint1']['mean abs'] == pytest.approx(0.1)


def test_later_start_and_other_clock_are_lined_up():
    """A run that starts moving 0.5 s later, on another clock, still lines up."""
    late = fake_run('late', start_s=2.5, clock_s=55.0)
    _, _, per_joint = compare.compare(fake_run('a'), late, HOME)
    assert per_joint['joint2_to_joint1']['max abs'] < 1e-3


def test_find_start_is_interpolated_between_samples():
    """At 30 Hz the crossing falls between two samples; it is found to well under one period."""
    run = fake_run('real', rate_hz=30.0, start_s=2.0)
    start_ns = compare.find_start(run.stamp_ns, run.reference, HOME)
    crossing_s = 2.0 + 4.0 * compare.START_THRESHOLD / 0.35
    assert (start_ns / compare.NS_PER_S - 1000.0) == pytest.approx(crossing_s, abs=1e-6)


def test_move_to_home_is_skipped():
    """The reference first moves to home; t = 0 is the trajectory leaving home, not that."""
    run = fake_run('real', before_home=[0.28, 0.0, 0.0, -0.23, 0.0, 0.0])
    start_s = compare.find_start(run.stamp_ns, run.reference, HOME) / compare.NS_PER_S - 1000.0
    assert start_s == pytest.approx(2.0, abs=0.01)


def test_reference_that_never_moves_is_an_error():
    run = fake_run('still', start_s=100.0)
    with pytest.raises(ValueError, match='never leaves'):
        compare.find_start(run.stamp_ns, run.reference, HOME)


def test_reference_that_never_reaches_home_is_an_error():
    run = fake_run('elsewhere')
    with pytest.raises(ValueError, match='never reaches'):
        compare.find_start(run.stamp_ns, run.reference, [0.5] * 6)


def test_grid_ends_at_the_shorter_run():
    """No extrapolation: the grid stops at the last sample of the shorter run."""
    grid, _, _ = compare.compare(fake_run('long', length_s=10.0), fake_run('short', length_s=8.0),
                                 HOME)
    assert grid[0] == 0.0
    assert grid[-1] <= 8.0 - 2.0
    assert grid[-1] == pytest.approx(5.99, abs=0.011)


def test_joints_in_another_order_are_matched_by_name():
    shuffled = fake_run('b', joints=list(reversed(JOINTS)), offset=0.1)
    _, _, per_joint = compare.compare(fake_run('a'), shuffled, HOME)
    assert list(per_joint) == JOINTS
    assert per_joint['joint6output_to_joint6']['signed mean'] == pytest.approx(0.1)


def test_missing_joint_is_an_error():
    with pytest.raises(ValueError, match='no data for'):
        compare.compare(fake_run('a'), fake_run('b', joints=JOINTS[:5]), HOME)


def test_repeated_stamps_are_dropped():
    run = fake_run('a')
    run.stamp_ns[11] = run.stamp_ns[10]
    assert len(compare.increasing(run).stamp_ns) == len(run.stamp_ns) - 1


def test_tracking_error_is_zero_when_feedback_follows():
    per_joint = compare.tracking(fake_run('sim'), HOME)
    assert all(values['max abs'] == 0.0 for values in per_joint.values())


def test_tracking_error_shows_a_lag():
    """Feedback 0.1 s behind the reference: a tracking error while it moves, none at the end."""
    run = fake_run('real')
    seconds = (run.stamp_ns - run.stamp_ns[0]) / compare.NS_PER_S
    run.feedback = reference_at(seconds - 0.1, 2.0)
    values = compare.tracking(run, HOME)['joint2_to_joint1']
    assert values['max abs'] == pytest.approx(0.35 * 0.1 / 4.0, rel=0.05)
    assert values['final abs'] == pytest.approx(0.0)


def test_table_prints_degrees():
    lines = compare.table('b - a', {'joint2_to_joint1': compare.metrics(np.array([0.1, 0.1]))})
    assert lines[0] == '### b - a'
    assert lines[-1] == '| joint2_to_joint1 | 5.73 | 5.73 | 5.73 | 5.73 | 5.73 |'


def test_warnings_for_failed_runs_and_other_trajectories(tmp_path):
    for name, sha in (('sim_1', 'aaa'), ('real_1', 'bbb'), ('sim_2_FAILED', 'aaa')):
        (tmp_path / name).mkdir()
        (tmp_path / name / 'run_info.txt').write_text(f'world: x\ntrajectory_sha256: {sha}\n')
    found = compare.warnings([str(tmp_path / 'sim_1'), str(tmp_path / 'real_1')])
    assert len(found) == 1 and 'different trajectory.yaml' in found[0]
    found = compare.warnings([str(tmp_path / 'sim_1'), str(tmp_path / 'sim_2_FAILED')])
    assert len(found) == 1 and 'failed run' in found[0]


def test_main_prints_the_tables(monkeypatch, capsys):
    runs = {'sim_6': fake_run('sim_6'), 'real_1': fake_run('real_1', rate_hz=30.0, offset=0.01)}
    monkeypatch.setattr(compare, 'read_bag', lambda folder: runs[folder])
    assert compare.main(['sim_6', 'real_1']) == 0
    out = capsys.readouterr().out
    assert '### real_1 - sim_6, feedback, degrees' in out
    assert '### sim_6 tracking error' in out and '### real_1 tracking error' in out
    assert '| joint2_to_joint1 | 0.57 | 0.57 | 0.57 | 0.57 | 0.57 |' in out


def test_main_reports_a_run_that_cannot_be_lined_up(monkeypatch, capsys):
    runs = {'a': fake_run('a'), 'b': fake_run('b', start_s=100.0)}
    monkeypatch.setattr(compare, 'read_bag', lambda folder: runs[folder])
    assert compare.main(['a', 'b']) == 1
    assert 'never leaves' in capsys.readouterr().err


def test_plot_writes_a_png(tmp_path):
    pytest.importorskip('matplotlib')
    path = tmp_path / 'figure.png'
    compare.plot(fake_run('sim'), [fake_run('real', rate_hz=30.0, offset=0.02)], HOME, str(path))
    assert path.read_bytes()[:8] == b'\x89PNG\r\n\x1a\n'
