# AI-generated (Claude, Anthropic).
# Prompt:
#   pytest for step_up_check.py, no ROS, no arm: mode folders written by hand like the arm run
#   leaves them (the stop lines are the format tap_speed.py printed for fixed50 on 24 Nov). A
#   byte at or below 30 has no condition; 70 starts after a full fixed50 and not without it;
#   a missing or refused stop, fewer than 5 stop lines, a travel just past the limit or NaN
#   refuse; exactly at the limit passes; a re-run folder counts; a dry-run folder counts only
#   for a dry run; bytes off the list (0, 80, 101) are refused; the command line exits 0/1/2.
"""Tests of step_up_check.py."""

import math

import pytest

import step_up_check

TRAVELS_FIXED50 = (1.93, 2.02, 2.03, 1.93, 1.85)


def make_mode(setup, name, travels=TRAVELS_FIXED50, triggers=(True,) * 5, dry=False):
    """A mode folder as arm_run_static.sh leaves it."""
    folder = setup / name
    folder.mkdir(parents=True)
    args = ('driver: DRY, fake backend send_speed:=50' if dry
            else 'driver: port /dev/pts/3 send_speed:=50')
    (folder / 'args.txt').write_text(args + '\n', encoding='utf-8')
    for run, ok in enumerate(triggers, start=1):
        (folder / f'stop_r{run}_trigger.txt').write_text(
            f'1795520118.483867 1795520118.539950 ok={ok}\n', encoding='utf-8')
    lines = ['## /ws/log/133s/setup1/fixed50/tap.csv: 8466 sends, 86286 poses']
    for i, travel in enumerate(travels):
        lines.append(f'stop at {141.4 + 80 * i:.3f} s: fastest joint 14.7 deg/s before, '
                     f'travel after {travel} deg (J1), still after 185 ms')
    lines.append('| file | status | waypoint | j1 | j2 | j3 | j4 | j5 | j6 | max |')
    (folder / 'summary.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return folder


@pytest.mark.parametrize('byte', [5, 10, 30])
def test_low_bytes_have_no_condition(tmp_path, byte):
    code, message = step_up_check.check(tmp_path / 'setup2', byte)
    assert code == 0, message


def test_70_after_a_full_fixed50(tmp_path):
    make_mode(tmp_path, 'fixed50')
    code, message = step_up_check.check(tmp_path, 70)
    assert code == 0, message
    assert '1.85-2.03 deg' in message


def test_70_refused_without_fixed50(tmp_path):
    make_mode(tmp_path, 'fixed30')
    code, message = step_up_check.check(tmp_path, 70)
    assert code == 1
    assert 'fixed50' in message


def test_no_setup_folder_refuses(tmp_path):
    code, _ = step_up_check.check(tmp_path / 'nothing', 90)
    assert code == 1


@pytest.mark.parametrize('byte, before', [(50, 30), (70, 50), (90, 70), (100, 90)])
def test_each_byte_needs_the_one_before(tmp_path, byte, before):
    make_mode(tmp_path, f'fixed{before}')
    assert step_up_check.check(tmp_path, byte)[0] == 0
    # Two steps up at once is refused: 100 does not follow 70.
    if byte == 90:
        assert step_up_check.check(tmp_path, 100)[0] == 1


def test_a_stop_the_driver_did_not_take_refuses(tmp_path):
    make_mode(tmp_path, 'fixed70', triggers=(True, True, False, True, True))
    code, message = step_up_check.check(tmp_path, 90)
    assert code == 1
    assert '[3]' in message


def test_a_missing_stop_refuses(tmp_path):
    make_mode(tmp_path, 'fixed70', triggers=(True,) * 4)
    assert step_up_check.check(tmp_path, 90)[0] == 1


def test_fewer_than_five_stop_lines_refuses(tmp_path):
    # tap_speed.py leaves out a stop with no rest after it.
    make_mode(tmp_path, 'fixed70', travels=TRAVELS_FIXED50[:4])
    code, message = step_up_check.check(tmp_path, 90)
    assert code == 1
    assert '4 stops' in message


def test_no_summary_refuses(tmp_path):
    folder = make_mode(tmp_path, 'fixed70')
    (folder / 'summary.txt').unlink()
    assert step_up_check.check(tmp_path, 90)[0] == 1


def test_travel_exactly_at_the_limit_passes(tmp_path):
    make_mode(tmp_path, 'fixed70', travels=(1.9, 2.0, 4.0, 1.8, 1.9))
    assert step_up_check.check(tmp_path, 90, max_travel_deg=4.0)[0] == 0


def test_travel_just_past_the_limit_refuses(tmp_path):
    make_mode(tmp_path, 'fixed70', travels=(1.9, 2.0, 4.01, 1.8, 1.9))
    code, message = step_up_check.check(tmp_path, 90, max_travel_deg=4.0)
    assert code == 1
    assert '4.01' in message


def test_nan_travel_refuses(tmp_path):
    make_mode(tmp_path, 'fixed70', travels=(1.9, 'nan', 2.0, 1.8, 1.9))
    assert step_up_check.check(tmp_path, 90)[0] == 1


def test_nan_limit_refuses(tmp_path):
    make_mode(tmp_path, 'fixed70')
    assert step_up_check.check(tmp_path, 90, max_travel_deg=math.nan)[0] == 1


def test_a_rerun_folder_counts(tmp_path):
    make_mode(tmp_path, 'fixed90', triggers=(True, True, False, True, True))
    make_mode(tmp_path, 'fixed90-143005')
    code, message = step_up_check.check(tmp_path, 100)
    assert code == 0, message
    assert 'fixed90-143005' in message


def test_a_folder_with_a_longer_name_does_not_count(tmp_path):
    make_mode(tmp_path, 'fixed900')
    make_mode(tmp_path, 'fixed90_old')
    assert step_up_check.check(tmp_path, 100)[0] == 1


def test_a_dry_run_never_counts_for_the_arm(tmp_path):
    make_mode(tmp_path, 'fixed70', travels=(), dry=True)
    code, message = step_up_check.check(tmp_path, 90)
    assert code == 1
    assert 'dry run' in message


def test_a_dry_run_counts_for_a_dry_run(tmp_path):
    make_mode(tmp_path, 'fixed70', travels=(), dry=True)
    code, message = step_up_check.check(tmp_path, 90, dry=True)
    assert code == 0, message
    assert 'travel not checked' in message


def test_a_dry_run_still_needs_its_stops(tmp_path):
    make_mode(tmp_path, 'fixed70', travels=(), triggers=(True,) * 3, dry=True)
    assert step_up_check.check(tmp_path, 90, dry=True)[0] == 1


@pytest.mark.parametrize('byte', [0, 1, 17, 80, 101, -30])
def test_bytes_off_the_list_are_refused(tmp_path, byte):
    assert step_up_check.check(tmp_path, byte)[0] == 2


def test_command_line_exit_codes(tmp_path, capsys):
    make_mode(tmp_path, 'fixed50')
    assert step_up_check.main([str(tmp_path), '70']) == 0
    assert 'may start' in capsys.readouterr().out
    assert step_up_check.main([str(tmp_path), '90']) == 1
    assert 'fixed70' in capsys.readouterr().err
    assert step_up_check.main([str(tmp_path), '80']) == 2
