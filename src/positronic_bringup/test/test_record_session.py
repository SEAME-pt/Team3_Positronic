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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# AI-generated (Claude, Anthropic).
# Prompt:
#   Write the pytest of record_session.py (#128), with no ROS and no arm: the fixed topic list;
#   the folder name for a normal issue, and refused for 0, negative, bool and non-int issues;
#   session.txt with who, issue and what, refused when who or what is empty, and kept on one
#   line when it has newlines or is very long; the ros2 bag command. Then main() with a fake
#   ros2 bag: the folder and session.txt it writes, the stop time and exit code, a Ctrl+C
#   while recording, an existing folder, ros2 missing, bad arguments, and --dry-run.
# Follow-up prompt:
#   A SIGINT sent to the script alone (kill, timeout) does not reach ros2 bag: after a grace
#   period the script sends it its own SIGINT. Test both stops with no real waiting.

"""Check record_session.py without ROS: the topics, the folder, session.txt and main()."""

import datetime
import importlib.util
import os
import signal
import subprocess

import pytest

SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'scripts',
    'record_session.py',
)

NOW = datetime.datetime(2026, 11, 24, 15, 30, 5)


def load_script():
    """Import the script as a module, without running its main()."""
    spec = importlib.util.spec_from_file_location('record_session', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def record():
    """Return the script as a module."""
    return load_script()


class FakeProcess:
    """
    Stand in for the `ros2 bag record` process.

    wait() returns `code`. With `interrupt`, the first wait() is cut by a Ctrl+C. With
    `deaf`, the process did not get that Ctrl+C: a wait with a timeout runs out until it is
    sent its own SIGINT. No real time passes.
    """

    def __init__(self, code=0, interrupt=False, deaf=False):
        """Keep the exit code and how the stop goes."""
        self.code = code
        self.interrupt = interrupt
        self.deaf = deaf
        self.waits = []  # the timeout of each wait()
        self.signals = []

    def wait(self, timeout=None):
        """Return the exit code, or raise as set up."""
        self.waits.append(timeout)
        if self.interrupt and len(self.waits) == 1:
            raise KeyboardInterrupt
        if self.deaf and timeout is not None and not self.signals:
            raise subprocess.TimeoutExpired('ros2', timeout)
        return self.code

    def send_signal(self, number):
        """Keep the signal sent."""
        self.signals.append(number)


@pytest.fixture
def fake_ros2(record, monkeypatch):
    """Make ros2 look installed and record the commands Popen gets instead of running them."""
    started = []
    process = FakeProcess()

    def popen(command):
        started.append(command)
        return process

    monkeypatch.setattr(record.shutil, 'which', lambda name: '/opt/ros/jazzy/bin/' + name)
    monkeypatch.setattr(record.subprocess, 'Popen', popen)
    return started, process


def args(root, *extra):
    """Return a valid command line for issue 128 under `root`, plus `extra`."""
    return ['--issue', '128', '--who', 'rui', '--what', 'T3 at 30 Hz', '--root', str(root),
            *extra]


# --- the topics and the command -------------------------------------------------------------

def test_topic_list_is_the_one_of_the_issue(record):
    """Check the six topics of #128, in order, and that none is repeated."""
    assert record.TOPICS == (
        '/joint_states',
        '/arm_controller/joint_trajectory',
        '/arm_controller/controller_state',
        '/dynamic_joint_states',
        '/diagnostics',
        '/mission/heartbeat',
    )
    assert len(set(record.TOPICS)) == len(record.TOPICS)


def test_record_command_writes_every_topic_into_the_bag_folder(record):
    """Check the ros2 bag command: output folder, then exactly the fixed topics."""
    command = record.record_command('bags/x/bag')
    assert command[:6] == ['ros2', 'bag', 'record', '--output', 'bags/x/bag', '--topics']
    assert tuple(command[6:]) == record.TOPICS


# --- the folder name ------------------------------------------------------------------------

def test_folder_name_has_date_time_and_issue(record):
    """Check the normal case: date, time to the second, issue number."""
    assert record.folder_name(128, NOW) == '2026-11-24_153005_issue-128'


def test_folder_name_pads_month_day_and_time(record):
    """Check the edge of single-digit fields: zero-padded, so folders sort by date."""
    early = datetime.datetime(2027, 1, 2, 3, 4, 5)
    assert record.folder_name(1, early) == '2027-01-02_030405_issue-1'


def test_folder_name_of_two_sessions_one_second_apart_differ(record):
    """Check that two sessions started in the same minute do not share a folder."""
    later = NOW + datetime.timedelta(seconds=1)
    assert record.folder_name(128, NOW) != record.folder_name(128, later)


@pytest.mark.parametrize('issue', [0, -3, True, 1.0, '128', None])
def test_folder_name_refuses_an_issue_that_is_not_a_positive_integer(record, issue):
    """Check the failure path: 0, negative, bool, float, string and None are refused."""
    with pytest.raises(ValueError):
        record.folder_name(issue, NOW)


# --- session.txt ----------------------------------------------------------------------------

def test_session_info_says_who_issue_what_host_start_and_topics(record):
    """Check the normal case of the lines written before the recording."""
    lines = record.session_info('rui', 128, 'T3 at 30 Hz', 'nano', NOW)
    assert lines == [
        'who: rui',
        'issue: #128',
        'what: T3 at 30 Hz',
        'host: nano',
        'started: 2026-11-24T15:30:05',
        'topics: ' + ' '.join(record.TOPICS),
    ]


@pytest.mark.parametrize('who, what', [('', 'T3'), ('   ', 'T3'), ('rui', ''), ('rui', '\n\t')])
def test_session_info_refuses_an_empty_who_or_what(record, who, what):
    """Check the failure path: a session without a person or a description is refused."""
    with pytest.raises(ValueError):
        record.session_info(who, 128, what, 'nano', NOW)


def test_session_info_keeps_a_multi_line_what_on_one_line(record):
    """Check that newlines and runs of spaces cannot break the key: value format."""
    lines = record.session_info('rui', 128, 'T3\nthen  T4\n', 'nano', NOW)
    assert lines[2] == 'what: T3 then T4'


def test_session_info_keeps_a_long_what_whole(record):
    """Check the edge of a very long description: nothing is cut."""
    what = 'x' * 10_000
    assert record.session_info('rui', 128, what, 'nano', NOW)[2] == 'what: ' + what


# --- main() ---------------------------------------------------------------------------------

def test_main_writes_session_txt_and_records_into_bag(record, fake_ros2, tmp_path):
    """Check the normal run: folder, session.txt before and after, and one ros2 bag command."""
    started, _ = fake_ros2
    assert record.main(args(tmp_path), now=NOW) == 0

    folder = tmp_path / '2026-11-24_153005_issue-128'
    assert started == [record.record_command(str(folder / 'bag'))]
    lines = (folder / 'session.txt').read_text(encoding='utf-8').splitlines()
    assert lines[:3] == ['who: rui', 'issue: #128', 'what: T3 at 30 Hz']
    assert lines[-2].startswith('stopped: ')
    assert lines[-1] == 'ros2 bag exit code: 0'
    # ros2 bag creates bag/ itself and refuses a folder that already exists.
    assert not (folder / 'bag').exists()


def test_main_waits_for_ros2_bag_after_ctrl_c(record, fake_ros2, tmp_path):
    """Check that a Ctrl+C while recording waits for ros2 bag to close the bag, then ends."""
    started, process = fake_ros2
    process.interrupt = True
    assert record.main(args(tmp_path), now=NOW) == 0
    # It got the same Ctrl+C: the second wait ends within the grace, no signal is sent.
    assert process.waits == [None, record.STOP_GRACE_S]
    assert process.signals == []
    assert len(started) == 1


def test_main_stops_ros2_bag_that_did_not_get_the_ctrl_c(record, fake_ros2, tmp_path):
    """Check a SIGINT to the script alone: after the grace, ros2 bag gets its own SIGINT."""
    _, process = fake_ros2
    process.interrupt = True
    process.deaf = True
    assert record.main(args(tmp_path), now=NOW) == 0
    assert process.signals == [signal.SIGINT]
    assert process.waits == [None, record.STOP_GRACE_S, None]


def test_main_returns_the_exit_code_of_ros2_bag(record, fake_ros2, tmp_path):
    """Check the failure path of the recorder: its exit code is returned and noted."""
    _, process = fake_ros2
    process.code = 1
    assert record.main(args(tmp_path), now=NOW) == 1
    info = tmp_path / '2026-11-24_153005_issue-128' / 'session.txt'
    assert info.read_text(encoding='utf-8').endswith('ros2 bag exit code: 1\n')


def test_main_refuses_a_folder_that_exists(record, fake_ros2, tmp_path):
    """Check that an existing session is never overwritten and nothing is recorded."""
    started, _ = fake_ros2
    folder = tmp_path / '2026-11-24_153005_issue-128'
    folder.mkdir()
    (folder / 'session.txt').write_text('earlier session\n', encoding='utf-8')

    assert record.main(args(tmp_path), now=NOW) == 1
    assert started == []
    assert (folder / 'session.txt').read_text(encoding='utf-8') == 'earlier session\n'


def test_main_without_ros2_creates_nothing(record, monkeypatch, tmp_path):
    """Check the failure path of an unsourced shell: an error, no folder, nothing started."""
    monkeypatch.setattr(record.shutil, 'which', lambda name: None)
    monkeypatch.setattr(record.subprocess, 'Popen', pytest.fail)
    assert record.main(args(tmp_path), now=NOW) == 1
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('bad', [['--issue', '0'], ['--who', ' '], ['--what', '']])
def test_main_refuses_bad_arguments_before_creating_anything(record, fake_ros2, tmp_path, bad):
    """Check that a bad issue, who or what stops main() with code 2 and no folder."""
    started, _ = fake_ros2
    assert record.main(args(tmp_path, *bad), now=NOW) == 2
    assert started == []
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('missing', ['--issue', '--who', '--what'])
def test_main_needs_issue_who_and_what(record, tmp_path, missing):
    """Check that each of the three required arguments is required."""
    argv = args(tmp_path)
    at = argv.index(missing)
    del argv[at:at + 2]
    with pytest.raises(SystemExit) as stop:
        record.main(argv, now=NOW)
    assert stop.value.code == 2


def test_main_refuses_an_issue_that_is_not_a_number(record, tmp_path):
    """Check that argparse refuses --issue abc."""
    argv = args(tmp_path)
    argv[1] = 'abc'
    with pytest.raises(SystemExit):
        record.main(argv, now=NOW)


def test_main_dry_run_prints_and_creates_nothing(record, fake_ros2, tmp_path, capsys):
    """Check --dry-run: the folder and the command are printed, nothing is created or run."""
    started, _ = fake_ros2
    assert record.main(args(tmp_path, '--dry-run'), now=NOW) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0] == str(tmp_path / '2026-11-24_153005_issue-128')
    assert out[1].startswith('ros2 bag record --output ')
    assert started == []
    assert list(tmp_path.iterdir()) == []
