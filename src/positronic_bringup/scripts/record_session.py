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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# AI-generated (Claude, Anthropic).
# Prompt:
#   Issue #128: one script in positronic_bringup records the same topics in every arm session:
#   /joint_states, the command and the state of arm_controller, the topic of driver/fault_code,
#   /diagnostics and the mission heartbeat, into a folder named by date and issue, with a text
#   file next to the bag (who ran it, the issue, what was run). Bags stay out of git. Keep the
#   topic list, the folder name and the text file testable without ROS; main() runs
#   `ros2 bag record`.

"""
Record an arm session into a bag, with the same topics every time (#128).

    ros2 run positronic_bringup record_session.py --issue 134 --who rui \
        --what "T3 of #33 at update_rate 50, speed of #133"

Creates <root>/<date>_<time>_issue-<n>/ with:

    session.txt   who ran it, the issue, what was run, the host, the topics, start and stop
    bag/          the bag, written by `ros2 bag record` (MCAP, the Jazzy default)

and records until Ctrl+C. <root> is `bags` in the current folder by default: /ws/bags in the
containers, the only folder that outlives them. It is in .gitignore, and `make nano-sync` leaves
it alone on the Jetson. --dry-run prints the folder and the command and records nothing.

A topic that nobody publishes yet is still recorded once it appears: start the recording before
or after the launch, it does not matter. The goals MoveIt sends are actions, which `ros2 bag`
does not record in Jazzy; what arm_controller does with them is on its state topic.
"""

import argparse
import datetime
import os
import shutil
import signal
import socket
import subprocess
import sys

# The topics of every session, in this order (#128). /dynamic_joint_states carries every state
# interface, the `driver` gpio included: driver/fault_code is read there (#45).
TOPICS = (
    '/joint_states',
    '/arm_controller/joint_trajectory',
    '/arm_controller/controller_state',
    '/dynamic_joint_states',
    '/diagnostics',
    '/mission/heartbeat',
)

DEFAULT_ROOT = 'bags'
BAG_DIR = 'bag'
INFO_FILE = 'session.txt'
# Seconds ros2 bag gets to close the bag after a Ctrl+C, before it is sent its own SIGINT.
STOP_GRACE_S = 10.0


def folder_name(issue, now):
    """
    Return the session folder's name, for example 2026-11-24_153005_issue-128.

    The seconds are in it so that two sessions started in the same minute get two folders.
    Raises ValueError for an issue number that is not a positive integer.
    """
    if isinstance(issue, bool) or not isinstance(issue, int) or issue <= 0:
        raise ValueError(f'the issue number must be a positive integer, not {issue!r}')
    return f'{now:%Y-%m-%d_%H%M%S}_issue-{issue}'


def one_line(text, field):
    """
    Return `text` on one line, its whitespace collapsed to single spaces.

    Raises ValueError when nothing is left: every session says who ran it and what was run.
    """
    collapsed = ' '.join(text.split())
    if not collapsed:
        raise ValueError(f'--{field} must not be empty')
    return collapsed


def session_info(who, issue, what, host, started):
    """Return the lines of session.txt written before the recording starts."""
    return [
        f'who: {one_line(who, "who")}',
        f'issue: #{issue}',
        f'what: {one_line(what, "what")}',
        f'host: {host}',
        f'started: {started.isoformat(timespec="seconds")}',
        'topics: ' + ' '.join(TOPICS),
    ]


def record_command(bag_path):
    """Return the `ros2 bag record` command that writes the fixed topics into `bag_path`."""
    return ['ros2', 'bag', 'record', '--output', bag_path, '--topics', *TOPICS]


def parse_args(argv):
    """Parse the command line."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--issue', type=int, required=True,
                        help='number of the issue this session is for')
    parser.add_argument('--who', required=True, help='who runs the session')
    parser.add_argument('--what', required=True,
                        help='what is run, in one line: launch, trajectory, speed, ...')
    parser.add_argument('--root', default=DEFAULT_ROOT,
                        help=f'folder that holds the sessions (default: {DEFAULT_ROOT})')
    parser.add_argument('--dry-run', action='store_true',
                        help='print the folder and the command, record nothing')
    return parser.parse_args(argv)


def run(command):
    """
    Run `command` until it ends and return its exit code.

    A Ctrl+C in the terminal reaches `ros2 bag record` too, since it is in the same process
    group, and it closes the bag by itself: this waits for it, so the bag is complete when the
    script ends. A SIGINT sent to this script alone (kill, timeout) does not reach it: if it
    still runs after STOP_GRACE_S, it gets its own SIGINT.
    """
    process = subprocess.Popen(command)
    try:
        return process.wait()
    except KeyboardInterrupt:
        try:
            return process.wait(timeout=STOP_GRACE_S)
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            return process.wait()


def main(argv=None, now=None):
    """Check the arguments, write session.txt, record until Ctrl+C, then note the stop time."""
    args = parse_args(argv)
    now = now or datetime.datetime.now()
    try:
        folder = os.path.join(args.root, folder_name(args.issue, now))
        lines = session_info(args.who, args.issue, args.what, socket.gethostname(), now)
    except ValueError as error:
        print(f'record_session: {error}', file=sys.stderr)
        return 2
    command = record_command(os.path.join(folder, BAG_DIR))

    if args.dry_run:
        print(folder)
        print(' '.join(command))
        return 0
    if shutil.which('ros2') is None:
        print('record_session: ros2 not found: source /opt/ros/jazzy/setup.bash', file=sys.stderr)
        return 1
    if os.path.exists(folder):
        print(f'record_session: {folder} already exists, nothing recorded', file=sys.stderr)
        return 1

    os.makedirs(folder)
    info_path = os.path.join(folder, INFO_FILE)
    with open(info_path, 'w', encoding='utf-8') as info:
        info.write('\n'.join(lines) + '\n')
    print(f'recording into {folder}, Ctrl+C to stop')

    code = run(command)
    with open(info_path, 'a', encoding='utf-8') as info:
        stopped = datetime.datetime.now().isoformat(timespec='seconds')
        info.write(f'stopped: {stopped}\nros2 bag exit code: {code}\n')
    print(f'session in {folder}')
    return code


if __name__ == '__main__':
    sys.exit(main())
