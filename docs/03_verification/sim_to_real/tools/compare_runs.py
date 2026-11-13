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
#   Write compare_runs.py for #37, the shape of the Sprint 02 guide: read
#   /arm_controller/controller_state from each run's bag (the only ROS part, imported inside
#   read_bag like check_joint_states.py), put t = 0 where the reference leaves the home pose of
#   trajectory.yaml (after it has been there, so the move to home is skipped), interpolated
#   between two samples, using header stamps and never receive times. Resample both runs onto
#   one 100 Hz grid from 0 to the end of the shorter run with numpy.interp, never extrapolating.
#   Per joint: max |d|, mean |d|, signed mean, RMS and final |d| of feedback B - feedback A, and
#   each run's own tracking error (feedback - reference). Compute in radians, print Markdown
#   tables in degrees. Optional PNG, six rows: positions left, differences right, matplotlib
#   Agg, skipped with a note if matplotlib is missing. Warn when the runs used different
#   trajectory files or a run is _FAILED.
#   Then: t = 0 is the departure from home that goes more than 5 deg, so a move to home that
#   only corrects the last run's 1 deg rest error is not taken for the trajectory (real_2).
"""Compare sim-to-real runs: per joint gap between two recordings of one trajectory (#37)."""

# How the runs are lined up:
#   - Time is the header stamp of each message: the time the controller acted, in that world's
#     own clock (simulation time in Gazebo, the Jetson's clock on the arm). Absolute times of two
#     worlds never compare, so each run gets its own t = 0.
#   - t = 0 is where the reference first leaves the home pose by more than 1e-4 rad, after it has
#     reached home. The reference is the same curve in both worlds, so this lines them up. The
#     crossing is interpolated between the two samples around it; what is left is the jitter of
#     the stamps (Gazebo 10 ms period, the Jetson 33 ms).
#   - Both runs are resampled onto one grid (100 Hz) from 0 to the end of the shorter run.
#
# The first run is the baseline A; every other run is compared with it (B - A).
#   python3 compare_runs.py runs/sim_6 runs/sim_7                   sim vs sim (noise floor)
#   python3 compare_runs.py runs/sim_6 runs/real_1 --plot fig.png   real vs sim, plus a figure
# Run it inside the ROS 2 container, where rosbag2_py and control_msgs exist.
# Exit code 0 when every run could be compared, 1 otherwise.

import argparse
import dataclasses
import math
import os
import sys

import numpy as np
from run_trajectory import DEFAULT_TRAJECTORY, load_trajectory

TOPIC = '/arm_controller/controller_state'
START_THRESHOLD = 1e-4  # rad, "leaves the home pose"
# The trajectory goes 20 to 30 deg from home; the move to home only corrects the rest error of
# the last run (about 1 deg on the arm). Only a departure that goes this far is the trajectory.
TRAJECTORY_DEPARTURE = math.radians(5.0)
GRID_RATE_HZ = 100.0
NS_PER_S = 1_000_000_000
METRICS = ('max abs', 'mean abs', 'signed mean', 'RMS', 'final abs')


@dataclasses.dataclass
class Run:
    """One recording of /arm_controller/controller_state. Arrays have one row per message."""

    name: str
    joint_names: list
    stamp_ns: np.ndarray  # header stamps, int64
    reference: np.ndarray  # rad, shape (messages, joints)
    feedback: np.ndarray  # rad, shape (messages, joints)


# --- Pure functions: no ROS, tested with pytest -------------------------------------------

def increasing(run):
    """Return run without messages whose stamp is not after the previous one (interp needs it)."""
    keep = np.ones(len(run.stamp_ns), dtype=bool)
    last = None
    for i, stamp in enumerate(run.stamp_ns):
        keep[i] = last is None or stamp > last
        if keep[i]:
            last = stamp
    return dataclasses.replace(run, stamp_ns=run.stamp_ns[keep], reference=run.reference[keep],
                               feedback=run.feedback[keep])


def ordered(run, joint_names):
    """Return run with its columns in the order of joint_names (two runs may list them apart)."""
    missing = sorted(set(joint_names) - set(run.joint_names))
    if missing:
        raise ValueError(f'{run.name}: no data for {", ".join(missing)}')
    columns = [run.joint_names.index(name) for name in joint_names]
    return dataclasses.replace(run, joint_names=list(joint_names),
                               reference=run.reference[:, columns],
                               feedback=run.feedback[:, columns])


def find_start(stamp_ns, reference, home, threshold=START_THRESHOLD,
               departure=TRAJECTORY_DEPARTURE):
    """Return the stamp (ns, float) where the trajectory leaves home.

    That is the last time the reference leaves home before it first goes further than
    departure from it. A smaller excursion is the move to home, not the trajectory: on the
    arm, a run that starts at home (the last run ended there) has the reference jump to the
    arm's real pose, about 1 deg away, when the home goal arrives (real_2, real_3).
    Interpolated between the last sample at home and the first one away from it.
    """
    distance = np.max(np.abs(reference - np.asarray(home)), axis=1)
    at_home = np.flatnonzero(distance <= threshold)
    if at_home.size == 0:
        raise ValueError('the reference never reaches the home pose: is this the same '
                         'trajectory.yaml?')
    far = np.flatnonzero(distance[at_home[0]:] > departure)
    if far.size == 0:
        raise ValueError('the reference never leaves the home pose: the trajectory did not run')
    first_far = at_home[0] + far[0]
    before = at_home[at_home < first_far][-1]
    after = before + 1
    fraction = (threshold - distance[before]) / (distance[after] - distance[before])
    return float(stamp_ns[before]) + fraction * float(stamp_ns[after] - stamp_ns[before])


def relative_time(stamp_ns, start_ns):
    """Return seconds since start_ns (negative before the trajectory started)."""
    return (stamp_ns - start_ns) / NS_PER_S


def common_grid(times_a, times_b, rate_hz=GRID_RATE_HZ):
    """Return the grid from 0 to the end of the shorter run, never past either run's samples."""
    end = min(times_a[-1], times_b[-1])
    if end <= 0.0 or times_a[0] > 0.0 or times_b[0] > 0.0:
        raise ValueError('the runs do not cover the start of the trajectory')
    return np.arange(0.0, end + 1e-9, 1.0 / rate_hz)


def resample(times, values, grid):
    """Return every column of values at the grid times, by linear interpolation."""
    return np.column_stack([np.interp(grid, times, values[:, j])
                            for j in range(values.shape[1])])


def metrics(difference):
    """Return {metric: value} of a 1-D difference, in its own unit."""
    return {
        'max abs': float(np.max(np.abs(difference))),
        'mean abs': float(np.mean(np.abs(difference))),
        'signed mean': float(np.mean(difference)),
        'RMS': float(math.sqrt(np.mean(difference ** 2))),
        'final abs': float(abs(difference[-1])),
    }


def aligned(run, home):
    """Return (times in s since the trajectory started, the run): what compare() and plot() use."""
    run = increasing(run)
    return relative_time(run.stamp_ns, find_start(run.stamp_ns, run.reference, home)), run


def compare(run_a, run_b, home, rate_hz=GRID_RATE_HZ):
    """Return (grid, B - A feedback on the grid, {joint: metrics}), in radians."""
    run_b = ordered(run_b, run_a.joint_names)
    times_a, run_a = aligned(run_a, home)
    times_b, run_b = aligned(run_b, home)
    grid = common_grid(times_a, times_b, rate_hz)
    difference = resample(times_b, run_b.feedback, grid) - resample(times_a, run_a.feedback, grid)
    return grid, difference, {name: metrics(difference[:, j])
                              for j, name in enumerate(run_a.joint_names)}


def tracking(run, home):
    """Return {joint: metrics} of feedback - reference from t = 0 on, in radians."""
    times, run = aligned(run, home)
    after = times >= 0.0
    error = run.feedback[after] - run.reference[after]
    return {name: metrics(error[:, j]) for j, name in enumerate(run.joint_names)}


def table(title, per_joint):
    """Return a Markdown table of {joint: metrics in rad}, printed in degrees."""
    lines = [f'### {title}', '', '| Joint | ' + ' | '.join(METRICS) + ' |',
             '|---|' + '---:|' * len(METRICS)]
    for name, values in per_joint.items():
        cells = ' | '.join(f'{math.degrees(values[metric]):.2f}' for metric in METRICS)
        lines.append(f'| {name} | {cells} |')
    return lines


def run_info(folder):
    """Return {key: value} of a run's run_info.txt (written by record_run.sh), or {}."""
    path = os.path.join(folder, 'run_info.txt')
    if not os.path.exists(path):
        return {}
    with open(path, encoding='utf-8') as info:
        return dict(line.rstrip('\n').split(': ', 1) for line in info if ': ' in line)


def warnings(folders):
    """Return warnings about runs that should not be compared without a second look."""
    found = []
    for folder in folders:
        if os.path.basename(os.path.normpath(folder)).endswith('_FAILED'):
            found.append(f'{folder} is a failed run')
    hashes = {folder: run_info(folder).get('trajectory_sha256') for folder in folders}
    if len({value for value in hashes.values() if value}) > 1:
        found.append('the runs used different trajectory.yaml files: '
                     + ', '.join(f'{folder} {value[:12]}' for folder, value in hashes.items()
                                 if value))
    return found


# --- ROS part and output -------------------------------------------------------------------

def read_bag(folder):
    """Return the controller_state messages of a run folder (or a bag folder) as a Run."""
    # Imported here so the functions above can be unit-tested without ROS 2.
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.serialization import deserialize_message
    import rosbag2_py

    bag = os.path.join(folder, 'bag') if os.path.isdir(os.path.join(folder, 'bag')) else folder
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    joint_names, stamps, reference, feedback = None, [], [], []
    while reader.has_next():
        _, data, _ = reader.read_next()  # the receive time is not used, see the top of the file
        message = deserialize_message(data, JointTrajectoryControllerState)
        joint_names = joint_names or list(message.joint_names)
        stamps.append(message.header.stamp.sec * NS_PER_S + message.header.stamp.nanosec)
        reference.append(list(message.reference.positions))
        feedback.append(list(message.feedback.positions))
    if not stamps:
        raise ValueError(f'{folder}: no {TOPIC} messages')
    return Run(os.path.basename(os.path.normpath(folder)), joint_names,
               np.array(stamps, dtype=np.int64), np.array(reference), np.array(feedback))


def plot(baseline, others, home, path):
    """Save one figure: per joint, positions (left) and B - A (right), in degrees."""
    try:
        import matplotlib
        matplotlib.use('Agg')  # no display in the container
        import matplotlib.pyplot as plt
    except ImportError:
        print(f'note: matplotlib is missing, {path} not written')
        return
    joints = baseline.joint_names
    figure, axes = plt.subplots(len(joints), 2, figsize=(12, 2.2 * len(joints)), sharex=True,
                                squeeze=False)
    times_a, run_a = aligned(baseline, home)
    for j, name in enumerate(joints):
        axes[j][0].plot(times_a, np.degrees(run_a.reference[:, j]), 'k--', label='reference')
        axes[j][0].plot(times_a, np.degrees(run_a.feedback[:, j]), label=run_a.name)
        axes[j][0].set_ylabel(f'{name}\ndeg')
    for other in others:
        times_b, run_b = aligned(ordered(other, joints), home)
        grid, difference, _ = compare(baseline, other, home)
        for j in range(len(joints)):
            axes[j][0].plot(times_b, np.degrees(run_b.feedback[:, j]), label=run_b.name)
            axes[j][1].plot(grid, np.degrees(difference[:, j]),
                            label=f'{run_b.name} - {run_a.name}')
    for j in range(len(joints)):
        for column in (0, 1):
            axes[j][column].grid(True, alpha=0.3)
            axes[j][column].set_xlim(left=-1.0)
    axes[0][0].legend(fontsize='small')
    axes[0][1].legend(fontsize='small')
    axes[0][0].set_title('Position (feedback), t = 0 at the trajectory start')
    axes[0][1].set_title('Difference B - A')
    axes[-1][0].set_xlabel('s')
    axes[-1][1].set_xlabel('s')
    figure.tight_layout()
    figure.savefig(path, dpi=100)
    print(f'Figure: {path}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', help='run folders (runs/sim_1 ...), the first is A')
    parser.add_argument('--trajectory', default=DEFAULT_TRAJECTORY,
                        help='the trajectory the runs used, for its home pose')
    parser.add_argument('--plot', help='write a PNG figure to this path')
    args = parser.parse_args(argv)
    if len(args.runs) < 2:
        parser.error('give at least two run folders')

    with open(args.trajectory, encoding='utf-8') as trajectory_file:
        home = load_trajectory(trajectory_file.read())['home']
    for warning in warnings(args.runs):
        print(f'WARNING: {warning}')

    try:
        runs = [read_bag(folder) for folder in args.runs]
        baseline = runs[0]
        lines = []
        for run in runs:
            times, clean = aligned(run, home)
            period_ms = 1000.0 * float(np.median(np.diff(times)))
            lines.append(f'{clean.name}: {len(times)} messages, period {period_ms:.1f} ms, '
                         f'{times[-1]:.1f} s after the start')
        for other in runs[1:]:
            grid, _, per_joint = compare(baseline, other, home)
            lines += ['', *table(f'{other.name} - {baseline.name}, feedback, degrees '
                                 f'({grid[-1]:.1f} s at {GRID_RATE_HZ:g} Hz)', per_joint)]
        for run in runs:
            lines += ['', *table(f'{run.name} tracking error, feedback - reference, degrees',
                                 tracking(run, home))]
        print('\n'.join(lines))
        if args.plot:
            plot(baseline, runs[1:], home, args.plot)
    except (ValueError, RuntimeError) as error:  # RuntimeError: rosbag2 cannot open a bag
        print(f'ERROR: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
