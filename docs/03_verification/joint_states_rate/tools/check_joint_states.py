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
"""Check a recorded /joint_states bag against the acceptance criteria of issue #23."""

# Criteria, one PASS/FAIL line each:
#   duration    at least --min-duration seconds of data (10 min by default)
#   rate        mean rate within --rate-tolerance of the nominal --rate (the #20 value)
#   periods     no period between two messages above 2x the nominal period
#   names       every message carries exactly the revolute joints of the URDF
#   radians     every position is finite and inside the URDF joint limits
#   reference   (optional) the first positions match angles read with pymycobot, in degrees
#
# Periods are measured on the time each message was recorded (what a subscriber sees, as
# ros2 topic hz does). The header stamps are reported too, to tell a slow control loop from
# a slow network.
#
# Run it inside the ROS 2 container, where rosbag2_py and sensor_msgs exist:
#   python3 docs/03_verification/joint_states_rate/tools/check_joint_states.py <bag_dir> --rate 50
# Exit code 0 when every criterion passes, 1 otherwise.

import argparse
import dataclasses
import math
import sys
import xml.etree.ElementTree as ElementTree

PERIOD_LIMIT_FACTOR = 2.0  # "no period above 2x nominal"
NS_PER_S = 1_000_000_000


@dataclasses.dataclass
class Sample:
    """One /joint_states message."""

    receive_ns: int  # when the bag recorded it
    header_ns: int  # header.stamp, set by the controller manager
    names: tuple
    positions: tuple


@dataclasses.dataclass
class PeriodStats:
    """Timing of a stream of stamps."""

    count: int
    duration_s: float
    mean_rate_hz: float
    p50_ms: float
    p99_ms: float
    max_ms: float
    max_at_s: float  # seconds from the first stamp to the end of the longest period


@dataclasses.dataclass
class Criterion:
    """The verdict on one acceptance criterion."""

    name: str
    passed: bool
    detail: str


def urdf_joint_limits(urdf_text):
    """Return {joint name: (lower, upper)} of the movable joints of a URDF, in file order."""
    limits = {}
    for joint in ElementTree.fromstring(urdf_text).findall('joint'):
        kind = joint.get('type')
        if kind == 'continuous':
            limits[joint.get('name')] = (-math.inf, math.inf)
        elif kind == 'revolute':
            limit = joint.find('limit')
            limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
    return limits


def percentile(sorted_values, percent):
    """Return the nearest-rank percentile of an already sorted list."""
    rank = max(1, math.ceil(percent / 100.0 * len(sorted_values)))
    return sorted_values[rank - 1]


def period_stats(stamps_ns):
    """Return the timing of a stream of at least two stamps, in nanoseconds."""
    if len(stamps_ns) < 2:
        raise ValueError('need at least two messages to measure periods')
    periods = [after - before for before, after in zip(stamps_ns, stamps_ns[1:])]
    longest = max(range(len(periods)), key=periods.__getitem__)
    duration_ns = stamps_ns[-1] - stamps_ns[0]
    ordered = sorted(periods)
    return PeriodStats(
        count=len(stamps_ns),
        duration_s=duration_ns / NS_PER_S,
        mean_rate_hz=(len(stamps_ns) - 1) * NS_PER_S / duration_ns if duration_ns > 0 else 0.0,
        p50_ms=percentile(ordered, 50) / 1e6,
        p99_ms=percentile(ordered, 99) / 1e6,
        max_ms=periods[longest] / 1e6,
        max_at_s=(stamps_ns[longest + 1] - stamps_ns[0]) / NS_PER_S,
    )


def periods_over(stamps_ns, limit_ns):
    """Return (seconds from start, period in ms) of every period longer than limit_ns."""
    return [
        ((after - stamps_ns[0]) / NS_PER_S, (after - before) / 1e6)
        for before, after in zip(stamps_ns, stamps_ns[1:])
        if after - before > limit_ns
    ]


def leading_zero_samples(samples):
    """Count the samples at the start whose positions are all exactly zero."""
    count = 0
    for sample in samples:
        if any(position != 0.0 for position in sample.positions):
            break
        count += 1
    return count


def evaluate(samples, rate_hz, limits, min_duration_s=600.0, rate_tolerance=0.05,
             limit_margin=0.05, reference_deg=None, reference_tolerance_deg=0.5):
    """Return the list of Criterion for the samples of one recording."""
    criteria = []
    receive = [sample.receive_ns for sample in samples]
    stats = period_stats(receive)

    criteria.append(Criterion(
        'duration', stats.duration_s >= min_duration_s,
        f'{stats.duration_s:.1f} s recorded, {min_duration_s:.0f} s required'))

    low, high = rate_hz * (1.0 - rate_tolerance), rate_hz * (1.0 + rate_tolerance)
    criteria.append(Criterion(
        'rate', low <= stats.mean_rate_hz <= high,
        f'mean {stats.mean_rate_hz:.2f} Hz, nominal {rate_hz:g} Hz '
        f'(accepted {low:.2f} to {high:.2f})'))

    limit_ns = PERIOD_LIMIT_FACTOR * NS_PER_S / rate_hz
    late = periods_over(receive, limit_ns)
    detail = (f'{len(late)} periods above {limit_ns / 1e6:.1f} ms; '
              f'p50 {stats.p50_ms:.1f} ms, p99 {stats.p99_ms:.1f} ms, '
              f'max {stats.max_ms:.1f} ms at {stats.max_at_s:.1f} s')
    if late:
        detail += '; first: ' + ', '.join(f'{ms:.1f} ms at {at:.1f} s' for at, ms in late[:5])
    criteria.append(Criterion('periods', not late, detail))

    expected = set(limits)
    wrong = [i for i, sample in enumerate(samples)
             if len(sample.names) != len(expected) or set(sample.names) != expected]
    if wrong:
        first = samples[wrong[0]].names
        detail = f'{len(wrong)} messages differ; first at index {wrong[0]}: {list(first)}'
    else:
        detail = f'all {len(samples)} messages carry the {len(expected)} URDF joints'
    criteria.append(Criterion('names', not wrong, detail))

    problems = []
    for index, sample in enumerate(samples):
        for name, position in zip(sample.names, sample.positions):
            lower, upper = limits.get(name, (-math.inf, math.inf))
            if not math.isfinite(position):
                problems.append(f'{name}={position} (not finite) at index {index}')
            elif not lower - limit_margin <= position <= upper + limit_margin:
                problems.append(f'{name}={position:.4f} outside [{lower}, {upper}] '
                                f'at index {index}')
        if len(problems) >= 5:
            break
    criteria.append(Criterion(
        'radians', not problems,
        '; '.join(problems) if problems else 'every position finite and inside the URDF limits'))

    if reference_deg is not None:
        criteria.append(_reference_criterion(
            samples, list(limits), reference_deg, reference_tolerance_deg))
    return criteria


def _reference_criterion(samples, joint_order, reference_deg, tolerance_deg):
    """Compare the mean of the first samples, in degrees, with angles read by pymycobot."""
    # Skip an all-zero placeholder pose at the start, should the driver ever publish one.
    first = samples[leading_zero_samples(samples):][:50]
    errors = []
    for name, expected in zip(joint_order, reference_deg):
        values = [s.positions[s.names.index(name)] for s in first if name in s.names]
        if not values:
            return Criterion('reference', False, f'{name} missing from the first messages')
        measured = math.degrees(sum(values) / len(values))
        errors.append((name, measured, expected, abs(measured - expected)))
    worst = max(errors, key=lambda error: error[3])
    detail = (f'worst {worst[0]}: {worst[1]:+.2f} deg published vs {worst[2]:+.2f} deg '
              f'from pymycobot (tolerance {tolerance_deg} deg)')
    if all(abs(expected) < 1.0 for expected in reference_deg):
        detail += '; weak evidence: every reference angle is within 1 deg of zero'
    return Criterion('reference', worst[3] <= tolerance_deg, detail)


def read_bag(bag_path, topic):
    """Return the /joint_states samples of a rosbag2 recording, in recording order."""
    # Imported here so the checks above can be unit-tested without ROS 2.
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import JointState

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    samples = []
    while reader.has_next():
        _, data, receive_ns = reader.read_next()
        message = deserialize_message(data, JointState)
        stamp = message.header.stamp
        samples.append(Sample(
            receive_ns=receive_ns,
            header_ns=stamp.sec * NS_PER_S + stamp.nanosec,
            names=tuple(message.name),
            positions=tuple(message.position)))
    return samples


def default_urdf_path():
    """Return the arm's URDF as installed by positronic_description."""
    from ament_index_python.packages import get_package_share_directory
    return (get_package_share_directory('positronic_description')
            + '/urdf/mycobot_280_jn_sim/mycobot_280_jn.urdf')


def report_lines(criteria, header_stats, leading_zeros, rate_hz):
    """Return the human-readable report as a list of lines."""
    lines = [f'{"PASS" if c.passed else "FAIL"}  {c.name:<10} {c.detail}' for c in criteria]
    lines.append(
        f'info  header     p50 {header_stats.p50_ms:.1f} ms, p99 {header_stats.p99_ms:.1f} ms, '
        f'max {header_stats.max_ms:.1f} ms (controller manager stamps, nominal '
        f'{1000.0 / rate_hz:.1f} ms)')
    if leading_zeros:
        lines.append(f'warn  start      {leading_zeros} leading messages are all zeros: a '
                     'placeholder pose, which #22 should not publish (report it there)')
    verdict = all(c.passed for c in criteria)
    lines.append(f'\nOVERALL: {"PASS" if verdict else "FAIL"}')
    return lines


def markdown_table(criteria):
    """Return the criteria as a Markdown table, to paste into the evidence document."""
    rows = ['| Criterion | Result | Detail |', '|---|---|---|']
    rows += [f'| {c.name} | {"PASS" if c.passed else "FAIL"} | {c.detail} |' for c in criteria]
    return '\n'.join(rows) + '\n'


def parse_reference(text):
    """Parse "a,b,c,d,e,f" (degrees, as pymycobot get_angles prints them) into six floats."""
    values = [float(value) for value in text.replace('[', '').replace(']', '').split(',')]
    if len(values) != 6:
        raise argparse.ArgumentTypeError('expected six comma-separated angles in degrees')
    return values


def main(argv=None):
    """Read the bag, check it, print the report and return the exit code."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 directory recorded from /joint_states')
    parser.add_argument('--rate', type=float, required=True,
                        help='nominal rate in Hz: the value fixed by #20')
    parser.add_argument('--topic', default='/joint_states')
    parser.add_argument('--urdf', help='URDF file (default: the installed positronic_description)')
    parser.add_argument('--min-duration', type=float, default=600.0, help='seconds (default 600)')
    parser.add_argument('--rate-tolerance', type=float, default=0.05,
                        help='accepted relative error of the mean rate (default 0.05)')
    parser.add_argument('--reference-deg', type=parse_reference,
                        help='pymycobot get_angles taken just before the run, e.g. "0.1,-2,..."')
    parser.add_argument('--report', help='also write the criteria as a Markdown table here')
    args = parser.parse_args(argv)

    with open(args.urdf or default_urdf_path(), encoding='utf-8') as urdf:
        limits = urdf_joint_limits(urdf.read())
    samples = read_bag(args.bag, args.topic)
    if len(samples) < 2:
        print(f'Only {len(samples)} messages on {args.topic} in {args.bag}: nothing to check.')
        return 1

    criteria = evaluate(samples, args.rate, limits, min_duration_s=args.min_duration,
                        rate_tolerance=args.rate_tolerance, reference_deg=args.reference_deg)
    header_stats = period_stats([sample.header_ns for sample in samples])
    print(f'{args.bag}: {len(samples)} messages on {args.topic}\n')
    print('\n'.join(report_lines(criteria, header_stats, leading_zero_samples(samples),
                                 args.rate)))
    if args.report:
        with open(args.report, 'w', encoding='utf-8') as report:
            report.write(markdown_table(criteria))
    return 0 if all(criterion.passed for criterion in criteria) else 1


if __name__ == '__main__':
    sys.exit(main())