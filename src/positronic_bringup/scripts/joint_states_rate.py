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

"""
Measure how often /joint_states carries a new pose, and the longest gap between two.

joint_state_broadcaster publishes once per controller cycle, so the topic rate is always
the update rate even when the driver has nothing new. This script ignores repeats and keeps
only the messages whose positions differ from the previous ones. For every new pose:
interval = its header.stamp - the previous new pose's header.stamp. Prints the count of new
poses and the longest interval every second, and at the end a summary: new poses, seconds,
min, median, p90 and max interval.

    ros2 run positronic_bringup joint_states_rate.py --duration 120 --out log/rate

A long interval is not a fault by itself: a standing arm reports the same pose over and over,
and the arm answers with the position it already had now and then. Whether the link went
quiet is measured on the serial line, see docs/03_verification/write_sweep.

Stops after --duration seconds, or with Ctrl+C. With --out, writes intervals.csv (one line
per new pose: seconds since start, interval in ms) and summary.txt.
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


NS_PER_S = 10**9
NS_PER_MS = 10**6


def stamp_ns(stamp):
    """Return a header stamp as nanoseconds."""
    return stamp.sec * NS_PER_S + stamp.nanosec


def summary(intervals_ms):
    """Return the line printed every second for a list of intervals, or None when it is empty."""
    if not intervals_ms:
        return None
    return f'{len(intervals_ms)} new poses  max gap {max(intervals_ms):.1f} ms'


def final_summary(samples):
    """
    Return the lines of the end-of-run summary.

    `samples` is a list of (seconds since start, interval in ms). Without samples the only
    line says so: nothing was measured.
    """
    if not samples:
        return ['no new poses received']
    intervals = sorted(interval for _, interval in samples)
    worst_at, worst = max(samples, key=lambda sample: sample[1])
    median = intervals[len(intervals) // 2]
    p90 = intervals[int(len(intervals) * 0.9)]
    return [
        f'{len(samples)} new poses in {samples[-1][0]:.0f} s',
        f'interval min {intervals[0]:.1f} ms  median {median:.1f} ms  p90 {p90:.1f} ms'
        f'  max {worst:.1f} ms at {worst_at:.1f} s',
    ]


def write_results(out_dir, samples, lines):
    """Write intervals.csv and summary.txt into `out_dir`, created if needed."""
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'intervals.csv'), 'w', encoding='utf-8') as csv:
        csv.write('seconds,interval_ms\n')
        csv.writelines(f'{at:.3f},{interval:.3f}\n' for at, interval in samples)
    with open(os.path.join(out_dir, 'summary.txt'), 'w', encoding='utf-8') as text:
        text.write('\n'.join(lines) + '\n')


class JointStatesRate(Node):
    """Subscribe to /joint_states and keep the interval between poses that actually changed."""

    def __init__(self):
        """Subscribe, and report once a second."""
        super().__init__('joint_states_rate')
        self.start = time.monotonic()
        self.last_positions = None  # of the previous new pose
        self.last_stamp_ns = None
        self.intervals_ms = []  # of the current second
        self.samples = []  # (seconds since start, interval in ms), of the whole run
        self.create_subscription(JointState, '/joint_states', self.on_message, 10)
        self.create_timer(1.0, self.report)

    def on_message(self, msg):
        """Keep the interval since the previous new pose; a repeat of the last pose is ignored."""
        positions = list(msg.position)
        if positions == self.last_positions:
            return  # the driver had nothing new: a repeat of the last pose
        now_ns = stamp_ns(msg.header.stamp)
        if self.last_stamp_ns is not None:
            interval = (now_ns - self.last_stamp_ns) / NS_PER_MS
            self.intervals_ms.append(interval)
            self.samples.append((time.monotonic() - self.start, interval))
        self.last_positions = positions
        self.last_stamp_ns = now_ns

    def report(self):
        """Log the second that just ended and start a new one."""
        line = summary(self.intervals_ms)
        self.get_logger().info(line if line else 'no new poses')
        self.intervals_ms = []


def main():
    """Parse the arguments, spin for --duration or until Ctrl+C, print and write the summary."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duration', type=float, default=None,
                        help='seconds to measure, then stop (default: until Ctrl+C)')
    parser.add_argument('--out', help='folder for intervals.csv and summary.txt')
    args = parser.parse_args()

    rclpy.init()
    node = JointStatesRate()
    try:
        if args.duration is None:
            rclpy.spin(node)
        else:
            end = time.monotonic() + args.duration
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    samples = node.samples
    node.destroy_node()
    rclpy.try_shutdown()

    lines = final_summary(samples)
    print('\n' + '\n'.join(lines))
    if args.out:
        write_results(args.out, samples, lines)
        print(f'written to {args.out}/intervals.csv and summary.txt')
    return 0


if __name__ == '__main__':
    sys.exit(main())
