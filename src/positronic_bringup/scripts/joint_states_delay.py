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
Measure the delay of /joint_states on this machine.

For every message: delay = now on this machine - header.stamp. Prints the
average and the maximum every second, and at the end a summary against a
limit: messages, seconds, average, worst, how many above the limit and when.
Only valid if this machine and the Jetson have their clocks synchronised (NTP).

    ros2 run positronic_bringup joint_states_delay.py --duration 300 --out log/delay

Stops after --duration seconds, or with Ctrl+C. With --out, writes delays.csv
(one line per message: seconds since start, delay in ms) and summary.txt.
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


def delay_ms(stamp, now_ns):
    """Return how old a message is, in ms: the time `now_ns` minus its header stamp."""
    stamp_ns = stamp.sec * NS_PER_S + stamp.nanosec
    return (now_ns - stamp_ns) / NS_PER_MS


def summary(delays_ms):
    """Return the line printed every second for a list of delays, or None when it is empty."""
    if not delays_ms:
        return None
    n = len(delays_ms)
    return f'{n} msgs  avg {sum(delays_ms) / n:.1f} ms  max {max(delays_ms):.1f} ms'


def final_summary(samples, limit_ms):
    """
    Return the lines of the end-of-run summary and whether it passed.

    `samples` is a list of (seconds since start, delay in ms). Passes when every delay
    is at or below `limit_ms`. Without samples it fails: nothing was measured.
    """
    if not samples:
        return ['no messages received'], False
    delays = [delay for _, delay in samples]
    late = [(at, delay) for at, delay in samples if delay > limit_ms]
    worst_at, worst = max(samples, key=lambda sample: sample[1])
    lines = [
        f'{len(samples)} msgs in {samples[-1][0]:.0f} s',
        f'avg {sum(delays) / len(delays):.1f} ms  max {worst:.1f} ms at {worst_at:.1f} s',
        f'{len(late)} msgs above {limit_ms:g} ms',
    ]
    if late:
        first = ', '.join(f'{delay:.1f} ms at {at:.1f} s' for at, delay in late[:5])
        lines.append('first: ' + first)
    lines.append(f'{"PASS" if not late else "FAIL"}: limit {limit_ms:g} ms')
    return lines, not late


def write_results(out_dir, samples, lines):
    """Write delays.csv and summary.txt into `out_dir`, created if needed."""
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'delays.csv'), 'w', encoding='utf-8') as csv:
        csv.write('seconds,delay_ms\n')
        csv.writelines(f'{at:.3f},{delay:.3f}\n' for at, delay in samples)
    with open(os.path.join(out_dir, 'summary.txt'), 'w', encoding='utf-8') as text:
        text.write('\n'.join(lines) + '\n')


class JointStatesDelay(Node):

    def __init__(self):
        super().__init__('joint_states_delay')
        self.start = time.monotonic()
        self.delays_ms = []  # of the current second
        self.samples = []  # (seconds since start, delay in ms), of the whole run
        self.create_subscription(JointState, '/joint_states', self.on_message, 10)
        self.create_timer(1.0, self.report)

    def on_message(self, msg):
        delay = delay_ms(msg.header.stamp, self.get_clock().now().nanoseconds)
        self.delays_ms.append(delay)
        self.samples.append((time.monotonic() - self.start, delay))

    def report(self):
        line = summary(self.delays_ms)
        self.get_logger().info(line if line else 'no messages')
        self.delays_ms = []


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--duration', type=float, default=None,
                        help='seconds to measure, then stop (default: until Ctrl+C)')
    parser.add_argument('--limit-ms', type=float, default=100.0,
                        help='a delay above this fails the run (default 100, the #24 limit)')
    parser.add_argument('--out', help='folder for delays.csv and summary.txt')
    args = parser.parse_args()

    rclpy.init()
    node = JointStatesDelay()
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

    lines, passed = final_summary(samples, args.limit_ms)
    print('\n' + '\n'.join(lines))
    if args.out:
        write_results(args.out, samples, lines)
        print(f'written to {args.out}/delays.csv and summary.txt')
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
