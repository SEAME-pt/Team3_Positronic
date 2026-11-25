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
#   #120: the driver publishes driver/pose_age_ms, the age in ms of the pose read() publishes,
#   on /dynamic_joint_states through joint_state_broadcaster. Write a tool for the 60 s run on
#   the arm: subscribe to /dynamic_joint_states for --duration seconds, keep every pose_age_ms
#   value, and print the count, the NaN count, min, mean, p50, p99 and max as a Markdown table
#   for the README of positronic_hardware, optionally with the raw values in a CSV. Keep the
#   parsing and the statistics in pure functions, unit-tested without ROS.
"""Record driver/pose_age_ms from /dynamic_joint_states and summarise it (#120)."""

# Run it inside the ROS 2 container, next to the driver, while the arm moves:
#   python3 docs/03_verification/pose_age/tools/pose_age_stats.py --duration 60 --csv ages.csv
# Exit code 0 when at least one finite age was recorded, 1 otherwise.

import argparse
import math
import statistics
import sys

GPIO = 'driver'
INTERFACE = 'pose_age_ms'
TOPIC = '/dynamic_joint_states'


def find_age(joint_names, interface_values, gpio=GPIO, interface=INTERFACE):
    """
    Return the value of `gpio`/`interface` in one DynamicJointState, or None when absent.

    `joint_names` and `interface_values` are the two lists of the message: the entry of a name
    has `interface_names` and `values`, in the same order.
    """
    for name, entry in zip(joint_names, interface_values):
        if name != gpio:
            continue
        for key, value in zip(entry.interface_names, entry.values):
            if key == interface:
                return float(value)
    return None


def percentile(sorted_values, fraction):
    """Return the value at `fraction` (0 to 1) of sorted, non-empty values, nearest rank."""
    rank = max(1, math.ceil(fraction * len(sorted_values)))
    return sorted_values[rank - 1]


def summarize(ages):
    """
    Return the statistics of the recorded ages, in ms, as a dict.

    NaN ages (no pose yet) are counted apart and left out of the statistics. min, mean, p50,
    p99 and max are None when no finite age was recorded.
    """
    finite = sorted(age for age in ages if math.isfinite(age))
    summary = {
        'count': len(ages),
        'nan': sum(1 for age in ages if math.isnan(age)),
        'min': None,
        'mean': None,
        'p50': None,
        'p99': None,
        'max': None,
    }
    if finite:
        summary.update(
            min=finite[0],
            mean=statistics.fmean(finite),
            p50=percentile(finite, 0.50),
            p99=percentile(finite, 0.99),
            max=finite[-1],
        )
    return summary


def markdown(summary, duration_s):
    """Return the summary as the Markdown table of the README, values in ms."""
    def cell(value):
        return '-' if value is None else f'{value:.1f}'

    rows = [
        '| Samples | NaN | min (ms) | mean (ms) | p50 (ms) | p99 (ms) | max (ms) |',
        '|---|---|---|---|---|---|---|',
        f"| {summary['count']} in {duration_s:.0f} s | {summary['nan']} | "
        f"{cell(summary['min'])} | {cell(summary['mean'])} | {cell(summary['p50'])} | "
        f"{cell(summary['p99'])} | {cell(summary['max'])} |",
    ]
    return '\n'.join(rows)


def record(duration_s):
    """Subscribe to TOPIC for `duration_s` seconds and return the ages seen, in order."""
    import rclpy  # Only here: the pure functions above run without ROS.
    from control_msgs.msg import DynamicJointState

    ages = []

    def on_message(message):
        age = find_age(message.joint_names, message.interface_values)
        if age is not None:
            ages.append(age)

    rclpy.init()
    node = rclpy.create_node('pose_age_stats')
    node.create_subscription(DynamicJointState, TOPIC, on_message, 100)
    end = node.get_clock().now().nanoseconds + int(duration_s * 1e9)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return ages


def main(argv=None):
    """Record, print the table, and write the raw ages when asked."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0, help='seconds (default 60)')
    parser.add_argument('--csv', help='also write every age, one per line, to this file')
    args = parser.parse_args(argv)

    ages = record(args.duration)
    if args.csv:
        with open(args.csv, 'w') as file:
            file.write('pose_age_ms\n')
            file.writelines(f'{age}\n' for age in ages)
    summary = summarize(ages)
    print(markdown(summary, args.duration))
    if summary['min'] is None:
        print(f'No finite {GPIO}/{INTERFACE} on {TOPIC}: is the driver up, with #120?')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
