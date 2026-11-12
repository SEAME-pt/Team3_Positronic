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
#   Write the recorder for the real-time profile run of #39, to run in the Jetson container
#   while driver.launch.py is up. write() is called once per controller_manager cycle and
#   joint_state_broadcaster publishes in that same cycle with header.stamp set to the cycle
#   time, so one row per /joint_states message is one row per write(). Subscribe to
#   /joint_states, keep every message (no de-duplication, unlike joint_states_rate.py), and
#   write cycles.csv with the arrival time, the cycle stamp and the six positions. With
#   --steps N, also step one joint: wait --settle seconds, read the pose the arm is resting
#   at, then publish N commands on /forward_position_controller/commands, --step-interval
#   apart, with that joint alternating between pose+delta and pose, so the arm stays inside a
#   band of delta rad; log each publish time to steps.csv and return the arm to the pose it
#   started from at the end. Keep the step vectors in a pure function, keep argument parsing
#   and the spin out of the tested code, and fail early when nothing publishes /joint_states
#   or nothing subscribes to the command topic.

"""Record one row per control loop cycle, and optionally step a joint (#39)."""

# One /joint_states message is one controller_manager cycle: joint_state_broadcaster publishes
# once per update, with header.stamp set to the cycle time, and write() runs in the same cycle.
# The interval between two cycle stamps is therefore the interval between two write() calls.
# What this does not measure is where inside a cycle write() runs; see rt_profile_v1.md.
#
# Run it inside the Jetson container while driver.launch.py is running:
#   python3 docs/03_verification/rt_profile/tools/rt_record.py --out /ws/log/rt/idle --duration 630
#
# Read-only without --steps: it only subscribes, so nothing it does can move the arm.

import argparse
import csv
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

COMMAND_TOPIC = '/forward_position_controller/commands'
STATE_TOPIC = '/joint_states'
NUM_JOINTS = 6
NS_PER_S = 10 ** 9
# How long to wait for the driver and for the controller before giving up.
STARTUP_TIMEOUT_S = 15.0


def stamp_ns(stamp):
    """Return a message stamp as nanoseconds."""
    return stamp.sec * NS_PER_S + stamp.nanosec


def step_targets(baseline, joint, delta, count):
    """
    Return `count` command vectors that step `joint` out and back.

    Even steps command baseline + delta, odd steps command baseline again, so the arm swings
    inside a band of `delta` rad around the pose it was resting at and ends there when
    `count` is even. Every vector carries all six joints: the controller writes all of them.
    """
    targets = []
    for index in range(count):
        target = list(baseline)
        target[joint] = baseline[joint] + (delta if index % 2 == 0 else 0.0)
        targets.append(target)
    return targets


def write_cycles(path, rows):
    """Write one row per cycle: arrival time, cycle stamp, six positions."""
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['receive_ns', 'header_ns'] + [f'pos{i}' for i in range(NUM_JOINTS)])
        for receive_ns, header_ns, positions in rows:
            writer.writerow([receive_ns, header_ns] + [f'{value:.6f}' for value in positions])


def write_steps(path, steps):
    """Write one row per commanded step: when it was published and what it asked for."""
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['index', 'publish_ns', 'joint', 'target_rad', 'baseline_rad'])
        for index, publish_ns, joint, target_rad, baseline_rad in steps:
            writer.writerow([index, publish_ns, joint, f'{target_rad:.6f}',
                             f'{baseline_rad:.6f}'])


class CycleRecorder(Node):
    """Keeps every /joint_states message of the run in memory."""

    def __init__(self):
        super().__init__('rt_record')
        self.rows = []  # (arrival in ns, cycle stamp in ns, six positions)
        self.create_subscription(JointState, STATE_TOPIC, self.on_message, 10)

    def on_message(self, message):
        """Store the cycle this message was published in."""
        self.rows.append((
            self.get_clock().now().nanoseconds,
            stamp_ns(message.header.stamp),
            tuple(message.position[:NUM_JOINTS])))

    def latest_positions(self):
        """Return the positions of the last message, or None when none arrived yet."""
        return self.rows[-1][2] if self.rows else None

    def wait_for_driver(self, timeout_s=STARTUP_TIMEOUT_S):
        """Spin until the first message arrives. False when the driver never published."""
        deadline = time.monotonic() + timeout_s
        while not self.rows and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return bool(self.rows)


def parse_arguments(argv):
    """Return the parsed command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, help='folder for cycles.csv and steps.csv')
    parser.add_argument('--duration', type=float, default=630.0,
                        help='seconds to record (default 630, the 10 minute run plus margin)')
    parser.add_argument('--steps', type=int, default=0,
                        help='commanded steps for the latency measurement (default 0: no motion)')
    parser.add_argument('--joint', type=int, default=0,
                        help='index of the joint to step, 0 is J1 at the base (default 0)')
    parser.add_argument('--delta-rad', type=float, default=0.0873,
                        help='how far the stepped joint moves, in rad (default 0.0873, 5 deg)')
    parser.add_argument('--step-interval', type=float, default=3.0,
                        help='seconds between two steps (default 3)')
    parser.add_argument('--settle', type=float, default=5.0,
                        help='seconds of recording before the first step (default 5)')
    return parser.parse_args(argv)


def needed_duration(args):
    """Return the recording time the run needs, stretched when the steps would not fit."""
    if args.steps <= 0:
        return args.duration
    return max(args.duration, args.settle + args.steps * args.step_interval + 3.0)


def send_steps(node, publisher, args, recorded):
    """
    Record for the whole duration, publishing the steps along the way.

    `recorded` is filled with one (index, publish time, joint, target, baseline) per step.
    Returns the baseline pose, or None when no step was sent.
    """
    deadline = time.monotonic() + needed_duration(args)
    next_step_at = time.monotonic() + args.settle
    baseline = None
    targets = []
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if publisher is None or len(recorded) >= args.steps:
            continue
        if time.monotonic() < next_step_at:
            continue
        if baseline is None:
            baseline = node.latest_positions()
            targets = step_targets(baseline, args.joint, args.delta_rad, args.steps)
            node.get_logger().info(f'baseline {[round(v, 4) for v in baseline]} rad')
        target = targets[len(recorded)]
        # Stamped before the publish, so the latency reported is never shorter than the real one.
        publish_ns = node.get_clock().now().nanoseconds
        publisher.publish(Float64MultiArray(data=target))
        recorded.append((len(recorded), publish_ns, args.joint, target[args.joint],
                         baseline[args.joint]))
        next_step_at += args.step_interval
    return baseline


def main(argv=None):  # pragma: no cover
    """Record the run, write the csv files and return the exit code."""
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = CycleRecorder()
    publisher = None
    steps = []
    code = 0
    try:
        if not node.wait_for_driver():
            node.get_logger().error(
                f'nothing published {STATE_TOPIC} within {STARTUP_TIMEOUT_S:.0f} s. '
                'Start driver.launch.py first.')
            return 1
        if args.steps > 0:
            publisher = node.create_publisher(Float64MultiArray, COMMAND_TOPIC, 10)
            deadline = time.monotonic() + STARTUP_TIMEOUT_S
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if publisher.get_subscription_count() == 0:
                node.get_logger().error(
                    f'nothing subscribes to {COMMAND_TOPIC}. Spawn forward_position_controller.')
                return 1
        node.get_logger().info(f'recording {needed_duration(args):.0f} s into {args.out}')
        baseline = send_steps(node, publisher, args, steps)
        if publisher is not None and baseline is not None:
            # Leave the arm where it was found, whatever the last step commanded.
            publisher.publish(Float64MultiArray(data=list(baseline)))
            time.sleep(1.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    rows = node.rows
    node.destroy_node()
    rclpy.try_shutdown()

    write_cycles(os.path.join(args.out, 'cycles.csv'), rows)
    print(f'{len(rows)} cycles written to {args.out}/cycles.csv')
    if steps:
        write_steps(os.path.join(args.out, 'steps.csv'), steps)
        print(f'{len(steps)} steps written to {args.out}/steps.csv')
    if len(rows) < 2:
        print('fewer than two cycles recorded: nothing to measure.', file=sys.stderr)
        return 1
    return code


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
