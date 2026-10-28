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
Compare one /joint_states message with the angles read by pymycobot just before.

The driver publishes radians, pymycobot get_angles prints degrees. One line per
joint: published (deg), reference (deg), difference. PASS when the worst
difference is below the tolerance. The arm must not move between the two readings.

    ros2 run positronic_bringup joint_states_check.py --reference-deg "17,-16,25,-10,1,0"
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Protocol order, J1 (base) to J6 (flange): the order of get_angles and of the URDF.
JOINTS = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]


def parse_reference(text):
    """Turn "a,b,c,d,e,f" (degrees, with or without brackets) into six floats."""
    values = [float(v) for v in text.replace('[', '').replace(']', '').split(',')]
    if len(values) != len(JOINTS):
        raise ValueError(f'expected {len(JOINTS)} angles, got {len(values)}')
    return values


def compare(names, positions_rad, reference_deg):
    """Return one (joint, published_deg, reference_deg, difference_deg) per joint, in order."""
    rows = []
    for joint, reference in zip(JOINTS, reference_deg):
        published = math.degrees(positions_rad[list(names).index(joint)])
        rows.append((joint, published, reference, abs(published - reference)))
    return rows


def report(rows, tolerance_deg):
    """Return the lines to print: one per joint and the verdict."""
    lines = [f'{joint:24} published {published:+8.2f} deg   reference {reference:+8.2f} deg'
             f'   diff {diff:5.2f}' for joint, published, reference, diff in rows]
    worst = max(diff for _, _, _, diff in rows)
    lines.append(f'{"PASS" if worst <= tolerance_deg else "FAIL"}: worst difference '
                 f'{worst:.2f} deg, tolerance {tolerance_deg} deg')
    return lines


def first_message(node, timeout_s):
    """Return the first /joint_states message within timeout_s, or None."""
    received = []
    node.create_subscription(JointState, '/joint_states', received.append, 10)
    end = time.time() + timeout_s
    while not received and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return received[0] if received else None


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--reference-deg', type=parse_reference, required=True,
                        help='the six angles printed by pymycobot get_angles, in degrees')
    parser.add_argument('--tolerance-deg', type=float, default=0.5)
    parser.add_argument('--timeout', type=float, default=5.0, help='seconds to wait for a message')
    args = parser.parse_args()

    rclpy.init()
    node = Node('joint_states_check')
    message = first_message(node, args.timeout)
    node.destroy_node()
    rclpy.try_shutdown()
    if message is None:
        print(f'FAIL: no /joint_states message in {args.timeout:g} s')
        return 1
    missing = [joint for joint in JOINTS if joint not in message.name]
    if missing:
        print(f'FAIL: joints missing from /joint_states: {missing}')
        return 1
    rows = compare(message.name, message.position, args.reference_deg)
    print('\n'.join(report(rows, args.tolerance_deg)))
    return 0 if max(diff for _, _, _, diff in rows) <= args.tolerance_deg else 1


if __name__ == '__main__':
    sys.exit(main())
