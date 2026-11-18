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

"""#124: send T2 goals from one persistent node, no new DDS participant per goal."""

# python3 goals_node.py --trajectory .../T2.yaml --period 23 --duration 600 --log goals.txt
#
# No T0: the arm must already rest at the first point of the trajectory (the T0 start pose),
# or arm_controller aborts the first goal on its path tolerance.
import argparse
import sys
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rosidl_runtime_py import set_message_fields
import yaml

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}


def wait(node, future, timeout):
    """Spin until the future completes, return its result or None on timeout."""
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        return None
    return future.result()


def send(node, client, goal, timeout):
    """Send one goal and return how it ended: succeeded, aborted, canceled, rejected or timeout."""
    handle = wait(node, client.send_goal_async(goal), timeout)
    if handle is None:
        return 'timeout'
    if not handle.accepted:
        return 'rejected'
    result = wait(node, handle.get_result_async(), timeout)
    if result is None:
        return 'timeout'
    return STATUS_NAMES.get(result.status, f'status {result.status}')


def main():
    """Send the goal every period until the duration ends, log each send and result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', required=True,
                        help='goal yaml, as ros2 action send_goal takes it')
    parser.add_argument('--period', type=float, default=23.0, help='seconds between two goals')
    parser.add_argument('--duration', type=float, default=600.0, help='seconds to keep sending')
    parser.add_argument('--timeout', type=float, default=60.0, help='seconds to wait for a result')
    parser.add_argument('--log', required=True, help='file for the send and result lines')
    args = parser.parse_args()

    goal = FollowJointTrajectory.Goal()
    with open(args.trajectory) as handle:
        set_message_fields(goal, yaml.safe_load(handle))

    rclpy.init()
    node = rclpy.create_node('goals_node_124')
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    try:
        if not client.wait_for_server(timeout_sec=15.0):
            print('no action server', file=sys.stderr)
            return 1
        with open(args.log, 'a', buffering=1) as log:
            n = 0
            start = time.monotonic()
            try:
                while n * args.period < args.duration:
                    time.sleep(max(0.0, start + n * args.period - time.monotonic()))
                    n += 1
                    log.write(f'goal {n} T2 {time.time():.9f}\n')
                    status = send(node, client, goal, args.timeout)
                    log.write(f'result {n} status {status} {time.time():.9f}\n')
            finally:
                log.write(f'goals_node end goals {n}\n')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
