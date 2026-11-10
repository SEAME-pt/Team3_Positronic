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

# AI-generated (Claude, Anthropic), from a pseudocode plan and loading code by the author.
# Prompt:
#   Write the sender for #37. Load trajectory.yaml, refuse it before ROS starts if the joint
#   names differ from the URDF, a position is not 10 deg inside the URDF limits, or a segment's
#   peak speed (1.9 x |dtheta| / dt) is above 30 % of the URDF velocity. Then wait 5 s for
#   /arm_controller/follow_joint_trajectory, send the home pose, wait for its result and 2 s,
#   send the whole trajectory as one goal, wait for its result with a timeout taken from the
#   file, print the error code and exit 0 or non-zero. Ctrl-C or a timeout cancels the goal.
#   Then: cap the allowed speed at 15 deg/s, the most the real arm follows within the 5 deg
#   trajectory tolerance of arm_controller (#33, docs/03_verification/jtc_on_the_arm), for both
#   the check and the home move.
"""Check trajectory.yaml against the URDF and the arm's speed, then run it on arm_controller."""

# Steps:
#   1. load trajectory.yaml (positions in rad, time_from_start in s)
#   2. refuse it before ROS starts: joint names, URDF limits minus 10 deg, peak speed
#   3. wait up to 5 s for the arm_controller action server
#   4. read the current pose and send the home pose, slow enough for the speed limit
#
# Speed limit per joint: the smaller of 30 % of the URDF velocity (#37) and 15 deg/s (#33).
# 30 % is 0.785 rad/s (45 deg/s), three times what the real arm follows: above about 15 deg/s
# a joint lags more than the 5 deg trajectory tolerance and arm_controller aborts the goal.
#   5. wait for its result, then 2 s so the arm settles
#   6. send the whole trajectory as one goal, wait for its result, print the error code
#   Ctrl-C, or a result that never comes, cancels the running goal before exiting.
#
# The same script runs in both worlds: ROS_DOMAIN_ID picks which one (record_run.sh checks it).
# Run it inside the ROS 2 container, where rclpy and control_msgs exist:
#   python3 docs/03_verification/sim_to_real/tools/run_trajectory.py
#   python3 docs/03_verification/sim_to_real/tools/run_trajectory.py --check-only  # no ROS
# Exit code 0 when the trajectory ran to the end, 1 otherwise.

import argparse
import math
import os
import sys
import time
import xml.etree.ElementTree as ElementTree

import yaml

ACTION_NAME = '/arm_controller/follow_joint_trajectory'
SPEED_FRACTION = 0.30  # #37: "motion on hardware at 30 % speed"
ARM_MAX_SPEED = math.radians(15.0)  # 0.262 rad/s, #33: "the working limit" on the real arm
PEAK_FACTOR = 1.9  # peak / average speed of a smooth segment that starts and stops at rest
LIMIT_MARGIN = math.radians(10.0)  # every position at least 10 deg inside the URDF limits
HOME_MIN_DURATION_S = 4.0
SETTLE_S = 2.0  # after home: the result says the plan ended, not that the servos stopped
SERVER_TIMEOUT_S = 5.0
RESULT_MARGIN_S = 5.0  # the real arm lags the plan; wait this much past the last point
NS_PER_S = 1_000_000_000

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.normpath(os.path.join(TOOLS_DIR, '..', '..', '..', '..'))
DEFAULT_TRAJECTORY = os.path.join(TOOLS_DIR, 'trajectory.yaml')
DEFAULT_URDF = os.path.join(REPO_DIR, 'src', 'positronic_description', 'urdf',
                            'mycobot_280_jn_sim', 'mycobot_280_jn.urdf')


# --- Pure functions: no ROS, tested with pytest -------------------------------------------

def load_trajectory(text):
    """Parse trajectory.yaml into {joint_names, home, points: [(positions, seconds)]}."""
    data = yaml.safe_load(text)
    trajectory = {
        'joint_names': list(data['joint_names']),
        'home': [float(value) for value in data['home']],
        'points': [([float(value) for value in point['positions']],
                    float(point['time_from_start']))
                   for point in data['points']],
    }
    return trajectory


def urdf_joint_limits(urdf_text):
    """Return {joint name: (lower, upper, velocity)} of the revolute joints of a URDF.

    Same parsing as joint_states_rate/tools/check_joint_states.py, plus the velocity limit.
    """
    limits = {}
    for joint in ElementTree.fromstring(urdf_text).findall('joint'):
        if joint.get('type') == 'revolute':
            limit = joint.find('limit')
            limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')),
                                         float(limit.get('velocity')))
    return limits


def labelled_poses(trajectory):
    """Return [(label, positions)]: home, then every point, for the error messages."""
    points = trajectory['points']
    return ([('home', trajectory['home'])]
            + [(f'point {i + 1}', positions) for i, (positions, _) in enumerate(points)])


def check_shape(trajectory):
    """Return problems with the file's shape: list lengths, times, numbers."""
    problems = []
    count = len(trajectory['joint_names'])
    for label, positions in labelled_poses(trajectory):
        if len(positions) != count:
            problems.append(f'{label}: {len(positions)} positions for {count} joint names')
        elif not all(math.isfinite(value) for value in positions):
            problems.append(f'{label}: a position is not a finite number')
    previous = 0.0
    for i, (_, seconds) in enumerate(trajectory['points']):
        if not seconds > previous:
            problems.append(
                f'point {i + 1}: time_from_start {seconds} s is not after {previous} s')
        previous = seconds
    if not trajectory['points']:
        problems.append('no points')
    return problems


def check_names(joint_names, limits):
    """Return problems if the names are not exactly the URDF's revolute joints, in any order."""
    if sorted(joint_names) == sorted(limits):
        return []
    problems = []
    duplicates = sorted({name for name in joint_names if joint_names.count(name) > 1})
    unknown = sorted(set(joint_names) - set(limits))
    missing = sorted(set(limits) - set(joint_names))
    if duplicates:
        problems.append(f'listed twice: {", ".join(duplicates)}')
    if unknown:
        problems.append(f'not in the URDF: {", ".join(unknown)}')
    if missing:
        problems.append(f'missing: {", ".join(missing)}')
    return problems


def check_positions(trajectory, limits, margin=LIMIT_MARGIN):
    """Return problems for every position less than margin inside its URDF limits."""
    problems = []
    for label, positions in labelled_poses(trajectory):
        for name, value in zip(trajectory['joint_names'], positions):
            lower, upper, _ = limits[name]
            if not lower + margin <= value <= upper - margin:
                problems.append(
                    f'{label} {name}: {math.degrees(value):.1f} deg, allowed '
                    f'{math.degrees(lower + margin):.1f} to {math.degrees(upper - margin):.1f}')
    return problems


def segment_peaks(trajectory):
    """Return [(segment label, joint, peak rad/s)] for every joint of every segment.

    Segment 1 starts at home: the script sends home first, so the trajectory starts there.
    """
    peaks = []
    previous_positions, previous_seconds = trajectory['home'], 0.0
    for i, (positions, seconds) in enumerate(trajectory['points']):
        duration = seconds - previous_seconds
        label = f'{"home" if i == 0 else f"point {i}"} -> point {i + 1}'
        for name, start, end in zip(trajectory['joint_names'], previous_positions, positions):
            peaks.append((label, name, PEAK_FACTOR * abs(end - start) / duration))
        previous_positions, previous_seconds = positions, seconds
    return peaks


def allowed_speed(name, limits):
    """Return the highest peak allowed for a joint, rad/s: 30 % of the URDF, at most 15 deg/s."""
    return min(SPEED_FRACTION * limits[name][2], ARM_MAX_SPEED)


def check_speeds(trajectory, limits):
    """Return problems for every segment whose peak is above the joint's allowed speed."""
    problems = []
    for label, name, peak in segment_peaks(trajectory):
        allowed = allowed_speed(name, limits)
        if peak > allowed:
            problems.append(f'{label} {name}: peak {peak:.3f} rad/s, allowed {allowed:.3f}')
    return problems


def check_trajectory(trajectory, limits):
    """Run every check in order; the later ones only make sense once the earlier ones pass."""
    problems = check_shape(trajectory)
    if problems:
        return problems
    problems = check_names(trajectory['joint_names'], limits)
    if problems:
        return problems
    return check_positions(trajectory, limits) + check_speeds(trajectory, limits)


def home_duration(current, home, joint_names, limits):
    """Seconds for the move from the current pose to home, so its peak stays allowed."""
    needed = max(PEAK_FACTOR * abs(end - start) / allowed_speed(name, limits)
                 for name, start, end in zip(joint_names, current, home))
    return max(HOME_MIN_DURATION_S, math.ceil(needed * 10.0) / 10.0)


def split_seconds(seconds):
    """Split seconds into (sec, nanosec), the two integers of builtin_interfaces/Duration."""
    nanoseconds = round(seconds * NS_PER_S)
    return nanoseconds // NS_PER_S, nanoseconds % NS_PER_S


# --- ROS part ------------------------------------------------------------------------------

def build_goal(joint_names, points):
    """Build one FollowJointTrajectory goal from [(positions, seconds)]: one goal, all points."""
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joint_names)
    for positions, seconds in points:
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec, point.time_from_start.nanosec = split_seconds(seconds)
        goal.trajectory.points.append(point)
    return goal


class Sender:
    """Sends goals to arm_controller and remembers the running one, so it can be cancelled."""

    def __init__(self, node):
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        self.node = node
        self.client = ActionClient(node, FollowJointTrajectory, ACTION_NAME)
        self.goal_handle = None  # the "ticket" of the goal that is running, if any

    def wait_for_server(self):
        return self.client.wait_for_server(timeout_sec=SERVER_TIMEOUT_S)

    def current_pose(self, joint_names, timeout_s=SERVER_TIMEOUT_S):
        """Return the positions of joint_names from one /joint_states message, or None."""
        import rclpy
        from rclpy.task import Future
        from sensor_msgs.msg import JointState

        future = Future()

        def on_message(message):
            by_name = dict(zip(message.name, message.position))
            if not future.done() and all(name in by_name for name in joint_names):
                future.set_result([by_name[name] for name in joint_names])

        subscription = self.node.create_subscription(JointState, '/joint_states', on_message, 10)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        self.node.destroy_subscription(subscription)
        return future.result() if future.done() else None

    def run(self, goal, result_timeout_s):
        """Send goal, wait for its result; return True when the controller reports success."""
        import rclpy
        from control_msgs.action import FollowJointTrajectory

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=SERVER_TIMEOUT_S)
        if not send_future.done():
            print(f'No answer from {ACTION_NAME} to the goal within {SERVER_TIMEOUT_S:.0f} s.')
            return False
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print('Goal rejected by arm_controller (check the joint names and its state).')
            return False

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=result_timeout_s)
        if not result_future.done():
            print(f'No result within {result_timeout_s:.1f} s: cancelling the goal.')
            self.cancel()
            return False
        self.goal_handle = None

        result = result_future.result().result
        print(f'error_code {result.error_code} {result.error_string}'.rstrip())
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def cancel(self):
        """Ask the controller to stop the running goal, if there is one."""
        import rclpy

        if self.goal_handle is None:
            return
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=2.0)
        print('Cancel sent.' if cancel_future.done() else 'Cancel not confirmed: use the switch.')
        self.goal_handle = None


def send(trajectory, limits):
    """Run home, settle, then the whole trajectory. Return the exit code."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    # Without this, rclpy's own Ctrl-C handler shuts ROS down before we can send the cancel.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('run_trajectory')
    sender = Sender(node)
    joint_names = trajectory['joint_names']
    try:
        if not sender.wait_for_server():
            print(f'{ACTION_NAME} not found within {SERVER_TIMEOUT_S:.0f} s: is the '
                  'controller running, and is ROS_DOMAIN_ID right?')
            return 1

        current = sender.current_pose(joint_names)
        if current is None:
            print('No /joint_states with every joint: cannot size the move to home.')
            return 1
        seconds = home_duration(current, trajectory['home'], joint_names, limits)
        print(f'Home in {seconds:.1f} s')
        home_goal = build_goal(joint_names, [(trajectory['home'], seconds)])
        if not sender.run(home_goal, seconds + RESULT_MARGIN_S):
            return 1
        time.sleep(SETTLE_S)

        duration = trajectory['points'][-1][1]
        print(f'Trajectory: {len(trajectory["points"])} points in {duration:.1f} s')
        goal = build_goal(joint_names, trajectory['points'])
        return 0 if sender.run(goal, duration + RESULT_MARGIN_S) else 1
    except KeyboardInterrupt:
        print('\nCtrl-C: cancelling the goal.')
        sender.cancel()
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    """Check the trajectory, then send it unless --check-only. Return the exit code."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', default=DEFAULT_TRAJECTORY,
                        help='trajectory file (default: trajectory.yaml next to this script)')
    parser.add_argument('--urdf', default=DEFAULT_URDF,
                        help='URDF with the joint limits '
                             "(default: the repo's mycobot_280_jn.urdf)")
    parser.add_argument('--check-only', action='store_true',
                        help='run the checks and print the peak speeds, send nothing')
    args = parser.parse_args(argv)

    with open(args.trajectory, encoding='utf-8') as trajectory_file:
        trajectory = load_trajectory(trajectory_file.read())
    with open(args.urdf, encoding='utf-8') as urdf:
        limits = urdf_joint_limits(urdf.read())

    problems = check_trajectory(trajectory, limits)
    if problems:
        print(f'REFUSED {args.trajectory}:')
        print('\n'.join(f'  {problem}' for problem in problems))
        return 1
    label, name, peak = max(segment_peaks(trajectory), key=lambda entry: entry[2])
    print(f'Checks pass. Highest peak: {peak:.3f} rad/s ({name}, {label}), allowed '
          f'{allowed_speed(name, limits):.3f} rad/s')
    if args.check_only:
        return 0
    return send(trajectory, limits)


if __name__ == '__main__':
    sys.exit(main())
