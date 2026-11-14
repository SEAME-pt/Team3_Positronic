# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import math

from builtin_interfaces.msg import Duration
from conftest import ARM_CONTROLLER, CONTROLLER_TIMEOUT_S, JOINTS, MAX_DRIFT, max_drift
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint


def test_hold_trajectory(arm):
    assert arm.wait_controller_active(ARM_CONTROLLER, CONTROLLER_TIMEOUT_S), \
        f'{ARM_CONTROLLER} not active after {CONTROLLER_TIMEOUT_S:.0f} s'

    reference = arm.samples[0][1]
    pose = [arm.samples[-1][1][name] for name in JOINTS]
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=pose, time_from_start=Duration(sec=1)),
        JointTrajectoryPoint(positions=pose, time_from_start=Duration(sec=2)),
    ]

    client = ActionClient(
        arm.node, FollowJointTrajectory, f'/{ARM_CONTROLLER}/follow_joint_trajectory')
    assert client.wait_for_server(timeout_sec=10.0), 'the trajectory action is not up'
    handle = arm.wait_future(client.send_goal_async(goal), 10.0, 'sending the goal')
    assert handle.accepted, 'the goal was rejected'
    result = arm.wait_future(handle.get_result_async(), 15.0, 'the trajectory')
    code = result.result.error_code
    assert code == FollowJointTrajectory.Result.SUCCESSFUL, f'error_code {code}'

    samples = arm.record(2.0)

    drift = max_drift(samples, reference)
    assert drift <= MAX_DRIFT, f'the arm moved {math.degrees(drift):.2f} deg'
