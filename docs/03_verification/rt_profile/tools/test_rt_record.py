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
#   #171: unit tests for rt_record.py with a /joint_states message that has the gripper mock
#   first (as since #151, velocity NaN): the six arm joints come out J1 first, in the
#   controller's order, whatever the order of the message; the first step built from them
#   moves one joint only; a missing arm joint or a field that does not match the names raises
#   instead of guessing, also through on_message(). No arm, no running ROS graph.

"""Unit tests for rt_record.py (#171): python3 -m pytest docs/03_verification/rt_profile/tools."""

import math
import random
from types import SimpleNamespace

import pytest
import rt_record as rr
from sensor_msgs.msg import JointState

# The fake backend's pose of the #133 spike, J1..J6, and the gripper mock's value.
POSE = (0.2822, 0.0061, -0.0106, -0.2330, 0.0075, -0.0106)
GRIPPER = 0.15


def gripper_first(pose=POSE):
    """Return /joint_states as published since #151: the gripper mock first, its speed NaN."""
    return JointState(
        name=['gripper_controller', *rr.ARM_JOINTS],
        position=[GRIPPER, *pose],
        velocity=[math.nan] + [0.0] * rr.NUM_JOINTS)


def test_gripper_first_message_gives_the_arm_joints_in_order():
    """Check that the gripper is dropped and J1..J6 come out in the controller's order."""
    message = gripper_first()
    assert rr.arm_values(message.name, message.position) == POSE


def test_any_order_gives_the_arm_joints_in_order():
    """Check that the order of the message does not matter."""
    pairs = list(zip(['gripper_controller', *rr.ARM_JOINTS], [GRIPPER, *POSE]))
    random.Random(171).shuffle(pairs)
    names, positions = zip(*pairs)
    assert rr.arm_values(names, positions) == POSE


def test_first_step_moves_one_joint_only():
    """Check the #171 failure: step 1 is the current pose with only the stepped joint changed."""
    message = gripper_first()
    baseline = rr.arm_values(message.name, message.position)
    for joint in range(rr.NUM_JOINTS):
        first, second = rr.step_targets(baseline, joint, 0.0873, 2)
        changed = [i for i in range(rr.NUM_JOINTS) if first[i] != POSE[i]]
        assert changed == [joint]
        assert math.isclose(first[joint], POSE[joint] + 0.0873)
        assert tuple(second) == POSE


def test_missing_arm_joint_raises():
    """Check that a message without one of the six joints stops the run."""
    names = ['gripper_controller', *rr.ARM_JOINTS[:5]]
    with pytest.raises(ValueError, match='joint6output_to_joint6'):
        rr.arm_values(names, [GRIPPER, *POSE[:5]])


def test_old_message_without_names_raises():
    """Check that positions without names are not taken by index."""
    with pytest.raises(ValueError):
        rr.arm_values([], [])
    with pytest.raises(ValueError):
        rr.arm_values([], list(POSE))


def test_field_shorter_than_the_names_raises():
    """Check that positions that do not match the names are refused."""
    with pytest.raises(ValueError, match='7 names and 6 values'):
        rr.arm_values(['gripper_controller', *rr.ARM_JOINTS], list(POSE))


def test_on_message_stores_the_arm_joints():
    """Check the subscriber callback: one row with the arm joints, the gripper ignored."""
    node = SimpleNamespace(rows=[], get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=5)))
    message = gripper_first()
    message.header.stamp.sec = 2
    message.header.stamp.nanosec = 7
    rr.CycleRecorder.on_message(node, message)
    assert node.rows == [(5, 2 * rr.NS_PER_S + 7, POSE)]
    assert rr.CycleRecorder.latest_positions(node) == POSE


def test_on_message_raises_on_a_missing_joint():
    """Check that the callback does not store a row it had to guess."""
    node = SimpleNamespace(rows=[], get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=5)))
    message = JointState(name=['gripper_controller'], position=[GRIPPER])
    with pytest.raises(ValueError):
        rr.CycleRecorder.on_message(node, message)
    assert node.rows == []
