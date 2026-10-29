# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check the joints that ros2_control sees in the Gazebo robot description."""

import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
import xacro

GRIPPER_JOINT = 'gripper_controller'
FINGER_JOINTS = [
    'gripper_base_to_gripper_left2',
    'gripper_left3_to_gripper_left1',
    'gripper_base_to_gripper_right3',
    'gripper_base_to_gripper_right2',
    'gripper_right3_to_gripper_right1',
]
ARM_JOINTS = 6


def robot():
    """Return the expanded positronic_gazebo.urdf.xacro, as installed, as an XML element."""
    share = get_package_share_directory('positronic_description')
    path = os.path.join(share, 'urdf', 'positronic_gazebo.urdf.xacro')
    return ElementTree.fromstring(xacro.process_file(path).toxml())


def control_joints(root):
    """Return {name: element} of the joints inside the <ros2_control> block."""
    return {joint.get('name'): joint for joint in root.findall('ros2_control/joint')}


def interfaces(joint, kind):
    """Return the names of one kind of interface of a joint, command or state."""
    return [interface.get('name') for interface in joint.findall(kind)]


def test_ros2_control_lists_the_arm_the_gripper_and_the_fingers():
    """Check that all twelve joints Gazebo drives are in the block."""
    joints = control_joints(robot())
    assert len(joints) == ARM_JOINTS + 1 + len(FINGER_JOINTS)
    assert GRIPPER_JOINT in joints
    for name in FINGER_JOINTS:
        assert name in joints, name


def test_finger_joints_have_no_command_interface():
    """Check the fingers are state only: a mimic joint with a command interface fails to load."""
    joints = control_joints(robot())
    for name in FINGER_JOINTS:
        assert interfaces(joints[name], 'command_interface') == [], name
        assert interfaces(joints[name], 'state_interface') == ['position'], name


def test_arm_and_gripper_joints_are_commanded_in_position():
    """Check that every other joint has exactly a position command interface."""
    for name, joint in control_joints(robot()).items():
        if name not in FINGER_JOINTS:
            assert interfaces(joint, 'command_interface') == ['position'], name


def test_finger_joints_mimic_the_gripper_joint_in_the_urdf():
    """Check the URDF mimic tags that ros2_control reads for the listed fingers."""
    urdf_joints = {joint.get('name'): joint for joint in robot().findall('joint')}
    for name in FINGER_JOINTS:
        mimic = urdf_joints[name].find('mimic')
        assert mimic is not None, name
        assert mimic.get('joint') == GRIPPER_JOINT, name
