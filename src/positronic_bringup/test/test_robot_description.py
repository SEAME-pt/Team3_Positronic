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
#   Write a pytest for positronic_real.urdf.xacro (in positronic_description, used by
#   driver.launch.py). Expand it with xacro (default arguments and backend:=fake) and check
#   that: the robot keeps all links and joints of the positronic_description URDF (the name is
#   positronic, as in the Gazebo xacro); the <ros2_control> block uses
#   positronic_hardware/RobotSystem; the backend, port and baud parameters (the names the
#   plugin reads in on_init, #22) carry
#   the xacro arguments; the joints of the
#   block are exactly the six revolute joints of the URDF, in URDF order; and every joint has
#   position and velocity state interfaces and a position command interface.
# Follow-up prompt:
#   #34: check that every joint of the <ros2_control> block turns ros2_control's limiter off
#   with <limits enable="false"/>, for both backends, and that every one of them still has a
#   <limit> in the URDF for the plugin to read.
# Follow-up prompt (#35):
#   Check that the <ros2_control> block has one <gpio name="driver"> with a fault_code state
#   interface and no fault_code command interface, for both backends. Other interfaces on it
#   are allowed (S3 adds a temperature, #115).
# Follow-up prompt (servo over-temperature):
#   The xacro now also passes servo_temp_limit_c (default 60) to the plugin.
# Follow-up prompt (#45):
#   The safety supervisor does not depend on positronic_hardware: it takes the names of the
#   hardware component and of the fault interface from config/safety_supervisor.yaml. Check
#   that those names are the ones of the <ros2_control> block.
# Follow-up prompt (#150):
#   The defaults of the driver's parameters now live only in the driver (HardwareConfig): the
#   xacro writes a <param> only for an argument that is set. Check that no argument, or an
#   empty one as the launch passes, writes no <param>.
# Follow-up prompt (real wrist camera):
#   The real xacro includes mycobot_280_jn_gripper_camera.urdf, so the real camera images can
#   use camera_link as their frame_id. Check that camera_link is in the description and hangs
#   under joint 6 (joint6_flange) through fixed joints only, for both backends.
# Follow-up prompt (#120):
#   The driver gpio also declares pose_age_s, the age of the published pose, as a state
#   interface and never as a command, for both backends. joint_state_broadcaster publishes it on
#   /dynamic_joint_states with driver/fault_code, as long as the yaml does not narrow it
#   (test_controllers_config.py checks that).

"""Check the robot description that the bringup launch gives to ros2_control."""

import os
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
import yaml

SUPERVISOR_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config',
    'safety_supervisor.yaml',
)


def xacro_file():
    """Return the real arm xacro as installed by positronic_description."""
    share = get_package_share_directory('positronic_description')
    return os.path.join(share, 'urdf', 'positronic_real.urdf.xacro')


def expand(**arguments):
    """Return the root element of the expanded xacro, with the given xacro arguments."""
    document = xacro.process_file(xacro_file(), mappings=arguments)
    return ElementTree.fromstring(document.toxml())


# The arm joints served by RobotSystem, in protocol order (J1 base to J6 flange).
ARM_JOINTS = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]


def arm_block(robot):
    """Return the <ros2_control> block of the arm and the gripper (RobotSystem)."""
    return robot.find("ros2_control[@name='positronic_arm']")


def original_urdf():
    """Return the arm's URDF with the gripper and the camera (#97), as in the Gazebo xacro."""
    path = os.path.join(
        get_package_share_directory('positronic_description'),
        'urdf',
        'mycobot_280_jn_sim',
        'mycobot_280_jn_gripper_camera.urdf',
    )
    return ElementTree.parse(path).getroot()


def revolute_joints(robot):
    """Return the names of the revolute joints of `robot`, in file order."""
    return [
        j.get('name') for j in robot.findall('joint') if j.get('type') == 'revolute'
    ]


def chain_to(robot, link, ancestor):
    """
    Return the joints from `ancestor` down to `link`, as (name, type), ancestor side first.

    Fails the test if `link` does not hang under `ancestor`.
    """
    parent_joint = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    chain = []
    while link != ancestor:
        assert link in parent_joint, f'{link} does not hang under {ancestor}'
        joint = parent_joint[link]
        chain.insert(0, (joint.get('name'), joint.get('type')))
        link = joint.find('parent').get('link')
    return chain


def hardware_params(robot):
    """Return the <param> entries of the <hardware> block as a dict."""
    hardware = arm_block(robot).find('hardware')
    return {param.get('name'): param.text for param in hardware.findall('param')}


def test_keeps_the_original_arm():
    """Check that the description is the original URDF, with nothing lost."""
    robot = expand()
    original = original_urdf()
    assert (
        robot.get('name') == 'positronic'
    )  # same name as positronic_gazebo.urdf.xacro
    assert len(robot.findall('link')) == len(original.findall('link'))
    assert revolute_joints(robot) == revolute_joints(original)


def test_uses_the_positronic_plugin():
    """Check that ros2_control loads RobotSystem from positronic_hardware."""
    assert (
        arm_block(expand()).find('hardware/plugin').text
        == 'positronic_hardware/RobotSystem'
    )


def test_unset_arguments_leave_the_driver_defaults():
    """
    Check that no argument writes no <param>: the driver keeps its own defaults.

    The defaults live in one place, HardwareConfig in positronic_hardware, whose test pins them
    (TheInfoLineHasEveryValue). A value written here as well would win over the driver's.
    """
    assert hardware_params(expand()) == {}


def test_an_empty_argument_is_unset():
    """Check that an empty argument, as the launch passes when not given, writes no <param>."""
    assert hardware_params(expand(moving_period_ms='', port='')) == {}


def test_arguments_reach_the_plugin():
    """Check that the xacro arguments become the plugin parameters."""
    params = hardware_params(
        expand(
            backend='fake',
            fake_mode='measured',
            port='/dev/null',
            baud='115200',
            moving_period_ms='40',
            echo_timeout_ms='45',
            pose_from_poll='false',
            servo_temp_limit_c='45',
            tracking_error_deg='6',
            tracking_error_ms='200',
            supervisor_timeout_ms='300',
        )
    )
    assert params == {
        'backend': 'fake',
        'fake_mode': 'measured',
        'port': '/dev/null',
        'baud': '115200',
        'moving_period_ms': '40',
        'echo_timeout_ms': '45',
        'pose_from_poll': 'false',
        'servo_temp_limit_c': '45',
        'tracking_error_deg': '6',
        'tracking_error_ms': '200',
        'supervisor_timeout_ms': '300',
    }


def test_joints_are_the_urdf_revolute_joints_in_order():
    """Check that the arm block has J1 to J6 in URDF order, then the gripper joint."""
    joints = [joint.get('name') for joint in arm_block(expand()).findall('joint')]
    assert joints == ARM_JOINTS + ['gripper_controller']
    urdf_order = [j for j in revolute_joints(original_urdf()) if j in ARM_JOINTS]
    assert urdf_order == ARM_JOINTS


@pytest.mark.parametrize('backend', ['serial', 'fake'])
def test_every_joint_is_commanded_in_position(backend):
    """Check that each joint has position and velocity states and a position command."""
    for joint in arm_block(expand(backend=backend)).findall('joint'):
        states = [state.get('name') for state in joint.findall('state_interface')]
        assert states == ['position', 'velocity'], joint.get('name')
        commands = [
            command.get('name') for command in joint.findall('command_interface')
        ]
        assert commands == ['position'], joint.get('name')


@pytest.mark.parametrize('backend', ['serial', 'fake'])
def test_ros2_control_limiter_is_off_on_every_joint(backend):
    """Check that the plugin, not ros2_control's limiter, sees a command past a limit (#34)."""
    for joint in arm_block(expand(backend=backend)).findall('joint'):
        limits = joint.findall('limits')
        assert len(limits) == 1, joint.get('name')
        assert limits[0].get('enable') == 'false', joint.get('name')


def test_every_commanded_joint_has_a_urdf_limit():
    """Check that the limits the plugin reads in on_init are in the URDF (#34)."""
    robot = expand()
    urdf_joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    for joint in arm_block(robot).findall('joint'):
        limit = urdf_joints[joint.get('name')].find('limit')
        assert limit is not None, joint.get('name')
        assert float(limit.get('lower')) < float(limit.get('upper')), joint.get('name')


@pytest.mark.parametrize('backend', ['serial', 'fake'])
def test_driver_reports_the_fault_code(backend):
    """Check that the driver gpio has a fault_code state and no fault_code command (#35)."""
    drivers = [
        gpio
        for gpio in expand(backend=backend).findall('ros2_control/gpio')
        if gpio.get('name') == 'driver'
    ]
    assert len(drivers) == 1
    states = [state.get('name') for state in drivers[0].findall('state_interface')]
    commands = [
        command.get('name') for command in drivers[0].findall('command_interface')
    ]
    assert 'fault_code' in states
    assert 'fault_code' not in commands


@pytest.mark.parametrize('backend', ['serial', 'fake'])
def test_driver_reports_the_pose_age(backend):
    """Check that the driver gpio has a pose_age_s state and no such command (#120)."""
    drivers = [
        gpio
        for gpio in expand(backend=backend).findall('ros2_control/gpio')
        if gpio.get('name') == 'driver'
    ]
    assert len(drivers) == 1
    states = [state.get('name') for state in drivers[0].findall('state_interface')]
    commands = [
        command.get('name') for command in drivers[0].findall('command_interface')
    ]
    assert states.count('pose_age_s') == 1
    assert 'pose_age_s' not in commands


def test_supervisor_reads_the_names_of_this_description():
    """Check safety_supervisor.yaml against the <ros2_control> block (#45)."""
    with open(SUPERVISOR_CONFIG) as file:
        params = yaml.safe_load(file)['safety_supervisor']['ros__parameters']
    block = arm_block(expand())
    assert params['hardware_component'] == block.get('name')
    gpios = {gpio.get('name'): gpio for gpio in block.findall('gpio')}
    assert params['fault_gpio'] in gpios
    driver = gpios[params['fault_gpio']]
    assert params['fault_interface'] in [
        s.get('name') for s in driver.findall('state_interface')
    ]


@pytest.mark.parametrize('backend', ['serial', 'fake'])
def test_the_gripper_is_served_by_the_arm_block(backend):
    """Check that RobotSystem is the only <ros2_control> block, gripper included (#123)."""
    blocks = expand(backend=backend).findall('ros2_control')
    assert [block.get('name') for block in blocks] == ['positronic_arm']


@pytest.mark.parametrize('backend', ['serial', 'fake'])
def test_camera_link_hangs_under_joint_6(backend):
    """Check the frame_id of the real camera images is under joint 6, by fixed joints only."""
    robot = expand(backend=backend)
    assert 'camera_link' in [link.get('name') for link in robot.findall('link')]
    # joint6_flange is the child of joint 6 (joint6output_to_joint6): the camera turns with it.
    joint6 = [joint for joint in robot.findall('joint')
              if joint.get('name') == 'joint6output_to_joint6']
    assert len(joint6) == 1
    flange = joint6[0].find('child').get('link')
    chain = chain_to(robot, 'camera_link', flange)
    assert chain  # at least one joint between the flange and the camera
    assert all(kind == 'fixed' for _, kind in chain), chain
