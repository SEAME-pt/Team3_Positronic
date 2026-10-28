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
#   position and velocity state interfaces and no command interface (read-only until #32).

"""Check the robot description that the bringup launch gives to ros2_control."""

import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


def xacro_file():
    """Return the real arm xacro as installed by positronic_description."""
    share = get_package_share_directory('positronic_description')
    return os.path.join(share, 'urdf', 'positronic_real.urdf.xacro')


def expand(**arguments):
    """Return the root element of the expanded xacro, with the given xacro arguments."""
    document = xacro.process_file(xacro_file(), mappings=arguments)
    return ElementTree.fromstring(document.toxml())


def original_urdf():
    """Return the root element of the arm's URDF as installed by positronic_description."""
    path = os.path.join(
        get_package_share_directory('positronic_description'),
        'urdf', 'mycobot_280_jn_sim', 'mycobot_280_jn.urdf')
    return ElementTree.parse(path).getroot()


def revolute_joints(robot):
    """Return the names of the revolute joints of `robot`, in file order."""
    return [j.get('name') for j in robot.findall('joint') if j.get('type') == 'revolute']


def hardware_params(robot):
    """Return the <param> entries of the <hardware> block as a dict."""
    hardware = robot.find('ros2_control/hardware')
    return {param.get('name'): param.text for param in hardware.findall('param')}


def test_keeps_the_original_arm():
    """Check that the description is the original URDF, with nothing lost."""
    robot = expand()
    original = original_urdf()
    assert robot.get('name') == 'positronic'  # same name as positronic_gazebo.urdf.xacro
    assert len(robot.findall('link')) == len(original.findall('link'))
    assert revolute_joints(robot) == revolute_joints(original)


def test_uses_the_positronic_plugin():
    """Check that ros2_control loads RobotSystem from positronic_hardware."""
    assert expand().find('ros2_control/hardware/plugin').text == 'positronic_hardware/RobotSystem'


def test_defaults_select_the_real_arm():
    """Check the default parameters: the serial backend on the Jetson UART."""
    assert hardware_params(expand()) == {
        'backend': 'serial', 'port': '/dev/ttyTHS1', 'baud': '1000000'}


def test_arguments_reach_the_plugin():
    """Check that the xacro arguments become the plugin parameters."""
    params = hardware_params(expand(backend='fake', port='/dev/null', baud='115200'))
    assert params == {'backend': 'fake', 'port': '/dev/null', 'baud': '115200'}


def test_joints_are_the_urdf_revolute_joints_in_order():
    """Check that the ros2_control joints are the six URDF joints, J1 to J6."""
    robot = expand()
    joints = [joint.get('name') for joint in robot.findall('ros2_control/joint')]
    assert len(joints) == 6
    assert joints == revolute_joints(original_urdf())


@pytest.mark.parametrize('backend', ['serial', 'fake'])
def test_every_joint_is_read_only(backend):
    """Check that each joint has position and velocity states and no command interface."""
    for joint in expand(backend=backend).findall('ros2_control/joint'):
        states = [state.get('name') for state in joint.findall('state_interface')]
        assert states == ['position', 'velocity'], joint.get('name')
        assert joint.findall('command_interface') == [], joint.get('name')