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
#   #45: write a pytest that loads every launch file of positronic_bringup that the Jetson
#   uses (driver.launch.py, supervised.launch.py) as a module and calls
#   generate_launch_description(), so a typo or a missing import fails in CI and not on the
#   arm. Nothing is launched. Also check that driver.launch.py declares start_inactive with
#   the default false (the launch behaves as before unless asked), that it has the two
#   variants of the controller manager and of the arm_controller spawner, and that
#   supervised.launch.py includes the driver launch and starts the safety supervisor.
# Follow-up prompt (#150):
#   The driver's arguments have no default of their own any more, and update_rate takes its
#   default from jetson_controllers.yaml. Check both.
# Follow-up prompt (real wrist camera):
#   Add camera.launch.py to the files that must build. Check that its frame_id default is a link
#   of positronic_real.urdf.xacro and the link the Gazebo camera sensor sits on (from the
#   expanded positronic_gazebo.urdf.xacro), and that its namespace gives the topics of the
#   Gazebo camera: the sensor's <topic> and the two bridged topics of gazebo.launch.py.
# Follow-up prompt (camera off by default):
#   driver.launch.py gets a camera argument, default false, that includes camera.launch.py only
#   when true. Check the default, and that the include is there, conditional, and resolves to
#   the camera launch.

"""Check that the launch files of the real arm build their description."""

import importlib.util
import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import LifecycleNode, Node
import pytest
import xacro
import yaml

LAUNCH_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch'
)


def load(name):
    """Import launch/<name> as a module, without launching anything."""
    spec = importlib.util.spec_from_file_location(
        name.replace('.', '_'), os.path.join(LAUNCH_DIR, name)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def describe(name):
    """Return the LaunchDescription of launch/<name>, without launching anything."""
    return load(name).generate_launch_description()


def expand_xacro(file_name, **arguments):
    """Return the root element of a positronic_description xacro, expanded."""
    path = os.path.join(get_package_share_directory('positronic_description'), 'urdf', file_name)
    return ElementTree.fromstring(xacro.process_file(path, mappings=arguments).toxml())


def gazebo_camera_sensors():
    """Return (reference link, sensor element) of every camera sensor of the Gazebo model."""
    robot = expand_xacro('positronic_gazebo.urdf.xacro', controllers_file='')
    return [(gazebo.get('reference'), sensor)
            for gazebo in robot.findall('gazebo')
            for sensor in gazebo.findall('sensor') if sensor.get('type') == 'camera']


@pytest.mark.parametrize('name',
                         ['driver.launch.py', 'supervised.launch.py', 'camera.launch.py'])
def test_launch_file_builds_its_description(name):
    """Check that the file imports and returns a LaunchDescription."""
    assert isinstance(describe(name), LaunchDescription)


def test_driver_starts_active_unless_asked():
    """Check that start_inactive exists and defaults to false: the launch is as it was."""
    arguments = {
        entity.name: entity
        for entity in describe('driver.launch.py').entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert 'start_inactive' in arguments
    default = ''.join(part.text for part in arguments['start_inactive'].default_value)
    assert default == 'false'


DRIVER_ARGUMENTS = [
    'backend', 'fake_mode', 'port', 'baud', 'moving_period_ms', 'echo_timeout_ms',
    'pose_from_poll', 'servo_temp_limit_c', 'tracking_error_deg', 'tracking_error_ms',
]


def launch_defaults(name):
    """
    Return {argument: default text} of launch/<name>, for the plain text defaults.

    A default built from substitutions (camera.launch.py's calibration_file, a path in the
    installed share) has no text until launch time, so it is left out.
    """
    defaults = {}
    for entity in describe(name).entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            parts = entity.default_value
            if all(hasattr(part, 'text') for part in parts):
                defaults[entity.name] = ''.join(part.text for part in parts)
    return defaults


def test_driver_arguments_have_no_default_of_their_own():
    """Check that every driver argument defaults to empty: the driver's default applies."""
    defaults = launch_defaults('driver.launch.py')
    assert {name: defaults[name] for name in DRIVER_ARGUMENTS} == dict.fromkeys(
        DRIVER_ARGUMENTS, '')


def test_update_rate_defaults_to_the_yaml():
    """Check that update_rate defaults to the value of jetson_controllers.yaml."""
    path = os.path.join(os.path.dirname(LAUNCH_DIR), 'config', 'jetson_controllers.yaml')
    with open(path, encoding='utf-8') as config:
        rate = yaml.safe_load(config)['controller_manager']['ros__parameters']['update_rate']
    assert launch_defaults('driver.launch.py')['update_rate'] == str(rate)


def test_driver_has_both_variants_of_what_start_inactive_changes():
    """Check the two controller managers and the two arm_controller spawners."""
    nodes = [
        entity
        for entity in describe('driver.launch.py').entities
        if isinstance(entity, Node)
    ]
    managers = [node for node in nodes if node.node_executable == 'ros2_control_node']
    spawners = [node for node in nodes if node.node_executable == 'spawner']
    # One of each pair runs, chosen by its condition.
    assert len(managers) == 2
    assert all(node.condition is not None for node in managers)
    assert len(spawners) == 3  # joint_state_broadcaster, and arm_controller twice
    assert sum(node.condition is not None for node in spawners) == 2


def test_supervised_launch_starts_the_driver_and_the_supervisor():
    """Check that supervised.launch.py includes a launch and runs the lifecycle node."""
    entities = describe('supervised.launch.py').entities
    assert any(isinstance(entity, IncludeLaunchDescription) for entity in entities)
    supervisors = [entity for entity in entities if isinstance(entity, LifecycleNode)]
    assert len(supervisors) == 1
    assert supervisors[0].node_executable == 'safety_supervisor'


def test_camera_frame_id_is_the_camera_link_of_both_models():
    """Check frame_id: a link of the real xacro, and the link the Gazebo camera sits on."""
    frame_id = launch_defaults('camera.launch.py')['frame_id']
    real_links = [link.get('name') for link in expand_xacro('positronic_real.urdf.xacro')
                  .findall('link')]
    assert frame_id in real_links
    sensors = gazebo_camera_sensors()
    assert len(sensors) == 1  # one camera in the Gazebo model: the wrist camera
    assert sensors[0][0] == frame_id


def test_camera_topics_are_the_gazebo_topics():
    """Check the namespace gives the Gazebo topics: the sensor's and the two bridged ones."""
    namespace = load('camera.launch.py').NAMESPACE
    _, sensor = gazebo_camera_sensors()[0]
    assert sensor.find('topic').text.strip('/') == f'{namespace}/image_raw'
    with open(os.path.join(LAUNCH_DIR, 'gazebo.launch.py'), encoding='utf-8') as file:
        gazebo_launch = file.read()
    # The bridge arguments are 'topic@ros_type[gz_type': the ROS side of both camera topics.
    assert f"'/{namespace}/image_raw@sensor_msgs/msg/Image[" in gazebo_launch
    assert f"'/{namespace}/camera_info@sensor_msgs/msg/CameraInfo[" in gazebo_launch


def test_camera_launch_runs_the_camera_node():
    """Check the launch starts one node: wrist_camera.py of this package."""
    nodes = [entity for entity in describe('camera.launch.py').entities
             if isinstance(entity, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_package == 'positronic_bringup'
    assert nodes[0].node_executable == 'wrist_camera.py'


def test_driver_starts_the_camera_only_when_asked():
    """Check camera defaults to false and camera.launch.py is included under a condition."""
    assert launch_defaults('driver.launch.py')['camera'] == 'false'
    includes = [entity for entity in describe('driver.launch.py').entities
                if isinstance(entity, IncludeLaunchDescription)]
    assert len(includes) == 1
    assert includes[0].condition is not None
    # Resolve the include (the installed camera.launch.py) and check it is the camera launch.
    included = includes[0].launch_description_source.get_launch_description(LaunchContext())
    names = [entity.name for entity in included.entities
             if isinstance(entity, DeclareLaunchArgument)]
    assert 'device' in names and 'frame_id' in names
