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

"""Check that the launch files of the real arm build their description."""

import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import LifecycleNode, Node
import pytest

LAUNCH_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch')


def describe(name):
    """Return the LaunchDescription of launch/<name>, without launching anything."""
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'),
                                                  os.path.join(LAUNCH_DIR, name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


@pytest.mark.parametrize('name', ['driver.launch.py', 'supervised.launch.py'])
def test_launch_file_builds_its_description(name):
    """Check that the file imports and returns a LaunchDescription."""
    assert isinstance(describe(name), LaunchDescription)


def test_driver_starts_active_unless_asked():
    """Check that start_inactive exists and defaults to false: the launch is as it was."""
    arguments = {entity.name: entity for entity in describe('driver.launch.py').entities
                 if isinstance(entity, DeclareLaunchArgument)}
    assert 'start_inactive' in arguments
    default = ''.join(part.text for part in arguments['start_inactive'].default_value)
    assert default == 'false'


def test_driver_has_both_variants_of_what_start_inactive_changes():
    """Check the two controller managers and the two arm_controller spawners."""
    nodes = [entity for entity in describe('driver.launch.py').entities
             if isinstance(entity, Node)]
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
