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
#   #214: the mission node never gets joint_limits.yaml, so its MoveGroupInterface falls back
#   to MoveIt's 0.1 velocity scaling. Write a pytest that builds the mission_node of
#   positronic_bringup's mission.launch.py (nothing is launched) and checks that its parameters
#   carry robot_description_planning from joint_limits.yaml: both default scaling factors and
#   the arm joints' max_velocity. It fails before the fix.
# Follow-up prompt (PR #215 review):
#   Compare the whole joint_limits.yaml with the node's robot_description_planning, read the
#   installed yaml (the one the launch reads), and say why the private attribute is safe.

"""
Check the parameters mission.launch.py gives the mission node.

The test lives here and not in positronic_bringup: this package already depends on bringup,
and bringup depending back on it for a test would be a cycle.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml

PLANNING = 'robot_description_planning'


def load_launch():
    """Import the installed mission.launch.py as a module, without launching anything."""
    path = os.path.join(
        get_package_share_directory('positronic_bringup'), 'launch', 'mission.launch.py')
    spec = importlib.util.spec_from_file_location('mission_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def mission_parameters():
    """Return the mission node's parameters, evaluated and merged into one dict."""
    context = LaunchContext()
    context.launch_configurations['use_sim_time'] = 'false'
    nodes = [entity for entity in load_launch().launch_setup(context)
             if isinstance(entity, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_executable == 'mission_node'
    # launch_ros's Node has no public getter for its parameters (Jazzy), so read the private
    # one. If launch_ros renames it, this fails with AttributeError, never a false pass.
    # evaluate_parameters is what Node._perform_substitutions calls before starting the node.
    merged = {}
    for entry in evaluate_parameters(context, nodes[0]._Node__parameters):
        assert isinstance(entry, dict)  # every entry is a dict, no parameter file
        merged.update(entry)
    return merged


@pytest.fixture(scope='module')
def joint_limits():
    """Return the installed config/joint_limits.yaml, the file mission.launch.py reads."""
    path = os.path.join(
        get_package_share_directory('positronic_moveit_config'), 'config', 'joint_limits.yaml')
    with open(path, encoding='utf-8') as file:
        return yaml.safe_load(file)


def flatten(tree, prefix):
    """Yield (dotted key, value) for every leaf of a nested dict, as ROS flattens parameters."""
    for key, value in tree.items():
        if isinstance(value, dict):
            yield from flatten(value, f'{prefix}.{key}')
        else:
            yield f'{prefix}.{key}', value


def test_mission_node_gets_all_of_joint_limits_yaml(mission_parameters, joint_limits):
    """Check every value of joint_limits.yaml reaches the node: without them MoveIt uses 0.1."""
    for key, value in flatten(joint_limits, PLANNING):
        assert mission_parameters[key] == value, key


def test_mission_node_velocity_scaling_is_0_3(mission_parameters):
    """Check the value the arm was run with on 27 Nov (#214), so a change to it is seen here."""
    assert mission_parameters[f'{PLANNING}.default_velocity_scaling_factor'] == 0.3
