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

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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
    # Node keeps the parameters it was given only in a private attribute; launch evaluates
    # them the same way when it starts the node.
    merged = {}
    for entry in evaluate_parameters(context, nodes[0]._Node__parameters):
        assert isinstance(entry, dict)  # every entry is a dict, no parameter file
        merged.update(entry)
    return merged


@pytest.fixture(scope='module')
def joint_limits():
    """Return the contents of config/joint_limits.yaml."""
    with open(os.path.join(PKG, 'config', 'joint_limits.yaml'), encoding='utf-8') as file:
        return yaml.safe_load(file)


@pytest.mark.parametrize('factor', [
    'default_velocity_scaling_factor', 'default_acceleration_scaling_factor'])
def test_mission_node_gets_the_scaling_of_joint_limits_yaml(
        mission_parameters, joint_limits, factor):
    """Check both scaling factors reach the node: without them MoveIt uses 0.1 (#214)."""
    assert mission_parameters[f'{PLANNING}.{factor}'] == joint_limits[factor]


def test_mission_node_velocity_scaling_is_0_3(mission_parameters):
    """Check the value the arm was run with on 27 Nov (#214), so a change to it is seen here."""
    assert mission_parameters[f'{PLANNING}.default_velocity_scaling_factor'] == 0.3


def test_mission_node_gets_every_joint_velocity_limit(mission_parameters, joint_limits):
    """Check every joint of joint_limits.yaml reaches the node with its max_velocity."""
    for joint, limits in joint_limits['joint_limits'].items():
        key = f'{PLANNING}.joint_limits.{joint}.max_velocity'
        assert mission_parameters[key] == limits['max_velocity'], joint
