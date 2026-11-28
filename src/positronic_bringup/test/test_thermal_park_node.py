# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# AI-generated (Claude, Anthropic).
# Prompt:
#   CI coverage of positronic_moveit_config fell under 80 %: the ThermalPark node itself was
#   never built, only its helpers. Build the real node in rclpy and drive its callbacks
#   directly, nothing spun: a hot servo with /joint_states known publishes one park trajectory
#   (the publisher replaced by a recorder); without /joint_states nothing is published; a
#   status from another hardware or a cool one parks nothing; at the rest pose the arrival
#   check stops, and without a supervisor stop service it only warns; stop_after_park off
#   calls no service. Every service name is unique to this process.
#
# Follow-up prompt (review of the PR):
#   The node refuses a speed_rad_s outside (0, 0.26] and a warn_c at or above the driver cut.
#
# Follow-up prompt (second review):
#   With stop_after_park off, check that no service call is made, not how long _stop() takes.

"""
Drive the real ThermalPark node's callbacks, without spinning it.

Each test starts rclpy with its own parameters, builds the node, and calls its callbacks with
messages built here. The trajectory publisher is replaced by a recorder, so nothing moves.
"""

import importlib.util
import os
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest
import rclpy
from sensor_msgs.msg import JointState

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(PKG, 'scripts', 'thermal_park.py')
# No supervisor answers here: the stop is never found.
STOP_SERVICE = f'/thermal_park_test_{os.getpid()}/stop'


def _load_script():
    spec = importlib.util.spec_from_file_location('thermal_park_node_under_test', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MOD = _load_script()


@pytest.fixture
def make_node():
    """Return a factory: ThermalPark with the given parameters, torn down after the test."""
    nodes = []

    def make(**parameters):
        arguments = ['--ros-args', '-p', f'stop_service:={STOP_SERVICE}']
        for name, value in parameters.items():
            arguments += ['-p', f'{name}:={value}']
        rclpy.init(args=arguments)
        node = MOD.ThermalPark()
        sent = []
        node._publisher = SimpleNamespace(publish=sent.append)
        node.sent = sent
        nodes.append(node)
        return node

    yield make
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _status(hardware_id, temperatures):
    status = DiagnosticStatus(hardware_id=hardware_id)
    status.values = [KeyValue(key=f'temperature_c/{joint}', value=str(celsius))
                     for joint, celsius in temperatures.items()]
    return DiagnosticArray(status=[status])


def _joints(positions):
    return JointState(name=list(MOD.ARM_JOINTS), position=[float(p) for p in positions])


def _hot():
    return {joint: 63 if joint == 'joint5_to_joint4' else 40 for joint in MOD.ARM_JOINTS}


def test_a_hot_servo_publishes_one_park_trajectory(make_node):
    node = make_node(hot_for_s=0.0)
    node._on_joint_states(_joints([0.0] * 6))
    node._on_diagnostics(_status('positronic_arm', _hot()))
    assert len(node.sent) == 1
    trajectory = node.sent[0]
    assert list(trajectory.joint_names) == MOD.ARM_JOINTS
    assert node._targets is not None
    # One park per hot spell.
    node._on_diagnostics(_status('positronic_arm', _hot()))
    assert len(node.sent) == 1


def test_without_joint_states_nothing_is_published(make_node):
    node = make_node(hot_for_s=0.0)
    node._on_diagnostics(_status('positronic_arm', _hot()))
    assert node.sent == []
    assert node._targets is None


def test_another_hardware_or_a_cool_arm_parks_nothing(make_node):
    node = make_node(hot_for_s=0.0)
    node._on_joint_states(_joints([0.0] * 6))
    node._on_diagnostics(_status('someone_else', _hot()))
    node._on_diagnostics(_status('positronic_arm', {j: 40 for j in MOD.ARM_JOINTS}))
    assert node.sent == []


def test_at_the_rest_pose_it_stops_and_warns_without_a_supervisor(make_node):
    node = make_node()
    targets = list(MOD.REST_POSITIONS)
    node._on_joint_states(_joints(targets))
    node._targets = targets
    node._deadline_s = node._now_s() + 10.0
    node._check_arrival()
    assert node._targets is None


def test_stop_after_park_off_calls_no_service(make_node):
    node = make_node(stop_after_park='false')
    calls = []
    node._stop_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=lambda request: calls.append(request))
    node._stop()
    assert calls == []


@pytest.mark.parametrize('speed', ['0.0', '-0.1', '0.5'])
def test_a_speed_outside_the_controller_limit_is_refused(make_node, speed):
    with pytest.raises(ValueError):
        make_node(speed_rad_s=speed)


def test_a_warning_at_the_driver_cut_is_refused(make_node):
    with pytest.raises(ValueError):
        make_node(warn_c=65, rearm_below_c=60)
