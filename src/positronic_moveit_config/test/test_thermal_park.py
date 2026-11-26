# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# AI-generated (Claude, Anthropic).
# Prompt:
#   Write pytest tests for thermal_park.py without spinning a node: temperatures_from() reads
#   the driver's "temperature_c/<joint>" keys of the positronic_arm status and ignores other
#   statuses, other keys and values that are not whole numbers; HotWatch parks once a servo
#   has stayed at warn_c for hot_for_s, not for one hot reading alone, not when the reading in
#   between is cool or missing, once per hot spell, and again only after every servo went
#   under rearm_below_c; park_targets() keeps J1 and J6 at their current values and refuses
#   without them; build_goal() asks to plan and execute the six joint targets on the arm group
#   at the given scaling; and REST_POSITIONS match the "rest" group_state of positronic.srdf.
#
# Follow-up prompt (park without MoveIt):
#   The park no longer goes through move_group: one trajectory straight to arm_controller.
#   Replace the build_goal() test: park_targets() needs all six joints; park_duration_s() is
#   the furthest joint's travel at the given speed, never under MIN_PARK_S; build_trajectory()
#   is one point with the six arm joints in the controller's order, zero velocities, reached
#   after the given time; arrived() is True only with every joint within the tolerance.
#   Test _check_arrival() on a stand-in for the node: at the rest pose it stops, before the
#   deadline away from it it waits, after the deadline it stops anyway.
#
# Follow-up prompt (supervisor stop after the park):
#   Test _stop() on a stand-in for the node: it calls the supervisor's stop when the service
#   is there, only warns when it is not, and does nothing with stop_after_park off.

import importlib.util
import os
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(PKG, 'scripts', 'thermal_park.py')
SRDF = os.path.join(PKG, 'config', 'positronic.srdf')


def _load_script():
    spec = importlib.util.spec_from_file_location('thermal_park', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MOD = _load_script()


def _array(hardware_id, pairs):
    status = DiagnosticStatus(hardware_id=hardware_id)
    status.values = [KeyValue(key=k, value=v) for k, v in pairs]
    return DiagnosticArray(status=[status])


def _all_at(celsius):
    return {joint: celsius for joint in MOD.ARM_JOINTS}


def test_temperatures_from_reads_the_driver_status_only():
    message = _array('positronic_arm', [
        ('temperature_c/joint5_to_joint4', '58'),
        ('temperature_c/joint2_to_joint1', 'hot'),
        ('other', '99'),
    ])
    message.status.append(DiagnosticStatus(
        hardware_id='someone_else', values=[KeyValue(key='temperature_c/x', value='90')]))
    assert MOD.temperatures_from(message, 'positronic_arm') == {'joint5_to_joint4': 58}
    assert MOD.temperatures_from(message, 'missing') is None


def test_one_hot_reading_alone_does_not_park():
    watch = MOD.HotWatch(63, 1.0, 60)
    hot = {**_all_at(40), 'joint2_to_joint1': 98}
    assert watch.update(hot, 0.0) is None
    assert watch.update(_all_at(40), 1.0) is None
    assert watch.update(hot, 2.0) is None


def test_a_servo_hot_for_one_second_parks_once():
    watch = MOD.HotWatch(63, 1.0, 60)
    hot = {**_all_at(40), 'joint5_to_joint4': 63}
    assert watch.update(hot, 0.0) is None
    assert watch.update(hot, 1.0) == ('joint5_to_joint4', 63)
    assert watch.update({**hot, 'joint5_to_joint4': 64}, 2.0) is None
    assert watch.update(hot, 3.0) is None


def test_a_missing_reading_stops_the_clock():
    watch = MOD.HotWatch(63, 1.0, 60)
    hot = {**_all_at(40), 'joint4_to_joint3': 63}
    assert watch.update(hot, 0.0) is None
    assert watch.update({}, 1.0) is None
    assert watch.update(hot, 1.5) is None
    assert watch.update(hot, 2.5) == ('joint4_to_joint3', 63)


def test_parks_again_only_after_every_servo_cooled_under_rearm():
    watch = MOD.HotWatch(63, 1.0, 60)
    hot = {**_all_at(40), 'joint5_to_joint4': 63}
    watch.update(hot, 0.0)
    assert watch.update(hot, 1.0) is not None
    assert watch.update({**_all_at(40), 'joint5_to_joint4': 61}, 2.0) is None
    watch.update(hot, 3.0)
    assert watch.update(hot, 4.0) is None
    assert watch.update(_all_at(59), 5.0) is None
    watch.update(hot, 6.0)
    assert watch.update(hot, 7.0) == ('joint5_to_joint4', 63)


def test_rearm_above_warn_is_refused():
    with pytest.raises(ValueError):
        MOD.HotWatch(63, 1.0, 64)


def _current(**overrides):
    current = {joint: 0.0 for joint in MOD.ARM_JOINTS}
    current.update(overrides)
    return current


def test_park_targets_keep_j1_and_j6():
    current = _current(joint2_to_joint1=-0.5, joint6output_to_joint6=0.25, joint3_to_joint2=1.0)
    targets = MOD.park_targets(current)
    assert targets[0] == -0.5
    assert targets[5] == 0.25
    assert targets[1:5] == MOD.REST_POSITIONS[1:5]


def test_park_targets_need_every_joint():
    current = _current()
    del current['joint4_to_joint3']
    assert MOD.park_targets(current) is None


def test_park_duration_is_the_furthest_joint_at_the_speed():
    current = _current()
    targets = [0.0, 0.0, -2.0, -0.5, 0.0, 0.0]
    assert MOD.park_duration_s(current, targets, 0.1) == pytest.approx(20.0)
    assert MOD.park_duration_s(current, [0.0] * 6, 0.1) == pytest.approx(MOD.MIN_PARK_S)


def test_build_trajectory_is_one_point_with_the_six_joints():
    targets = [0.1, 0.0, -2.3, -0.2, -0.4, 1.9]
    trajectory = MOD.build_trajectory(targets, 12.5)
    assert trajectory.joint_names == MOD.ARM_JOINTS
    (point,) = trajectory.points
    assert point.positions == pytest.approx(targets)
    assert point.velocities == [0.0] * 6
    assert point.time_from_start.sec == 12
    assert point.time_from_start.nanosec == 500_000_000


def test_arrived_needs_every_joint_within_the_tolerance():
    targets = [0.0, 0.0, -2.0, 0.0, 0.0, 0.0]
    assert MOD.arrived(_current(joint4_to_joint3=-1.95), targets, 0.07)
    assert not MOD.arrived(_current(joint4_to_joint3=-1.9), targets, 0.07)
    current = _current(joint4_to_joint3=-2.0)
    del current['joint6_to_joint5']
    assert not MOD.arrived(current, targets, 0.07)


def test_rest_positions_match_the_srdf_rest_state():
    root = ET.parse(SRDF).getroot()
    (rest,) = [s for s in root.findall('group_state') if s.get('name') == 'rest']
    values = {j.get('name'): float(j.get('value')) for j in rest.findall('joint')}
    assert list(values) == MOD.ARM_JOINTS
    assert [values[j] for j in MOD.ARM_JOINTS] == pytest.approx(MOD.REST_POSITIONS)


class _Client:

    def __init__(self, available):
        self.available = available
        self.calls = 0

    def wait_for_service(self, timeout_sec):
        return self.available

    def call_async(self, request):
        self.calls += 1
        return SimpleNamespace(add_done_callback=lambda callback: None)


class _Logger:

    def __init__(self):
        self.warnings = []

    def warn(self, text):
        self.warnings.append(text)

    def info(self, text):
        pass

    def error(self, text):
        self.warnings.append(text)


def _node(available, enabled=True):
    logger = _Logger()
    return SimpleNamespace(
        _stop_after_park=enabled, _stop_client=_Client(available), _on_stopped=None,
        get_logger=lambda: logger, logger=logger)


def test_stop_calls_the_supervisor_when_it_is_there():
    node = _node(available=True)
    MOD.ThermalPark._stop(node)
    assert node._stop_client.calls == 1
    assert node.logger.warnings == []


def test_stop_only_warns_without_a_supervisor_or_when_off():
    node = _node(available=False)
    MOD.ThermalPark._stop(node)
    assert node._stop_client.calls == 0
    assert len(node.logger.warnings) == 1
    off = _node(available=True, enabled=False)
    MOD.ThermalPark._stop(off)
    assert off._stop_client.calls == 0


def _parking(joints, now_s, deadline_s):
    logger = _Logger()
    node = SimpleNamespace(
        _joints=joints, _targets=[0.0, 0.0, -2.0, 0.0, 0.0, 0.0], _tolerance_rad=0.07,
        _deadline_s=deadline_s, _now_s=lambda: now_s, get_logger=lambda: logger, stops=[])
    node._stop = lambda: node.stops.append(True)
    return node


def test_check_arrival_stops_at_the_rest_pose():
    node = _parking(_current(joint4_to_joint3=-2.0), now_s=1.0, deadline_s=10.0)
    MOD.ThermalPark._check_arrival(node)
    assert node.stops == [True]
    assert node._targets is None


def test_check_arrival_waits_then_stops_anyway_after_the_deadline():
    node = _parking(_current(), now_s=1.0, deadline_s=10.0)
    MOD.ThermalPark._check_arrival(node)
    assert node.stops == []
    node._now_s = lambda: 11.0
    MOD.ThermalPark._check_arrival(node)
    assert node.stops == [True]
    assert node._targets is None
