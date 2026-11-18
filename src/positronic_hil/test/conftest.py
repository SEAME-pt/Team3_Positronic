# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import math
import os
import signal
import subprocess
import time

from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from controller_manager_msgs.srv import SetHardwareComponentState, SwitchController
from lifecycle_msgs.msg import State
import pytest
import rclpy
from sensor_msgs.msg import JointState

COMPONENT = 'positronic_arm'
JOINTS = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]
MAX_DRIFT = math.radians(0.5)
ARM_CONTROLLER = 'arm_controller'
CONTROLLER_TIMEOUT_S = 30.0


def pytest_collection_modifyitems(config, items):
    if os.environ.get('POSITRONIC_HIL') == '1':
        return
    skip = pytest.mark.skip(reason='HIL: needs the real arm, run with POSITRONIC_HIL=1')
    for item in items:
        item.add_marker(skip)


class Probe:

    def __init__(self, node):
        self.node = node
        self.samples = []
        node.create_subscription(JointState, '/joint_states', self._on_state, 100)
        self.list_hardware = node.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.set_hardware = node.create_client(
            SetHardwareComponentState, '/controller_manager/set_hardware_component_state')
        self.list_controllers = node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.switch = node.create_client(
            SwitchController, '/controller_manager/switch_controller')

    def _on_state(self, msg):
        self.samples.append((time.monotonic(), dict(zip(msg.name, msg.position))))

    def wait_first(self, timeout):
        end = time.monotonic() + timeout
        while not self.samples and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return bool(self.samples)

    def record(self, seconds):
        start = len(self.samples)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.samples[start:]

    def wait_future(self, future, timeout, what):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        assert future.done(), f'{what} did not finish in {timeout} s'
        return future.result()

    def call(self, client, request):
        assert client.wait_for_service(timeout_sec=10.0), f'{client.srv_name} is not up'
        return self.wait_future(client.call_async(request), 10.0, client.srv_name)

    def component_state(self):
        reply = self.call(self.list_hardware, ListHardwareComponents.Request())
        for component in reply.component:
            if component.name == COMPONENT:
                return component.state.label
        raise AssertionError(f'{COMPONENT} is not loaded')

    def set_component_state(self, state_id, label):
        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state = State(id=state_id, label=label)
        return self.call(self.set_hardware, request)

    def controller_state(self, name):
        reply = self.call(self.list_controllers, ListControllers.Request())
        for controller in reply.controller:
            if controller.name == name:
                return controller.state
        return None

    def wait_controller_active(self, name, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.controller_state(name) == 'active':
                return True
            self.record(0.5)
        return False

    def switch_controllers(self, activate=(), deactivate=()):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        return self.call(self.switch, request)


def max_drift(samples, reference):
    return max(abs(pose[name] - reference[name]) for _, pose in samples for name in JOINTS)


def driver_log_path(request):
    xml = request.config.option.xmlpath
    folder = os.path.dirname(os.path.abspath(xml)) if xml else os.getcwd()
    return os.path.join(folder, f'{request.module.__name__}.driver.log')


@pytest.fixture(scope='module')
def arm(request):
    log = open(driver_log_path(request), 'w', encoding='utf-8')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'positronic_bringup', 'driver.launch.py'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        env={**os.environ, 'PYTHONUNBUFFERED': '1'})
    rclpy.init()
    node = rclpy.create_node('hil_test')
    try:
        probe = Probe(node)
        assert probe.wait_first(20.0), 'no /joint_states in 20 s: is the arm powered?'
        yield probe
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
        log.close()
