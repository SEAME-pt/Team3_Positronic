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
#   #45: the safety supervisor reads driver/fault_code on /dynamic_joint_states, which
#   joint_state_broadcaster publishes with every state interface as long as it keeps its
#   default parameters. Write a pytest that reads config/jetson_controllers.yaml and fails
#   when a setting would take the fault code off that topic: the broadcaster missing from the
#   controller manager, both `joints` and `interfaces` set, publish_dynamic_joint_states
#   false, or use_local_topics true.
# Follow-up prompt:
#   Also read config/safety_supervisor.yaml and check it against jetson_controllers.yaml: a
#   halt deactivates every controller that writes a command interface (every declared
#   controller that is not a broadcaster), a resume only activates declared controllers and
#   includes joint_state_broadcaster, and every timeout is positive, with the check period at
#   most a quarter of the timeouts it watches.

"""Check the controllers config of the real arm against what the safety supervisor needs."""

import os

import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
CONFIG = os.path.join(CONFIG_DIR, 'jetson_controllers.yaml')
SUPERVISOR_CONFIG = os.path.join(CONFIG_DIR, 'safety_supervisor.yaml')


def load_config():
    """Return jetson_controllers.yaml as a dict."""
    with open(CONFIG) as file:
        return yaml.safe_load(file)


def supervisor_params():
    """Return the parameters of the safety supervisor from safety_supervisor.yaml."""
    with open(SUPERVISOR_CONFIG) as file:
        return yaml.safe_load(file)['safety_supervisor']['ros__parameters']


def declared_controllers():
    """Return {name: type} of the controllers the controller manager can load."""
    manager = load_config()['controller_manager']['ros__parameters']
    return {name: value['type'] for name, value in manager.items()
            if isinstance(value, dict) and 'type' in value}


def broadcaster_params():
    """Return the parameters the yaml gives joint_state_broadcaster, {} when it gives none."""
    return load_config().get('joint_state_broadcaster', {}).get('ros__parameters', {})


def test_joint_state_broadcaster_is_declared():
    """Check that the controller manager knows the broadcaster the supervisor depends on."""
    manager = load_config()['controller_manager']['ros__parameters']
    broadcaster = manager['joint_state_broadcaster']
    assert broadcaster['type'] == 'joint_state_broadcaster/JointStateBroadcaster'


def test_broadcaster_publishes_every_state_interface():
    """Check that the broadcaster is not narrowed to a list that leaves the driver gpio out."""
    params = broadcaster_params()
    # It publishes everything while either list is empty; with both set, only those.
    narrowed = bool(params.get('joints')) and bool(params.get('interfaces'))
    assert not narrowed, 'joints and interfaces are both set: driver/fault_code is not published'


def test_fault_code_stays_on_dynamic_joint_states():
    """Check that /dynamic_joint_states is published, under that name."""
    params = broadcaster_params()
    assert params.get('publish_dynamic_joint_states', True) is True
    assert params.get('use_local_topics', False) is False


def test_a_halt_deactivates_every_command_controller():
    """Check that no controller that can move the arm is left out of the halt (#45)."""
    # A broadcaster only reads state interfaces; everything else writes commands.
    command_controllers = {name for name, kind in declared_controllers().items()
                           if not kind.endswith('Broadcaster')}
    assert command_controllers, 'jetson_controllers.yaml declares no command controller'
    missing = command_controllers - set(supervisor_params()['controllers_to_stop'])
    assert not missing, f'not deactivated by a halt: {sorted(missing)}'


def test_a_resume_activates_declared_controllers_and_the_fault_reader():
    """Check the controllers a resume activates, joint_state_broadcaster among them."""
    to_start = supervisor_params()['controllers_to_start']
    unknown = set(to_start) - set(declared_controllers())
    assert not unknown, f'not in jetson_controllers.yaml: {sorted(unknown)}'
    # The controller manager deactivates it with the others when the driver returns an
    # error, and without it the supervisor has no fault code.
    assert 'joint_state_broadcaster' in to_start


def test_supervisor_timeouts_are_positive():
    """Check the timeouts: the node refuses to configure with one that is not positive."""
    params = supervisor_params()
    for name in ('heartbeat_timeout_ms', 'fault_timeout_ms', 'halt_step_timeout_ms',
                 'resume_step_timeout_ms', 'check_period_ms'):
        assert isinstance(params[name], int) and params[name] > 0, name
    # The check must run several times inside the shortest timeout it watches.
    assert params['check_period_ms'] * 4 <= params['heartbeat_timeout_ms']
    assert params['check_period_ms'] * 4 <= params['fault_timeout_ms']
