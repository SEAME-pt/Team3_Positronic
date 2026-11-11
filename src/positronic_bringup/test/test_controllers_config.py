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

"""Check the controllers config of the real arm against what the safety supervisor needs."""

import os

import yaml

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'config', 'jetson_controllers.yaml')


def load_config():
    """Return jetson_controllers.yaml as a dict."""
    with open(CONFIG) as file:
        return yaml.safe_load(file)


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
