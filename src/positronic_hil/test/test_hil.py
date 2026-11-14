# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import math

from conftest import ARM_CONTROLLER, CONTROLLER_TIMEOUT_S, JOINTS, MAX_DRIFT, max_drift
from lifecycle_msgs.msg import State

MIN_RATE_HZ = 28.0
MAX_GAP_S = 0.2
HOLD_S = 60.0
CYCLES = 3


def test_read_and_hold(arm):
    assert arm.component_state() == 'active'
    reference = arm.samples[0][1]
    assert set(reference) == set(JOINTS)

    samples = arm.record(HOLD_S)

    rate = len(samples) / HOLD_S
    assert rate >= MIN_RATE_HZ, f'/joint_states at {rate:.1f} Hz'
    gap = max(b[0] - a[0] for a, b in zip(samples, samples[1:]))
    assert gap <= MAX_GAP_S, f'/joint_states silent for {gap:.2f} s'
    assert all(math.isfinite(p) for _, pose in samples for p in pose.values()), 'NaN'
    drift = max_drift(samples, reference)
    assert drift <= MAX_DRIFT, f'the arm moved {math.degrees(drift):.2f} deg'


def test_stop_and_resume(arm):
    assert arm.wait_controller_active(ARM_CONTROLLER, CONTROLLER_TIMEOUT_S), \
        f'{ARM_CONTROLLER} not active after {CONTROLLER_TIMEOUT_S:.0f} s'
    reference = arm.samples[0][1]
    for cycle in range(CYCLES):
        reply = arm.set_component_state(State.PRIMARY_STATE_INACTIVE, 'inactive')
        assert reply.ok, f'cycle {cycle}: deactivate (stop) failed'
        reply = arm.switch_controllers(deactivate=[ARM_CONTROLLER])
        assert reply.ok, f'cycle {cycle}: {ARM_CONTROLLER} did not deactivate'
        reply = arm.set_component_state(State.PRIMARY_STATE_ACTIVE, 'active')
        assert reply.ok, f'cycle {cycle}: activate (power_on) failed'
        reply = arm.switch_controllers(activate=[ARM_CONTROLLER])
        assert reply.ok, f'cycle {cycle}: {ARM_CONTROLLER} did not activate'
    assert arm.component_state() == 'active'
    assert arm.controller_state(ARM_CONTROLLER) == 'active'

    samples = arm.record(2.0)

    assert samples, 'no /joint_states after the cycles'
    drift = max_drift(samples, reference)
    assert drift <= MAX_DRIFT, f'the arm moved {math.degrees(drift):.2f} deg'
