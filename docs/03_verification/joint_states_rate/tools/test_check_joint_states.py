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
#   Write a pytest suite for check_joint_states.py that needs no ROS 2: build synthetic
#   /joint_states samples in memory and test the pure checks. Cover: the URDF joint limits are
#   read from the real mycobot_280_jn.urdf in file order; period statistics on a steady stream
#   (rate, p50, max and where it happened); a clean 10-minute stream at the nominal rate passes
#   every criterion; a recording shorter than 10 minutes fails duration; a mean rate off by more
#   than the tolerance fails rate; a single gap above 2x the nominal period fails periods and is
#   located in time; a wrong or missing joint name fails names; positions in degrees, outside
#   the limits, or NaN fail radians; the pymycobot reference passes when it matches and fails
#   when the driver publishes degrees; and leading all-zero samples (the start pose published
#   before the first serial reply) are counted and left out of the reference comparison.

"""Unit tests for the /joint_states checks of issue #23 (no ROS 2 needed)."""

import math
import os

import check_joint_states as check
import pytest

REPO_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..')
URDF = os.path.join(
    REPO_ROOT, 'src', 'positronic_description',
    'urdf', 'mycobot_280_jn_sim', 'mycobot_280_jn.urdf')
JOINTS = ('joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6')
POSE = (-0.0014, -0.0030, -0.0106, -0.0215, 0.0075, 0.0106)  # a real start pose, in radians
RATE = 50.0
PERIOD_NS = int(check.NS_PER_S / RATE)


def limits():
    """Return the joint limits of the real URDF."""
    with open(URDF, encoding='utf-8') as urdf:
        return check.urdf_joint_limits(urdf.read())


def stream(seconds=600.0, period_ns=PERIOD_NS, names=JOINTS, positions=POSE):
    """Return a steady stream of identical samples covering `seconds`."""
    count = int(seconds * check.NS_PER_S / period_ns) + 1
    return [check.Sample(i * period_ns, i * period_ns, names, positions) for i in range(count)]


def verdicts(samples, **options):
    """Return {criterion name: passed} for the samples."""
    return {c.name: c.passed for c in check.evaluate(samples, RATE, limits(), **options)}


def test_reads_the_six_urdf_joints_in_order():
    """Check that the revolute joints of the real URDF are read, in file order."""
    joint_limits = limits()
    assert tuple(joint_limits) == JOINTS
    assert joint_limits['joint2_to_joint1'] == (-2.9321, 2.9321)


def test_period_stats_of_a_steady_stream():
    """Check rate, median and longest period on a stream with one late message."""
    stamps = [i * PERIOD_NS for i in range(101)]
    stamps[50] += PERIOD_NS // 2  # one message 10 ms late
    stats = check.period_stats(stamps)
    assert stats.mean_rate_hz == pytest.approx(RATE)
    assert stats.p50_ms == pytest.approx(20.0)
    assert stats.max_ms == pytest.approx(30.0)
    assert stats.max_at_s == pytest.approx(1.01)


def test_clean_ten_minutes_passes_everything():
    """Check that a steady 10-minute stream at the nominal rate passes every criterion."""
    assert verdicts(stream()) == {
        'duration': True, 'rate': True, 'periods': True, 'names': True, 'radians': True}


def test_short_recording_fails_duration():
    """Check that 5 minutes is not enough."""
    assert verdicts(stream(seconds=300.0))['duration'] is False


def test_wrong_rate_fails_rate():
    """Check that 40 Hz against a nominal 50 Hz fails the rate criterion."""
    assert verdicts(stream(period_ns=int(check.NS_PER_S / 40.0)))['rate'] is False


def test_one_gap_above_twice_nominal_fails_periods():
    """Check that a single 45 ms gap (nominal 20 ms) fails and is located in time."""
    samples = stream()
    for sample in samples[1000:]:
        sample.receive_ns += 25_000_000  # message 1000 arrives 25 ms late: a 45 ms period
    result = {c.name: c for c in check.evaluate(samples, RATE, limits())}
    assert result['periods'].passed is False
    assert '1 periods above 40.0 ms' in result['periods'].detail
    assert '45.0 ms at 20.0 s' in result['periods'].detail


def test_gap_of_exactly_twice_nominal_passes():
    """Check that a period equal to 2x nominal is still accepted."""
    samples = stream()
    for sample in samples[1000:]:
        sample.receive_ns += PERIOD_NS
    assert verdicts(samples)['periods'] is True


@pytest.mark.parametrize('names', [
    JOINTS[:5],
    JOINTS[:5] + ('joint7',),
    JOINTS + ('gripper',),
])
def test_wrong_names_fail(names):
    """Check that a missing, renamed or extra joint fails the names criterion."""
    samples = stream(seconds=1.0)
    samples[10] = check.Sample(samples[10].receive_ns, samples[10].header_ns, names, POSE)
    assert verdicts(samples, min_duration_s=0.0)['names'] is False


def test_names_in_another_order_pass():
    """Check that the criterion is the set of names, not their order."""
    reordered = tuple(reversed(JOINTS))
    samples = stream(seconds=1.0, names=reordered, positions=tuple(reversed(POSE)))
    assert verdicts(samples, min_duration_s=0.0)['names'] is True


@pytest.mark.parametrize('positions', [
    tuple(math.degrees(p) + 30.0 for p in POSE),  # degrees, far from zero
    (0.0, 3.0, 0.0, 0.0, 0.0, 0.0),  # J2 beyond its 2.4434 rad limit
    (0.0, math.nan, 0.0, 0.0, 0.0, 0.0),
])
def test_bad_positions_fail_radians(positions):
    """Check that degrees, out-of-limit values and NaN fail the radians criterion."""
    assert verdicts(stream(seconds=1.0, positions=positions), min_duration_s=0.0)['radians'] \
        is False


def test_reference_matches_pymycobot():
    """Check that published radians match pymycobot degrees within tolerance."""
    reference = [math.degrees(p) for p in POSE]
    assert verdicts(stream(seconds=1.0), min_duration_s=0.0,
                    reference_deg=reference)['reference'] is True


def test_reference_catches_degrees_published_as_radians():
    """Check that a driver publishing degrees fails, even near zero."""
    reference = [-0.08, -0.17, -0.61, -1.23, 0.43, 0.61]  # get_angles_19 of the capture
    samples = stream(seconds=1.0, positions=tuple(reference))  # degrees used as radians
    assert verdicts(samples, min_duration_s=0.0, reference_deg=reference)['reference'] is False


def test_reference_ignores_the_leading_start_pose():
    """Check that all-zero samples before the first serial reply do not bias the reference."""
    samples = stream(seconds=1.0)
    for sample in samples[:10]:
        sample.positions = (0.0,) * 6
    reference = [math.degrees(p) for p in POSE]
    result = {c.name: c for c in check.evaluate(
        samples, RATE, limits(), min_duration_s=0.0, reference_deg=reference)}
    assert result['reference'].passed is True
    assert 'worst' in result['reference'].detail


def test_counts_leading_zero_samples():
    """Check that the start pose published before the first serial reply is counted."""
    samples = stream(seconds=1.0)
    zeros = (0.0,) * 6
    for sample in samples[:3]:
        sample.positions = zeros
    assert check.leading_zero_samples(samples) == 3


def test_parse_reference_accepts_pymycobot_output():
    """Check that the list printed by pymycobot can be pasted as is."""
    assert check.parse_reference('[-0.08, -0.17, -0.61, -1.23, 0.43, 0.61]') == [
        -0.08, -0.17, -0.61, -1.23, 0.43, 0.61]
