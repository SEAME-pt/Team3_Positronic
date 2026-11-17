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
#   #45: write the script for the fault-injection tests of the safety supervisor on the arm.
#   It listens to /safety_supervisor/status, /joint_states and /mission/heartbeat and prints
#   one CSV line per halt: the reason, whether the hardware component confirmed the stop, the
#   time the halt requests took, the time from the trigger until every joint is at rest, the
#   speed at the trigger, how far the arm travelled after it, and the time from the last
#   heartbeat to the trigger. Times between messages of the Jetson come from their stamps, so
#   the network delay to the script does not count. Pure functions for the maths, unit-tested
#   without ROS; rclpy is imported only in main().
"""One CSV line per halt of the safety supervisor: how long the arm took to stop (#45)."""

# Run it where the supervisor runs (the Jetson), before the test, and leave it running:
#
#     python3 halt_latency.py | tee halts.csv
#
# Every time the supervisor halts the arm, one line comes out:
#
#   trial                 1, 2, ...
#   reason                why the supervisor halted (SafetyStatus.msg)
#   hardware_stopped      1 when the hardware component confirmed it left ACTIVE
#   halt_ms               trigger -> the three halt requests answered (stamps of the status)
#   rest_ms               trigger -> every joint at rest (stamp of /joint_states)
#   speed_deg_s           fastest joint in the last /joint_states before the trigger
#   travel_deg            how far the joint that moved most went after the trigger
#   heartbeat_to_halt_ms  last heartbeat received -> trigger received, on this machine's clock
#
# "At rest" is every joint slower than --rest-speed for --rest-samples messages in a row; the
# time is the stamp of the first of them. The status and /joint_states are stamped by the same
# machine, so halt_ms and rest_ms do not depend on where this script runs. heartbeat_to_halt_ms
# does: it uses the times the messages arrived here, so it is only right on the Jetson.
# A node killed mid-trajectory sent its last heartbeat before it died, so
# heartbeat_to_halt_ms + rest_ms is an upper bound of "killed -> at rest".

import argparse
from collections import deque
import math
import sys
import time

MODE_STOPPED = 0
MODE_HALTING = 3
REASONS = {
    0: 'NONE', 1: 'STOP_SERVICE', 2: 'HEARTBEAT_LOST', 3: 'DRIVER_FAULT',
    4: 'FAULT_DATA_LOST', 5: 'DEACTIVATED', 6: 'RESUME_FAILED',
}
HEADER = ('trial,reason,hardware_stopped,halt_ms,rest_ms,speed_deg_s,travel_deg,'
          'heartbeat_to_halt_ms')


def stamp_ns(stamp):
    """Return a builtin_interfaces/Time as nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def at_rest(velocities, rest_speed):
    """Return True when every joint is slower than rest_speed (rad/s). NaN is not at rest."""
    return bool(velocities) and all(
        not math.isnan(v) and abs(v) <= rest_speed for v in velocities)


def rest_time_ns(samples, halt_ns, rest_speed, needed):
    """
    Return the stamp of the moment the arm came to rest after halt_ns, or None.

    samples is a list of (stamp_ns, positions, velocities), oldest first. The arm is at rest
    from the first of `needed` samples in a row, stamped at or after halt_ns, that are all
    at rest.
    """
    run_start = None
    run_length = 0
    for stamp, _, velocities in samples:
        if stamp < halt_ns:
            continue
        if at_rest(velocities, rest_speed):
            if run_length == 0:
                run_start = stamp
            run_length += 1
            if run_length >= needed:
                return run_start
        else:
            run_length = 0
    return None


def sample_before(samples, halt_ns):
    """Return the last sample stamped at or before halt_ns, or None."""
    last = None
    for sample in samples:
        if sample[0] > halt_ns:
            break
        last = sample
    return last


def speed_before(samples, halt_ns):
    """Return the speed of the fastest joint (rad/s) just before halt_ns, or None."""
    sample = sample_before(samples, halt_ns)
    if sample is None or not sample[2] or any(math.isnan(v) for v in sample[2]):
        return None
    return max(abs(v) for v in sample[2])


def travel_after(samples, halt_ns, rest_ns):
    """Return how far (rad) the joint that moved most went between halt_ns and rest_ns."""
    start = sample_before(samples, halt_ns)
    end = sample_before(samples, rest_ns)
    if start is None or end is None or len(start[1]) != len(end[1]) or not start[1]:
        return None
    return max(abs(b - a) for a, b in zip(start[1], end[1]))


def ms(nanoseconds):
    """Return nanoseconds as a text in ms with one decimal, empty for None."""
    return '' if nanoseconds is None else f'{nanoseconds / 1e6:.1f}'


def degrees(radians):
    """Return radians as a text in degrees with two decimals, empty for None."""
    return '' if radians is None else f'{math.degrees(radians):.2f}'


def csv_line(trial, reason, hardware_stopped, halt_ns, stopped_ns, samples,
             heartbeat_to_halt_ns, rest_speed, needed):
    """Return the CSV line of one halt."""
    rest_ns = rest_time_ns(samples, halt_ns, rest_speed, needed)
    return ','.join([
        str(trial),
        REASONS.get(reason, str(reason)),
        '1' if hardware_stopped else '0',
        ms(None if stopped_ns is None else stopped_ns - halt_ns),
        ms(None if rest_ns is None else rest_ns - halt_ns),
        degrees(speed_before(samples, halt_ns)),
        degrees(None if rest_ns is None else travel_after(samples, halt_ns, rest_ns)),
        ms(heartbeat_to_halt_ns),
    ])


def main(argv=None):
    """Listen until Ctrl-C and print one CSV line per halt."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rest-speed', type=float, default=0.02,
                        help='rad/s below which a joint counts as at rest (default 0.02)')
    parser.add_argument('--rest-samples', type=int, default=3,
                        help='/joint_states messages in a row at rest (default 3)')
    parser.add_argument('--wait', type=float, default=5.0,
                        help='seconds to wait for the arm to rest before printing (default 5)')
    args = parser.parse_args(argv)

    # Only here, so the functions above can be imported and tested without ROS.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from positronic_interfaces.msg import Heartbeat, SafetyStatus
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node('halt_latency')
    samples = deque(maxlen=2000)  # about a minute of /joint_states at 30 Hz
    state = {'trial': 0, 'halt': None, 'last_heartbeat': None}

    def on_joint_states(message):
        samples.append((stamp_ns(message.header.stamp), tuple(message.position),
                        tuple(message.velocity)))

    def on_heartbeat(_):
        state['last_heartbeat'] = time.monotonic_ns()

    def on_status(message):
        if message.mode == MODE_HALTING:
            arrived = time.monotonic_ns()
            last = state['last_heartbeat']
            state['halt'] = {
                'reason': message.reason,
                'halt_ns': stamp_ns(message.stamp),
                'stopped_ns': None,
                'hardware_stopped': False,
                'heartbeat_to_halt_ns': None if last is None else arrived - last,
                'deadline': time.monotonic() + args.wait,
            }
        elif message.mode == MODE_STOPPED and state['halt'] is not None:
            state['halt']['stopped_ns'] = stamp_ns(message.stamp)
            state['halt']['hardware_stopped'] = message.hardware_stopped

    def report():
        halt = state['halt']
        if halt is None:
            return
        rested = rest_time_ns(list(samples), halt['halt_ns'], args.rest_speed, args.rest_samples)
        done = halt['stopped_ns'] is not None and rested is not None
        if not done and time.monotonic() < halt['deadline']:
            return
        state['trial'] += 1
        print(csv_line(state['trial'], halt['reason'], halt['hardware_stopped'],
                       halt['halt_ns'], halt['stopped_ns'], list(samples),
                       halt['heartbeat_to_halt_ns'], args.rest_speed, args.rest_samples),
              flush=True)
        state['halt'] = None

    latched = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    best_effort = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(SafetyStatus, '/safety_supervisor/status', on_status, latched)
    node.create_subscription(JointState, '/joint_states', on_joint_states, 50)
    node.create_subscription(Heartbeat, '/mission/heartbeat', on_heartbeat, best_effort)
    node.create_timer(0.05, report)

    print(HEADER, flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
