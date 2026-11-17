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
#   #45: write the laptop side of an unattended heartbeat link campaign. It stands in for the
#   mission node: it publishes positronic_interfaces/Heartbeat on /mission/heartbeat with the
#   same QoS (best effort, keep last 1) and runs a fixed schedule of phases by itself: 10 Hz
#   quiet, 25 Hz quiet, 25 Hz busy (runs ros2 command-line tools every 30 s, like a person at
#   the laptop), 50 Hz quiet, 10 minutes each. Each heartbeat carries its phase name in
#   node_name, so the Jetson side can tell the phases apart with no clock shared between the
#   machines. At the end it sends "done". It refuses to start while another node publishes the
#   heartbeat. A --confirm mode publishes at one rate for a given time, for the final run with
#   the supervisor active. Publish on an absolute schedule so the period does not drift.
# Follow-up prompt:
#   The busy phases started each ros2 tool from inside the publishing process, which can stall
#   it and inflate the gaps. Run the tools from one separate shell process for the whole phase
#   instead, as a person in another terminal would, and stop it at the end of the phase.
"""Publish the mission heartbeat on a schedule of rates, for the link campaign (#45)."""

# Run it in the laptop container (make shell), with the mission node STOPPED:
#
#     python3 heartbeat_source.py                    # the campaign, 40 minutes
#     python3 heartbeat_source.py --confirm 25 --minutes 15
#
# The Jetson side, record_gaps.py, must be started first.

import argparse
import subprocess
import sys
import time

PHASE_MINUTES = 10
# (phase name, rate in Hz, busy). The name starts with the rate: analyze_gaps.py reads it.
PHASES = (
    ('10hz_quiet', 10, False),
    ('25hz_quiet', 25, False),
    ('25hz_busy', 25, True),
    ('50hz_quiet', 50, False),
)
DONE = 'done'
# What a person does at the laptop during a session: one tool every 30 s. It runs in a shell
# of its own, started once per phase, so the publishing process never starts a program itself.
BUSY_SCRIPT = (
    'while true; do'
    ' sleep 30; ros2 topic list > /dev/null 2>&1;'
    ' sleep 30; ros2 node list > /dev/null 2>&1;'
    ' sleep 30; timeout 20 ros2 topic echo --once /joint_states > /dev/null 2>&1;'
    ' done'
)


def start_busy():
    """Start the shell that runs the ros2 tools; return its process."""
    return subprocess.Popen(['bash', '-c', BUSY_SCRIPT], start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_busy(process):
    """Stop the shell of start_busy() and the tool it may be running."""
    import os
    import signal
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    process.wait(timeout=10)


def publish_phase(publisher, message_type, name, rate_hz, seconds):
    """Publish heartbeats named `name` at `rate_hz` for `seconds`, on an absolute schedule."""
    period = 1.0 / rate_hz
    end = time.monotonic() + seconds
    next_time = time.monotonic()
    sent = 0
    while next_time < end:
        message = message_type()
        message.node_name = name
        publisher.publish(message)
        sent += 1
        next_time += period
        time.sleep(max(0.0, next_time - time.monotonic()))
    return sent


def main(argv=None):
    """Run the campaign, or one confirmation phase."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm', type=int, metavar='HZ',
                        help='publish at this rate only, for the confirmation run')
    parser.add_argument('--minutes', type=float, default=15.0,
                        help='length of the confirmation run (default 15)')
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from positronic_interfaces.msg import Heartbeat

    rclpy.init()
    node = rclpy.create_node('heartbeat_source')
    # Give discovery a moment, then make sure nobody else feeds the supervisor.
    time.sleep(3.0)
    others = node.count_publishers('/mission/heartbeat')
    if others:
        print(f'{others} other node(s) already publish /mission/heartbeat: stop the mission node '
              "(and anyone else's) first.", file=sys.stderr)
        return 1
    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(Heartbeat, '/mission/heartbeat', qos)
    time.sleep(2.0)  # let the Jetson match the new publisher before the first phase

    if args.confirm:
        phases = ((f'{args.confirm}hz_confirm', args.confirm, True),)
        minutes = args.minutes
    else:
        phases = PHASES
        minutes = PHASE_MINUTES
    total = len(phases) * minutes
    print(f'{len(phases)} phase(s) of {minutes:g} min, {total:g} min in all. Leave it running.',
          flush=True)

    try:
        for name, rate, busy in phases:
            print(time.strftime('%H:%M:%S'), f'phase {name}: {rate} Hz'
                  + (', running ros2 tools every 30 s' if busy else ''), flush=True)
            helper = start_busy() if busy else None
            try:
                sent = publish_phase(publisher, Heartbeat, name, rate, minutes * 60)
            finally:
                if helper is not None:
                    stop_busy(helper)
            print(time.strftime('%H:%M:%S'), f'phase {name} done: {sent} heartbeats sent',
                  flush=True)
        publish_phase(publisher, Heartbeat, DONE, 10, 2.0)
    except KeyboardInterrupt:
        print('Stopped before the end.', file=sys.stderr)
        return 1
    print('Finished. The results print on the Jetson.', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
