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
#   #45: write the Jetson side of the unattended heartbeat link campaign. Record the arrival
#   time (monotonic clock) of every /mission/heartbeat message with its phase (the node_name
#   heartbeat_source.py puts in it), of every /joint_states message (published on the Jetson,
#   the control channel), and of every halt the supervisor publishes on its status during the
#   run. Stop when the phase "done" arrives, or after --max-minutes. Then write the CSV and
#   print the analysis of analyze_gaps.py, with the recommended rate and timeout. Print a
#   progress line every minute.
"""Record the heartbeat link campaign on the Jetson and print the result (#45)."""

# Run it in the Jetson container (make nano-shell), with supervised.launch.py running, BEFORE
# starting heartbeat_source.py on the laptop:
#
#     python3 record_gaps.py
#
# It waits for the heartbeats, follows the phases by itself and prints the tables and the
# recommendation at the end. The CSV (phase, channel, t_ns) is kept for the record.

import argparse
import sys
import time

import analyze_gaps

DONE = 'done'
MODE_HALTING = 3


def main(argv=None):
    """Record until the source says "done", then analyze."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default=time.strftime('heartbeat_%Y-%m-%d_%H%M.csv'),
                        help='CSV file to write (default heartbeat_<date>_<time>.csv)')
    parser.add_argument('--max-minutes', type=float, default=60.0,
                        help='stop after this long even without "done" (default 60)')
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from positronic_interfaces.msg import Heartbeat, SafetyStatus
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node('record_gaps')
    started_ros = node.get_clock().now()
    rows = []
    state = {'phase': None, 'done': False, 'beats': 0}

    def on_heartbeat(message):
        now = time.monotonic_ns()
        if message.node_name == DONE:
            state['done'] = True
            return
        if message.node_name != state['phase']:
            state['phase'] = message.node_name
            print(time.strftime('%H:%M:%S'), f'phase {message.node_name}', flush=True)
        rows.append((message.node_name, 'heartbeat', now))
        state['beats'] += 1

    def on_joint_states(_):
        rows.append(('', 'joint_states', time.monotonic_ns()))

    def on_status(message):
        # The status is latched: skip the one from before the recording started.
        stamp = rclpy.time.Time.from_msg(message.stamp)
        if message.mode == MODE_HALTING and stamp > started_ros:
            rows.append((state['phase'] or '', 'halt', time.monotonic_ns()))
            print(time.strftime('%H:%M:%S'), f'SUPERVISOR HALT, reason {message.reason}',
                  flush=True)

    best_effort = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Heartbeat, '/mission/heartbeat', on_heartbeat, best_effort)
    node.create_subscription(JointState, '/joint_states', on_joint_states, 50)
    node.create_subscription(SafetyStatus, '/safety_supervisor/status', on_status, latched)

    print('Waiting for heartbeat_source.py on the laptop. Leave this running.', flush=True)
    start = time.monotonic()
    next_report = start + 60.0
    try:
        while not state['done'] and time.monotonic() - start < args.max_minutes * 60:
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.monotonic() >= next_report:
                next_report += 60.0
                print(time.strftime('%H:%M:%S'),
                      f'{state["beats"]} heartbeats so far, phase {state["phase"]}', flush=True)
    except KeyboardInterrupt:
        print('Stopped by hand: analysing what was recorded.', file=sys.stderr)

    with open(args.out, 'w') as out:
        out.write('phase,channel,t_ns\n')
        for phase, channel, t_ns in rows:
            out.write(f'{phase},{channel},{t_ns}\n')
    print(f'\nWrote {args.out}\n', flush=True)
    if not state['beats']:
        print('No heartbeat received.', file=sys.stderr)
        return 1
    print(analyze_gaps.report(analyze_gaps.read_run(args.out)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
