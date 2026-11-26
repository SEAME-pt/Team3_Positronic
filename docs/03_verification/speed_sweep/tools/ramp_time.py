# AI-generated (Claude, Anthropic).
# Prompt:
#   #133 "Ramp": from the bags of the sweep, the time a joint takes from the start of a segment
#   to its steady speed, at each byte. /arm_controller/controller_state has the JTC's reference
#   (positions only in our goals, so its velocity steps at a segment start) and the feedback
#   position. Per segment of a joint: the start is where the reference speed jumps above 1
#   deg/s, the plateau is the median reference speed of the segment, and the ramp is the time
#   until the speed from the feedback positions (over 3 samples, ~100 ms) first reaches 90 % of
#   it. Print, per joint and commanded speed, the number of segments and the median and range
#   of the ramp, in ms.
"""Ramp time per joint and commanded speed, from a sweep bag's controller_state."""

import argparse
import math
import statistics
import sys

from control_msgs.msg import JointTrajectoryControllerState
from rclpy.serialization import deserialize_message
import rosbag2_py

TOPIC = '/arm_controller/controller_state'
MOVING_DEG_S = 1.0
REACHED = 0.9
SPAN = 3


def read_states(bag):
    """(names, rows): rows of (t, reference velocities, feedback positions)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    names, rows = None, []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, JointTrajectoryControllerState)
        if not msg.reference.velocities or not msg.feedback.positions:
            continue
        names = list(msg.joint_names)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rows.append((t, list(msg.reference.velocities), list(msg.feedback.positions)))
    return names, rows


def ramps(rows, joint):
    """(commanded deg/s, ramp ms) for each segment of one joint."""
    out = []
    k = SPAN
    while k < len(rows):
        ref = math.degrees(rows[k][1][joint])
        before = math.degrees(rows[k - 1][1][joint])
        if abs(ref) < MOVING_DEG_S or abs(before) >= MOVING_DEG_S:
            k += 1
            continue
        start = k
        end = start
        while end < len(rows) and abs(math.degrees(rows[end][1][joint])) >= MOVING_DEG_S:
            end += 1
        plateau = statistics.median(abs(math.degrees(r[1][joint])) for r in rows[start:end])
        ramp = None
        for m in range(start, end):
            t0, p0 = rows[m - SPAN][0], rows[m - SPAN][2][joint]
            t1, p1 = rows[m][0], rows[m][2][joint]
            if t1 > t0 and abs(math.degrees(p1 - p0) / (t1 - t0)) >= REACHED * plateau:
                ramp = 1000 * (t1 - rows[start][0])
                break
        if ramp is not None:
            out.append((plateau, ramp))
        k = end
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--joints', default='joint2_to_joint1,joint4_to_joint3')
    args = parser.parse_args(argv)
    names, rows = read_states(args.bag)
    if not rows:
        print(f'{args.bag}: no {TOPIC}', file=sys.stderr)
        return 1
    for joint_name in args.joints.split(','):
        groups = {}
        for speed, ramp in ramps(rows, names.index(joint_name)):
            groups.setdefault(round(speed * 2) / 2, []).append(ramp)
        for speed in sorted(groups):
            values = groups[speed]
            print(f'{joint_name} {speed:5.1f} deg/s: {len(values):3d} segments, ramp '
                  f'{statistics.median(values):4.0f} ms ({min(values):.0f}-{max(values):.0f})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
