# AI-generated (Claude, Anthropic).
# Prompt:
#   #133, 26 Nov: with the merged driver (pose from the servo-bus poll, #170/#182) the tap no
#   longer sees the pose, so tap_speed.py's stop travel is target minus a stale pose. Measure the
#   stop travel from the mode's bag instead: /joint_states J1 from each stop_r*_trigger.txt time
#   until J1 changes less than 0.1 deg for 0.3 s. Print one 'stop at' line per stop, the format
#   step_up_check.py reads. On the 24 Nov folders it reads 0.3-0.5 deg more than the tap: the
#   safe side for the step-up rule.
"""Stop travel of J1 from a mode folder's bag, one 'stop at' line per stop_r*_trigger.txt."""

import glob
import math
import os
import sys

from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import JointState

JOINT = 'joint2_to_joint1'
STILL_DEG = 0.1
STILL_S = 0.3
WINDOW_S = 1.5


def read_j1(bag):
    """(times, J1 deg) from /joint_states in the bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/joint_states']))
    times, j1 = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, JointState)
        if JOINT in msg.name:
            times.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            j1.append(math.degrees(msg.position[msg.name.index(JOINT)]))
    return times, j1


def stop_line(name, t_stop, times, j1):
    """The 'stop at' line of one stop, or None when the bag does not cover it."""
    first = next((k for k, t in enumerate(times) if t >= t_stop), None)
    if first is None or first < 3:
        return None
    speed = (j1[first] - j1[first - 3]) / (times[first] - times[first - 3])
    after = [(t, a) for t, a in zip(times[first:], j1[first:]) if t < t_stop + WINDOW_S]
    t_still, a_still = after[-1]
    for k, (t, a) in enumerate(after):
        near = [b for (u, b) in after[k:] if u < t + STILL_S]
        if max(near) - min(near) < STILL_DEG:
            t_still, a_still = t, a
            break
    return (f'stop at {name} (bag): J1 {abs(speed):.1f} deg/s before, travel after '
            f'{abs(a_still - j1[first]):.2f} deg (J1), still after '
            f'{1000 * (t_still - t_stop):.0f} ms')


def main(folder):
    bags = glob.glob(f'{folder}/bag/*.mcap')
    triggers = sorted(glob.glob(f'{folder}/stop_r*_trigger.txt'))
    if not bags or not triggers:
        return 0
    times, j1 = read_j1(bags[0])
    for path in triggers:
        name = os.path.basename(path).split('_')[1]
        with open(path, encoding='utf-8') as f:
            line = stop_line(name, float(f.read().split()[0]), times, j1)
        if line:
            print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
