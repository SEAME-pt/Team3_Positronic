#!/usr/bin/env python3
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
#   Experiment E5 for #144: split the full clang-tidy run over N CI runners. Read the files
#   run-clang-tidy would check (build/compile_commands.json filtered by the same regex as
#   make tidy-strict), give each file its measured clang-tidy cost, and split them into N shards
#   with the longest-first greedy rule so each shard costs about the same. Print the regex
#   run-clang-tidy takes for one shard. A file with no measured cost gets the median, so a new
#   file is still checked by exactly one shard.

"""Split the clang-tidy files into balanced shards (experiment E5, #144)."""

import argparse
import json
import re
import statistics
import sys

# CPU seconds per file: clang-tidy-18, 4 files at a time, local container, 2026-11-18
# (#144 notes, section 1). Only the ratios matter.
COSTS = {
    'positronic_hardware/src/positronic_hardware.cpp': 63.6,
    'positronic_hardware/src/serial_arm.cpp': 10.6,
    'positronic_hardware/test/hil_link_timeout.cpp': 5.6,
    'positronic_hardware/test/test_joint_limit_check.cpp': 27.5,
    'positronic_hardware/test/test_link_watchdog.cpp': 10.0,
    'positronic_hardware/test/test_load_robot_system.cpp': 183.1,
    'positronic_hardware/test/test_reply_assembler.cpp': 11.3,
    'positronic_hardware/test/test_serial_arm.cpp': 78.9,
    'positronic_hardware/test/test_servo_temperature.cpp': 27.7,
    'positronic_hardware/test/test_velocity_estimator.cpp': 26.6,
    'positronic_mission/src/mission_node.cpp': 59.2,
    'positronic_mission/src/moveit_backend.cpp': 100.0,
    'positronic_mission/src/pose_validation.cpp': 7.7,
    'positronic_mission/test/test_failures.cpp': 86.9,
    'positronic_mission/test/test_goals.cpp': 112.7,
    'positronic_mission/test/test_lifecycle.cpp': 76.6,
    'positronic_mission/test/test_pose_validation.cpp': 62.6,
    'positronic_protocol/src/decode.cpp': 1.7,
    'positronic_protocol/src/encode.cpp': 3.5,
    'positronic_protocol/src/units.cpp': 3.7,
    'positronic_protocol/test/test_decode.cpp': 57.9,
    'positronic_protocol/test/test_encode.cpp': 19.7,
    'positronic_protocol/test/test_recorded_frames.cpp': 31.6,
    'positronic_protocol/test/test_smoke.cpp': 8.5,
    'positronic_protocol/test/test_types.cpp': 19.5,
    'positronic_protocol/test/test_units.cpp': 24.9,
    'positronic_safety/src/safety_supervisor.cpp': 70.1,
    'positronic_safety/src/safety_supervisor_main.cpp': 34.2,
    'positronic_safety/test/test_deadline_watchdog.cpp': 10.9,
    'positronic_safety/test/test_fault_code.cpp': 13.0,
    'positronic_safety/test/test_safety_supervisor.cpp': 70.9,
    'positronic_safety/test/test_supervisor_logic.cpp': 38.1,
}


def cost(path):
    """Measured cost of a file, the median for a file not in COSTS."""
    for suffix, seconds in COSTS.items():
        if path.endswith('/' + suffix):
            return seconds
    return statistics.median(COSTS.values())


def split(files, shards):
    """Longest file first, each to the shard with the least cost so far."""
    buckets = [[0.0, []] for _ in range(shards)]
    for path in sorted(files, key=lambda p: (-cost(p), p)):
        bucket = min(buckets, key=lambda b: b[0])
        bucket[0] += cost(path)
        bucket[1].append(path)
    return buckets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--db', default='build/compile_commands.json')
    parser.add_argument('--filter', required=True, help='regex of make tidy-strict')
    parser.add_argument('--shards', type=int, default=3)
    parser.add_argument('--shard', type=int, required=True, help='0 to shards - 1')
    args = parser.parse_args()

    with open(args.db) as db:
        files = sorted({entry['file'] for entry in json.load(db)
                        if re.search(args.filter, entry['file'])})
    buckets = split(files, args.shards)
    for index, (total, paths) in enumerate(buckets):
        print(f'shard {index}: {len(paths)} files, {total:.0f} s', file=sys.stderr)
        for path in paths:
            print(f'  {cost(path):6.1f}  {path}', file=sys.stderr)
    print(f'all shards: {len(files)} files', file=sys.stderr)
    mine = buckets[args.shard][1]
    print('^(' + '|'.join(re.escape(p) for p in mine) + ')$')


if __name__ == '__main__':
    main()
