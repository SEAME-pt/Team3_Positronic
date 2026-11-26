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
"""Record the raw serial bytes of the gripper commands of the myCobot 280, one section per step."""

import argparse
import hashlib
import os
import platform
import time

import capture_frames as cf
import pymycobot
from pymycobot import MyCobot280

GRIPPER_SPEED = 20
START_READS = 5
READS_PER_TARGET = 3
MOVE_WINDOW_S = 4.0
TARGETS = (
    ("set_gripper_open", 100),
    ("set_gripper_half", 50),
    ("set_gripper_closed", 0),
    ("set_gripper_open_again", 100),
)


def move_gripper(log, mc, name, value):
    cf.confirm(
        f"{name}: the gripper goes to {value} at speed {GRIPPER_SPEED}. "
        "Nothing between the fingers and a hand on the power switch?"
    )
    cf.window(
        log,
        name,
        mc,
        MOVE_WINDOW_S,
        actions=[(0.0, lambda: mc.set_gripper_value(value, GRIPPER_SPEED))],
    )
    label = name[len("set_gripper_"):]
    for i in range(READS_PER_TARGET):
        cf.step(log, f"get_gripper_value_{label}_{i}", mc.get_gripper_value)


def run(log, mc):
    for i in range(START_READS):
        cf.step(log, f"get_gripper_value_start_{i}", mc.get_gripper_value)
    for name, value in TARGETS:
        move_gripper(log, mc, name, value)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--operator", required=True, help="who is running the capture")
    parser.add_argument("--arm", required=True, help="label of the arm being recorded")
    parser.add_argument("--out-dir", default=".", help="where to write the capture file")
    parser.add_argument("--port", default=cf.PORT)
    args = parser.parse_args()

    out_dir = os.path.abspath(args.out_dir)
    if f"{os.sep}positronic{os.sep}" in out_dir + os.sep:
        print(f"WARNING: {out_dir} is inside the folder nano-sync mirrors with --delete.")
        cf.confirm("The capture would be lost on the next make nano-*. Continue anyway?")
    out = os.path.join(out_dir, f"frames_gripper_{time.strftime('%Y%m%d_%H%M%S')}.txt")

    print("Before starting:")
    print("  - the driver is stopped and nobody else has the port (fuser /dev/ttyTHS1)")
    print("  - the arm is powered on and stands still")
    print("  - nothing between the gripper fingers")
    print("  - one person keeps a hand on the power switch the whole time")
    cf.confirm("Ready?")

    mc = MyCobot280(args.port, cf.BAUD)
    time.sleep(0.5)

    with open(out, "w") as log:
        date = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        with open(os.path.abspath(__file__), "rb") as script:
            script_sha = hashlib.sha256(script.read()).hexdigest()
        header = [
            f"# pymycobot {pymycobot.__version__}",
            f"# date: {date}  operator: {args.operator}  arm: {args.arm}",
            f"# port: {args.port}  baud: {cf.BAUD}  python: {platform.python_version()}",
            f"# script: scripts/capture_frames/capture_gripper.py  sha256: {script_sha}",
            f"# gripper_speed: {GRIPPER_SPEED}",
            "# TXT/RXT: bursts of bytes, ms after the first TX of the section",
            "# (or after the section start when nothing was sent)",
        ]
        log.write("\n".join(header) + "\n\n")
        try:
            run(log, mc)
        except BaseException:
            cf.emergency_stop(mc)
            raise

    print(f"\nSaved to {out}")
    print("The gripper is open. Copy the file to src/positronic_protocol/test/data/.")


if __name__ == "__main__":
    main()
