# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check the zero of joint 6 in the eight arm URDFs, so model and real arm agree."""

import glob
import math
import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory

JOINT = 'joint6output_to_joint6'
URDF_COUNT = 8

# Measured on the real arm with every joint at zero and only joint 6 released: the flange
# matched the upstream model's zero when joint 6 read this angle (readings of -46, -49 and
# -42, and the mark on the flange sits at 45: the holes then form a square). At zero the real
# gripper, mounted on the diagonal of those holes, sits 45 deg from the links while the
# upstream model's is parallel: flange and gripper are off by the same angle, so it is the
# zero of the joint. The model's zero turns the other way.
MEASURED_AT_MODEL_ZERO_DEG = -45.0

# The origin turns the joint frame by -90 deg about x, so a turn of the zero about the joint
# axis (z) is a turn about y before it: it goes in the pitch, the second rpy value.
EXPECTED_RPY = (-1.5708, math.radians(-MEASURED_AT_MODEL_ZERO_DEG), 0.0)


def urdf_files():
    """Return the arm URDFs installed by positronic_description."""
    share = get_package_share_directory('positronic_description')
    return sorted(glob.glob(os.path.join(share, 'urdf', 'mycobot_280_jn_sim', '*.urdf')))


def joint6_origin(path):
    """Return the <origin> attributes of joint 6 in one URDF, as a dict."""
    joint = ElementTree.parse(path).getroot().find(f"joint[@name='{JOINT}']")
    assert joint is not None, path
    return joint.find('origin').attrib


def test_all_eight_files_have_joint6():
    """Check the glob finds the eight variants, so a missing file fails here."""
    assert len(urdf_files()) == URDF_COUNT


def test_joint6_origin_is_the_same_in_every_file():
    """Check the eight files agree, so Gazebo, RViz and MoveIt use one zero."""
    origins = {os.path.basename(path): joint6_origin(path) for path in urdf_files()}
    first = next(iter(origins.values()))
    for name, origin in origins.items():
        assert origin == first, name


def test_joint6_zero_is_the_measured_offset():
    """Check the rpy of joint 6 carries the offset measured on the arm."""
    for path in urdf_files():
        rpy = [float(value) for value in joint6_origin(path)['rpy'].split()]
        assert rpy == [round(value, 4) for value in EXPECTED_RPY], os.path.basename(path)
