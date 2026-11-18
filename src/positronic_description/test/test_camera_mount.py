# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# AI-generated (Claude, Anthropic).
# Prompt:
#   #97: on the real arm the camera module is mounted 90 deg from the line the fingers open
#   along, looking into the gap between them so the gripper does not cover the view, on the side
#   the Gazebo model of #91 already has; the raw image is upright once turned 90 deg
#   counterclockwise. Write the URDF check, as test_joint6_zero.py: the four camera variants
#   under mycobot_280_jn_sim give the camera joint, the camera_link frame and (with the gripper)
#   the gripper mount the same origin, and the geometry matches that mount.
"""Check the wrist camera mount in the four camera URDFs, so model and real arm agree (#97)."""

import glob
import math
import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory

CAMERA_JOINT = 'joint6output_to_camera_flange'
CAMERA_LINK_JOINT = 'camera_flange_to_camera_link'
GRIPPER_JOINT = 'joint6output_to_gripper_base'
CAMERA_URDF_COUNT = 4
GRIPPER_CAMERA_URDF_COUNT = 2

# The camera module on the real arm, 90 deg from the line the fingers open along
# (docs/03_verification/camera_mount): the yaw, third rpy value, turns it about the flange axis.
EXPECTED_CAMERA = {'xyz': (0.0, 0.0, 0.01), 'rpy': (1.5708, 0.0, -1.5708)}
# The camera frame inside the camera module. The roll puts the gripper at the bottom of the
# image (#91); the real raw image matches it once turned 90 deg counterclockwise.
EXPECTED_CAMERA_LINK = {'xyz': (0.0415, 0.0083, 0.0), 'rpy': (1.5708, 0.0, 1.5708)}
# The gripper turned 180 deg on the flange, as on the real arm (#94).
EXPECTED_GRIPPER = {'xyz': (0.0, 0.0, 0.0505), 'rpy': (1.5708, 0.0, 3.1416)}

# Tolerance of the angle checks, in degrees. The URDF writes pi/2 as 1.5708, 4e-4 deg off.
ANGLE_TOLERANCE_DEG = 0.01


def camera_urdf_files():
    """Return the arm URDFs with the wrist camera installed by positronic_description."""
    share = get_package_share_directory('positronic_description')
    pattern = os.path.join(share, 'urdf', 'mycobot_280_jn_sim', '*camera*.urdf')
    return sorted(glob.glob(pattern))


def joint_origin(path, name):
    """Return the <origin> of one joint in one URDF as {'xyz': tuple, 'rpy': tuple}."""
    joint = ElementTree.parse(path).getroot().find(f"joint[@name='{name}']")
    assert joint is not None, (os.path.basename(path), name)
    origin = joint.find('origin').attrib
    return {key: tuple(float(value) for value in origin[key].split()) for key in ('xyz', 'rpy')}


def rotation(rpy):
    """Return the 3x3 rotation of a URDF rpy (fixed axes x, y, z), as a list of rows."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def apply(matrix, vector):
    """Return matrix times vector."""
    return [sum(row[i] * vector[i] for i in range(3)) for row in matrix]


def camera_in_flange(camera, camera_link):
    """Return position, optical axis (+x) and image up (+z) of camera_link in joint6_flange.

    Gazebo cameras look along +x of their link, with +z up in the image.
    """
    turn = rotation(camera['rpy'])
    offset = apply(turn, camera_link['xyz'])
    position = [camera['xyz'][i] + offset[i] for i in range(3)]
    link_turn = rotation(camera_link['rpy'])
    axis = apply(turn, apply(link_turn, [1.0, 0.0, 0.0]))
    image_up = apply(turn, apply(link_turn, [0.0, 0.0, 1.0]))
    return position, axis, image_up


def finger_line_in_flange(gripper):
    """Return the line the fingers open along in the joint6_flange frame.

    The finger joints sit at x = -0.005 and x = +0.005 of gripper_base and the fingers grow
    along its y: they open and close along its x axis.
    """
    return apply(rotation(gripper['rpy']), [1.0, 0.0, 0.0])


def angle_about_flange_axis_deg(first, second):
    """Return the angle in degrees, 0 to 180, between two vectors seen along the flange axis."""
    dot = first[0] * second[0] + first[1] * second[1]
    norm = math.hypot(first[0], first[1]) * math.hypot(second[0], second[1])
    return math.degrees(math.acos(max(-1.0, min(1.0, dot / norm))))


def is_across_the_fingers(position, fingers):
    """Return True when the camera sits 90 deg from the line the fingers open along."""
    angle = angle_about_flange_axis_deg(position, fingers)
    return abs(angle - 90.0) < ANGLE_TOLERANCE_DEG


def is_on_the_real_side(position):
    """Return True when the camera sits on -y of joint6_flange.

    Seen from the front at all-zero, that puts the camera module up and to the left, as on the
    real arm (docs/03_verification/camera_mount).
    """
    return position[1] < 0.0


def assert_origin(actual, expected, label):
    """Check one origin against the expected values, to the 4 decimals the URDF writes."""
    for key in ('xyz', 'rpy'):
        assert actual[key] == tuple(round(value, 4) for value in expected[key]), (label, key)


def camera_of(path):
    """Return position, optical axis and image up of the camera of one URDF, in the flange."""
    return camera_in_flange(joint_origin(path, CAMERA_JOINT),
                            joint_origin(path, CAMERA_LINK_JOINT))


def test_all_four_camera_files_are_found():
    """Check the glob finds the four camera variants, so a missing file fails here."""
    files = camera_urdf_files()
    assert len(files) == CAMERA_URDF_COUNT
    with_gripper = [path for path in files if 'gripper' in os.path.basename(path)]
    assert len(with_gripper) == GRIPPER_CAMERA_URDF_COUNT


def test_camera_joint_is_the_real_mount_in_every_file():
    """Check the four files place the camera module where it is on the real arm."""
    for path in camera_urdf_files():
        assert_origin(joint_origin(path, CAMERA_JOINT), EXPECTED_CAMERA, os.path.basename(path))


def test_camera_frame_is_the_same_in_every_file():
    """Check camera_link sits the same in the camera module of the four files."""
    for path in camera_urdf_files():
        assert_origin(joint_origin(path, CAMERA_LINK_JOINT), EXPECTED_CAMERA_LINK,
                      os.path.basename(path))


def test_gripper_mount_is_the_same_in_the_camera_files_with_gripper():
    """Check the gripper sits as on the real arm where the camera is checked against it."""
    for path in camera_urdf_files():
        if 'gripper' in os.path.basename(path):
            assert_origin(joint_origin(path, GRIPPER_JOINT), EXPECTED_GRIPPER,
                          os.path.basename(path))


def test_camera_module_is_across_the_fingers_on_the_real_side():
    """Check the decision of #97 in geometry: 90 deg from the fingers, on the real side."""
    for path in camera_urdf_files():
        if 'gripper' not in os.path.basename(path):
            continue
        position, _, _ = camera_of(path)
        fingers = finger_line_in_flange(joint_origin(path, GRIPPER_JOINT))
        assert is_across_the_fingers(position, fingers), (os.path.basename(path), position)
        assert is_on_the_real_side(position), (os.path.basename(path), position)


def test_camera_looks_along_the_tool_axis():
    """Check the optical axis of camera_link is the flange axis, where the gripper points."""
    for path in camera_urdf_files():
        _, axis, _ = camera_of(path)
        assert math.isclose(axis[2], 1.0, abs_tol=1e-6), (os.path.basename(path), axis)


def test_gripper_is_at_the_bottom_of_the_image():
    """Check image up points away from the flange axis, so the gripper is at the bottom."""
    for path in camera_urdf_files():
        position, _, image_up = camera_of(path)
        radius = math.hypot(position[0], position[1])
        outward = (position[0] * image_up[0] + position[1] * image_up[1]) / radius
        assert math.isclose(outward, 1.0, abs_tol=1e-6), (os.path.basename(path), image_up)


def test_the_check_rejects_the_camera_in_line_with_the_fingers():
    """Check the 90 deg check fails with the camera on the finger line, behind a finger."""
    for yaw in (0.0, 3.1416):
        in_line = {'xyz': (0.0, 0.0, 0.01), 'rpy': (1.5708, 0.0, yaw)}
        position, _, _ = camera_in_flange(in_line, EXPECTED_CAMERA_LINK)
        assert not is_across_the_fingers(position, finger_line_in_flange(EXPECTED_GRIPPER)), yaw


def test_the_check_rejects_the_camera_on_the_other_side():
    """Check the two 90 deg sides are told apart: across the fingers both, real side one."""
    other_side = {'xyz': (0.0, 0.0, 0.01), 'rpy': (1.5708, 0.0, 1.5708)}
    position, _, _ = camera_in_flange(other_side, EXPECTED_CAMERA_LINK)
    assert is_across_the_fingers(position, finger_line_in_flange(EXPECTED_GRIPPER))
    assert not is_on_the_real_side(position)
