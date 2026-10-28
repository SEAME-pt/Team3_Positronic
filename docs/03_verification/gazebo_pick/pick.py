#!/usr/bin/env python3
# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Pick the cube of workcell_pick.sdf with MoveIt and the gripper, one wrist camera frame per step.

Needs gazebo.launch.py with world:=workcell_pick.sdf, the three controllers active, and
move_group.launch.py with the planning scene. Exit code 0 when the cube was lifted, 1 otherwise.
"""

import argparse
import math
import re
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, OrientationConstraint, PositionConstraint
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from shape_msgs.msg import SolidPrimitive

WORLD = 'positronic_workcell'
# Cube centre in g_base (the bench top is z = 0), where workcell_pick.sdf puts it.
CUBE = (0.17, 0.0, 0.0925)
# Heights of gripper_base above the cube centre, in m. The palm is 3 cm below gripper_base and
# the finger pads reach 7 cm (collision meshes), so at +0.065 the pads surround the upper half
# of the cube and the palm stays clear of it. Lower, the palm or the pads land on something.
PRE_GRASP = 0.16
GRASP = 0.065
LIFT = 0.20
# The jaw centre sits 4 mm from the gripper_base origin along x.
JAW_OFFSET_X = 0.004
OPEN = 0.1
CLOSED = -0.6
MIN_RISE = 0.05  # m the cube must go up for PASS

# gripper_base with its +y axis (the fingers) pointing down: a rotation of -90 deg about x.
FINGERS_DOWN = Quaternion(x=-math.sin(math.pi / 4), y=0.0, z=0.0, w=math.cos(math.pi / 4))


def cube_z():
    """Return the cube's world z from the Gazebo pose topic, or None if it cannot be read."""
    command = ['bash', '-c', f'timeout 8 gz topic -e -t /world/{WORLD}/pose/info -n 1']
    out = subprocess.run(command, capture_output=True, text=True, check=False).stdout
    match = re.search(r'name: "cube"[\s\S]*?position \{\s*x: [-\d.e]+\s*y: [-\d.e]+\s*z: ([-\d.e]+)',
                      out)
    return float(match.group(1)) if match else None


class Pick(Node):
    """One pick: open, approach, straight down, close, straight up."""

    def __init__(self, out_dir):
        """Connect to MoveIt, the gripper and the wrist camera."""
        super().__init__('gazebo_pick')
        self.out_dir = out_dir
        self.image = None
        self.create_subscription(Image, '/wrist_camera/image_raw', self._on_image, 10)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.gripper = ActionClient(self, GripperCommand, '/gripper_action_controller/gripper_cmd')
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        for name, client in [('move_group', self.move_group), ('execute', self.execute),
                             ('gripper', self.gripper)]:
            if not client.wait_for_server(timeout_sec=60):
                raise RuntimeError(f'{name} action server not found')
        if not self.cartesian.wait_for_service(timeout_sec=30):
            raise RuntimeError('compute_cartesian_path service not found')

    def _on_image(self, message):
        self.image = message

    def spin(self, seconds):
        """Process callbacks for `seconds`."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def save_frame(self, name):
        """Write the latest wrist camera frame as <out_dir>/<name>.png. False if none came."""
        self.spin(1.0)
        if self.image is None:
            self.get_logger().error('no wrist camera image')
            return False
        rgb = np.frombuffer(bytes(self.image.data), np.uint8)
        rgb = rgb.reshape(self.image.height, self.image.width, 3)
        cv2.imwrite(f'{self.out_dir}/{name}.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        return True

    def move_gripper(self, position):
        """Send a GripperCommand goal. A gripper blocked on the object never reports back."""
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 5.0
        future = self.gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20)
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=6)
        if result.result() is None:
            print(f'gripper -> {position:+.2f}: blocked on the object', flush=True)
        else:
            print(f'gripper -> {position:+.2f}: at {result.result().result.position:+.3f}',
                  flush=True)

    def goto(self, xyz, label):
        """Plan and execute a free move of gripper_base to `xyz` in g_base, fingers down."""
        position = PositionConstraint()
        position.header.frame_id = 'g_base'
        position.link_name = 'gripper_base'
        position.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
        centre = Pose()
        centre.position.x, centre.position.y, centre.position.z = xyz
        centre.orientation.w = 1.0
        position.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[centre])
        orientation = OrientationConstraint()
        orientation.header.frame_id = 'g_base'
        orientation.link_name = 'gripper_base'
        orientation.orientation = FINGERS_DOWN
        orientation.absolute_x_axis_tolerance = 0.02
        orientation.absolute_y_axis_tolerance = 0.02
        orientation.absolute_z_axis_tolerance = 0.02
        orientation.weight = 1.0
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.allowed_planning_time = 10.0
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.goal_constraints = [Constraints(position_constraints=[position],
                                                     orientation_constraints=[orientation])]
        for _ in range(3):
            future = self.move_group.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30)
            result = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result, timeout_sec=120)
            code = result.result().result.error_code.val
            print(f'{label} {xyz}: error_code {code}', flush=True)
            if code != -4:  # CONTROL_FAILED: the previous trajectory was still running
                return code == 1
            self.spin(3.0)
        return False

    def straight(self, xyz, label):
        """Move gripper_base to `xyz` in a straight line: the way down onto the cube, and up."""
        target = Pose()
        target.position.x, target.position.y, target.position.z = xyz
        target.orientation = FINGERS_DOWN
        request = GetCartesianPath.Request()
        request.header.frame_id = 'g_base'
        request.group_name = 'arm'
        request.link_name = 'gripper_base'
        request.waypoints = [target]
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.avoid_collisions = False
        request.max_velocity_scaling_factor = 0.3
        request.max_acceleration_scaling_factor = 0.3
        future = self.cartesian.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30)
        path = future.result()
        print(f'{label} {xyz}: straight path {path.fraction:.0%}', flush=True)
        if path.fraction < 0.5:
            return False
        goal = ExecuteTrajectory.Goal(trajectory=path.solution)
        future = self.execute.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30)
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=120)
        code = result.result().result.error_code.val
        print(f'{label}: executed, error_code {code}', flush=True)
        return code == 1


def main():
    """Run the pick and return the exit code."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='.', help='folder for the camera frames')
    args = parser.parse_args()

    start_z = cube_z()
    if start_z is None:
        print('FAIL: cannot read the cube pose from Gazebo', flush=True)
        return 1

    rclpy.init()
    pick = Pick(args.out)
    x, y, z = CUBE
    ok = True
    pick.move_gripper(OPEN)
    ok &= pick.save_frame('1_open')
    ok &= pick.goto((x, y, z + PRE_GRASP), 'pre-grasp')
    ok &= pick.save_frame('2_above_cube')
    ok = ok and pick.straight((x + JAW_OFFSET_X, y, z + GRASP), 'grasp')
    ok &= pick.save_frame('3_at_cube')
    if ok:
        pick.move_gripper(CLOSED)
        ok &= pick.save_frame('4_closed')
    ok = ok and pick.straight((x + JAW_OFFSET_X, y, z + LIFT), 'lift')
    pick.spin(2.0)
    ok &= pick.save_frame('5_lifted')
    rclpy.shutdown()

    end_z = cube_z()
    rise = (end_z - start_z) if end_z is not None else 0.0
    print(f'cube: z {start_z:.3f} -> {end_z if end_z is not None else float("nan"):.3f} m, '
          f'rise {rise:.3f} m, needed {MIN_RISE:.2f}', flush=True)
    passed = ok and rise >= MIN_RISE
    print('PASS' if passed else 'FAIL', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
