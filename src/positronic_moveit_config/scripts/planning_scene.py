#!/usr/bin/env python3
# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
import yaml


def _pose_from_dict(d):
    """Build a geometry_msgs/Pose from a dict of floats (missing keys default to identity)."""
    p = Pose()
    p.position.x = float(d.get('x', 0.0))
    p.position.y = float(d.get('y', 0.0))
    p.position.z = float(d.get('z', 0.0))
    p.orientation.x = float(d.get('qx', 0.0))
    p.orientation.y = float(d.get('qy', 0.0))
    p.orientation.z = float(d.get('qz', 0.0))
    p.orientation.w = float(d.get('qw', 1.0))
    return p


def _primitive_from_object(obj):
    """Build a shape_msgs/SolidPrimitive from one yaml object entry (box only)."""
    kind = obj['type']
    if kind != 'box':
        raise ValueError(
            f"unsupported object type '{kind}' (id={obj.get('id')}); only 'box' is supported"
        )
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(v) for v in obj['size']]
    return prim


def build_collision_objects(cfg, remove=False):
    """Return a list of CollisionObject built from a parsed planning-scene yaml config."""
    frame_id = cfg.get('frame_id', 'g_base')
    operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
    objects = []
    for obj in cfg.get('objects', []):
        co = CollisionObject()
        co.header.frame_id = frame_id
        co.id = obj['id']
        co.operation = operation
        if not remove:
            co.primitives = [_primitive_from_object(obj)]
            co.primitive_poses = [_pose_from_dict(obj.get('pose', {}))]
        objects.append(co)
    return objects


def load_scene_file(path):
    """Parse the planning-scene yaml file."""
    with open(path, encoding='utf-8') as fh:
        return yaml.safe_load(fh)


class PlanningSceneLoader(Node):
    """Push the workcell obstacles into move_group, then exit."""

    def __init__(self):
        super().__init__('planning_scene_loader')
        default_file = os.path.join(
            get_package_share_directory('positronic_moveit_config'),
            'config',
            'planning_scene.yaml',
        )
        self.declare_parameter('scene_file', default_file)
        self.declare_parameter('remove', False)
        # move_group needs a moment to create its service; retry instead of failing.
        self.declare_parameter('timeout_sec', 30.0)

        self._scene_file = self.get_parameter('scene_file').value
        self._remove = bool(self.get_parameter('remove').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)
        self._client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def apply(self):
        cfg = load_scene_file(self._scene_file)
        objects = build_collision_objects(cfg, remove=self._remove)

        if not self._client.wait_for_service(timeout_sec=self._timeout):
            self.get_logger().error(
                '/apply_planning_scene not available: is move_group running?'
            )
            return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        result = future.result()
        if result is None or not result.success:
            self.get_logger().error('move_group refused the planning scene update')
            return False

        verb = 'removed from' if self._remove else 'loaded into'
        ids = ', '.join(o.id for o in objects)
        self.get_logger().info(f'{len(objects)} objects {verb} the planning scene: {ids}')
        return True


def main(argv=None):
    rclpy.init(args=sys.argv)
    node = PlanningSceneLoader()
    try:
        ok = node.apply()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
