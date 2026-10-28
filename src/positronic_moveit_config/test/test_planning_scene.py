# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import importlib.util
import os

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(PKG, 'scripts', 'planning_scene.py')
YAML = os.path.join(PKG, 'config', 'planning_scene.yaml')


def _load_script():
    spec = importlib.util.spec_from_file_location('planning_scene', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scene_file_builds_collision_objects():
    mod = _load_script()
    cfg = mod.load_scene_file(YAML)
    objects = mod.build_collision_objects(cfg)

    assert objects, 'planning_scene.yaml must define at least one object'
    assert cfg['frame_id'] == 'g_base'

    by_id = {o.id: o for o in objects}
    walls = {'wall_left', 'wall_right', 'wall_back', 'wall_front'}
    assert {'bench'} | walls <= set(by_id)
    for obj in objects:
        assert obj.header.frame_id == 'g_base'
        assert obj.operation == CollisionObject.ADD
        assert len(obj.primitives) == 1
        assert len(obj.primitive_poses) == 1
        assert obj.primitives[0].type == SolidPrimitive.BOX
        assert len(obj.primitives[0].dimensions) == 3

    bench = by_id['bench']
    top = bench.primitive_poses[0].position.z + bench.primitives[0].dimensions[2] / 2.0
    assert -0.01 < top < 0.0

    reach = 0.28
    for wall_id in walls:
        wall = by_id[wall_id]
        p = wall.primitive_poses[0].position
        sx, sy, _ = wall.primitives[0].dimensions
        inner = min(abs(p.x) - sx / 2.0 if abs(p.x) > abs(p.y) else abs(p.y) - sy / 2.0, 1.0)
        assert reach < inner < 0.35, f'{wall_id} inner face at {inner:.3f} m'
    assert 'roof' in by_id


def test_remove_sets_remove_operation():
    mod = _load_script()
    cfg = mod.load_scene_file(YAML)
    objects = mod.build_collision_objects(cfg, remove=True)
    assert objects
    for obj in objects:
        assert obj.operation == CollisionObject.REMOVE
