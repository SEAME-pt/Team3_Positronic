# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# AI-generated (Claude, Anthropic).
# Prompt:
#   Test PlanningSceneLoader.apply() and main() without MoveIt (#46, test review C1): a fake
#   /apply_planning_scene answers success or refuses, or is not there at all. apply() must
#   send every object as a diff and return True only on success; main() must exit 1 on failure.
#
# Follow-up prompt (#170, a flaky test found while running make test):
#   The first test of the file failed twice under make test, at 3.1 s, and passes alone in
#   1.7 s: with every package testing at once, finding the fake service took more than the
#   2 s the tests that expect it gave it. Give those tests a bound that only a missing service
#   reaches; the no-service tests keep their 0.2 s.

"""
Check the planning scene loader against a fake move_group service.

The fake lives in its own rclpy context, spun in a thread, so the loader under test runs in the
default context exactly as in main(). Every test remaps /apply_planning_scene to a name unique to
this process, so a fake from another test run on the same machine is never found.
"""

import importlib.util
import os
import sys
import threading
import time

from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(PKG, 'scripts', 'planning_scene.py')

SERVICE = f'/test_{os.getpid()}/apply_planning_scene'
REMAP = ['-r', f'/apply_planning_scene:={SERVICE}']
# The tests that expect the fake service: wait_for_service() returns once it is found, so a long
# bound costs nothing (2 s failed under make test). The no-service tests use NO_WAIT.
TIMEOUT = ['-p', 'timeout_sec:=15.0']
NO_WAIT = ['-p', 'timeout_sec:=0.2']


def load_script():
    """Import the script as a module, without running its main()."""
    spec = importlib.util.spec_from_file_location('planning_scene', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeMoveGroup:
    """An /apply_planning_scene service that answers `success` and keeps every request."""

    def __init__(self, success):
        self.success = success
        self.requests = []
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('fake_move_group', context=self.context)
        self.node.create_service(ApplyPlanningScene, SERVICE, self.on_request)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def on_request(self, request, response):
        self.requests.append(request)
        response.success = self.success
        return response

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.thread.join(timeout=2.0)


@pytest.fixture
def fake_ok():
    fake = FakeMoveGroup(success=True)
    yield fake
    fake.close()


@pytest.fixture
def fake_refuses():
    fake = FakeMoveGroup(success=False)
    yield fake
    fake.close()


def make_loader(args):
    """Initialise the default context with `args` and build the loader in it."""
    rclpy.init(args=['planning_scene.py', '--ros-args', *args])
    return load_script().PlanningSceneLoader()


def close_loader(node):
    node.destroy_node()
    rclpy.try_shutdown()


def test_apply_sends_every_object_as_a_diff(fake_ok):
    """move_group accepts: apply() is True and the request carries the whole scene file."""
    node = make_loader([*REMAP, *TIMEOUT])
    try:
        assert node.apply() is True
    finally:
        close_loader(node)

    assert len(fake_ok.requests) == 1
    scene = fake_ok.requests[0].scene
    assert scene.is_diff, 'a full scene would wipe what move_group already has'
    mod = load_script()
    expected = mod.build_collision_objects(mod.load_scene_file(
        os.path.join(PKG, 'config', 'planning_scene.yaml')))
    assert [o.id for o in scene.world.collision_objects] == [o.id for o in expected]
    assert all(o.operation == CollisionObject.ADD for o in scene.world.collision_objects)


def test_apply_with_remove_sends_remove(fake_ok):
    """remove:=true reaches the request: every object is removed, not added."""
    node = make_loader([*REMAP, *TIMEOUT, '-p', 'remove:=true'])
    try:
        assert node.apply() is True
    finally:
        close_loader(node)

    objects = fake_ok.requests[0].scene.world.collision_objects
    assert objects
    assert all(o.operation == CollisionObject.REMOVE for o in objects)


def test_apply_is_false_when_move_group_refuses(fake_refuses):
    """move_group answers success = false: the loader must not report the scene as loaded."""
    node = make_loader([*REMAP, *TIMEOUT])
    try:
        assert node.apply() is False
    finally:
        close_loader(node)
    assert len(fake_refuses.requests) == 1


def test_apply_is_false_quickly_without_move_group():
    """No service: apply() gives up after timeout_sec instead of waiting forever."""
    node = make_loader([*REMAP, *NO_WAIT])
    try:
        start = time.monotonic()
        assert node.apply() is False
        # 0.2 s timeout; 3 s leaves room for a slow runner and still catches the 30 s default.
        assert time.monotonic() - start < 3.0
    finally:
        close_loader(node)


def test_main_exits_0_when_the_scene_is_loaded(fake_ok, monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['planning_scene.py', '--ros-args', *REMAP, *TIMEOUT])
    assert load_script().main() == 0
    assert len(fake_ok.requests) == 1


def test_main_exits_1_without_move_group(monkeypatch):
    """The exit code is the only sign a launch gets that the obstacles are missing."""
    monkeypatch.setattr(sys, 'argv', ['planning_scene.py', '--ros-args', *REMAP, *NO_WAIT])
    assert load_script().main() == 1
    assert not rclpy.ok(), 'main() must shut rclpy down'
