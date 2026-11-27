# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

cmake_minimum_required(VERSION 3.8)
project(positronic_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/planning_scene.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_planning_scene test/test_planning_scene.py TIMEOUT 60)
  ament_add_pytest_test(test_planning_scene_node test/test_planning_scene_node.py TIMEOUT 60)
  # The mission node gets robot_description_planning from joint_limits.yaml (#214).
  ament_add_pytest_test(test_mission_launch test/test_mission_launch.py TIMEOUT 60)
endif()

ament_package()
