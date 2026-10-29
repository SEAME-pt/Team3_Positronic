# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

cmake_minimum_required(VERSION 3.8)
project(positronic_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Expands the Gazebo xacro and checks the joints ros2_control sees, gripper fingers included.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gazebo_description test/test_gazebo_description.py)
endif()

ament_package()
