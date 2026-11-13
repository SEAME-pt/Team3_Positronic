# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

cmake_minimum_required(VERSION 3.8)
project(positronic_hil)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_hil test/test_hil.py TIMEOUT 300)
  ament_add_pytest_test(test_hil_trajectory test/test_hil_trajectory.py TIMEOUT 300)
endif()

ament_package()
  