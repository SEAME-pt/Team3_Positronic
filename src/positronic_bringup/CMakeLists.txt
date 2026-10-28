cmake_minimum_required(VERSION 3.8)
project(positronic_bringup)

# ROS 2 macros for CMake.
find_package(ament_cmake REQUIRED)

# Run the ROS linters in make test.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Expands the real arm xacro and checks its <ros2_control> block against the URDF (#23).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_description test/test_robot_description.py)
endif()

# Copy the folders to install/.
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()