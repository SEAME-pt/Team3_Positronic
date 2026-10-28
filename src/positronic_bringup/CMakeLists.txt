# Nothing to compile. Installs the launch, config and rviz folders.
cmake_minimum_required(VERSION 3.8)
project(positronic_bringup)

# ROS 2 macros for CMake.
find_package(ament_cmake REQUIRED)

# Run the ROS linters in make test.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Expands the real arm xacro and checks its <ros2_control> block against the URDF (#23).
  # The maths of the delay and check scripts, without ROS running.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_description test/test_robot_description.py)
  # The maths of the delay script, without ROS running.
  ament_add_pytest_test(test_joint_states_delay test/test_joint_states_delay.py)
  ament_add_pytest_test(test_joint_states_check test/test_joint_states_check.py)
endif()

# Copy the folders to install/.
install(DIRECTORY launch config worlds rviz DESTINATION share/${PROJECT_NAME})

# Scripts go to lib/, where `ros2 run` looks for executables.
install(PROGRAMS scripts/joint_states_delay.py scripts/joint_states_check.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
