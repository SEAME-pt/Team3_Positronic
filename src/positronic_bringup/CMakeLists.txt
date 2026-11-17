# Nothing to compile. Installs the launch, config and rviz folders.
cmake_minimum_required(VERSION 3.8)
project(positronic_bringup)

# ROS 2 macros for CMake.
find_package(ament_cmake REQUIRED)

# Run the ROS linters in make test.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  # Expands the real arm xacro and checks its <ros2_control> block against the URDF (#23).
  ament_add_pytest_test(test_robot_description test/test_robot_description.py)
  # The broadcaster settings the safety supervisor depends on to read driver/fault_code (#45).
  ament_add_pytest_test(test_controllers_config test/test_controllers_config.py)
  # Every launch file builds its description: a typo or a missing import fails here, not on
  # the Jetson.
  ament_add_pytest_test(test_launch_files test/test_launch_files.py)
  # The maths of the delay script, without ROS running (#24).
  ament_add_pytest_test(test_joint_states_delay test/test_joint_states_delay.py)
  ament_add_pytest_test(test_joint_states_rate test/test_joint_states_rate.py)
  # main() of the rate script, alone: it initialises rclpy itself, so it needs its own process.
  ament_add_pytest_test(test_joint_states_rate_main test/test_joint_states_rate_main.py)
endif()

# Copy the folders to install/.
install(DIRECTORY launch config worlds rviz DESTINATION share/${PROJECT_NAME})

# Scripts go to lib/, where `ros2 run` looks for executables.
install(PROGRAMS scripts/joint_states_delay.py scripts/joint_states_rate.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
