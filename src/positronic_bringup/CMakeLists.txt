# Nothing to compile. Installs the launch, config, urdf and rviz folders and the scripts.
cmake_minimum_required(VERSION 3.8)
project(positronic_bringup)

# ROS 2 macros for CMake.
find_package(ament_cmake REQUIRED)

# Run the ROS linters in make test.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Copy the folders to install/.
install(DIRECTORY launch config urdf rviz DESTINATION share/${PROJECT_NAME})

# Scripts go to lib/, where `ros2 run` looks for executables.
install(PROGRAMS scripts/joint_states_delay.py DESTINATION lib/${PROJECT_NAME})

ament_package()
