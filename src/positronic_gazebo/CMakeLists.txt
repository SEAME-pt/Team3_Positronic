# Nothing to compile. Only installs the urdf, config and launch folders.
cmake_minimum_required(VERSION 3.8)
project(positronic_gazebo)

# ROS 2 macros for CMake.
find_package(ament_cmake REQUIRED)

# Run the ROS linters in make test.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Copy the folders to install/.
install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

ament_package()
