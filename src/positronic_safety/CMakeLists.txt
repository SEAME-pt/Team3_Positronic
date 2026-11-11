cmake_minimum_required(VERSION 3.8)
project(positronic_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# The decisions of the supervisor are header-only and have no ROS in them, so they are unit
# tested without a node: deadline_watchdog.hpp and supervisor_logic.hpp.
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Pure-logic unit test for the timeouts (heartbeat, fault data, request in flight).
  ament_add_gtest(test_deadline_watchdog test/test_deadline_watchdog.cpp)
  target_include_directories(test_deadline_watchdog PRIVATE include)

  # Pure-logic unit test for the halt and resume order and the stay-stopped rule.
  ament_add_gtest(test_supervisor_logic test/test_supervisor_logic.cpp)
  target_include_directories(test_supervisor_logic PRIVATE include)
endif()

ament_package()
