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
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(positronic_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

# The node, as a library: the executable below only adds main(), and a test can link the node
# instead of compiling it a second time. The decisions it takes are header-only and have no
# ROS in them (deadline_watchdog.hpp, supervisor_logic.hpp).
add_library(${PROJECT_NAME} SHARED
  src/safety_supervisor.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${control_msgs_TARGETS}
  ${controller_manager_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${positronic_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_srvs_TARGETS}
)

# ros2 run positronic_safety safety_supervisor
add_executable(safety_supervisor src/safety_supervisor_main.cpp)
target_link_libraries(safety_supervisor ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS safety_supervisor
  DESTINATION lib/${PROJECT_NAME}
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

  # Finding driver/fault_code in a /dynamic_joint_states message: only the message type, no node.
  ament_add_gtest(test_fault_code test/test_fault_code.cpp)
  target_include_directories(test_fault_code PRIVATE include)
  target_link_libraries(test_fault_code ${control_msgs_TARGETS})

  # The node against a fake controller manager, trajectory controller and mission node, all in
  # one process: the order of the requests, every trigger, the stay-stopped rule. It has its
  # own main() (rclcpp::init), and every test waits for real timeouts, hence the longer limit.
  ament_add_gtest(test_safety_supervisor test/test_safety_supervisor.cpp
    TIMEOUT 120
    SKIP_LINKING_MAIN_LIBRARIES
  )
  target_link_libraries(test_safety_supervisor ${PROJECT_NAME})
endif()

ament_package()
