cmake_minimum_required(VERSION 3.8)
project(positronic_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(positronic_interfaces REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  rclcpp_lifecycle
  rclcpp_action
  positronic_interfaces
)

include_directories(
  include
)

add_executable(mission_node src/mission_node.cpp)
ament_target_dependencies(mission_node ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mission_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mission_unit
    test/test_mission_unit.cpp
  )
  ament_target_dependencies(test_mission_unit
    ${dependencies}
  )
  target_include_directories(test_mission_unit PRIVATE
    include
  )
endif()

ament_package()
