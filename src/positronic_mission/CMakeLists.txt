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
find_package(rclcpp_components REQUIRED)
find_package(positronic_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  rclcpp_lifecycle
  rclcpp_action
  rclcpp_components
  positronic_interfaces
  geometry_msgs
  lifecycle_msgs
)

include_directories(
  include
)

# add_executable(mission_node src/mission_node.cpp)
# ament_target_dependencies(mission_node ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#  TARGETS mission_node
#  DESTINATION lib/${PROJECT_NAME}
# )

add_library(mission_node_component SHARED
  src/mission_node.cpp
  src/pose_validation.cpp
)

target_include_directories(mission_node_component PUBLIC
  include
)

ament_target_dependencies(mission_node_component
  ${dependencies}
)

rclcpp_components_register_node(mission_node_component
  PLUGIN "positronic_mission::MissionNode"
  EXECUTABLE mission_node
)

install(TARGETS
  mission_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)

  foreach(test_name test_pose_validation test_goals)   # add test_pose_validation test_lifecycle test_failures as you write them
    ament_add_ros_isolated_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} mission_node_component)
    ament_target_dependencies(${test_name} ${dependencies})
  endforeach()
endif()

ament_package()
