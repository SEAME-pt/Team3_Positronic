cmake_minimum_required(VERSION 3.8)
project(positronic_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# The plugin: a shared library that pluginlib loads at runtime
add_library(${PROJECT_NAME} SHARED
  src/positronic_hardware.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Registers positronic_hardware.xml with hardware_interface, so the
# ResourceManager can find RobotSystem by the name used in the URDF
pluginlib_export_plugin_description_file(hardware_interface positronic_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # M2: load RobotSystem through the ResourceManager, as the controller manager does.
  # It does not link the plugin: pluginlib must find it, so registration is tested too.
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(test_load_robot_system test/test_load_robot_system.cpp)
  # Only the headers (FakeArm::START_POSE), not the plugin library itself.
  target_include_directories(test_load_robot_system PRIVATE include)
  target_link_libraries(test_load_robot_system
    ament_index_cpp::ament_index_cpp
    hardware_interface::hardware_interface
    ${lifecycle_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
