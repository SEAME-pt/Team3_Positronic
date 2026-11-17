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
  positronic_protocol
  realtime_tools
  urdf
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# The plugin: a shared library that pluginlib loads at runtime
add_library(${PROJECT_NAME} SHARED
  src/positronic_hardware.cpp
  src/serial_arm.cpp
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
  positronic_protocol::positronic_protocol
  realtime_tools::realtime_tools
  urdf::urdf
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
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  # M2: load RobotSystem through the ResourceManager, as the controller manager does.
  # It does not link the plugin: pluginlib must find it, so registration is tested too.
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

  # Pure-logic unit test for the velocity estimator: no ROS, no arm, header-only.
  ament_add_gtest(test_velocity_estimator test/test_velocity_estimator.cpp)
  target_include_directories(test_velocity_estimator PRIVATE include)

  # Pure-logic unit test for the joint limit check (#34): no ROS, no arm, header-only.
  ament_add_gtest(test_joint_limit_check test/test_joint_limit_check.cpp)
  target_include_directories(test_joint_limit_check PRIVATE include)

  # Links the library rather than compiling src/serial_arm.cpp a second time.
  ament_add_gtest(test_serial_arm test/test_serial_arm.cpp)
  target_link_libraries(test_serial_arm ${PROJECT_NAME})

  # Pure-logic unit test for the reply assembler: no port, no thread, header-only.
  ament_add_gtest(test_reply_assembler test/test_reply_assembler.cpp)
  target_include_directories(test_reply_assembler PRIVATE include)
  target_link_libraries(test_reply_assembler positronic_protocol::positronic_protocol)

  # Pure-logic unit test for the link watchdog (#35): no port, no thread, header-only.
  ament_add_gtest(test_link_watchdog test/test_link_watchdog.cpp)
  target_include_directories(test_link_watchdog PRIVATE include)

  # Pure-logic unit test for the servo over-temperature cut: no port, no thread, header-only.
  ament_add_gtest(test_servo_temperature test/test_servo_temperature.cpp)
  target_include_directories(test_servo_temperature PRIVATE include)
  # #35 hardware check: built with the tests, run by hand on the Jetson, never by ctest. It
  # moves the real arm (see the header of the file).
  add_executable(hil_link_timeout test/hil_link_timeout.cpp)
  target_link_libraries(hil_link_timeout ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()