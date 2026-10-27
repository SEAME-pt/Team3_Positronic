// Copyright 2026 Positronic
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// AI-generated (Claude, Anthropic).
// Prompt:
//   Write the M2 load test for the positronic_hardware plugin (#22). Load RobotSystem the
//   way the controller manager does: a ResourceManager (ROS 2 Jazzy, ros2_control 4.48,
//   ResourceManagerParams API) given the real URDF from positronic_description plus a
//   <ros2_control> block, so the test fails if pluginlib registration or the joint names
//   drift. Check that on_init succeeds and exactly the six <joint>/position state
//   interfaces are exported, and that on_init rejects five joints and a joint without a
//   position state interface.
// Follow-up prompt:
//   Add tests for the fake arm: positions are NaN after configure and before any read,
//   read() reports FakeArm::START_POSE, and the component activates and still reads.
// Follow-up prompt:
//   Apply the review of #22: each joint has position and velocity state interfaces and no
//   command interface; test twelve exported states, a joint without velocity, a joint with
//   a command interface, and check loaded_ in the rejection tests.
// Follow-up prompt:
//   on_init reads the <hardware> parameters backend, port, baud and velocity_filter_alpha.
//   Every test URDF selects backend=fake; test that valid values load and that an unknown
//   parameter or a bad value is rejected.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/resource_manager_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "positronic_hardware/fake_arm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace
{

/// Name of the <hardware> component in the test URDFs.
constexpr std::string_view HARDWARE_NAME{"PositronicArm"};

/// Revolute joints of mycobot_280_jn.urdf, J1 (base) to J6 (flange).
const std::vector<std::string> JOINTS{"joint2_to_joint1", "joint3_to_joint2",
  "joint4_to_joint3", "joint5_to_joint4",
  "joint6_to_joint5", "joint6output_to_joint6"};

/// The arm's URDF as installed by positronic_description, without a <ros2_control> block.
std::string read_description()
{
  const std::string path{
    ament_index_cpp::get_package_share_directory("positronic_description") +
    "/urdf/mycobot_280_jn_sim/mycobot_280_jn.urdf"};
  const std::ifstream file{path};
  std::stringstream content;
  content << file.rdbuf();
  return content.str();
}

constexpr std::string_view POSITION{"<state_interface name=\"position\"/>"};
constexpr std::string_view VELOCITY{"<state_interface name=\"velocity\"/>"};
constexpr std::string_view COMMAND{"<command_interface name=\"position\"/>"};

/// One <joint> entry of the <ros2_control> block, with the given interface tags.
std::string joint_block(const std::string & name, std::string_view interfaces)
{
  return "<joint name=\"" + name + "\">" + std::string{interfaces} + "</joint>";
}

/// A valid joint: position and velocity state interfaces.
std::string good_joint(const std::string & name)
{
  return joint_block(name, std::string{POSITION} + std::string{VELOCITY});
}

/// All six joints valid, except joint `index`, which gets `interfaces` instead.
std::string joints_with_one_bad(std::size_t index, const std::string & interfaces)
{
  std::string blocks;
  for (std::size_t i{0}; i < JOINTS.size(); ++i) {
    blocks += i == index ? joint_block(JOINTS.at(i), interfaces) : good_joint(JOINTS.at(i));
  }
  return blocks;
}

/// One <param> of the <hardware> tag.
std::string param(std::string_view name, std::string_view value)
{
  return "<param name=\"" + std::string{name} + "\">" + std::string{value} + "</param>";
}

/// Tests never open a serial port.
constexpr std::string_view FAKE_BACKEND{"<param name=\"backend\">fake</param>"};

/// The arm's URDF with a <ros2_control> block that uses RobotSystem for `joint_blocks`,
/// with `parameters` in its <hardware> tag.
std::string urdf_with(const std::string & joint_blocks, std::string_view parameters = FAKE_BACKEND)
{
  std::string urdf{read_description()};
  const std::string ros2_control{
    std::string{"<ros2_control name=\""} + std::string{HARDWARE_NAME} +
    "\" type=\"system\">"
    "<hardware><plugin>positronic_hardware/RobotSystem</plugin>" +
    std::string{parameters} + "</hardware>" + joint_blocks + "</ros2_control>"};
  urdf.insert(urdf.rfind("</robot>"), ros2_control);
  return urdf;
}

std::string all_good_joints()
{
  std::string blocks;
  for (const auto & joint : JOINTS) {
    blocks += good_joint(joint);
  }
  return blocks;
}

class LoadRobotSystem : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  /// Loads `urdf` into a fresh ResourceManager, as the controller manager does at start-up.
  hardware_interface::ResourceManager & load(const std::string & urdf)
  {
    hardware_interface::ResourceManagerParams params;
    params.robot_description = urdf;
    params.clock = std::make_shared<rclcpp::Clock>();
    params.logger = rclcpp::get_logger("test_load_robot_system");
    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(params, false);
    loaded_ = resource_manager_->load_and_initialize_components(params);
    return *resource_manager_;
  }

  /// Lifecycle state id of the component, or PRIMARY_STATE_UNKNOWN if it is not listed.
  std::uint8_t component_state()
  {
    const auto & status{resource_manager_->get_components_status()};
    const auto it{status.find(std::string{HARDWARE_NAME})};
    if (it == status.end()) {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    return it->second.state.id();
  }

  /// Moves the component to another lifecycle state, as the controller manager does.
  hardware_interface::return_type go_to(std::uint8_t id, const char * label)
  {
    rclcpp_lifecycle::State target{id, label};
    return resource_manager_->set_component_state(std::string{HARDWARE_NAME}, target);
  }

  /// One controller-manager cycle of read(), 10 ms long.
  hardware_interface::return_type read_once()
  {
    return resource_manager_->read(clock_.now(), rclcpp::Duration::from_seconds(0.01)).result;
  }

  /// The value a controller would see for `joint`/`interface`.
  std::optional<double> state(const std::string & joint, const std::string & interface)
  {
    const auto loaned{resource_manager_->claim_state_interface(joint + "/" + interface)};
    return loaned.get_optional();
  }

  rclcpp::Clock clock_;
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;
  bool loaded_{false};
};

}  // namespace

TEST_F(LoadRobotSystem, DescriptionHasTheSixJoints)
{
  // Guards the other tests: if this fails, the URDF moved or a joint was renamed.
  const std::string urdf{read_description()};
  ASSERT_FALSE(urdf.empty());
  for (const auto & joint : JOINTS) {
    EXPECT_NE(urdf.find("<joint name=\"" + joint + "\" type=\"revolute\""), std::string::npos)
      << joint;
  }
}

TEST_F(LoadRobotSystem, InitSucceedsWithSixPositionJoints)
{
  load(urdf_with(all_good_joints()));

  EXPECT_TRUE(loaded_);
  // on_init returned SUCCESS: the component sits in UNCONFIGURED.
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, ExportsExactlyTheTwelveStates)
{
  auto & resource_manager{load(urdf_with(all_good_joints()))};
  ASSERT_TRUE(loaded_);

  std::vector<std::string> expected;
  expected.reserve(2 * JOINTS.size());
  for (const auto & joint : JOINTS) {
    expected.push_back(joint + "/position");
    expected.push_back(joint + "/velocity");
  }
  auto keys{resource_manager.state_interface_keys()};
  std::sort(keys.begin(), keys.end());
  std::sort(expected.begin(), expected.end());
  EXPECT_EQ(keys, expected);
}

TEST_F(LoadRobotSystem, InitRejectsFiveJoints)
{
  std::string blocks;
  for (std::size_t i{0}; i + 1 < JOINTS.size(); ++i) {
    blocks += good_joint(JOINTS.at(i));
  }
  load(urdf_with(blocks));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsJointWithoutPositionState)
{
  load(urdf_with(joints_with_one_bad(2, std::string{VELOCITY})));  // J3: velocity only

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsJointWithoutVelocityState)
{
  load(urdf_with(joints_with_one_bad(2, std::string{POSITION})));  // J3: position only

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsCommandInterface)
{
  // J1 asks to be commanded: refused until #32, so no controller believes it moves the arm.
  load(urdf_with(
    joints_with_one_bad(0, std::string{COMMAND} + std::string{POSITION} + std::string{VELOCITY})));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitAcceptsEveryParameter)
{
  load(urdf_with(
    all_good_joints(), std::string{FAKE_BACKEND} + param("port", "/dev/ttyUSB0") +
                         param("baud", "115200") + param("velocity_filter_alpha", "0.5")));

  EXPECT_TRUE(loaded_);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsBadParameters)
{
  // Each case is the fake backend plus one wrong <param>: a typo must stop the driver.
  const std::vector<std::pair<std::string, std::string>> bad{
    {"baudrate", "115200"},             // unknown name
    {"backend", "simulated"},           // only serial or fake
    {"port", ""},                       // empty
    {"baud", "9600"},                   // a rate PosixSerialPort cannot set
    {"baud", "115200x"},                // not only a number
    {"velocity_filter_alpha", "0"},     // the filter would never move
    {"velocity_filter_alpha", "1.5"},   // above 1
    {"velocity_filter_alpha", "fast"},  // not a number
    {"velocity_filter_alpha", "nan"},   // from_chars reads it, the range check must not
  };
  for (const auto & [name, value] : bad) {
    SCOPED_TRACE(::testing::Message{} << name << "=" << value);
    // "backend" replaces the fake one rather than appearing twice.
    const std::string parameters{
      name == "backend" ? param(name, value) : std::string{FAKE_BACKEND} + param(name, value)};
    load(urdf_with(all_good_joints(), parameters));

    EXPECT_FALSE(loaded_);
    EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_F(LoadRobotSystem, PositionsAreUnknownBeforeTheFirstRead)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);

  for (const auto & joint : JOINTS) {
    for (const std::string interface : {"position", "velocity"}) {
      const auto value{state(joint, interface)};
      ASSERT_TRUE(value.has_value()) << joint << "/" << interface;
      EXPECT_TRUE(std::isnan(value.value_or(0.0))) << joint << "/" << interface;
    }
  }
}

TEST_F(LoadRobotSystem, ReadReportsTheFakeArmPose)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);

  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  for (std::size_t i{0}; i < JOINTS.size(); ++i) {
    const auto position{state(JOINTS.at(i), "position")};
    ASSERT_TRUE(position.has_value()) << JOINTS.at(i);
    EXPECT_DOUBLE_EQ(
      position.value_or(std::nan("")), positronic_hardware::FakeArm::START_POSE.at(i))
      << JOINTS.at(i);
    // The fake arm stands still.
    const auto velocity{state(JOINTS.at(i), "velocity")};
    ASSERT_TRUE(velocity.has_value()) << JOINTS.at(i);
    EXPECT_DOUBLE_EQ(velocity.value_or(std::nan("")), 0.0) << JOINTS.at(i);
  }
}

TEST_F(LoadRobotSystem, ActivatesAndReadStillWorks)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);

  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(read_once(), hardware_interface::return_type::OK);
}
