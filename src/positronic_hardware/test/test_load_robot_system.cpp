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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/resource_manager_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

/// Name of the <hardware> component in the test URDFs.
const char HARDWARE_NAME[]{"PositronicArm"};

/// Revolute joints of mycobot_280_jn.urdf, J1 (base) to J6 (flange).
const std::vector<std::string> JOINTS{
  "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
  "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"};

/// The arm's URDF as installed by positronic_description, without a <ros2_control> block.
std::string read_description()
{
  const std::string path{
    ament_index_cpp::get_package_share_directory("positronic_description") +
    "/urdf/mycobot_280_jn_sim/mycobot_280_jn.urdf"};
  std::ifstream file{path};
  std::stringstream content;
  content << file.rdbuf();
  return content.str();
}

/// One <joint> entry of the <ros2_control> block.
std::string joint_block(const std::string & name, bool with_position_state)
{
  std::string block{"<joint name=\"" + name + "\">"};
  if (with_position_state) {
    block += "<state_interface name=\"position\"/>";
  }
  return block + "</joint>";
}

/// The arm's URDF with a <ros2_control> block that uses RobotSystem for `joint_blocks`.
std::string urdf_with(const std::string & joint_blocks)
{
  std::string urdf{read_description()};
  const std::string ros2_control{
    std::string{"<ros2_control name=\""} + HARDWARE_NAME + "\" type=\"system\">"
    "<hardware><plugin>positronic_hardware/RobotSystem</plugin></hardware>" +
    joint_blocks + "</ros2_control>"};
  urdf.insert(urdf.rfind("</robot>"), ros2_control);
  return urdf;
}

std::string all_joints_with_position()
{
  std::string blocks;
  for (const auto & joint : JOINTS) {
    blocks += joint_block(joint, true);
  }
  return blocks;
}

class LoadRobotSystem : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

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
    const auto it{status.find(HARDWARE_NAME)};
    if (it == status.end()) {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    return it->second.state.id();
  }

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
  load(urdf_with(all_joints_with_position()));

  EXPECT_TRUE(loaded_);
  // on_init returned SUCCESS: the component sits in UNCONFIGURED.
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, ExportsExactlyTheSixPositionStates)
{
  auto & resource_manager{load(urdf_with(all_joints_with_position()))};
  ASSERT_TRUE(loaded_);

  std::vector<std::string> expected;
  for (const auto & joint : JOINTS) {
    expected.push_back(joint + "/position");
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
    blocks += joint_block(JOINTS.at(i), true);
  }
  load(urdf_with(blocks));

  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsJointWithoutPositionState)
{
  std::string blocks;
  for (std::size_t i{0}; i < JOINTS.size(); ++i) {
    blocks += joint_block(JOINTS.at(i), i != 2);  // J3 has no position state interface
  }
  load(urdf_with(blocks));

  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}
