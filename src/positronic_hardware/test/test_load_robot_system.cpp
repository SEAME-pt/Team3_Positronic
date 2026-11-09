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
// Follow-up prompt:
//   on_configure creates SerialArm or FakeArm from the backend parameter and on_cleanup
//   destroys it. Test that configure fails cleanly with backend=serial on a port that does not
//   exist, and that the component can be cleaned up and configured again.
// Follow-up prompt:
//   The states stay NaN until the arm's first reply, and on_activate fails if no reply comes.
//   Test it on a pseudo-terminal that opens like a serial port but never answers.
// Follow-up prompt (#36):
//   on_deactivate halts the arm and on_error releases it, on_activate powers the servos on, and
//   write() returns early when the component is not active. With the fake arm, check that a
//   command written after deactivate never reaches the arm, that activate resumes commands from
//   the measured pose, and that cleanup still works afterwards. On the silent pseudo-terminal,
//   check that a failed activate leaves the component UNCONFIGURED and configurable again.
// Follow-up prompt (#36):
//   on_deactivate returning ERROR is the safety path of the change and no test reaches it,
//   because the fake arm's halt() always succeeds. Use a pseudo-terminal that answers get_angles
//   so activate succeeds, then close its controller side so every write fails: the stop cannot
//   leave, deactivate must return ERROR and the component must end in UNCONFIGURED.
// Follow-up prompt (#34):
//   #34: the plugin reads the joint limits from the URDF and write() refuses a command past a
//   limit, NaN, and halts. The test joints carry <limits enable="false"/> like the real xacro.
//   Test that: a command exactly at a limit is written; limit + 5 deg (upper side, and the lower
//   side of the asymmetric J5) and NaN make write() return ERROR and take the component out of
//   ACTIVE; after a refusal the component configures and activates again; on_init refuses a
//   joint without a revolute <limit> and swapped limits; on_activate refuses an arm that stands
//   outside its limits.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/resource_manager_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "positronic_hardware/arm_backend.hpp"
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
/// ros2_control's clamping limiter off, as in positronic_real.urdf.xacro (#34).
constexpr std::string_view LIMITS_OFF{"<limits enable=\"false\"/>"};

/// One <joint> entry of the <ros2_control> block, with the given interface tags.
std::string joint_block(const std::string & name, std::string_view interfaces)
{
  return "<joint name=\"" + name + "\">" + std::string{interfaces} + "</joint>";
}

/// A valid joint: position and velocity state interfaces, position command interface, and the
/// ros2_control limiter off, exactly like the real xacro.
std::string good_joint(const std::string & name)
{
  return joint_block(
    name, std::string{LIMITS_OFF} + std::string{COMMAND} + std::string{POSITION} +
    std::string{VELOCITY});
}

/// `text` with its one occurrence of `from` replaced by `to`. Used to edit the URDF limits.
std::string replaced(std::string text, std::string_view from, std::string_view to)
{
  const std::size_t at{text.find(from)};
  if (at != std::string::npos) {
    text.replace(at, from.size(), to);
  }
  return text;
}

/// The <limit> tags of J1, J2 and J5 in mycobot_280_jn.urdf, verbatim. Each is unique.
constexpr std::string_view J1_LIMIT{
  R"(<limit effort="10.0" lower="-2.9321" upper="2.9321" velocity="2.618"/>)"};
constexpr std::string_view J2_LIMIT{
  R"(<limit effort="10.0" lower="-2.4434" upper="2.4434" velocity="2.618"/>)"};

/// The URDF limits the refusal tests aim at, in radians.
constexpr double J2_UPPER{2.4434};
constexpr double J5_LOWER{-2.7052};
constexpr double DEG{3.14159265358979323846 / 180.0};

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

/// `description` (the arm's URDF by default) with a <ros2_control> block that uses RobotSystem
/// for `joint_blocks`, with `parameters` in its <hardware> tag.
std::string urdf_with(
  const std::string & joint_blocks, std::string_view parameters = FAKE_BACKEND,
  const std::string & description = read_description())
{
  std::string urdf{description};
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

/// Opens a pseudo-terminal, which opens and configures like a serial port but never answers,
/// and writes its device path into `port`. Returns the controller descriptor, which the caller
/// closes, or -1.
int open_pseudo_terminal(std::array<char, 128> & port)
{
  const int controller{::posix_openpt(O_RDWR | O_NOCTTY)};
  if (controller < 0) {
    return -1;
  }
  if (::grantpt(controller) != 0 || ::unlockpt(controller) != 0 ||
    ::ptsname_r(controller, port.data(), port.size()) != 0)
  {
    ::close(controller);
    return -1;
  }
  return controller;
}

/// One get_angles reply as the arm sends it, recorded on the real arm (the pose the SerialArm
/// tests decode). Written back for every request, so a power_on also gets a pose: the arm
/// answers that one with an ack, but the thread only looks for poses.
constexpr std::array<std::uint8_t, 17> RECORDED_REPLY{0xFE, 0xFE, 0x0E, 0x20, 0xFF, 0xF8, 0xFF,
  0xEF, 0xFF, 0xC3, 0xFF, 0x85, 0x00, 0x2B, 0x00, 0x3D, 0xFA};

/// Answers every request on the controller side of a pseudo-terminal with RECORDED_REPLY until
/// stopped: a silent-free stand-in for the arm, enough for activate to succeed.
class ReplyingArm
{
public:
  explicit ReplyingArm(int controller)
  : thread_{[this, controller]() {
        std::array<std::uint8_t, 64> request{};
        while (!stop_.load()) {
          pollfd waiting{controller, POLLIN, 0};
          if (::poll(&waiting, 1, 10) <= 0) {
            continue;
          }
          const ssize_t got{::read(controller, request.data(), request.size())};
          if (got > 0) {
            static_cast<void>(::write(controller, RECORDED_REPLY.data(), RECORDED_REPLY.size()));
          }
        }
      }}
  {
  }

  ~ReplyingArm()
  {
    stop_.store(true);
    thread_.join();
  }

  ReplyingArm(const ReplyingArm &) = delete;
  ReplyingArm & operator=(const ReplyingArm &) = delete;
  ReplyingArm(ReplyingArm &&) = delete;
  ReplyingArm & operator=(ReplyingArm &&) = delete;

private:
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

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

  /// Configures (if needed) and activates the component, as the controller manager does.
  hardware_interface::return_type activate()
  {
    return go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
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

  /// One controller-manager cycle of write(), 10 ms long.
  hardware_interface::return_type write_once()
  {
    return resource_manager_->write(clock_.now(), rclcpp::Duration::from_seconds(0.01)).result;
  }

  /// Writes `value` to `joint`/position the way a controller does.
  bool command(const std::string & joint, double value)
  {
    auto loaned{resource_manager_->claim_command_interface(joint + "/position")};
    return loaned.set_value(value);
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

TEST_F(LoadRobotSystem, DescriptionHasTheLimitsTheTestsEdit)
{
  // Guards the #34 tests that edit a <limit>: if the URDF text changes, they must not pass
  // for the wrong reason.
  const std::string urdf{read_description()};
  EXPECT_NE(urdf.find(J1_LIMIT), std::string::npos);
  EXPECT_NE(urdf.find(J2_LIMIT), std::string::npos);
  EXPECT_NE(urdf.find("lower=\"-2.7052\""), std::string::npos);
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

TEST_F(LoadRobotSystem, InitRejectsJointWithoutCommandInterface)
{
  // J1 cannot be commanded: a controller could never move it, so the driver refuses to load.
  load(urdf_with(joints_with_one_bad(0, std::string{POSITION} + std::string{VELOCITY})));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsVelocityCommandInterface)
{
  // J1 asks for a velocity command: the arm only takes angles, so nothing could honour it.
  constexpr std::string_view VELOCITY_COMMAND{"<command_interface name=\"velocity\"/>"};
  load(urdf_with(joints_with_one_bad(
      0, std::string{VELOCITY_COMMAND} + std::string{POSITION} + std::string{VELOCITY})));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitAcceptsEveryParameter)
{
  load(urdf_with(
    all_good_joints(), std::string{FAKE_BACKEND} + param("port", "/dev/ttyUSB0") +
                         param("baud", "115200") + param("velocity_filter_alpha", "0.5") +
                         param("moving_period_ms", "40") + param("send_settle_ms", "25")));

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
    {"moving_period_ms", "10"},         // below the arm's own busy time
    {"moving_period_ms", "66.5"},       // whole ms only
    {"send_settle_ms", "250"},          // longer than the #35 timeout
    {"send_settle_ms", "slow"},         // not a number
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

TEST_F(LoadRobotSystem, InitRejectsASettleThatFillsTheTurn)
{
  // Both in range on their own, but the settle leaves no room for the reply in the turn.
  load(urdf_with(
    all_good_joints(),
    std::string{FAKE_BACKEND} + param("moving_period_ms", "30") + param("send_settle_ms", "30")));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
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

TEST_F(LoadRobotSystem, WriteSendsTheCommandToTheFakeArm)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);

  // A controller asks J1 for 0.25 rad. The fake arm gets there at once, so the next read()
  // reports it: the whole path from command interface to backend and back.
  ASSERT_TRUE(command(JOINTS.at(0), 0.25));
  ASSERT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(state(JOINTS.at(0), "position").value_or(std::nan("")), 0.25);
  // The other joints were commanded "stay" by on_activate, so they still report START_POSE.
  EXPECT_DOUBLE_EQ(
    state(JOINTS.at(1), "position").value_or(std::nan("")),
    positronic_hardware::FakeArm::START_POSE.at(1));
}

TEST_F(LoadRobotSystem, WriteSendsACommandExactlyAtTheLimit)
{
  // The URDF says "up to here": the limit itself is allowed.
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  ASSERT_TRUE(command(JOINTS.at(1), J2_UPPER));
  EXPECT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state(JOINTS.at(1), "position").value_or(std::nan("")), J2_UPPER);
}

TEST_F(LoadRobotSystem, WriteRefusesACommandPastTheLimit)
{
  // Acceptance criterion of #34: a goal at the limit + 5 deg is refused and the arm halts.
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  ASSERT_TRUE(command(JOINTS.at(1), J2_UPPER + 5.0 * DEG));
  EXPECT_EQ(write_once(), hardware_interface::return_type::ERROR);
  // on_error ran: the component left ACTIVE, so no controller can write to it any more.
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LoadRobotSystem, WriteRefusesACommandBelowAnAsymmetricLimit)
{
  // J5 goes to +2.7925 but only to -2.7052: the lower side has its own end.
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  ASSERT_TRUE(command(JOINTS.at(4), J5_LOWER - 5.0 * DEG));
  EXPECT_EQ(write_once(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LoadRobotSystem, WriteRefusesNaN)
{
  // Bad data is refused the same way. Every comparison with NaN is false, so a plain range
  // check would let it through.
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  ASSERT_TRUE(command(JOINTS.at(3), std::nan("")));
  EXPECT_EQ(write_once(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LoadRobotSystem, ComesBackAfterARefusal)
{
  // The refusal is not forever: configure and activate start again from "stay where you are".
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_TRUE(command(JOINTS.at(1), J2_UPPER + 5.0 * DEG));
  ASSERT_EQ(write_once(), hardware_interface::return_type::ERROR);

  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(
    state(JOINTS.at(1), "position").value_or(std::nan("")),
    positronic_hardware::FakeArm::START_POSE.at(1));
}

TEST_F(LoadRobotSystem, InitRejectsAJointWithoutALimit)
{
  // J1 made continuous, without its <limit>: ros2_control accepts it, the driver must not.
  const std::string description{replaced(
      replaced(
        read_description(), R"(<joint name="joint2_to_joint1" type="revolute">)",
        R"(<joint name="joint2_to_joint1" type="continuous">)"),
      J1_LIMIT, "")};
  load(urdf_with(all_good_joints(), FAKE_BACKEND, description));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsSwappedLimits)
{
  // J2 with lower above upper: the check would refuse every command, so the driver stops here.
  const std::string description{replaced(
      read_description(), J2_LIMIT,
      R"(<limit effort="10.0" lower="2.4434" upper="-2.4434" velocity="2.618"/>)")};
  load(urdf_with(all_good_joints(), FAKE_BACKEND, description));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, ActivateRefusesAnArmOutsideItsLimits)
{
  // J2 narrowed to [-0.2, 0.2] rad: the fake arm stands at 0.5 rad, outside. The first
  // command would be refused at once, so activation fails with the reason instead.
  const std::string description{replaced(
      read_description(), J2_LIMIT,
      R"(<limit effort="10.0" lower="-0.2" upper="0.2" velocity="2.618"/>)")};
  load(urdf_with(all_good_joints(), FAKE_BACKEND, description));
  ASSERT_TRUE(loaded_);

  EXPECT_EQ(activate(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LoadRobotSystem, ConfigureFailsWhenTheSerialPortIsMissing)
{
  // The serial path without an arm: opening the port throws, configure must report an error.
  load(urdf_with(
    all_good_joints(), param("backend", "serial") + param("port", "/dev/positronic_no_such_port")));
  ASSERT_TRUE(loaded_);

  EXPECT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(LoadRobotSystem, CleanupThenConfigureAgain)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED),
    hardware_interface::return_type::OK);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);

  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  const auto position{state(JOINTS.at(1), "position")};
  ASSERT_TRUE(position.has_value());
  EXPECT_DOUBLE_EQ(position.value_or(std::nan("")), positronic_hardware::FakeArm::START_POSE.at(1));
}

TEST_F(LoadRobotSystem, ActivateFailsWhenTheArmIsSilent)
{
  // A pseudo-terminal stands in for the serial port: it opens and configures like one, but
  // nothing ever answers get_angles. No arm is needed.
  std::array<char, 128> port{};
  const int controller{open_pseudo_terminal(port)};
  ASSERT_GE(controller, 0);

  load(urdf_with(all_good_joints(), param("backend", "serial") + param("port", port.data())));
  ASSERT_TRUE(loaded_);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);

  // No reply yet: read() works but reports "unknown", not the placeholder pose of zeros.
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  for (const auto & joint : JOINTS) {
    for (const std::string interface : {"position", "velocity"}) {
      const auto value{state(joint, interface)};
      ASSERT_TRUE(value.has_value()) << joint << "/" << interface;
      EXPECT_TRUE(std::isnan(value.value_or(0.0))) << joint << "/" << interface;
    }
  }

  // And the component refuses to become active on an arm that never answered.
  EXPECT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  resource_manager_.reset();  // closes the port before its pseudo-terminal goes away
  ::close(controller);
}

TEST_F(LoadRobotSystem, DeactivateHoldsCommandsAndActivateResumesThem)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);
  ASSERT_TRUE(command(JOINTS.at(0), 0.25));
  ASSERT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  ASSERT_DOUBLE_EQ(state(JOINTS.at(0), "position").value_or(std::nan("")), 0.25);

  // Deactivate: on_deactivate halts the arm (nothing to interrupt on the fake) and succeeds.
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // A controller still writing its command interface reaches nothing: the resource manager
  // skips an inactive component's write(), and write() itself returns early when not active.
  ASSERT_TRUE(command(JOINTS.at(0), 0.5));
  EXPECT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state(JOINTS.at(0), "position").value_or(std::nan("")), 0.25);

  // Activate again: power_on, commands start from the measured pose, and then follow.
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(command(JOINTS.at(0), 0.5));
  ASSERT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state(JOINTS.at(0), "position").value_or(std::nan("")), 0.5);

  // The rest of the lifecycle still works after a deactivate.
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED),
    hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, AFailedActivateLeavesTheComponentConfigurable)
{
  // Same silent pseudo-terminal: activate fails, and on_error must take the component back to
  // UNCONFIGURED, never FINALIZED (which needs a plugin reload), so a supervisor can configure
  // it again.
  std::array<char, 128> port{};
  const int controller{open_pseudo_terminal(port)};
  ASSERT_GE(controller, 0);

  load(urdf_with(all_good_joints(), param("backend", "serial") + param("port", port.data())));
  ASSERT_TRUE(loaded_);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // on_error kept the backend, and with it the port: configure drops it before opening again.
  EXPECT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  resource_manager_.reset();
  ::close(controller);
}

TEST_F(LoadRobotSystem, DeactivateFailsWhenTheStopCannotLeaveAndTheComponentIsConfigurable)
{
  // A pseudo-terminal with an arm behind it that answers every request, so the component gets
  // its first reply and becomes ACTIVE like on the bench.
  std::array<char, 128> port{};
  const int controller{open_pseudo_terminal(port)};
  ASSERT_GE(controller, 0);
  auto arm{std::make_unique<ReplyingArm>(controller)};

  load(urdf_with(all_good_joints(), param("backend", "serial") + param("port", port.data())));
  ASSERT_TRUE(loaded_);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::OK);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_FALSE(std::isnan(state(JOINTS.at(0), "position").value_or(std::nan(""))));

  // The arm goes away: with the controller side closed every write on the port fails, so the
  // stop can never leave. halt() gives up after REQUEST_TIMEOUT and on_deactivate must report
  // it: a component that could not stop the arm must not stay ACTIVE, and on_error takes it to
  // UNCONFIGURED, where a supervisor can configure it again. (The thread does not spin
  // meanwhile: a failed get_angles write skips the read and sleeps out the turn.)
  arm.reset();
  ::close(controller);
  const auto start{std::chrono::steady_clock::now()};
  EXPECT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::ERROR);
  const auto elapsed{std::chrono::steady_clock::now() - start};
  EXPECT_GE(elapsed, positronic_hardware::ArmBackend::REQUEST_TIMEOUT);
  EXPECT_LT(elapsed, 2 * positronic_hardware::ArmBackend::REQUEST_TIMEOUT);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  resource_manager_.reset();
}
