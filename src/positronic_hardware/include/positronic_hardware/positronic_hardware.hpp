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

#ifndef POSITRONIC_HARDWARE__POSITRONIC_HARDWARE_HPP_
#define POSITRONIC_HARDWARE__POSITRONIC_HARDWARE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "positronic_hardware/fake_arm.hpp"
#include "positronic_hardware/joint_limit_check.hpp"
#include "positronic_hardware/serial_arm.hpp"
#include "positronic_hardware/velocity_estimator.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace positronic_hardware
{
class RobotSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Where the joint angles come from.
  enum class Backend : std::uint8_t
  {
    SERIAL,
    FAKE
  };

  /// The <param> values of the <hardware> tag, checked in on_init. A parameter the URDF does
  /// not set keeps its default.
  struct Config
  {
    Backend backend{Backend::SERIAL};
    std::string port{"/dev/ttyTHS1"};
    int baud{1000000};
    double velocity_filter_alpha{VelocityEstimator::DEFAULT_ALPHA};
    SerialArm::Timing timing{};
  };

  /// Fills config_ from the <hardware> parameters. False, with an error logged, on an unknown
  /// parameter or a bad value: a typo in the URDF must stop the driver, not be ignored.
  bool read_parameters(const std::unordered_map<std::string, std::string> & parameters);

  bool load_limits(const hardware_interface::HardwareInfo & info);

  /// Refuses a bad command: stops the arm, logs the joint name and value, latches halted_.
  void refuse(const CommandCheck & check);

  /// The interfaces of one joint, looked up once so read() and write() build no string: two
  /// states the plugin fills, one command the controller fills and the plugin sends.
  struct JointHandles
  {
    hardware_interface::CommandInterface::SharedPtr command;
    hardware_interface::StateInterface::SharedPtr position;
    hardware_interface::StateInterface::SharedPtr velocity;
  };

  /// Asks the arm where the joints are, estimates the velocity from the change since the last
  /// call `dt` seconds ago, and stores both in the state interfaces. No allocation, no blocking.
  void update_states(double dt);

  Config config_{};
  std::unique_ptr<ArmBackend> arm_;
  VelocityEstimator velocity_estimator_;
  std::array<JointHandles, NUM_JOINTS> handles_{};

  /// Position limits of J1..J6 from the URDF, read once in on_init. write() checks every
  /// command against them before it reaches the arm.
  PositionLimits limits_{};

  /// Set when a command was refused. write() sends nothing more until the component is
  /// configured and activated again: a refused command means something upstream is wrong.
  bool halted_{false};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__POSITRONIC_HARDWARE_HPP_