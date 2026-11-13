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
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "positronic_hardware/fake_arm.hpp"
#include "positronic_hardware/joint_limit_check.hpp"
#include "positronic_hardware/serial_arm.hpp"
#include "positronic_hardware/servo_temperature.hpp"
#include "positronic_hardware/velocity_estimator.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace positronic_hardware
{

/// What the driver reports in its fault state interface (#35), as a whole number in a double.
/// The supervisor (#45) stops the arm on any value other than NONE, so a new fault needs no
/// change there. The values are part of the interface: add new ones at the end, never reuse or
/// renumber one.
enum class FaultCode : std::uint8_t
{
  /// No fault: the link is alive (or the component is not active).
  NONE = 0,
  /// No valid reply from the arm for LinkWatchdog::DEFAULT_TIMEOUT while active. Latched until
  /// the next activate.
  LINK_TIMEOUT = 1,
  /// Reserved for S3, "the arm does not follow the target". Not raised yet.
  TRACKING_ERROR = 2,
};

/// The <gpio> of the <ros2_control> block that carries the fault, and its one state interface.
/// The supervisor reads "driver/fault_code".
inline constexpr std::string_view FAULT_GPIO{"driver"};
inline constexpr std::string_view FAULT_STATE{"fault_code"};

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

  hardware_interface::CallbackReturn on_deactivate(
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
    int servo_temp_limit_c{SERVO_TEMP_LIMIT_C};
  };

  /// Fills config_ from the <hardware> parameters. False, with an error logged, on an unknown
  /// parameter or a bad value: a typo in the URDF must stop the driver, not be ignored.
  bool read_parameters(const std::unordered_map<std::string, std::string> & parameters);

  /// Fills limits_ from the <limit> tags of the URDF (#34). It parses the URDF itself
  /// (info.original_xml): with <limits enable="false"/> on the joints, ros2_control leaves no
  /// position limit in info.limits. False, with an error logged, when a joint has no usable
  /// position limit: the driver must not run without its last guard.
  bool load_limits(const hardware_interface::HardwareInfo & info);

  /// Latches `code` (#35): stores it, asks the backend to stop the arm, and logs it once. Only
  /// the first fault is kept; on_activate clears it. No allocation: safe from read().
  void raise_fault(FaultCode code);

  /// Refuses a bad command (#34): asks the backend to stop the arm and logs the joint name and
  /// value. write() then returns ERROR, which takes the component out of ACTIVE.
  void refuse(const CommandCheck & check);

  /// on_activate's temperature check, before the servos get torque: waits for the first reading
  /// and refuses an arm that was cut or is within ACTIVATE_COOLDOWN_C of the limit. False, with
  /// the reason logged, when activation must not go on.
  bool temperatures_allow_activation();

  /// read()'s temperature check. ERROR, with one log line, when the serial thread has cut the
  /// arm for an overheat, or when the component is active and the temperatures went stale.
  hardware_interface::return_type check_temperatures();

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
  /// "driver/fault_code", looked up once like the joint handles.
  hardware_interface::StateInterface::SharedPtr fault_handle_;
  /// The latched fault. read() runs on the control loop and the lifecycle callbacks may run on
  /// another thread: an atomic keeps both sides safe without a lock.
  std::atomic<FaultCode> fault_{FaultCode::NONE};

  /// Position limits of J1..J6 from the URDF, read once in on_init. write() checks every
  /// command against them before it reaches the arm.
  PositionLimits limits_{};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__POSITRONIC_HARDWARE_HPP_
