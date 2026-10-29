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

#include "positronic_hardware/positronic_hardware.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <exception>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <tuple>
#include <unordered_map>

namespace positronic_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
/// True if the joint has exactly two state interfaces: position and velocity, in any order.
bool has_position_and_velocity(const hardware_interface::ComponentInfo & joint)
{
  const auto has{[&joint](const std::string & name) {
      return std::any_of(
      joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [&name](const hardware_interface::InterfaceInfo & interface) {
          return interface.name == name;
      });
    }};
  return joint.state_interfaces.size() == 2 && has(hardware_interface::HW_IF_POSITION) &&
         has(hardware_interface::HW_IF_VELOCITY);
}

/// How long on_activate waits for the arm's first reply. One get_angles round trip takes a few
/// milliseconds, so this is generous; the arm not answering at all is what it catches.
constexpr std::chrono::milliseconds FIRST_SAMPLE_TIMEOUT{1000};

/// The <param> names the <hardware> tag may use. Anything else is a typo.
constexpr std::array<std::string_view, 4> KNOWN_PARAMETERS{
  "backend", "port", "baud", "velocity_filter_alpha"};

/// The value of `key`, or nullptr if the URDF does not set it.
const std::string * find_parameter(
  const std::unordered_map<std::string, std::string> & parameters, const std::string & key)
{
  const auto it{parameters.find(key)};
  return it == parameters.end() ? nullptr : &it->second;
}

/// `text` as a number, or nullopt unless the whole text is one. from_chars ignores the
/// locale and throws nothing, unlike std::stoi/std::stod.
template<typename Number>
std::optional<Number> parse_number(const std::string & text)
{
  Number value{};
  const char * const end{std::next(text.data(), static_cast<std::ptrdiff_t>(text.size()))};
  const auto [stop, error]{std::from_chars(text.data(), end, value)};
  if (error != std::errc{} || stop != end) {
    return std::nullopt;
  }
  return value;
}
}  // namespace

CallbackReturn RobotSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const hardware_interface::HardwareInfo & info{params.hardware_info};

  // The URDF must list the joints in protocol order: J1 (base) first, J6 (flange) last.
  if (info.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(
      get_logger(), "The URDF must have exactly %zu joints, %zu were read.", NUM_JOINTS,
      info.joints.size());
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : info.joints) {
    if (!has_position_and_velocity(joint)) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must have two state interfaces: 'position' and 'velocity'.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    // Read-only driver until #32: no controller may believe it moves the arm.
    if (!joint.command_interfaces.empty()) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must have no command interface: this driver is read-only.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  if (!read_parameters(info.hardware_parameters)) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool RobotSystem::read_parameters(const std::unordered_map<std::string, std::string> & parameters)
{
  for (const auto & [key, value] : parameters) {
    if (std::find(KNOWN_PARAMETERS.begin(), KNOWN_PARAMETERS.end(),
        key) == KNOWN_PARAMETERS.end())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Unknown hardware parameter '%s'. Known: backend, port, baud, velocity_filter_alpha.",
        key.c_str());
      return false;
    }
  }

  Config config{};

  if (const std::string * backend{find_parameter(parameters, "backend")}) {
    if (*backend == "serial") {
      config.backend = Backend::SERIAL;
    } else if (*backend == "fake") {
      config.backend = Backend::FAKE;
    } else {
      RCLCPP_ERROR(
        get_logger(), "Parameter 'backend' is '%s'; use 'serial' or 'fake'.", backend->c_str());
      return false;
    }
  }

  if (const std::string * port{find_parameter(parameters, "port")}) {
    if (port->empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'port' is empty.");
      return false;
    }
    config.port = *port;
  }

  if (const std::string * baud{find_parameter(parameters, "baud")}) {
    // Only the rates PosixSerialPort can set.
    const std::optional<int> value{parse_number<int>(*baud)};
    if (value != 115200 && value != 1000000) {
      RCLCPP_ERROR(get_logger(), "Parameter 'baud' is '%s'; use 115200 or 1000000.", baud->c_str());
      return false;
    }
    config.baud = *value;
  }

  if (const std::string * alpha{find_parameter(parameters, "velocity_filter_alpha")}) {
    // Not a number becomes NaN. from_chars also accepts "nan" itself. Every comparison with
    // NaN is false, so both fail the range check.
    const double value{
      parse_number<double>(*alpha).value_or(std::numeric_limits<double>::quiet_NaN())};
    const bool in_range{value > 0.0 && value <= 1.0};
    if (!in_range) {
      RCLCPP_ERROR(
        get_logger(), "Parameter 'velocity_filter_alpha' is '%s'; use a number in (0, 1].",
        alpha->c_str());
      return false;
    }
    config.velocity_filter_alpha = value;
  }

  config_ = config;
  RCLCPP_INFO(
    get_logger(), "Backend '%s', port %s at %d baud, velocity filter alpha %.3f.",
    config_.backend == Backend::SERIAL ? "serial" : "fake", config_.port.c_str(), config_.baud,
    config_.velocity_filter_alpha);
  return true;
}

CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // "Plug in the cable". SerialArm opens the port and starts its polling thread; it throws if
  // the port cannot be opened. Allocating and throwing are fine here, outside the control loop.
  try {
    if (config_.backend == Backend::SERIAL) {
      arm_ = std::make_unique<SerialArm>(config_.port, config_.baud);
    } else {
      arm_ = std::make_unique<FakeArm>();
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Cannot start the serial backend: %s", error.what());
    return CallbackReturn::ERROR;
  }
  // A fresh history: the first read() after (re)configure has no earlier sample to difference.
  velocity_estimator_ = VelocityEstimator{config_.velocity_filter_alpha};

  // Look the handles up once, here, so read() builds no string. This throws if a name is
  // missing, which is fine outside the control loop.
  try {
    for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
      const std::string & name{info_.joints.at(i).name};
      handles_.at(i).position =
        get_state_interface_handle(name + "/" + hardware_interface::HW_IF_POSITION);
      handles_.at(i).velocity =
        get_state_interface_handle(name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Cannot find a state interface: %s", error.what());
    return CallbackReturn::ERROR;
  }

  // Nothing measured yet. NaN means "unknown"; 0.0 would claim the arm is at zero.
  for (const auto & [name, description] : joint_state_interfaces_) {
    set_state(name, std::numeric_limits<double>::quiet_NaN());
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // "Unplug the cable": SerialArm's destructor stops its thread and closes the port.
  arm_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // "Allow motion": every command starts at the measured position, so the first thing a
  // controller sees is "stay where you are". There are no command interfaces until #32.
  // Never activate on a placeholder pose: wait for the arm to answer. Sleeping is fine here,
  // outside the control loop.
  const auto deadline{std::chrono::steady_clock::now() + FIRST_SAMPLE_TIMEOUT};
  while (!arm_->has_sample()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(
        get_logger(), "The arm did not answer within %d ms. Is it powered and on %s?",
        static_cast<int>(FIRST_SAMPLE_TIMEOUT.count()), config_.port.c_str());
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }

  // dt = 0: no time has passed, so the velocity estimate is held, not recomputed.
  update_states(0.0);
  for (const auto & [name, description] : joint_command_interfaces_) {
    set_command(name, get_state(name));
  }
  return CallbackReturn::SUCCESS;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  update_states(period.seconds());
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read-only driver (#22): nothing is sent, so the arm cannot move. Commands come in #32.
  return return_type::OK;
}

void RobotSystem::update_states(double dt)
{
  // No reply from the arm yet: keep the states NaN ("unknown") rather than publish the
  // placeholder pose, and keep the velocity estimator from using it as a baseline.
  if (!arm_->has_sample()) {
    return;
  }
  const JointPositions angles{arm_->read_angles()};
  const JointPositions velocities{velocity_estimator_.update(angles, dt)};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    // wait_until_set = false: never block on a controller that is reading the handle.
    std::ignore = set_state(handles_.at(i).position, angles.at(i), false);
    std::ignore = set_state(handles_.at(i).velocity, velocities.at(i), false);
  }
}

}  // namespace positronic_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(positronic_hardware::RobotSystem, hardware_interface::SystemInterface)
