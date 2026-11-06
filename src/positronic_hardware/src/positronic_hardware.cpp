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
#include "urdf/model.h"

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
constexpr std::array<std::string_view, 6> KNOWN_PARAMETERS{
  "backend", "port", "baud", "velocity_filter_alpha", "moving_period_ms", "send_settle_ms"};

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

/// Reads `name` into `out` when the URDF sets it: a whole number of ms in [min, max]. True
/// when the parameter is absent or valid, false with the error logged otherwise.
bool read_milliseconds(
  const rclcpp::Logger & logger, const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, int min, int max, std::chrono::milliseconds & out)
{
  const std::string * text{find_parameter(parameters, name)};
  if (text == nullptr) {
    return true;
  }
  const std::optional<int> value{parse_number<int>(*text)};
  if (!value.has_value() || *value < min || *value > max) {
    RCLCPP_ERROR(
      logger, "Parameter '%s' is '%s'; use a whole number in [%d, %d].", name.c_str(),
      text->c_str(), min, max);
    return false;
  }
  out = std::chrono::milliseconds{*value};
  return true;
}

/// The two timings of the serial thread (#32), from the URDF when set. The bounds only keep out
/// nonsense: a period below the arm's own busy time, or a settle longer than the #35 timeout.
bool read_timing(
  const rclcpp::Logger & logger, const std::unordered_map<std::string, std::string> & parameters,
  SerialArm::Timing & out)
{
  if (!read_milliseconds(logger, parameters, "moving_period_ms", 20, 500, out.moving_period) ||
    !read_milliseconds(logger, parameters, "send_settle_ms", 0, 200, out.send_settle))
  {
    return false;
  }
  // The settle is part of a turn, with the reply before it. A settle that fills the turn would
  // only stretch every turn by itself and hide the value under test in a sweep.
  if (out.send_settle >= out.moving_period) {
    RCLCPP_ERROR(
      logger, "send_settle_ms (%d) must be below moving_period_ms (%d).",
      static_cast<int>(out.send_settle.count()), static_cast<int>(out.moving_period.count()));
    return false;
  }
  return true;
}

/// For the log lines only: people read degrees faster than radians.
constexpr double RAD_TO_DEG{180.0 / 3.14159265358979323846};

/// Logs a refused check (#34) as "<what>: joint '<name>' = <rad> rad (<deg> deg), <fault>
/// [<lower>, <upper>] rad.", the joint name and value the issue asks for.
void log_fault(
  const rclcpp::Logger & logger, const char * what, const CommandCheck & check,
  const PositionLimits & limits, const hardware_interface::HardwareInfo & info)
{
  if (check.fault == CommandFault::WRONG_COUNT) {
    RCLCPP_ERROR(
      logger, "%s: %s (%.0f values, %zu expected).", what, to_string(check.fault), check.value,
      NUM_JOINTS);
    return;
  }
  const PositionLimit & limit{limits.at(check.joint)};
  RCLCPP_ERROR(
    logger, "%s: joint '%s' = %.4f rad (%.2f deg), %s [%.4f, %.4f] rad.", what,
    info.joints.at(check.joint).name.c_str(), check.value, check.value * RAD_TO_DEG,
    to_string(check.fault), limit.lower, limit.upper);
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
    // The arm only takes angles (send_angles), so position is the one command we can honour.
    // A velocity or effort command interface would be claimed by a controller and never sent.
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces.front().name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must have exactly one command interface: 'position'.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    if (joint.enable_limits) {
      RCLCPP_WARN(
        get_logger(),
        "Joint '%s' has the ros2_control limiter on: commands past a limit are clamped, not "
        "refused. Add <limits enable=\"false\"/> to the joint, as positronic_real.urdf.xacro does.",
        joint.name.c_str());
    }
  }

  if (!load_limits(info)) {
    return CallbackReturn::ERROR;
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
        "Unknown hardware parameter '%s'. Known: backend, port, baud, velocity_filter_alpha, "
        "moving_period_ms, send_settle_ms.",
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

  if (!read_timing(get_logger(), parameters, config.timing)) {
    return false;
  }

  config_ = config;
  RCLCPP_INFO(
    get_logger(),
    "Backend '%s', port %s at %d baud, velocity filter alpha %.3f, moving period %d ms, "
    "send settle %d ms.",
    config_.backend == Backend::SERIAL ? "serial" : "fake", config_.port.c_str(), config_.baud,
    config_.velocity_filter_alpha, static_cast<int>(config_.timing.moving_period.count()),
    static_cast<int>(config_.timing.send_settle.count()));
  return true;
}

bool RobotSystem::load_limits(const hardware_interface::HardwareInfo & info)
{
  urdf::Model model{};
  if (!model.initString(info.original_xml)) {
    RCLCPP_ERROR(get_logger(), "Cannot parse the URDF to read the joint limits.");
    return false;
  }

  PositionLimits limits{};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    const std::string & name{info.joints.at(i).name};
    const urdf::JointConstSharedPtr joint{model.getJoint(name)};
    if (joint == nullptr || joint->type != urdf::Joint::REVOLUTE || joint->limits == nullptr) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' has no revolute <limit> in the URDF. Refusing to start.",
        name.c_str());
      return false;
    }
    limits.at(i) = PositionLimit{joint->limits->lower, joint->limits->upper};
  }

  if (!limits_valid(limits)) {
    RCLCPP_ERROR(
      get_logger(), "The URDF position limits are not usable: each needs lower < upper, finite.");
    return false;
  }

  limits_ = limits;
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    RCLCPP_INFO(
      get_logger(), "Joint '%s' limits: [%.4f, %.4f] rad.", info.joints.at(i).name.c_str(),
      limits_.at(i).lower, limits_.at(i).upper);
  }
  return true;
}

CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // "Plug in the cable". SerialArm opens the port and starts its polling thread; it throws if
  // the port cannot be opened. Allocating and throwing are fine here, outside the control loop.
  // Drop the old backend first: after on_error the component is UNCONFIGURED with its SerialArm
  // still holding the port, and TIOCEXCL refuses a second open of it, even from this process.
  arm_.reset();
  try {
    if (config_.backend == Backend::SERIAL) {
      arm_ = std::make_unique<SerialArm>(config_.port, config_.baud, config_.timing);
    } else {
      arm_ = std::make_unique<FakeArm>();
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Cannot start the serial backend: %s", error.what());
    return CallbackReturn::ERROR;
  }
  // A fresh history: the first read() after (re)configure has no earlier sample to difference.
  velocity_estimator_ = VelocityEstimator{config_.velocity_filter_alpha};

  // Look the handles up once, here, so read() and write() build no string. This throws if a
  // name is missing, which is fine outside the control loop.
  try {
    for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
      const std::string & name{info_.joints.at(i).name};
      handles_.at(i).command =
        get_command_interface_handle(name + "/" + hardware_interface::HW_IF_POSITION);
      handles_.at(i).position =
        get_state_interface_handle(name + "/" + hardware_interface::HW_IF_POSITION);
      handles_.at(i).velocity =
        get_state_interface_handle(name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Cannot find an interface: %s", error.what());
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
  // controller sees is "stay where you are" and write() sends nothing new until a controller
  // writes.
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

  const CommandCheck start{check_command(arm_->read_angles(), limits_)};
  if (!start.ok()) {
    log_fault(get_logger(), "Not activated, the arm is outside its limits", start, limits_, info_);
    RCLCPP_ERROR(get_logger(), "Move the arm inside its limits by hand, then activate again.");
    return CallbackReturn::ERROR;
  }
  halted_ = false;

  // dt = 0: no time has passed, so the velocity estimate is held, not recomputed.
  update_states(0.0);
  for (const auto & [name, description] : joint_command_interfaces_) {
    set_command(name, get_state(name));
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Whatever led here (a refused command, a failed write, a failed transition), the arm stops.
  // SUCCESS takes the component to UNCONFIGURED: configure and activate bring it back, and
  // on_activate starts again from "stay where you are".
  halted_ = true;
  if (arm_ != nullptr) {
    arm_->halt();
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
  if (halted_) {
    return return_type::ERROR;
  }

  JointPositions commands{};
  std::size_t received{0};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    // wait_until_get = false: never block on a controller that is writing the handle.
    if (!get_command(handles_.at(i).command, commands.at(i), false)) {
      return return_type::OK;  // A handle was busy: keep the last command, try next tick.
    }
    ++received;
  }

  const CommandCheck check{check_command(CommandView{commands.data(), received}, limits_)};
  if (!check.ok()) {
    refuse(check);
    return return_type::ERROR;
  }

  arm_->write_angles(commands);
  return return_type::OK;
}

void RobotSystem::refuse(const CommandCheck & check)
{
  halted_ = true;
  arm_->halt();
  log_fault(get_logger(), "Command refused, arm halted", check, limits_, info_);
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