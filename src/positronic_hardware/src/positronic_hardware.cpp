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
#include "lifecycle_msgs/msg/state.hpp"
#include "positronic_hardware/link_watchdog.hpp"
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

/// True if the <ros2_control> block has exactly the fault <gpio> (#35): one <gpio> named
/// FAULT_GPIO with one state interface FAULT_STATE and no command interface.
bool has_fault_gpio(const hardware_interface::HardwareInfo & info)
{
  if (info.gpios.size() != 1) {
    return false;
  }
  const hardware_interface::ComponentInfo & gpio{info.gpios.front()};
  return gpio.name == FAULT_GPIO && gpio.command_interfaces.empty() &&
         gpio.state_interfaces.size() == 1 && gpio.state_interfaces.front().name == FAULT_STATE;
}

/// A FaultCode as the value of the fault state interface: a small whole number, exact in a double.
constexpr double to_state(FaultCode code)
{
  return static_cast<double>(std::underlying_type_t<FaultCode>(code));
}

/// #35: one lost reply costs up to moving_period + REPLY_TIMEOUT + a late reply of silence. A
/// period at most half the link timeout keeps room for at least one lost reply, so a single
/// bad turn never trips the fault.
constexpr int MAX_MOVING_PERIOD_MS{static_cast<int>(LinkWatchdog::DEFAULT_TIMEOUT.count() / 2)};

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

/// The two timings of the serial thread (#32), from the URDF when set. The bounds keep out a
/// period below the arm's own busy time or too close to the #35 link timeout, and a settle
/// longer than that timeout.
bool read_timing(
  const rclcpp::Logger & logger, const std::unordered_map<std::string, std::string> & parameters,
  SerialArm::Timing & out)
{
  if (!read_milliseconds(
      logger, parameters, "moving_period_ms", 20, MAX_MOVING_PERIOD_MS, out.moving_period) ||
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
    // #34: this plugin refuses an out-of-range command and halts the arm. ros2_control's own
    // limiter would clamp the command to the limit before write() sees it, so the refusal would
    // never happen. Still safe (the arm stops at the limit), hence a warning, not an error.
    if (joint.enable_limits) {
      RCLCPP_WARN(
        get_logger(),
        "Joint '%s' has the ros2_control limiter on: commands past a limit are clamped, not "
        "refused. Add <limits enable=\"false\"/> to the joint, as positronic_real.urdf.xacro does.",
        joint.name.c_str());
    }
  }

  // #35: the fault reaches the supervisor through this one state interface. Required, like the
  // joints: without it the supervisor would read nothing and never stop the arm on a link fault.
  if (!has_fault_gpio(info)) {
    RCLCPP_ERROR(
      get_logger(),
      "The <ros2_control> block must have exactly one <gpio name=\"%s\"> with one "
      "<state_interface name=\"%s\"/>, as positronic_real.urdf.xacro does.",
      std::string{FAULT_GPIO}.c_str(), std::string{FAULT_STATE}.c_str());
    return CallbackReturn::ERROR;
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
  // The limits come from the URDF itself, never from a table in the code: one source of truth.
  // Parsing it here, not reading info.limits, because <limits enable="false"/> (which turns
  // ros2_control's clamping limiter off, see on_init) also wipes the joint's info.limits.
  urdf::Model model{};
  if (!model.initString(info.original_xml)) {
    RCLCPP_ERROR(get_logger(), "Cannot parse the URDF to read the joint limits.");
    return false;
  }

  PositionLimits limits{};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    const std::string & name{info.joints.at(i).name};
    const urdf::JointConstSharedPtr joint{model.getJoint(name)};
    // Only a revolute joint has a position range: a continuous one has none, and a missing
    // joint or <limit> tag leaves nothing to check against.
    if (joint == nullptr || joint->type != urdf::Joint::REVOLUTE || joint->limits == nullptr) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' has no revolute <limit> in the URDF. Refusing to start.",
        name.c_str());
      return false;
    }
    limits.at(i) = PositionLimit{joint->limits->lower, joint->limits->upper};
  }

  // A swapped or infinite limit would refuse everything, or nothing.
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
  // After on_error the old backend is still alive and holds the port (TIOCEXCL refuses a second
  // open, even from this process): drop it first. On a first configure there is nothing to drop.
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
    fault_handle_ =
      get_state_interface_handle(std::string{FAULT_GPIO} + "/" + std::string{FAULT_STATE});
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Cannot find an interface: %s", error.what());
    return CallbackReturn::ERROR;
  }

  // Nothing measured yet. NaN means "unknown"; 0.0 would claim the arm is at zero.
  for (const auto & [name, description] : joint_state_interfaces_) {
    set_state(name, std::numeric_limits<double>::quiet_NaN());
  }
  // #35: a fault latched before an on_error stays visible; only on_activate clears it.
  std::ignore = set_state(fault_handle_, to_state(fault_.load()), true);
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

  // #35: has_sample() stays true once the arm has answered, so it says nothing about a link that
  // died later. Never activate on a silent link: read() would raise the fault again at once.
  if (arm_->link_timed_out()) {
    RCLCPP_ERROR(
      get_logger(), "The arm stopped answering on %s; not activating.", config_.port.c_str());
    return CallbackReturn::ERROR;
  }

  // #34: the measured pose becomes the first command, so it must be inside the limits, or the
  // first write() would refuse it at once. Checked before power_on: an arm outside its limits
  // is not torqued. ERROR goes through on_error, which requests a stop: the arm stays where it is.
  const CommandCheck start{check_command(arm_->read_angles(), limits_)};
  if (!start.ok()) {
    log_fault(get_logger(), "Not activated, the arm is outside its limits", start, limits_, info_);
    RCLCPP_ERROR(get_logger(), "Move the arm inside its limits by hand, then activate again.");
    return CallbackReturn::ERROR;
  }

  // "Power the servos on" (#36), after the first reply: a released arm still answers
  // get_angles, so the wait above does not need torque, and a silent arm fails there first.
  // power_on() also lets targets out again after a halt(). If its frame cannot be written, the
  // port is in trouble: ERROR hands the component to on_error rather than leaving it ACTIVE.
  if (!arm_->power_on()) {
    RCLCPP_ERROR(
      get_logger(), "power_on was not sent to the arm within %d ms; not activating.",
      static_cast<int>(ArmBackend::REQUEST_TIMEOUT.count()));
    return CallbackReturn::ERROR;
  }

  // dt = 0: no time has passed, so the velocity estimate is held, not recomputed.
  update_states(0.0);
  for (const auto & [name, description] : joint_command_interfaces_) {
    set_command(name, get_state(name));
  }
  // #35: the one place a latched fault is cleared. Someone asked for motion again on purpose,
  // and the link was checked alive above.
  fault_.store(FaultCode::NONE);
  std::ignore = set_state(fault_handle_, to_state(FaultCode::NONE), true);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // "Cut actuation": the arm gets a stop and holds where it is, and no target goes out until
  // the next activate. halt() waits for the stop to reach the port, so SUCCESS means the arm
  // was told to stop, not only that the component changed state.
  if (arm_->halt()) {
    return CallbackReturn::SUCCESS;
  }
  // The stop never left: nobody knows what the arm is doing. FAILURE would keep the component
  // ACTIVE and the controllers commanding it; ERROR takes it out of service through on_error,
  // and the supervisor sees a component that must be configured and activated again.
  RCLCPP_ERROR(
    get_logger(), "stop was not sent to the arm within %d ms.",
    static_cast<int>(ArmBackend::REQUEST_TIMEOUT.count()));
  return CallbackReturn::ERROR;
}

CallbackReturn RobotSystem::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Any transition that returned ERROR lands here, and so does a write() that returned ERROR
  // (#34: a refused command), on the control loop thread. The arm may still be moving towards
  // its last valid target, so it gets a stop, without waiting here: the backend stays alive,
  // its thread sends the stop at the start of its next turn and holds every target, and keeps
  // reading the pose. Dropping it here would join its thread on the control loop; on_configure
  // drops it instead, before opening the port again. arm_ is null when on_configure itself
  // failed to open the port.
  // SUCCESS takes the component to UNCONFIGURED, where configure starts it afresh; the base
  // class would return FAILURE and finalize it, which only a plugin reload undoes.
  if (arm_ != nullptr) {
    arm_->request_halt();
  }
  return CallbackReturn::SUCCESS;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  update_states(period.seconds());
  // #35: a link that goes silent while active is a fault. While inactive nothing can move the
  // arm, and on_activate refuses a silent link anyway.
  const bool active{get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE};
  if (active && arm_->link_timed_out()) {
    raise_fault(FaultCode::LINK_TIMEOUT);
  }
  // Reported, not returned as ERROR: the component stays ACTIVE and readable, the backend holds
  // every target, and the supervisor (#45) decides what happens next. Written every cycle, so
  // anyone reading it sees the latched value.
  std::ignore = set_state(fault_handle_, to_state(fault_.load()), false);
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // The resource manager never calls write() on an inactive component, and after a halt() the
  // backend holds every target anyway. This is a second guard for a caller outside the
  // controller manager (a test, a script). No log: nothing here is RT-safe to log, and the
  // normal path never reaches it.
  const bool active{get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE};
  if (!active) {
    return return_type::OK;
  }
  // #35: after a fault the backend already holds every target; do not even hand them over.
  if (fault_.load() != FaultCode::NONE) {
    return return_type::OK;
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

  // #34: check before anything reaches the arm. The view carries the count actually read, so
  // the wrong-count rule of the issue sits here too, although on_init already wants six joints.
  const CommandCheck check{check_command(CommandView{commands.data(), received}, limits_)};
  if (!check.ok()) {
    refuse(check);
    // ERROR: the controller manager deactivates the controllers on this component and the
    // resource manager calls on_error(), which takes the component out of ACTIVE (#36).
    return return_type::ERROR;
  }

  arm_->write_angles(commands);
  return return_type::OK;
}

void RobotSystem::raise_fault(FaultCode code)
{
  // Only the first fault is kept: it is the cause, later ones are its consequences.
  FaultCode expected{FaultCode::NONE};
  if (!fault_.compare_exchange_strong(expected, code)) {
    return;
  }
  // Going silent does not stop the arm: it finishes its last target (#20, serial_link.md
  // section 5). request_halt() holds every target and leaves a stop for the serial thread; on a
  // dead link that stop may never arrive, which is why #45 also needs the hardware stop.
  arm_->request_halt();
  // Logged once, at the transition, like a refusal (#34): read() does not log on later cycles.
  RCLCPP_ERROR(
    get_logger(),
    "Fault %d: no valid reply from the arm for over %d ms. Targets held until the next activate.",
    static_cast<int>(code), static_cast<int>(LinkWatchdog::DEFAULT_TIMEOUT.count()));
}

void RobotSystem::refuse(const CommandCheck & check)
{
  // Going silent does not stop the arm: it keeps going to its last target. request_halt() is
  // the non-blocking stop of #36 (two atomic stores, safe here on the control loop); on_error
  // asks for it again, which changes nothing. The refused command never reached the backend.
  arm_->request_halt();
  // One log line per refusal: write() is not called again once the component leaves ACTIVE.
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
