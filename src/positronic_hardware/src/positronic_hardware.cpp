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
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <vector>
#include "lifecycle_msgs/msg/state.hpp"
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

/// True if one of `interfaces` is called `name`.
bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, std::string_view name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [name](const hardware_interface::InterfaceInfo & candidate) {
      return candidate.name == name;
    });
}

/// True if the <ros2_control> block has a <gpio> named FAULT_GPIO with a FAULT_STATE state
/// interface, and no command interface of that name (#35). Only the fault is required: other
/// gpios, and other interfaces on this one, are allowed, so S3 can add driver states such as
/// a temperature (#115) without touching this check.
bool has_fault_gpio(const hardware_interface::HardwareInfo & info)
{
  for (const hardware_interface::ComponentInfo & gpio : info.gpios) {
    if (gpio.name == FAULT_GPIO) {
      // A command interface of the same name could be claimed by a controller and overwrite
      // the fault, so it is refused.
      return has_interface(gpio.state_interfaces, FAULT_STATE) &&
             !has_interface(gpio.command_interfaces, FAULT_STATE);
    }
  }
  return false;
}

/// A FaultCode as the value of the fault state interface: a small whole number, exact in a double.
constexpr double to_state(FaultCode code)
{
  return static_cast<double>(std::underlying_type_t<FaultCode>(code));
}

/// How long on_activate waits for the arm's first reply. One get_angles round trip takes a few
/// milliseconds, so this is generous; the arm not answering at all is what it catches.
constexpr std::chrono::milliseconds FIRST_SAMPLE_TIMEOUT{1000};

/// How long on_activate waits for the first servo temperatures. The serial thread asks on its
/// first idle turn and then once a second, so two seconds is two tries.
constexpr std::chrono::milliseconds FIRST_TEMPERATURE_TIMEOUT{2000};

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

std::uint8_t to_gripper_value(double radians, double closed, double open)
{
  const double fraction{std::clamp((radians - closed) / (open - closed), 0.0, 1.0)};
  const double value{GRIPPER_CLOSED_VALUE + fraction * (GRIPPER_OPEN_VALUE - GRIPPER_CLOSED_VALUE)};
  return static_cast<std::uint8_t>(std::lround(value));
}

double to_gripper_radians(std::uint8_t value, double closed, double open)
{
  const double fraction{
    (static_cast<double>(value) - GRIPPER_CLOSED_VALUE) /
    (GRIPPER_OPEN_VALUE - GRIPPER_CLOSED_VALUE)};
  return std::clamp(closed + std::clamp(fraction, 0.0, 1.0) * (open - closed), closed, open);
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
  if (info.joints.size() != NUM_JOINTS && info.joints.size() != NUM_JOINTS + 1) {
    RCLCPP_ERROR(
      get_logger(), "The URDF must have %zu joints, or %zu with the gripper; %zu were read.",
      NUM_JOINTS, NUM_JOINTS + 1, info.joints.size());
    return CallbackReturn::ERROR;
  }
  has_gripper_ = info.joints.size() == NUM_JOINTS + 1;

  for (const auto & joint : info.joints) {
    if (!has_position_and_velocity(joint)) {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must have two state interfaces: 'position' and 'velocity'.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    // The arm only takes angles (send_angles), so position is the one command we can honour.
    // A velocity or effort command interface would be claimed by a controller and never sent.
    if (
      joint.command_interfaces.size() != 1 ||
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
      "The <ros2_control> block must have a <gpio name=\"%s\"> with a "
      "<state_interface name=\"%s\"/> and no command interface of that name, as "
      "positronic_real.urdf.xacro does.",
      std::string{FAULT_GPIO}.c_str(), std::string{FAULT_STATE}.c_str());
    return CallbackReturn::ERROR;
  }

  if (!load_limits(info)) {
    return CallbackReturn::ERROR;
  }

  if (!read_parameters(info.hardware_parameters)) {
    return CallbackReturn::ERROR;
  }
  if (!start_supervisor_check()) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool RobotSystem::start_supervisor_check()
{
  if (config_.supervisor_timeout_ms == 0) {
    RCLCPP_WARN(
      get_logger(),
      "Dead-man check off (supervisor_timeout_ms = 0): if the safety supervisor dies, nothing "
      "stops the arm. Only for a run without the supervisor.");
    return true;
  }
  // The controller manager gives each component a node of its own, spun by its executor, so
  // the callback runs on an executor thread and never on the control loop. No node means the
  // component was loaded without an executor: never run with the check silently skipped.
  const rclcpp::Node::SharedPtr node{get_node()};
  if (node == nullptr) {
    RCLCPP_ERROR(
      get_logger(),
      "supervisor_timeout_ms is %d but the component has no ROS node to subscribe to %s with. "
      "Set it to 0 for a run without the safety supervisor.",
      config_.supervisor_timeout_ms, std::string{SUPERVISOR_HEARTBEAT_TOPIC}.c_str());
    return false;
  }
  supervisor_watchdog_.emplace(std::chrono::milliseconds{config_.supervisor_timeout_ms});
  // Only the arrival counts, so best effort and keep last 1, like the supervisor's publisher: a
  // lost heartbeat is not worth sending again, the next one is 20 ms away.
  heartbeat_subscription_ = node->create_subscription<positronic_interfaces::msg::Heartbeat>(
    std::string{SUPERVISOR_HEARTBEAT_TOPIC}, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    [this](const positronic_interfaces::msg::Heartbeat & /*message*/) {
      // When it arrived on this machine's steady clock, not the stamp it carries: a clock that
      // jumps, or sim time, cannot fake or hide a heartbeat.
      supervisor_watchdog_->on_reply(LinkWatchdog::Clock::now());
    });
  RCLCPP_INFO(
    get_logger(), "Dead-man check on: the arm halts after %d ms without a heartbeat on %s.",
    config_.supervisor_timeout_ms, std::string{SUPERVISOR_HEARTBEAT_TOPIC}.c_str());
  return true;
}

bool RobotSystem::read_parameters(const std::unordered_map<std::string, std::string> & parameters)
{
  const ParsedParameters parsed{parse_hardware_parameters(parameters)};
  if (!parsed.config.has_value()) {
    RCLCPP_ERROR(get_logger(), "%s", parsed.error.c_str());
    return false;
  }
  config_ = parsed.config.value();
  RCLCPP_INFO(get_logger(), "%s", describe(config_).c_str());
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
  if (has_gripper_) {
    const std::string & name{info.joints.at(NUM_JOINTS).name};
    const urdf::JointConstSharedPtr gripper{model.getJoint(name)};
    if (
      gripper == nullptr || gripper->type != urdf::Joint::REVOLUTE || gripper->limits == nullptr ||
      !std::isfinite(gripper->limits->lower) || !std::isfinite(gripper->limits->upper) ||
      gripper->limits->lower >= gripper->limits->upper)
    {
      RCLCPP_ERROR(
        get_logger(), "Gripper joint '%s' has no usable revolute <limit>. Refusing to start.",
        name.c_str());
      return false;
    }
    gripper_closed_ = gripper->limits->lower;
    gripper_open_ = gripper->limits->upper;
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
  echo_report_.reset();
  tracking_ = TrackingCheck{TrackingLimits{
      config_.tracking_error_deg * TRACKING_DEG_TO_RAD,
      std::chrono::milliseconds{config_.tracking_error_ms}}};
  try {
    if (config_.backend == Backend::SERIAL) {
      TemperatureGuard guard{};
      guard.limit_c = config_.servo_temp_limit_c;
      arm_ = std::make_unique<SerialArm>(
        config_.port, config_.baud, config_.timing, guard, has_gripper_);
    } else {
      // The fake arm's time is the sum of the periods read() got: on the control loop that is
      // the time that passed, and a test that passes the periods in moves the arm without
      // sleeping (#122). Not the `time` argument: the resource manager replaces it with its own
      // clock.
      arm_ = std::make_unique<FakeArm>(config_.fake_mode, [this]() {
            return FakeArm::Clock::time_point{std::chrono::duration_cast<FakeArm::Clock::duration>(
          std::chrono::nanoseconds{control_time_ns_.load()})};
      });
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Cannot start the serial backend: %s", error.what());
    return CallbackReturn::ERROR;
  }
  // A fresh history: the first read() after (re)configure has no earlier sample to difference.
  velocity_estimator_ = VelocityEstimator{config_.velocity_filter_alpha};
  gripper_previous_valid_ = false;

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
    if (has_gripper_) {
      const std::string & name{info_.joints.at(NUM_JOINTS).name};
      gripper_handles_.command =
        get_command_interface_handle(name + "/" + hardware_interface::HW_IF_POSITION);
      gripper_handles_.position =
        get_state_interface_handle(name + "/" + hardware_interface::HW_IF_POSITION);
      gripper_handles_.velocity =
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

CallbackReturn RobotSystem::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  // The resource manager deactivates an ACTIVE component before shutting it down, so the stop
  // of on_deactivate has gone out by now. Not every path does (HardwareComponent::shutdown()
  // takes ACTIVE straight here), so send it ourselves when the arm may still be moving (#103).
  if (arm_ != nullptr && previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    std::ignore = arm_->halt();
  }
  arm_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // #136: no motion without someone watching it. Otherwise read() would raise the fault again
  // at once.
  if (!supervisor_allows_activation()) {
    return CallbackReturn::ERROR;
  }
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

  while (has_gripper_ && !arm_->read_gripper().has_value()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(
        get_logger(), "The gripper did not answer within %d ms; not activating.",
        static_cast<int>(FIRST_SAMPLE_TIMEOUT.count()));
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

  // The over-temperature guard must be able to see before the servos get torque, and a hot arm
  // is not powered on at all.
  if (!temperatures_allow_activation()) {
    return CallbackReturn::ERROR;
  }

  // "Power the servos on" (#36), after the first reply: a released arm still answers
  // get_angles, so the wait above does not need torque, and a silent arm fails there first.
  // power_on() also lets targets out again after a halt(). If the arm does not acknowledge it,
  // the request was lost or the port is in trouble: ERROR hands the component to on_error
  // rather than leaving it ACTIVE with an arm nobody confirmed powered (#103).
  if (!arm_->power_on()) {
    RCLCPP_ERROR(
      get_logger(), "power_on was not acknowledged by the arm within %d ms; not activating.",
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
  tracking_.reset();
  std::ignore = set_state(fault_handle_, to_state(FaultCode::NONE), true);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // "Cut actuation": the arm gets a stop and holds where it is, and no target goes out until
  // the next activate. halt() waits for the arm to acknowledge the stop, so SUCCESS means the
  // arm confirmed it, not only that the frame left or the component changed state (#103).
  if (arm_->halt()) {
    return CallbackReturn::SUCCESS;
  }
  // The arm never confirmed the stop: nobody knows what it is doing. FAILURE would keep the
  // component ACTIVE and the controllers commanding it; ERROR takes it out of service through
  // on_error, and the supervisor sees a component that must be configured and activated again.
  RCLCPP_ERROR(
    get_logger(), "stop was not acknowledged by the arm within %d ms.",
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
  // A period that is not positive is no time passing: the fake arm's clock never goes back.
  if (period.nanoseconds() > 0) {
    control_time_ns_.fetch_add(period.nanoseconds());
  }
  update_states(period.seconds());
  // First, before the link check: the power off of an overheat cut silences the arm, so a link
  // timeout can follow the cut, and only the first fault is kept. The cause must win. Also
  // before the fault is written, so OVERHEAT is on the interface in the cycle that returns ERROR.
  const return_type temperatures{check_temperatures()};
  // #35: a link that goes silent while active is a fault. While inactive nothing can move the
  // arm, and on_activate refuses a silent link anyway.
  const bool active{is_active()};
  if (active && arm_->link_timed_out()) {
    raise_fault(FaultCode::LINK_TIMEOUT);
  }
  // #136: the supervisor went silent while active. After the link check, so a link that died
  // first keeps its fault, the cause. Returned as ERROR even then: with no supervisor left, the
  // controller manager is the one that takes the controllers and the component out of ACTIVE.
  // Logged here, once, for the same reason as the overheat: read() is not called again until
  // the component is configured.
  const bool supervisor_lost{active && supervisor_silent()};
  if (supervisor_lost) {
    raise_fault(FaultCode::SUPERVISOR_LOST);
    RCLCPP_ERROR(
      get_logger(),
      "No heartbeat from the safety supervisor for over %d ms (fault %d latched). Arm halted, "
      "targets held. Start the supervisor again and activate it to resume.",
      config_.supervisor_timeout_ms, static_cast<int>(fault_.load()));
  }
  // #103: a target the arm did not follow. After the link check: a silent arm has a stale
  // pose, and the silence is the cause to report, not the lag that follows it.
  if (active) {
    check_tracking();
    report_echo_timeouts();
  }
  // Reported, not returned as ERROR: the component stays ACTIVE and readable, the backend holds
  // every target, and the supervisor (#45) decides what happens next. Written every cycle, so
  // anyone reading it sees the latched value. Two exceptions return ERROR as well: the
  // overheat, because the arm is already cut and must not be driven again before a configure,
  // and the supervisor loss, because there is no supervisor to decide.
  std::ignore = set_state(fault_handle_, to_state(fault_.load()), false);
  if (supervisor_lost) {
    return return_type::ERROR;
  }
  return temperatures;
}

bool RobotSystem::supervisor_allows_activation()
{
  if (!supervisor_watchdog_.has_value()) {
    return true;
  }
  // The supervisor publishes from its configure on, so the heartbeat is normally there already;
  // the wait only covers a late one. Sleeping is fine here, outside the control loop.
  const auto deadline{LinkWatchdog::Clock::now() + supervisor_watchdog_->timeout()};
  while (supervisor_silent()) {
    if (LinkWatchdog::Clock::now() >= deadline) {
      RCLCPP_ERROR(
        get_logger(),
        "No heartbeat from the safety supervisor on %s for %d ms; not activating. Start "
        "supervised.launch.py, or set supervisor_timeout_ms:=0 for a run without the supervisor.",
        std::string{SUPERVISOR_HEARTBEAT_TOPIC}.c_str(), config_.supervisor_timeout_ms);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  return true;
}

bool RobotSystem::supervisor_silent() const
{
  return supervisor_watchdog_.has_value() &&
         supervisor_watchdog_->timed_out(LinkWatchdog::Clock::now());
}

void RobotSystem::report_echo_timeouts()
{
  const std::optional<std::uint64_t> fresh{
    echo_report_.due(arm_->echo_timeouts(), EchoTimeoutReport::Clock::now())};
  if (!fresh.has_value()) {
    return;
  }
  RCLCPP_WARN(
    get_logger(),
    "%" PRIu64
    " send_angles got no sync-write from the arm since the last report: each of their "
    "turns waited the whole echo timeout. A target outside the arm's own joint limits does this, "
    "and so does a send that reached the arm while it was still answering a request.",
    fresh.value());
}

bool RobotSystem::temperatures_allow_activation()
{
  const auto deadline{std::chrono::steady_clock::now() + FIRST_TEMPERATURE_TIMEOUT};
  std::optional<ServoTemperatures> temperatures{arm_->read_temperatures()};
  while (!temperatures.has_value()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(
        get_logger(),
        "No servo temperature within %d ms: the arm did not answer get_servo_temps (0xE5). Not "
        "activating without the over-temperature guard.",
        static_cast<int>(FIRST_TEMPERATURE_TIMEOUT.count()));
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
    temperatures = arm_->read_temperatures();
  }
  if (const std::optional<Overheat> cut{arm_->overheat()}) {
    RCLCPP_ERROR(
      get_logger(),
      "Not activated: joint '%s' overheated (%d degC) and the arm was cut. "
      "Let it cool, power it on again, then configure.",
      info_.joints.at(cut->joint).name.c_str(), cut->celsius);
    return false;
  }
  if (!cool_enough_to_activate(temperatures.value(), config_.servo_temp_limit_c)) {
    const Overheat hot{hottest(temperatures.value())};
    RCLCPP_ERROR(
      get_logger(),
      "Not activated: joint '%s' is at %d degC; every servo must be below %d degC "
      "(limit %d minus %d). Let the arm cool down, then activate again.",
      info_.joints.at(hot.joint).name.c_str(), hot.celsius,
      config_.servo_temp_limit_c - ACTIVATE_COOLDOWN_C, config_.servo_temp_limit_c,
      ACTIVATE_COOLDOWN_C);
    return false;
  }
  return true;
}

return_type RobotSystem::check_temperatures()
{
  // One log line each: ERROR takes the component through on_error to UNCONFIGURED, and read()
  // is not called again until it is configured. Same rule as the #34 refusal in write().
  if (const std::optional<Overheat> cut{arm_->overheat()}) {
    // The serial thread has already sent the stop and the power off, on its own: nothing is
    // left to do here but report it, through the fault (#35) and the ERROR. on_error asks for a
    // stop again, which changes nothing.
    raise_fault(FaultCode::OVERHEAT);
    RCLCPP_ERROR(
      get_logger(),
      "Servo overheated: joint '%s' at %d degC (limit %d degC). The arm was "
      "stopped and its servos powered off, so it is limp now. Find the cause, let it cool, "
      "power the arm on again, then configure and activate.",
      info_.joints.at(cut->joint).name.c_str(), cut->celsius, config_.servo_temp_limit_c);
    return return_type::ERROR;
  }
  // Blind while moving: halt (no power off, nothing is known to be hot). Not while inactive,
  // where nothing moves and on_activate checks the temperatures again.
  const bool active{is_active()};
  if (active && arm_->temperature_stale()) {
    arm_->request_halt();
    RCLCPP_ERROR(
      get_logger(),
      "No servo temperature for %d ms: the over-temperature guard is blind. Arm "
      "halted.",
      static_cast<int>(TemperatureGuard{}.stale_timeout.count()));
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // The resource manager never calls write() on an inactive component, and after a halt() the
  // backend holds every target anyway. This is a second guard for a caller outside the
  // controller manager (a test, a script). No log: nothing here is RT-safe to log, and the
  // normal path never reaches it.
  const bool active{is_active()};
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

  double gripper_target{0.0};
  const bool gripper_read{
    has_gripper_ && get_command(gripper_handles_.command, gripper_target, false)};
  if (
    gripper_read && (!std::isfinite(gripper_target) || gripper_target < gripper_closed_ ||
    gripper_target > gripper_open_))
  {
    arm_->request_halt();
    RCLCPP_ERROR(
      get_logger(), "Command refused, arm halted: joint '%s' = %.4f rad, outside [%.4f, %.4f] rad.",
      info_.joints.at(NUM_JOINTS).name.c_str(), gripper_target, gripper_closed_, gripper_open_);
    return return_type::ERROR;
  }

  arm_->write_angles(commands);
  if (gripper_read) {
    arm_->write_gripper(to_gripper_value(gripper_target, gripper_closed_, gripper_open_));
  }
  return return_type::OK;
}

bool RobotSystem::is_active() const
{
  return get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

void RobotSystem::check_tracking()
{
  if (fault_.load() != FaultCode::NONE || !arm_->has_sample()) {
    return;
  }
  const std::optional<JointPositions> target{arm_->last_target()};
  if (!target.has_value()) {
    // Nothing on the wire to follow: before the first send, or after a stop. A clock that was
    // running belongs to a target that is gone.
    tracking_.reset();
    return;
  }
  const std::optional<TrackingError> error{
    tracking_.update(target.value(), arm_->read_angles(), TrackingCheck::Clock::now())};
  if (!error.has_value()) {
    return;
  }
  // Logged here, once: raise_fault() does not know the joint and the error.
  RCLCPP_ERROR(
    get_logger(),
    "Fault %d: joint '%s' is %.1f deg from its target for over %d ms (limit %.1f deg). The arm "
    "did not follow the command. Targets held until the next activate.",
    static_cast<int>(FaultCode::TRACKING_ERROR), info_.joints.at(error->joint).name.c_str(),
    error->error_rad * RAD_TO_DEG, config_.tracking_error_ms, config_.tracking_error_deg);
  raise_fault(FaultCode::TRACKING_ERROR);
}

void RobotSystem::raise_fault(FaultCode code)
{
  // Only the first fault is kept: it is the cause, later ones are its consequences.
  FaultCode expected{FaultCode::NONE};
  if (!fault_.compare_exchange_strong(expected, code)) {
    return;
  }
  // Going silent does not stop the arm: it finishes its last target (#20, serial_link.md
  // section 5). request_halt() holds every target and leaves a stop for the serial thread. On a
  // dead line that stop never arrives, and no software can stop a move already sent, #45
  // included: only cutting the servo power does.
  arm_->request_halt();
  // Logged once, at the transition, like a refusal (#34): read() does not log on later cycles.
  // An overheat is logged by check_temperatures(), which knows the joint and the value, and a
  // supervisor loss by read(), which logs it even when another fault came first.
  if (code == FaultCode::LINK_TIMEOUT) {
    RCLCPP_ERROR(
      get_logger(),
      "Fault %d: no valid reply from the arm for over %d ms. Targets held until the next "
      "activate.",
      static_cast<int>(code), static_cast<int>(LinkWatchdog::DEFAULT_TIMEOUT.count()));
  }
}

void RobotSystem::refuse(const CommandCheck & check)
{
  // Going silent does not stop the arm: it keeps going to its last target. request_halt() is
  // the non-blocking stop of #36 (two atomic stores and a short lock, safe here on the control
  // loop); on_error asks for it again, which changes nothing. The refused command never reached
  // the backend.
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
  const std::optional<std::uint8_t> gripper{has_gripper_ ? arm_->read_gripper() : std::nullopt};
  if (gripper.has_value()) {
    const double position{to_gripper_radians(gripper.value(), gripper_closed_, gripper_open_)};
    const double velocity{
      gripper_previous_valid_ && dt > 0.0 ? (position - gripper_previous_) / dt : 0.0};
    gripper_previous_ = position;
    gripper_previous_valid_ = true;
    std::ignore = set_state(gripper_handles_.position, position, false);
    std::ignore = set_state(gripper_handles_.velocity, velocity, false);
  }
}

}  // namespace positronic_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(positronic_hardware::RobotSystem, hardware_interface::SystemInterface)
