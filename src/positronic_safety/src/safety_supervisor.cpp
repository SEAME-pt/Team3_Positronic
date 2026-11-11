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

#include "positronic_safety/safety_supervisor.hpp"

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "positronic_safety/fault_code.hpp"

namespace positronic_safety
{

namespace
{
using LifecycleState = lifecycle_msgs::msg::State;
using std::chrono::milliseconds;
using Status = positronic_interfaces::msg::SafetyStatus;

// The status message carries Mode and Reason as numbers, and SafetyStatus.msg names them again
// for the subscribers. If the two lists ever disagree, the build stops here.
constexpr bool matches(std::uint8_t constant, Mode mode)
{
  return constant == static_cast<std::uint8_t>(mode);
}
constexpr bool matches(std::uint8_t constant, Reason reason)
{
  return constant == static_cast<std::uint8_t>(reason);
}
static_assert(matches(Status::MODE_STOPPED, Mode::STOPPED));
static_assert(matches(Status::MODE_RESUMING, Mode::RESUMING));
static_assert(matches(Status::MODE_MONITORING, Mode::MONITORING));
static_assert(matches(Status::MODE_HALTING, Mode::HALTING));
static_assert(matches(Status::REASON_NONE, Reason::NONE));
static_assert(matches(Status::REASON_STOP_SERVICE, Reason::STOP_SERVICE));
static_assert(matches(Status::REASON_HEARTBEAT_LOST, Reason::HEARTBEAT_LOST));
static_assert(matches(Status::REASON_DRIVER_FAULT, Reason::DRIVER_FAULT));
static_assert(matches(Status::REASON_FAULT_DATA_LOST, Reason::FAULT_DATA_LOST));
static_assert(matches(Status::REASON_DEACTIVATED, Reason::DEACTIVATED));
static_assert(matches(Status::REASON_RESUME_FAILED, Reason::RESUME_FAILED));

/// How long `watchdog` has been silent at `now`, in ms, for a log line. -1 when it was never
/// fed.
std::int64_t silence_ms(const DeadlineWatchdog & watchdog, DeadlineWatchdog::Clock::time_point now)
{
  const std::optional<DeadlineWatchdog::Clock::duration> silence = watchdog.silence(now);
  if (!silence.has_value()) {
    return -1;
  }
  return std::chrono::duration_cast<milliseconds>(*silence).count();
}
}  // namespace

SafetySupervisor::SafetySupervisor(const rclcpp::NodeOptions & options)
: LifecycleNode("safety_supervisor", options)
{
  // The defaults are our arm on the Jetson (jetson_controllers.yaml, positronic_real.urdf.xacro).
  declare_parameter<std::string>("hardware_component", "positronic_arm");
  declare_parameter<std::string>("controller_manager", "/controller_manager");
  declare_parameter<std::string>("trajectory_action", "/arm_controller/follow_joint_trajectory");
  declare_parameter<std::vector<std::string>>(
    "controllers_to_start", std::vector<std::string>{"joint_state_broadcaster", "arm_controller"});
  // Every controller that writes a command interface. One left active here would write its old
  // target again as soon as the hardware component is activated, and the arm would move to it.
  declare_parameter<std::vector<std::string>>(
    "controllers_to_stop",
    std::vector<std::string>{"arm_controller", "forward_position_controller"});
  declare_parameter<std::string>("heartbeat_topic", "/mission/heartbeat");
  declare_parameter<std::string>("fault_topic", "/dynamic_joint_states");
  // FAULT_GPIO and FAULT_STATE in positronic_hardware.hpp.
  declare_parameter<std::string>("fault_gpio", "driver");
  declare_parameter<std::string>("fault_interface", "fault_code");
  // The mission node publishes at 10 Hz: 200 ms tolerates one lost heartbeat.
  declare_parameter<int>("heartbeat_timeout_ms", 200);
  // The fault code comes at the control rate, 30 Hz: 200 ms is six messages in a row lost.
  declare_parameter<int>("fault_timeout_ms", 200);
  // Above the 300 ms the driver waits for its stop to leave the port before it answers.
  declare_parameter<int>("halt_step_timeout_ms", 500);
  // Above the 1.3 s an activation takes at worst: 1 s for the arm's first reply, 300 ms for
  // the power on.
  declare_parameter<int>("resume_step_timeout_ms", 3000);
  declare_parameter<int>("check_period_ms", 20);
}

bool SafetySupervisor::read_parameters()
{
  bool valid = true;
  const auto read_name = [this, &valid](const std::string & name, std::string & value) {
      value = get_parameter(name).as_string();
      if (value.empty()) {
        RCLCPP_ERROR(get_logger(), "Parameter '%s' must not be empty.", name.c_str());
        valid = false;
      }
    };
  const auto read_ms = [this, &valid](const std::string & name) {
      const std::int64_t value = get_parameter(name).as_int();
      if (value <= 0) {
        RCLCPP_ERROR(
          get_logger(), "Parameter '%s' must be a positive number of ms, it is %" PRId64 ".",
          name.c_str(), value);
        valid = false;
      }
      return milliseconds{value};
    };

  read_name("hardware_component", config_.hardware_component);
  read_name("controller_manager", config_.controller_manager);
  read_name("trajectory_action", config_.trajectory_action);
  config_.controllers_to_start = get_parameter("controllers_to_start").as_string_array();
  config_.controllers_to_stop = get_parameter("controllers_to_stop").as_string_array();
  read_name("heartbeat_topic", config_.heartbeat_topic);
  read_name("fault_topic", config_.fault_topic);
  read_name("fault_gpio", config_.fault_gpio);
  read_name("fault_interface", config_.fault_interface);
  config_.heartbeat_timeout = read_ms("heartbeat_timeout_ms");
  config_.fault_timeout = read_ms("fault_timeout_ms");
  config_.halt_step_timeout = read_ms("halt_step_timeout_ms");
  config_.resume_step_timeout = read_ms("resume_step_timeout_ms");
  config_.check_period = read_ms("check_period_ms");
  return valid;
}

SafetySupervisor::CallbackReturn SafetySupervisor::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!read_parameters()) {
    return CallbackReturn::FAILURE;
  }
  logic_ = SupervisorLogic{};
  heartbeat_watchdog_ = DeadlineWatchdog{config_.heartbeat_timeout};
  fault_watchdog_ = DeadlineWatchdog{config_.fault_timeout};
  fault_code_ = std::numeric_limits<double>::quiet_NaN();

  // Only the newest message matters, and one that was lost is not worth sending again: the
  // next one is already on its way. Best effort also matches the mission node's publisher (a
  // reliable subscription would get nothing from it).
  const rclcpp::QoS newest_only = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  heartbeat_subscription_ = create_subscription<Heartbeat>(
    config_.heartbeat_topic, newest_only, [this](const Heartbeat & /*message*/) {
      heartbeat_watchdog_.feed(Clock::now());
    });
  fault_subscription_ = create_subscription<DynamicJointState>(
    config_.fault_topic, newest_only, [this](const DynamicJointState & message) {
      on_joint_states(message);
    });

  // Latched: whoever subscribes later still gets the current status.
  status_publisher_ = rclcpp::create_publisher<SafetyStatus>(
    *this, "~/status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  // rclcpp picks a service callback by its exact argument types: shared pointers by value.
  // Naming that std::function type lets the lambda take references instead of copies.
  using StopCallback =
    std::function<void (std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response>)>;
  const StopCallback on_stop =
    [this](
    const std::shared_ptr<Trigger::Request> & /*request*/,
    const std::shared_ptr<Trigger::Response> & response) {
      // The answer only says that the halt is on its way; ~/status tells when it is over.
      const bool started = begin_halt(Reason::STOP_SERVICE);
      response->success = true;
      response->message = started ? "Halt started." : "A halt is already running.";
    };
  stop_service_ = create_service<Trigger>("~/stop", on_stop);

  hardware_client_ = create_client<SetHardwareComponentState>(
    config_.controller_manager + "/set_hardware_component_state");
  switch_client_ = create_client<SwitchController>(
    config_.controller_manager + "/switch_controller");
  trajectory_client_ =
    rclcpp_action::create_client<FollowJointTrajectory>(this, config_.trajectory_action);

  timer_ = create_wall_timer(config_.check_period, [this]() {check();});

  publish_status();
  return CallbackReturn::SUCCESS;
}

SafetySupervisor::CallbackReturn SafetySupervisor::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (logic_.mode() != Mode::STOPPED) {
    RCLCPP_ERROR(get_logger(), "A halt is still running. Activate again when it is over.");
    return CallbackReturn::FAILURE;
  }
  if (heartbeat_watchdog_.expired(Clock::now())) {
    RCLCPP_ERROR(
      get_logger(), "No mission heartbeat on '%s'. Start and activate the mission node first.",
      config_.heartbeat_topic.c_str());
    return CallbackReturn::FAILURE;
  }
  if (!hardware_client_->service_is_ready() || !switch_client_->service_is_ready()) {
    RCLCPP_ERROR(
      get_logger(), "The services of '%s' are not available. Is the driver launched?",
      config_.controller_manager.c_str());
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Resuming: hardware component first, then the controllers.");
  const Order order = logic_.resume();
  publish_status();
  run(order);
  if (logic_.mode() == Mode::STOPPED) {
    // The resume failed on the spot and the halt that followed is over: stay INACTIVE.
    return CallbackReturn::FAILURE;
  }
  return LifecycleNode::on_activate(previous_state);
}

SafetySupervisor::CallbackReturn SafetySupervisor::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  // A person deactivated the supervisor: the same halt as any trigger. When the supervisor
  // deactivates itself at the end of a halt, the arm is already stopped.
  if (logic_.mode() == Mode::RESUMING || logic_.mode() == Mode::MONITORING) {
    begin_halt(Reason::DEACTIVATED);
  }
  return LifecycleNode::on_deactivate(previous_state);
}

SafetySupervisor::CallbackReturn SafetySupervisor::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reached from INACTIVE only, where the arm is stopped or a halt is running. Dropping the
  // clients drops the rest of a running halt: wait for STOPPED on ~/status before a cleanup.
  if (logic_.mode() != Mode::STOPPED) {
    RCLCPP_WARN(
      get_logger(), "Cleanup during %s: the requests not sent yet are dropped.",
      to_string(logic_.mode()));
  }
  timer_.reset();
  heartbeat_subscription_.reset();
  fault_subscription_.reset();
  stop_service_.reset();
  hardware_client_.reset();
  switch_client_.reset();
  trajectory_client_.reset();
  status_publisher_.reset();
  return CallbackReturn::SUCCESS;
}

SafetySupervisor::CallbackReturn SafetySupervisor::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // A shutdown can come straight from ACTIVE, with no on_deactivate before it. Nothing is
  // released here: the clients must live for the halt to go out.
  if (logic_.mode() == Mode::RESUMING || logic_.mode() == Mode::MONITORING) {
    begin_halt(Reason::DEACTIVATED);
  }
  return CallbackReturn::SUCCESS;
}

void SafetySupervisor::on_joint_states(const DynamicJointState & message)
{
  const std::optional<double> code =
    find_fault_code(message, config_.fault_gpio, config_.fault_interface);
  if (!code.has_value()) {
    return;  // bad data: the fault watchdog is not fed, and expires if this goes on
  }
  if (rclcpp::Time{message.header.stamp} < fault_valid_from_) {
    return;  // read before the last activation: may still carry the fault it cleared
  }
  fault_watchdog_.feed(Clock::now());
  fault_code_ = *code;
  if (*code != 0.0 && logic_.mode() == Mode::MONITORING) {
    RCLCPP_ERROR(
      get_logger(), "The driver reports a fault: %s/%s = %.0f.", config_.fault_gpio.c_str(),
      config_.fault_interface.c_str(), *code);
    begin_halt(Reason::DRIVER_FAULT);
  }
}

void SafetySupervisor::check()
{
  const Clock::time_point now = Clock::now();

  if (logic_.step() != Step::NONE && step_watchdog_.expired(now)) {
    RCLCPP_ERROR(
      get_logger(), "No answer to %s in %" PRId64 " ms: going on without it.",
      to_string(logic_.step()), static_cast<std::int64_t>(step_watchdog_.timeout().count()));
    // Only one request is ever in flight, so everything still pending is this one. Forget it,
    // or the clients keep it forever.
    hardware_client_->prune_pending_requests();
    switch_client_->prune_pending_requests();
    advance(pending_.ticket, false);
  }

  if (logic_.mode() != Mode::MONITORING) {
    return;
  }
  if (heartbeat_watchdog_.expired(now)) {
    RCLCPP_ERROR(
      get_logger(), "No mission heartbeat for %" PRId64 " ms (limit %" PRId64 " ms).",
      silence_ms(heartbeat_watchdog_, now),
      static_cast<std::int64_t>(config_.heartbeat_timeout.count()));
    begin_halt(Reason::HEARTBEAT_LOST);
  } else if (fault_watchdog_.expired(now)) {
    RCLCPP_ERROR(
      get_logger(), "No usable %s/%s on '%s' for %" PRId64 " ms (limit %" PRId64 " ms).",
      config_.fault_gpio.c_str(), config_.fault_interface.c_str(), config_.fault_topic.c_str(),
      silence_ms(fault_watchdog_, now),
      static_cast<std::int64_t>(config_.fault_timeout.count()));
    begin_halt(Reason::FAULT_DATA_LOST);
  }
}

bool SafetySupervisor::begin_halt(Reason reason)
{
  const Order order = logic_.halt(reason);
  if (order.step == Step::NONE) {
    return false;  // a halt is already running, and it keeps its first reason
  }
  RCLCPP_ERROR(get_logger(), "HALT (%s).", to_string(reason));
  publish_status();
  run(order);
  return true;
}

void SafetySupervisor::run(Order order)
{
  // Usually one turn: the request leaves and its answer, or its timeout, calls advance()
  // later. More turns only when a request cannot even be sent: its outcome is known at once
  // and the next one goes out. It ends: every turn uses up one step of a sequence.
  while (order.step != Step::NONE) {
    const std::optional<bool> outcome = send(order);
    if (!outcome.has_value()) {
      return;
    }
    order = report(order.ticket, *outcome);
  }
}

void SafetySupervisor::advance(std::uint64_t ticket, bool ok)
{
  run(report(ticket, ok));
}

Order SafetySupervisor::report(std::uint64_t ticket, bool ok)
{
  const Mode before = logic_.mode();
  const Order next = logic_.step_done(ticket, ok);
  if (logic_.mode() != before) {
    on_mode_changed();
  }
  return next;
}

std::optional<bool> SafetySupervisor::send(const Order & order)
{
  pending_ = order;
  const bool halting = logic_.mode() == Mode::HALTING;
  const milliseconds timeout =
    halting ? config_.halt_step_timeout : config_.resume_step_timeout;
  step_watchdog_ = DeadlineWatchdog{timeout};
  step_watchdog_.feed(Clock::now());

  switch (order.step) {
    case Step::HARDWARE_INACTIVE:
      return set_hardware_state(order.ticket, LifecycleState::PRIMARY_STATE_INACTIVE, "inactive");
    case Step::CANCEL_GOALS:
      return cancel_goals(order.ticket);
    case Step::CONTROLLERS_OFF:
      return switch_controllers(order.ticket, {}, config_.controllers_to_stop, timeout);
    case Step::HARDWARE_ACTIVE:
      return set_hardware_state(order.ticket, LifecycleState::PRIMARY_STATE_ACTIVE, "active");
    case Step::CONTROLLERS_ON:
      return switch_controllers(order.ticket, config_.controllers_to_start, {}, timeout);
    case Step::NONE:
      break;
  }
  return std::nullopt;
}

void SafetySupervisor::on_mode_changed()
{
  publish_status();
  switch (logic_.mode()) {
    case Mode::MONITORING:
      // The fault code must show up within its timeout from now on.
      fault_watchdog_.feed(Clock::now());
      RCLCPP_INFO(get_logger(), "Resumed: the arm may move, every trigger is watched.");
      break;
    case Mode::HALTING:
      RCLCPP_ERROR(get_logger(), "The resume failed: HALT (%s).", to_string(logic_.reason()));
      break;
    case Mode::STOPPED:
      if (logic_.hardware_stopped()) {
        RCLCPP_WARN(
          get_logger(), "Stopped (%s). Activate the supervisor to resume.",
          to_string(logic_.reason()));
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Stopped (%s), but the hardware component did not confirm it: THE ARM MAY STILL MOVE. "
          "Cut the servo power.", to_string(logic_.reason()));
      }
      // Stay stopped until a person activates the supervisor again. During a transition
      // (a halt that ended inside on_activate or on_deactivate) the state is not ACTIVE.
      if (get_current_state().id() == LifecycleState::PRIMARY_STATE_ACTIVE) {
        deactivate();
      }
      break;
    case Mode::RESUMING:
      break;
  }
}

std::optional<bool> SafetySupervisor::set_hardware_state(
  std::uint64_t ticket, std::uint8_t state_id, const char * label)
{
  if (!hardware_client_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "'%s' is not available.", hardware_client_->get_service_name());
    return false;
  }
  const auto request = std::make_shared<SetHardwareComponentState::Request>();
  request->name = config_.hardware_component;
  request->target_state.id = state_id;
  request->target_state.label = label;
  // rclcpp picks the callback by its exact argument type: the future by value. Naming that
  // std::function type lets the lambda take a reference instead of a copy.
  const HardwareClient::CallbackType on_answer =
    [this, ticket, state_id](const HardwareClient::SharedFuture & future) {
      on_hardware_answer(ticket, state_id, *future.get());
    };
  hardware_client_->async_send_request(request, on_answer);
  return std::nullopt;
}

void SafetySupervisor::on_hardware_answer(
  std::uint64_t ticket, std::uint8_t wanted_state,
  const SetHardwareComponentState::Response & response)
{
  if (!logic_.waiting_for(ticket)) {
    return;  // it came after its timeout, or a halt replaced the resume it belonged to
  }
  const std::uint8_t state = response.state.id;
  const char * label = response.state.label.c_str();
  // In these two states the controller manager no longer calls the driver's write().
  const bool cannot_command = state == LifecycleState::PRIMARY_STATE_UNCONFIGURED ||
    state == LifecycleState::PRIMARY_STATE_FINALIZED;
  bool ok = response.ok;

  if (wanted_state == LifecycleState::PRIMARY_STATE_ACTIVE) {
    if (ok) {
      fault_valid_from_ = now();
      RCLCPP_INFO(get_logger(), "The hardware component is active.");
    } else {
      RCLCPP_ERROR(
        get_logger(), "The hardware component did not activate (it is '%s'). See the driver log.",
        label);
    }
  } else if (ok) {
    RCLCPP_INFO(get_logger(), "The hardware component is inactive: the driver sent the stop.");
  } else if (cannot_command) {
    // Expected on a link fault: the stop could not leave the port, so on_deactivate returned
    // ERROR and the component went to UNCONFIGURED. In that state it sends nothing to the arm:
    // stopped for the supervisor. The next resume configures it again.
    RCLCPP_WARN(
      get_logger(),
      "The hardware component is '%s', not inactive: its stop did not reach the arm (link "
      "fault?). It sends no more commands. An arm with a dead line finishes its last move.",
      label);
    ok = true;
  } else {
    RCLCPP_ERROR(
      get_logger(), "The hardware component did not leave ACTIVE (it is '%s').", label);
  }
  advance(ticket, ok);
}

std::optional<bool> SafetySupervisor::switch_controllers(
  std::uint64_t ticket, const std::vector<std::string> & activate,
  const std::vector<std::string> & deactivate, milliseconds timeout)
{
  if (!switch_client_->service_is_ready()) {
    RCLCPP_ERROR(get_logger(), "'%s' is not available.", switch_client_->get_service_name());
    return false;
  }
  const auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  // A controller that is already in the wanted state, or is not loaded, is not an error.
  request->strictness = SwitchController::Request::BEST_EFFORT;
  request->activate_asap = false;
  // The controller manager gives up on the switch when the supervisor stops waiting for it.
  request->timeout = rclcpp::Duration{timeout};
  const SwitchClient::CallbackType on_answer =
    [this, ticket](const SwitchClient::SharedFuture & future) {
      on_switch_answer(ticket, *future.get());
    };
  switch_client_->async_send_request(request, on_answer);
  return std::nullopt;
}

void SafetySupervisor::on_switch_answer(
  std::uint64_t ticket, const SwitchController::Response & response)
{
  if (!logic_.waiting_for(ticket)) {
    return;
  }
  if (response.ok) {
    RCLCPP_INFO(get_logger(), "%s: done.", to_string(logic_.step()));
  } else {
    RCLCPP_ERROR(
      get_logger(), "%s: the controller manager refused. See its log.", to_string(logic_.step()));
  }
  advance(ticket, response.ok);
}

std::optional<bool> SafetySupervisor::cancel_goals(std::uint64_t ticket)
{
  if (!trajectory_client_->action_server_is_ready()) {
    // No trajectory controller, so no goal to cancel either: not a failure of the halt.
    RCLCPP_WARN(
      get_logger(), "The action '%s' is not available: no goal to cancel.",
      config_.trajectory_action.c_str());
    return true;
  }
  // A goal id and a stamp of zero mean "every goal", whoever sent it.
  trajectory_client_->async_cancel_all_goals(
    [this, ticket](const TrajectoryClient::CancelResponse::SharedPtr & response) {
      on_cancel_answer(ticket, *response);
    });
  return std::nullopt;
}

void SafetySupervisor::on_cancel_answer(
  std::uint64_t ticket, const TrajectoryClient::CancelResponse & response)
{
  if (!logic_.waiting_for(ticket)) {
    return;
  }
  const bool ok = response.return_code == TrajectoryClient::CancelResponse::ERROR_NONE;
  if (ok) {
    RCLCPP_INFO(
      get_logger(), "Trajectory goals being cancelled: %zu.", response.goals_canceling.size());
  } else {
    RCLCPP_ERROR(
      get_logger(), "The trajectory controller refused to cancel (code %d).",
      static_cast<int>(response.return_code));
  }
  advance(ticket, ok);
}

void SafetySupervisor::publish_status() const
{
  if (!status_publisher_) {
    return;
  }
  SafetyStatus status;
  status.stamp = now();
  status.mode = static_cast<std::uint8_t>(logic_.mode());
  status.reason = static_cast<std::uint8_t>(logic_.reason());
  status.fault_code = fault_code_;
  status.hardware_stopped = logic_.hardware_stopped();
  status_publisher_->publish(status);
}

}  // namespace positronic_safety
