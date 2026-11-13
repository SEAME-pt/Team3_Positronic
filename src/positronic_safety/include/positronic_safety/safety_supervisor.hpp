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

#ifndef POSITRONIC_SAFETY__SAFETY_SUPERVISOR_HPP_
#define POSITRONIC_SAFETY__SAFETY_SUPERVISOR_HPP_

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "positronic_interfaces/msg/heartbeat.hpp"
#include "positronic_interfaces/msg/safety_status.hpp"
#include "positronic_safety/deadline_watchdog.hpp"
#include "positronic_safety/supervisor_logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace positronic_safety
{

/// The safety supervisor (#45): a lifecycle node that halts the arm and keeps it halted.
///
/// ACTIVE means the arm may move and is watched. Any trigger runs the one halt path (hardware
/// component inactive, trajectory goals cancelled, command controllers deactivated), then the
/// supervisor deactivates itself. It stays INACTIVE, and the arm stopped, until a person
/// activates it again; activating runs the resume path (hardware component active, then the
/// controllers).
///
/// Triggers: the ~/stop service, no mission heartbeat for heartbeat_timeout_ms, a
/// driver/fault_code other than 0, no usable fault code for fault_timeout_ms, and a person
/// deactivating the supervisor.
///
/// This class only talks to ROS. What to do next is decided by SupervisorLogic, and "too long
/// ago" by DeadlineWatchdog; both are tested without ROS. Every callback runs on one thread
/// (spin it in a single-threaded executor), so there is no lock and no atomic. Every request
/// is asynchronous: no callback ever waits for an answer.
///
/// Limits: it stops the arm through the controller manager and the driver, so it cannot stop
/// an arm whose serial line is dead (see "Link timeout" in the positronic_hardware README), and
/// nothing stops the arm for it if its own process dies.
class SafetySupervisor : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SafetySupervisor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Reads the parameters and creates the subscriptions, the clients, the stop service and the
  /// timer. From here on the heartbeat and the fault code are followed, so on_activate can tell
  /// whether they are there.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /// Starts the resume. Refused while a halt is still running, without a mission heartbeat,
  /// or without the controller manager.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /// Runs the halt, unless the supervisor is deactivating itself at the end of one.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  /// Runs the halt if the arm could still move. Keeps the clients, so the requests go out.
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  using Clock = DeadlineWatchdog::Clock;
  using DynamicJointState = control_msgs::msg::DynamicJointState;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using Heartbeat = positronic_interfaces::msg::Heartbeat;
  using SafetyStatus = positronic_interfaces::msg::SafetyStatus;
  using SetHardwareComponentState = controller_manager_msgs::srv::SetHardwareComponentState;
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using Trigger = std_srvs::srv::Trigger;

  using HardwareClient = rclcpp::Client<SetHardwareComponentState>;
  using SwitchClient = rclcpp::Client<SwitchController>;
  using TrajectoryClient = rclcpp_action::Client<FollowJointTrajectory>;

  /// The parameters, read in on_configure.
  struct Config
  {
    /// Name of the <ros2_control> block of the URDF.
    std::string hardware_component;
    /// Node whose set_hardware_component_state and switch_controller services are called.
    std::string controller_manager;
    /// FollowJointTrajectory action whose goals a halt cancels.
    std::string trajectory_action;
    /// Controllers a resume activates, after the hardware component.
    std::vector<std::string> controllers_to_start;
    /// Controllers a halt deactivates: every controller that writes a command interface.
    std::vector<std::string> controllers_to_stop;
    std::string heartbeat_topic;
    /// Where joint_state_broadcaster publishes every state interface, the driver gpio included.
    std::string fault_topic;
    std::string fault_gpio;
    std::string fault_interface;
    std::chrono::milliseconds heartbeat_timeout{0};
    std::chrono::milliseconds fault_timeout{0};
    /// How long a halt waits for the answer to one request before it sends the next one.
    std::chrono::milliseconds halt_step_timeout{0};
    /// The same for a resume, which then turns into a halt.
    std::chrono::milliseconds resume_step_timeout{0};
    /// Period of the timer that checks the timeouts.
    std::chrono::milliseconds check_period{0};
  };

  /// Fills config_. False, with an error logged, on an empty name or a timeout that is not
  /// positive: a bad parameter must stop the supervisor, not be ignored.
  bool read_parameters();

  /// A /dynamic_joint_states message: follows the fault code and halts on a fault.
  void on_joint_states(const DynamicJointState & message);

  /// The timer: gives up on a request that was not answered, then checks the heartbeat and
  /// the fault data.
  void check();

  /// Starts the halt for `reason`. False when one is already running.
  bool begin_halt(Reason reason);

  /// Sends `order`, and the orders after it for as long as their outcome is known at once.
  void run(Order order);

  /// The request `ticket` was answered, or timed out: tells the logic and sends what is next.
  void advance(std::uint64_t ticket, bool ok);

  /// Tells the logic how request `ticket` ended. Returns what it orders next.
  Order report(std::uint64_t ticket, bool ok);

  /// Sends the request of `order` and starts counting its timeout. Empty when the request
  /// left: its answer comes later. A value when the outcome is known at once, because the
  /// request cannot be sent: true or false is what the logic must be told.
  std::optional<bool> send(const Order & order);

  /// Called when the logic moved to another mode.
  void on_mode_changed();

  // The three kinds of request. They return what send() returns.
  std::optional<bool> set_hardware_state(
    std::uint64_t ticket, std::uint8_t state_id, const char * label);
  void on_hardware_answer(
    std::uint64_t ticket, std::uint8_t wanted_state,
    const SetHardwareComponentState::Response & response);

  std::optional<bool> switch_controllers(
    std::uint64_t ticket, const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate, std::chrono::milliseconds timeout);
  void on_switch_answer(std::uint64_t ticket, const SwitchController::Response & response);

  std::optional<bool> cancel_goals(std::uint64_t ticket);
  void on_cancel_answer(
    std::uint64_t ticket, const TrajectoryClient::CancelResponse & response);

  void publish_status() const;

  Config config_{};
  SupervisorLogic logic_{};
  DeadlineWatchdog heartbeat_watchdog_{std::chrono::milliseconds{0}};
  DeadlineWatchdog fault_watchdog_{std::chrono::milliseconds{0}};
  /// Counts the wait for the answer to the request in flight.
  DeadlineWatchdog step_watchdog_{std::chrono::milliseconds{0}};
  /// The request in flight.
  Order pending_{};
  /// The last usable fault code read, NaN before the first one.
  double fault_code_{std::numeric_limits<double>::quiet_NaN()};
  /// When the hardware component was last activated. Activating clears the driver's latched
  /// fault, so a message stamped before this may still carry the old code and is ignored.
  rclcpp::Time fault_valid_from_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<Heartbeat>::SharedPtr heartbeat_subscription_;
  rclcpp::Subscription<DynamicJointState>::SharedPtr fault_subscription_;
  /// A plain publisher, not a lifecycle one: the status must also go out while INACTIVE.
  rclcpp::Publisher<SafetyStatus>::SharedPtr status_publisher_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  HardwareClient::SharedPtr hardware_client_;
  SwitchClient::SharedPtr switch_client_;
  TrajectoryClient::SharedPtr trajectory_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace positronic_safety

#endif  // POSITRONIC_SAFETY__SAFETY_SUPERVISOR_HPP_
