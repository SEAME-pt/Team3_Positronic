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
//   Write a GoogleTest suite for the SafetySupervisor node (issue #45) that needs no arm and no
//   controller manager. One fake node plays everything the supervisor talks to: the two
//   controller manager services, the FollowJointTrajectory action server, the mission
//   heartbeat and /dynamic_joint_states with the driver's fault code. It records the requests
//   in the order they arrive. Spin the fake and the supervisor in one single-threaded executor.
//   Check the acceptance criteria and the team's decisions: activation is refused without a
//   heartbeat; a resume is hardware active, then controllers on; the stop service halts in the
//   order hardware inactive, cancel the active goal, controllers off, and the supervisor ends
//   INACTIVE; a lost heartbeat, a fault code other than 0, missing fault data and a fault code
//   that is not a number each halt with their own reason; a deactivate that fails and leaves
//   the hardware UNCONFIGURED (link fault) counts as stopped, one that leaves it ACTIVE does
//   not; the halt goes on when the controller manager does not answer; a failed resume becomes
//   a halt; the supervisor stays stopped until it is activated again; it resumes after a driver
//   fault once the driver cleared it; a person deactivating the supervisor halts the arm.
//   Every test uses its own namespace, so tests running at the same time do not hear each other.
// Follow-up prompt (#136):
//   The supervisor publishes ~/heartbeat at every check for the driver's dead-man check, and
//   halt_before_exit() is main()'s Ctrl+C hook. Test that the heartbeat arrives while STOPPED
//   and while MONITORING; that the hook, called from another thread while the test spins,
//   sends "hardware inactive" while MONITORING and returns; and that it sends nothing while
//   STOPPED.
// Follow-up prompt (#136 review):
//   Test that the hook returns after halt_step_timeout_ms when the controller manager never
//   answers, and that a supervisor shut down while monitoring stops its heartbeat.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "control_msgs/msg/interface_value.hpp"
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "positronic_interfaces/msg/heartbeat.hpp"
#include "positronic_interfaces/msg/safety_status.hpp"
#include "positronic_safety/safety_supervisor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
using control_msgs::action::FollowJointTrajectory;
using control_msgs::msg::DynamicJointState;
using control_msgs::msg::InterfaceValue;
using controller_manager_msgs::srv::SetHardwareComponentState;
using controller_manager_msgs::srv::SwitchController;
using lifecycle_msgs::msg::State;
using positronic_interfaces::msg::Heartbeat;
using positronic_interfaces::msg::SafetyStatus;
using positronic_safety::SafetySupervisor;
using std::chrono::milliseconds;
using std_srvs::srv::Trigger;

using Calls = std::vector<std::string>;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

/// Everything the supervisor talks to, in one node: the controller manager's two services, the
/// trajectory controller's action, the mission node's heartbeat and the broadcaster's
/// /dynamic_joint_states. The public members are the knobs and the records of a test.
class FakeArmSide : public rclcpp::Node
{
public:
  explicit FakeArmSide(const std::string & ns)
  : Node("fake_arm_side", ns)
  {
    create_topics();
    create_services();
    create_action();
  }

  /// A trajectory goal from "MoveIt", for the supervisor to cancel.
  void send_goal()
  {
    trajectory_client_->async_send_goal(FollowJointTrajectory::Goal{});
  }

  /// Calls the supervisor's stop service, like `ros2 service call`.
  void call_stop()
  {
    stop_client_->async_send_request(std::make_shared<Trigger::Request>());
  }

  bool ready() const
  {
    return stop_client_->service_is_ready() && trajectory_client_->action_server_is_ready();
  }

  bool goal_running() const
  {
    return goal_ != nullptr;
  }

  /// The controller manager crashed: its two services are gone. Returns once this node no
  /// longer sees them, so the supervisor (same process, same graph) does not either.
  bool controller_manager_gone(milliseconds timeout)
  {
    hardware_service_.reset();
    switch_service_.reset();
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (hardware_probe_->service_is_ready() || switch_probe_->service_is_ready()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(milliseconds{10});
    }
    return true;
  }

  /// What the supervisor asked for, in the order it arrived: "hardware:active",
  /// "hardware:inactive", "cancel", "controllers:on", "controllers:off".
  Calls calls;
  /// The controller lists of the last switch_controller requests.
  Calls activated;
  Calls deactivated;
  /// Every status the supervisor published, oldest first.
  std::vector<SafetyStatus> statuses;
  /// How many heartbeats of the supervisor arrived (#136), and the state of the last one.
  std::size_t supervisor_heartbeats{0};
  std::uint8_t supervisor_heartbeat_state{Heartbeat::STATE_IDLE};

  bool publish_heartbeat{true};
  bool publish_fault_data{true};
  double fault_code{0.0};
  /// False: set_hardware_component_state is never answered, like a hung controller manager.
  bool answer_hardware{true};
  bool activation_succeeds{true};
  bool deactivation_succeeds{true};
  /// Where a failed deactivation leaves the hardware component. UNCONFIGURED is what the
  /// driver does on a link fault.
  std::uint8_t state_after_failed_deactivation{State::PRIMARY_STATE_UNCONFIGURED};
  /// True: for STALE_WINDOW after answering an activation, every other message has fault
  /// code 1 and a stamp from before the activation, like messages still on their way from
  /// before the driver cleared its fault.
  bool stale_fault_after_activation{false};
  static constexpr milliseconds STALE_WINDOW{500};

private:
  void create_topics()
  {
    heartbeat_publisher_ = create_publisher<Heartbeat>(
      "mission/heartbeat", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    joint_states_publisher_ =
      create_publisher<DynamicJointState>("dynamic_joint_states", rclcpp::SystemDefaultsQoS());
    // 100 Hz: well inside the supervisor's timeouts, so a halt in a test is never an accident.
    publish_timer_ = create_wall_timer(milliseconds{10}, [this]() {
          publish();
    });
    status_subscription_ = create_subscription<SafetyStatus>(
      "safety_supervisor/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](const SafetyStatus & status) {
        statuses.push_back(status);
      });
    supervisor_heartbeat_subscription_ = create_subscription<Heartbeat>(
      "safety_supervisor/heartbeat", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const Heartbeat & heartbeat) {
        ++supervisor_heartbeats;
        supervisor_heartbeat_state = heartbeat.state;
      });
  }

  void create_services()
  {
    // This callback form answers by hand (send_response), so a test can also not answer.
    using HardwareCallback = std::function<void(
          std::shared_ptr<rmw_request_id_t>, std::shared_ptr<SetHardwareComponentState::Request>)>;
    const HardwareCallback on_hardware =
      [this](
      const std::shared_ptr<rmw_request_id_t> & header,
      const std::shared_ptr<SetHardwareComponentState::Request> & request) {
        on_hardware_request(*header, *request);
      };
    hardware_service_ = create_service<SetHardwareComponentState>(
      "controller_manager/set_hardware_component_state", on_hardware);

    using SwitchCallback = std::function<void(
          std::shared_ptr<SwitchController::Request>, std::shared_ptr<SwitchController::Response>)>;
    const SwitchCallback on_switch =
      [this](
      const std::shared_ptr<SwitchController::Request> & request,
      const std::shared_ptr<SwitchController::Response> & response) {
        if (request->activate_controllers.empty()) {
          calls.emplace_back("controllers:off");
          deactivated = request->deactivate_controllers;
        } else {
          calls.emplace_back("controllers:on");
          activated = request->activate_controllers;
        }
        response->ok = true;
      };
    switch_service_ =
      create_service<SwitchController>("controller_manager/switch_controller", on_switch);

    stop_client_ = create_client<Trigger>("safety_supervisor/stop");
    hardware_probe_ =
      create_client<SetHardwareComponentState>("controller_manager/set_hardware_component_state");
    switch_probe_ = create_client<SwitchController>("controller_manager/switch_controller");
  }

  void create_action()
  {
    trajectory_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this, "arm_controller/follow_joint_trajectory",
      [](
        const rclcpp_action::GoalUUID & /*uuid*/,
        const std::shared_ptr<const FollowJointTrajectory::Goal> & /*goal*/) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> & /*goal*/) {
        calls.emplace_back("cancel");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> & goal) {
        goal_ = goal;  // kept, so the goal stays active until someone cancels it
      });
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "arm_controller/follow_joint_trajectory");
  }

  void on_hardware_request(
    rmw_request_id_t & header, const SetHardwareComponentState::Request & request)
  {
    const bool to_active = request.target_state.id == State::PRIMARY_STATE_ACTIVE;
    calls.emplace_back(to_active ? "hardware:active" : "hardware:inactive");
    if (!answer_hardware) {
      return;
    }
    SetHardwareComponentState::Response response;
    if (to_active) {
      response.ok = activation_succeeds;
      response.state.id =
        activation_succeeds ? State::PRIMARY_STATE_ACTIVE : State::PRIMARY_STATE_UNCONFIGURED;
      if (activation_succeeds) {
        fault_code = 0.0;  // the driver clears its latched fault when it activates
        if (stale_fault_after_activation) {
          stale_stamp_ = now() - rclcpp::Duration{milliseconds{50}};
          stale_until_ = now() + rclcpp::Duration{STALE_WINDOW};
        }
      }
    } else {
      response.ok = deactivation_succeeds;
      response.state.id =
        deactivation_succeeds ? State::PRIMARY_STATE_INACTIVE : state_after_failed_deactivation;
    }
    response.state.label = "state " + std::to_string(response.state.id);
    hardware_service_->send_response(header, response);
  }

  void publish()
  {
    if (publish_heartbeat) {
      Heartbeat heartbeat;
      heartbeat.stamp = now();
      heartbeat.node_name = get_name();
      heartbeat_publisher_->publish(heartbeat);
    }
    if (publish_fault_data) {
      InterfaceValue driver;
      driver.interface_names = {"fault_code"};
      driver.values = {fault_code};
      DynamicJointState message;
      message.header.stamp = now();
      message.joint_names = {"driver"};
      message.interface_values = {driver};
      // In the stale window every other tick sends the stale message instead: the supervisor
      // keeps only the newest message, so two in one tick would hide one of them. A fresh one
      // still comes every 20 ms, well inside the fault timeout.
      if (now() < stale_until_ && (++stale_tick_ % 2) == 0) {
        message.header.stamp = stale_stamp_;
        message.interface_values.front().values = {1.0};
      }
      joint_states_publisher_->publish(message);
    }
  }

  rclcpp::Publisher<Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<DynamicJointState>::SharedPtr joint_states_publisher_;
  rclcpp::Subscription<SafetyStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<Heartbeat>::SharedPtr supervisor_heartbeat_subscription_;
  rclcpp::Service<SetHardwareComponentState>::SharedPtr hardware_service_;
  rclcpp::Service<SwitchController>::SharedPtr switch_service_;
  rclcpp::Client<Trigger>::SharedPtr stop_client_;
  rclcpp::Client<SetHardwareComponentState>::SharedPtr hardware_probe_;
  rclcpp::Client<SwitchController>::SharedPtr switch_probe_;
  rclcpp::Time stale_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stale_until_{0, 0, RCL_ROS_TIME};
  std::uint64_t stale_tick_{0};
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr trajectory_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::shared_ptr<GoalHandle> goal_;
};

const Calls HALT{"hardware:inactive", "controllers:off"};
const Calls HALT_WITH_A_GOAL{"hardware:inactive", "cancel", "controllers:off"};
const Calls RESUME{"hardware:active", "controllers:on"};
}  // namespace

/// One fake and one supervisor, in a namespace of their own, spun by one executor: every
/// callback of both runs on the test's thread, between two lines of a test.
class SafetySupervisorTest : public testing::Test
{
protected:
  void SetUp() override
  {
    // The clock makes it unique across processes, the test name across tests.
    const std::string ns =
      "/supervisor_test_" +
      std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + "_" +
      testing::UnitTest::GetInstance()->current_test_info()->name();
    fake_ = std::make_shared<FakeArmSide>(ns);

    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=" + ns});
    options.append_parameter_override("controller_manager", ns + "/controller_manager");
    options.append_parameter_override(
      "trajectory_action", ns + "/arm_controller/follow_joint_trajectory");
    options.append_parameter_override("heartbeat_topic", ns + "/mission/heartbeat");
    options.append_parameter_override("fault_topic", ns + "/dynamic_joint_states");
    // Shorter than on the arm, so the test that waits for a timeout stays quick.
    options.append_parameter_override("halt_step_timeout_ms", 200);
    options.append_parameter_override("resume_step_timeout_ms", 500);
    supervisor_ = std::make_shared<SafetySupervisor>(options);

    executor_.add_node(fake_);
    executor_.add_node(supervisor_->get_node_base_interface());
  }

  /// Spins until `done` says so. False when it did not within `timeout`.
  bool spin_until(const std::function<bool()> & done, milliseconds timeout = milliseconds{5000})
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      executor_.spin_once(milliseconds{10});
    }
    return true;
  }

  void spin_for(milliseconds duration)
  {
    spin_until(
      []() {
        return false;
      },
      duration);
  }

  /// Lets some time pass, then activates the supervisor. True when it accepted.
  bool try_activate()
  {
    spin_for(milliseconds{50});
    return supervisor_->activate().id() == State::PRIMARY_STATE_ACTIVE;
  }

  /// Configures and activates the supervisor and waits until it watches the arm.
  void start_monitoring()
  {
    ASSERT_EQ(supervisor_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    // Discovery takes a moment. Until the supervisor has heard a heartbeat and found the
    // services it refuses to activate, as it should: ask until it accepts.
    ASSERT_TRUE(spin_until([this]() {
        return try_activate();
    }));
    ASSERT_TRUE(spin_until([this]() {
        return mode_is(SafetyStatus::MODE_MONITORING);
    }));
    ASSERT_TRUE(spin_until([this]() {
        return fake_->ready();
    }));
    ASSERT_EQ(fake_->calls, RESUME);
    fake_->calls.clear();
  }

  bool mode_is(std::uint8_t mode) const
  {
    return !fake_->statuses.empty() && fake_->statuses.back().mode == mode;
  }

  /// True once the last status says STOPPED for `reason`.
  bool stopped_because(std::uint8_t reason) const
  {
    return mode_is(SafetyStatus::MODE_STOPPED) && fake_->statuses.back().reason == reason;
  }

  std::uint8_t supervisor_state() const
  {
    return supervisor_->get_current_state().id();
  }

  std::shared_ptr<FakeArmSide> fake_;
  std::shared_ptr<SafetySupervisor> supervisor_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(SafetySupervisorTest, ActivationIsRefusedWithoutAHeartbeat)
{
  fake_->publish_heartbeat = false;
  ASSERT_EQ(supervisor_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(milliseconds{500});  // long enough to find the services: only the heartbeat lacks

  EXPECT_EQ(supervisor_->activate().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(milliseconds{100});
  EXPECT_TRUE(fake_->calls.empty());  // nothing was enabled
  EXPECT_TRUE(mode_is(SafetyStatus::MODE_STOPPED));
}

TEST_F(SafetySupervisorTest, ResumeIsHardwareThenControllers)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());  // checks the order of the two requests

  const Calls expected{"joint_state_broadcaster", "arm_controller"};
  EXPECT_EQ(fake_->activated, expected);
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(fake_->statuses.back().reason, SafetyStatus::REASON_NONE);
  EXPECT_FALSE(fake_->statuses.back().hardware_stopped);
}

TEST_F(SafetySupervisorTest, StopServiceHaltsInOrderAndCancelsTheGoal)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());
  fake_->send_goal();
  ASSERT_TRUE(spin_until([this]() {
      return fake_->goal_running();
  }));

  fake_->call_stop();
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_STOP_SERVICE);
  }));

  EXPECT_EQ(fake_->calls, HALT_WITH_A_GOAL);
  const Calls expected{"arm_controller", "forward_position_controller"};
  EXPECT_EQ(fake_->deactivated, expected);
  EXPECT_TRUE(fake_->statuses.back().hardware_stopped);
  // Stays stopped: the supervisor deactivated itself.
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SafetySupervisorTest, LostHeartbeatHalts)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  fake_->publish_heartbeat = false;  // the mission node died, or its network did
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_HEARTBEAT_LOST);
  }));

  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SafetySupervisorTest, DriverFaultHalts)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  fake_->fault_code = 1.0;  // LINK_TIMEOUT in positronic_hardware
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_DRIVER_FAULT);
  }));

  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_DOUBLE_EQ(fake_->statuses.back().fault_code, 1.0);
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SafetySupervisorTest, MissingFaultDataHalts)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  fake_->publish_fault_data = false;  // the broadcaster was deactivated
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_FAULT_DATA_LOST);
  }));

  EXPECT_EQ(fake_->calls, HALT);
}

TEST_F(SafetySupervisorTest, FaultCodeThatIsNotANumberHalts)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  // Bad data: messages keep coming, but none of them says whether there is a fault.
  fake_->fault_code = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_FAULT_DATA_LOST);
  }));

  EXPECT_EQ(fake_->calls, HALT);
}

TEST_F(SafetySupervisorTest, UnconfiguredAfterAFailedDeactivateCountsAsStopped)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  // A link fault: the driver's stop cannot leave the port, on_deactivate returns ERROR and the
  // hardware component ends UNCONFIGURED.
  fake_->deactivation_succeeds = false;
  fake_->fault_code = 1.0;
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_DRIVER_FAULT);
  }));

  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_TRUE(fake_->statuses.back().hardware_stopped);
}

TEST_F(SafetySupervisorTest, HardwareStillActiveIsNotReportedAsStopped)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  fake_->deactivation_succeeds = false;
  fake_->state_after_failed_deactivation = State::PRIMARY_STATE_ACTIVE;
  fake_->call_stop();
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_STOP_SERVICE);
  }));

  EXPECT_EQ(fake_->calls, HALT);  // the controllers were still deactivated
  EXPECT_FALSE(fake_->statuses.back().hardware_stopped);
}

TEST_F(SafetySupervisorTest, HaltGoesOnWhenTheControllerManagerDoesNotAnswer)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  fake_->answer_hardware = false;
  const auto start = std::chrono::steady_clock::now();
  fake_->call_stop();
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_STOP_SERVICE);
  }));

  // It waited for the answer (halt_step_timeout_ms is 200 here), then sent the rest anyway.
  EXPECT_GE(std::chrono::steady_clock::now() - start, milliseconds{200});
  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_FALSE(fake_->statuses.back().hardware_stopped);
}

TEST_F(SafetySupervisorTest, FailedResumeBecomesAHalt)
{
  fake_->activation_succeeds = false;  // the driver refuses to activate
  ASSERT_EQ(supervisor_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(spin_until([this]() {
      return try_activate();
  }));
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_RESUME_FAILED);
  }));

  // The controllers were never activated, and the full halt ran.
  const Calls expected{"hardware:active", "hardware:inactive", "controllers:off"};
  EXPECT_EQ(fake_->calls, expected);
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SafetySupervisorTest, StaysStoppedUntilAPersonActivatesAgain)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());
  fake_->call_stop();
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_STOP_SERVICE);
  }));
  fake_->calls.clear();

  // Heartbeat and fault code are fine, and still nothing comes back by itself.
  spin_for(milliseconds{500});
  EXPECT_TRUE(fake_->calls.empty());
  EXPECT_TRUE(stopped_because(SafetyStatus::REASON_STOP_SERVICE));
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);

  ASSERT_TRUE(spin_until([this]() {
      return try_activate();
  }));
  ASSERT_TRUE(spin_until([this]() {
      return mode_is(SafetyStatus::MODE_MONITORING);
  }));
  EXPECT_EQ(fake_->calls, RESUME);
}

TEST_F(SafetySupervisorTest, ResumesAfterADriverFaultOnceTheDriverClearedIt)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());
  fake_->fault_code = 1.0;
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_DRIVER_FAULT);
  }));

  // The fake clears the code when it is activated, like the driver. Messages that still carry
  // the old code may be on their way: they must not halt the arm again.
  ASSERT_TRUE(spin_until([this]() {
      return try_activate();
  }));
  ASSERT_TRUE(spin_until([this]() {
      return mode_is(SafetyStatus::MODE_MONITORING);
  }));
  spin_for(milliseconds{500});
  EXPECT_TRUE(mode_is(SafetyStatus::MODE_MONITORING));
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(SafetySupervisorTest, DeactivatingTheSupervisorHalts)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  EXPECT_EQ(supervisor_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_DEACTIVATED);
  }));

  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_TRUE(fake_->statuses.back().hardware_stopped);
}

// AI-generated (Claude, Anthropic).
// Prompt:
//   #46 test review S1-S3. S1: a shutdown requested while MONITORING must send "hardware
//   inactive". S2: after a resume from a driver fault, messages
//   stamped before the activation and still carrying the old code must not halt again, while a
//   new fault still does. S3: with the controller manager gone, a halt must end STOPPED at once
//   and report the hardware as not stopped.

TEST_F(SafetySupervisorTest, ShutdownWhileMonitoringSendsHardwareInactive)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  // A shutdown request from ACTIVE. In the installed rclcpp_lifecycle this runs on_deactivate
  // before on_shutdown (found by deleting on_shutdown's halt: this test still passed, with
  // "HALT (DEACTIVATED)" from on_deactivate). on_shutdown's own halt is a second guard for a
  // version that would not. The test checks the outcome, whichever callback gives it.
  EXPECT_EQ(supervisor_->shutdown().id(), State::PRIMARY_STATE_FINALIZED);

  // Step 1 is the one that stops the arm: it must leave even though the node is finalizing.
  ASSERT_TRUE(spin_until([this]() {
      return !fake_->calls.empty() && fake_->calls.front() == "hardware:inactive";
  }));
}

TEST_F(SafetySupervisorTest, StaleFaultCodeAfterAResumeDoesNotHaltButANewOneDoes)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());
  fake_->fault_code = 1.0;
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_DRIVER_FAULT);
  }));

  // Resume. For 500 ms after it activates the hardware, every other message from the fake has
  // code 1 and a stamp from before the activation. Without the filter, the first one seen in
  // MONITORING halts.
  fake_->stale_fault_after_activation = true;
  ASSERT_TRUE(spin_until([this]() {
      return try_activate();
  }));
  ASSERT_TRUE(spin_until([this]() {
      return mode_is(SafetyStatus::MODE_MONITORING);
  }));
  spin_for(FakeArmSide::STALE_WINDOW + milliseconds{100});  // the whole stale window, and more
  EXPECT_TRUE(mode_is(SafetyStatus::MODE_MONITORING)) << "A stale fault code halted the arm";
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_ACTIVE);

  // The filter is not too wide: a fault stamped after the activation still halts.
  fake_->calls.clear();
  fake_->fault_code = 1.0;
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_DRIVER_FAULT);
  }));
  EXPECT_EQ(fake_->calls, HALT);
}

TEST_F(SafetySupervisorTest, HaltWithTheControllerManagerGoneEndsAtOnceAndSaysNotStopped)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  // The controller manager crashed (and the driver with it: no stop was sent).
  ASSERT_TRUE(fake_->controller_manager_gone(milliseconds{5000}));

  const auto start = std::chrono::steady_clock::now();
  fake_->call_stop();
  ASSERT_TRUE(spin_until([this]() {
      return stopped_because(SafetyStatus::REASON_STOP_SERVICE);
  }));

  // Every step failed at once ("not available"): no waiting for the 200 ms step timeouts.
  EXPECT_LT(std::chrono::steady_clock::now() - start, milliseconds{200});
  EXPECT_TRUE(fake_->calls.empty());  // nothing reached the (gone) controller manager
  // The honest report: the supervisor could not stop the arm.
  EXPECT_FALSE(fake_->statuses.back().hardware_stopped);
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SafetySupervisorTest, HeartbeatGoesOutWhileStoppedAndWhileMonitoring)
{
  // #136: from configure on, so a resume finds the driver already hearing it.
  ASSERT_EQ(supervisor_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(spin_until([this]() {
      return fake_->supervisor_heartbeats > 0;
  }));
  EXPECT_EQ(fake_->supervisor_heartbeat_state, Heartbeat::STATE_STOPPED);

  ASSERT_TRUE(spin_until([this]() {
      return try_activate();
  }));
  ASSERT_TRUE(spin_until([this]() {
      return mode_is(SafetyStatus::MODE_MONITORING);
  }));
  const std::size_t before = fake_->supervisor_heartbeats;
  // 20 ms period: 500 ms holds about 25, so 5 leaves room for a slow runner.
  spin_for(milliseconds{500});
  EXPECT_GE(fake_->supervisor_heartbeats, before + 5);
  EXPECT_EQ(fake_->supervisor_heartbeat_state, Heartbeat::STATE_MOVING);
}

TEST_F(SafetySupervisorTest, ExitWhileMonitoringSendsHardwareInactiveFirst)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  // As rclcpp's signal thread does on Ctrl+C, while the executor (here the test) still spins.
  std::atomic<bool> returned{false};
  std::thread hook{[this, &returned]() {
      supervisor_->halt_before_exit();
      returned = true;
    }};
  const bool done = spin_until([&returned]() {
        return returned.load();
  });
  hook.join();

  ASSERT_TRUE(done);
  ASSERT_FALSE(fake_->calls.empty());
  EXPECT_EQ(fake_->calls.front(), "hardware:inactive");
}

TEST_F(SafetySupervisorTest, ExitWhileStoppedSendsNothing)
{
  ASSERT_EQ(supervisor_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  spin_for(milliseconds{100});

  // Returns at once: nothing to wait for, so no thread and no spinning needed.
  supervisor_->halt_before_exit();
  spin_for(milliseconds{100});
  EXPECT_TRUE(fake_->calls.empty());
}

TEST_F(SafetySupervisorTest, ExitReturnsAfterTheStepTimeoutWhenNobodyAnswers)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());
  fake_->answer_hardware = false;

  std::atomic<bool> returned{false};
  const auto start = std::chrono::steady_clock::now();
  std::thread hook{[this, &returned]() {
      supervisor_->halt_before_exit();
      returned = true;
    }};
  const bool done = spin_until([&returned]() {
        return returned.load();
  });
  const auto waited = std::chrono::steady_clock::now() - start;
  hook.join();

  ASSERT_TRUE(done);
  // halt_step_timeout_ms is 200 in this fixture. The hook never returns before it: the request
  // was not answered. 500 ms over it is the margin for a loaded runner, not a requirement.
  EXPECT_GE(waited, milliseconds{200});
  EXPECT_LT(waited, milliseconds{700});
  ASSERT_FALSE(fake_->calls.empty());
  EXPECT_EQ(fake_->calls.front(), "hardware:inactive");
}

TEST_F(SafetySupervisorTest, ShutDownSupervisorStopsItsHeartbeat)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  // From ACTIVE straight to FINALIZED: it halts the arm, then must go silent for the driver,
  // even though the process (here the executor) keeps spinning it.
  EXPECT_EQ(supervisor_->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
  spin_for(milliseconds{100});  // heartbeats already sent arrive
  const std::size_t after_shutdown = fake_->supervisor_heartbeats;
  spin_for(milliseconds{300});  // 15 periods of 20 ms
  EXPECT_EQ(fake_->supervisor_heartbeats, after_shutdown);
  ASSERT_FALSE(fake_->calls.empty());
  EXPECT_EQ(fake_->calls.front(), "hardware:inactive");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
