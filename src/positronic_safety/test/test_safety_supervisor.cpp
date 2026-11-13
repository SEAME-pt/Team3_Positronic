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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
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

  /// What the supervisor asked for, in the order it arrived: "hardware:active",
  /// "hardware:inactive", "cancel", "controllers:on", "controllers:off".
  Calls calls;
  /// The controller lists of the last switch_controller requests.
  Calls activated;
  Calls deactivated;
  /// Every status the supervisor published, oldest first.
  std::vector<SafetyStatus> statuses;

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

private:
  void create_topics()
  {
    heartbeat_publisher_ = create_publisher<Heartbeat>(
      "mission/heartbeat", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    joint_states_publisher_ = create_publisher<DynamicJointState>(
      "dynamic_joint_states", rclcpp::SystemDefaultsQoS());
    // 100 Hz: well inside the supervisor's timeouts, so a halt in a test is never an accident.
    publish_timer_ = create_wall_timer(milliseconds{10}, [this]() {publish();});
    status_subscription_ = create_subscription<SafetyStatus>(
      "safety_supervisor/status",
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](const SafetyStatus & status) {
        statuses.push_back(status);
      });
  }

  void create_services()
  {
    // This callback form answers by hand (send_response), so a test can also not answer.
    using HardwareCallback = std::function<void (
          std::shared_ptr<rmw_request_id_t>,
          std::shared_ptr<SetHardwareComponentState::Request>)>;
    const HardwareCallback on_hardware =
      [this](
      const std::shared_ptr<rmw_request_id_t> & header,
      const std::shared_ptr<SetHardwareComponentState::Request> & request) {
        on_hardware_request(*header, *request);
      };
    hardware_service_ = create_service<SetHardwareComponentState>(
      "controller_manager/set_hardware_component_state", on_hardware);

    using SwitchCallback = std::function<void (
          std::shared_ptr<SwitchController::Request>,
          std::shared_ptr<SwitchController::Response>)>;
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
      joint_states_publisher_->publish(message);
    }
  }

  rclcpp::Publisher<Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<DynamicJointState>::SharedPtr joint_states_publisher_;
  rclcpp::Subscription<SafetyStatus>::SharedPtr status_subscription_;
  rclcpp::Service<SetHardwareComponentState>::SharedPtr hardware_service_;
  rclcpp::Service<SwitchController>::SharedPtr switch_service_;
  rclcpp::Client<Trigger>::SharedPtr stop_client_;
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
    const std::string ns = "/supervisor_test_" +
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
    spin_until([]() {return false;}, duration);
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
    ASSERT_TRUE(spin_until([this]() {return try_activate();}));
    ASSERT_TRUE(spin_until([this]() {return mode_is(SafetyStatus::MODE_MONITORING);}));
    ASSERT_TRUE(spin_until([this]() {return fake_->ready();}));
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
  ASSERT_TRUE(spin_until([this]() {return fake_->goal_running();}));

  fake_->call_stop();
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_STOP_SERVICE);}));

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
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_HEARTBEAT_LOST);}));

  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SafetySupervisorTest, DriverFaultHalts)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  fake_->fault_code = 1.0;  // LINK_TIMEOUT in positronic_hardware
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_DRIVER_FAULT);}));

  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_DOUBLE_EQ(fake_->statuses.back().fault_code, 1.0);
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SafetySupervisorTest, MissingFaultDataHalts)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  fake_->publish_fault_data = false;  // the broadcaster was deactivated
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_FAULT_DATA_LOST);}));

  EXPECT_EQ(fake_->calls, HALT);
}

TEST_F(SafetySupervisorTest, FaultCodeThatIsNotANumberHalts)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  // Bad data: messages keep coming, but none of them says whether there is a fault.
  fake_->fault_code = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_FAULT_DATA_LOST);}));

  EXPECT_EQ(fake_->calls, HALT);
}

TEST_F(SafetySupervisorTest, UnconfiguredAfterAFailedDeactivateCountsAsStopped)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  // A link fault: the driver's stop cannot leave the port, on_deactivate returns ERROR and the
  // hardware component ends UNCONFIGURED.
  fake_->deactivation_succeeds = false;
  fake_->fault_code = 1.0;
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_DRIVER_FAULT);}));

  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_TRUE(fake_->statuses.back().hardware_stopped);
}

TEST_F(SafetySupervisorTest, HardwareStillActiveIsNotReportedAsStopped)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  fake_->deactivation_succeeds = false;
  fake_->state_after_failed_deactivation = State::PRIMARY_STATE_ACTIVE;
  fake_->call_stop();
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_STOP_SERVICE);}));

  EXPECT_EQ(fake_->calls, HALT);  // the controllers were still deactivated
  EXPECT_FALSE(fake_->statuses.back().hardware_stopped);
}

TEST_F(SafetySupervisorTest, HaltGoesOnWhenTheControllerManagerDoesNotAnswer)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  fake_->answer_hardware = false;
  const auto start = std::chrono::steady_clock::now();
  fake_->call_stop();
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_STOP_SERVICE);}));

  // It waited for the answer (halt_step_timeout_ms is 200 here), then sent the rest anyway.
  EXPECT_GE(std::chrono::steady_clock::now() - start, milliseconds{200});
  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_FALSE(fake_->statuses.back().hardware_stopped);
}

TEST_F(SafetySupervisorTest, FailedResumeBecomesAHalt)
{
  fake_->activation_succeeds = false;  // the driver refuses to activate
  ASSERT_EQ(supervisor_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(spin_until([this]() {return try_activate();}));
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_RESUME_FAILED);}));

  // The controllers were never activated, and the full halt ran.
  const Calls expected{"hardware:active", "hardware:inactive", "controllers:off"};
  EXPECT_EQ(fake_->calls, expected);
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SafetySupervisorTest, StaysStoppedUntilAPersonActivatesAgain)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());
  fake_->call_stop();
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_STOP_SERVICE);}));
  fake_->calls.clear();

  // Heartbeat and fault code are fine, and still nothing comes back by itself.
  spin_for(milliseconds{500});
  EXPECT_TRUE(fake_->calls.empty());
  EXPECT_TRUE(stopped_because(SafetyStatus::REASON_STOP_SERVICE));
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_INACTIVE);

  ASSERT_TRUE(spin_until([this]() {return try_activate();}));
  ASSERT_TRUE(spin_until([this]() {return mode_is(SafetyStatus::MODE_MONITORING);}));
  EXPECT_EQ(fake_->calls, RESUME);
}

TEST_F(SafetySupervisorTest, ResumesAfterADriverFaultOnceTheDriverClearedIt)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());
  fake_->fault_code = 1.0;
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_DRIVER_FAULT);}));

  // The fake clears the code when it is activated, like the driver. Messages that still carry
  // the old code may be on their way: they must not halt the arm again.
  ASSERT_TRUE(spin_until([this]() {return try_activate();}));
  ASSERT_TRUE(spin_until([this]() {return mode_is(SafetyStatus::MODE_MONITORING);}));
  spin_for(milliseconds{500});
  EXPECT_TRUE(mode_is(SafetyStatus::MODE_MONITORING));
  EXPECT_EQ(supervisor_state(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(SafetySupervisorTest, DeactivatingTheSupervisorHalts)
{
  ASSERT_NO_FATAL_FAILURE(start_monitoring());

  EXPECT_EQ(supervisor_->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(spin_until([this]() {return stopped_because(SafetyStatus::REASON_DEACTIVATED);}));

  EXPECT_EQ(fake_->calls, HALT);
  EXPECT_TRUE(fake_->statuses.back().hardware_stopped);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
