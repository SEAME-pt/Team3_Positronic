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

#ifndef POSITRONIC_MISSION__MISSION_NODE_HPP_
#define POSITRONIC_MISSION__MISSION_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <thread>
#include <rclcpp/node_options.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <rclcpp/rate.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "positronic_interfaces/action/go_to_pose.hpp"
#include "positronic_interfaces/msg/heartbeat.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "positronic_interfaces/srv/state_trigger.hpp"
#include "positronic_mission/motion_backend.hpp"
#include "positronic_mission/pose_validation.hpp"

namespace positronic_mission
{
using positronic_interfaces::action::GoToPose;
using positronic_interfaces::msg::Heartbeat;
using positronic_interfaces::srv::StateTrigger;
using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPose>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum class State : unsigned char
{
  IDLE,
  INACTIVE,
  MOVING,
  STOPPED,
  ERROR
};

// Make sure that the enum State is the same value as the Heartbeat.msg
static_assert(static_cast<uint8_t>(State::IDLE) == Heartbeat::STATE_IDLE);
static_assert(static_cast<uint8_t>(State::INACTIVE) == Heartbeat::STATE_INACTIVE);
static_assert(static_cast<uint8_t>(State::MOVING) == Heartbeat::STATE_MOVING);
static_assert(static_cast<uint8_t>(State::STOPPED) == Heartbeat::STATE_STOPPED);
static_assert(static_cast<uint8_t>(State::ERROR) == Heartbeat::STATE_ERROR);

class MissionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MissionNode() override;

  MissionNode(const MissionNode & rhs) = delete;
  MissionNode & operator=(const MissionNode & rhs) = delete;
  MissionNode(MissionNode && rhs) = delete;
  MissionNode & operator=(MissionNode && rhs) = delete;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /* state */) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /* state */) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /* state */) override;

  // Backend interface (mock or moveit)
  void set_backend(std::shared_ptr<MotionBackend> backend)
  {
    backend_ = std::move(backend);
  }

private:
  void reset_ptrs();
  void heart_beat();
  // bool validate_pose(const std::shared_ptr<const GoToPose::Goal> & goal);
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const GoToPose::Goal> & goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> & goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> & goal_handle);
  void execute(const std::shared_ptr<GoalHandle> & goal_handle);
  void handle_reset(
    const std::shared_ptr<StateTrigger::Request> & request,
    const std::shared_ptr<StateTrigger::Response> & response);
  void handle_emergency_stop(
    const std::shared_ptr<StateTrigger::Request> & request,
    const std::shared_ptr<StateTrigger::Response> & response);

  // Mission Node action server
  rclcpp_action::Server<GoToPose>::SharedPtr mission_server_{nullptr};

  // Heartbeat of the Node
  rclcpp_lifecycle::LifecyclePublisher<Heartbeat>::SharedPtr publisher_{nullptr};

  // Server that resets the State to IDLE
  rclcpp::Service<StateTrigger>::SharedPtr reset_state_{nullptr};

  // Server that forces a STOPPED State. Doesn't override ERROR State
  rclcpp::Service<StateTrigger>::SharedPtr emergency_stop_{nullptr};

  std::thread exec_thread_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::atomic<State> state_{State::INACTIVE};

  // Backend interface (mock or moveit)
  std::shared_ptr<MotionBackend> backend_{nullptr};
};
}  // namespace positronic_mission

#endif  // POSITRONIC_MISSION__MISSION_NODE_HPP_
