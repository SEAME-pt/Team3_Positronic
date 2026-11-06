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

#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <thread>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <shared_mutex>
#include "positronic_interfaces/msg/heartbeat.hpp"
#include "positronic_interfaces/action/go_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace positronic_mission
{
using positronic_interfaces::msg::Heartbeat;
using positronic_interfaces::action::GoToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPose>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum class State : unsigned char {
    IDLE,
    INACTIVE,
    MOVING,
    STOPPED,
    ERROR
};

class MissionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    MissionNode();
    ~MissionNode() override;

    MissionNode(const MissionNode & rhs) = delete;
    MissionNode &operator=(const MissionNode & rhs) = delete;
    MissionNode(const MissionNode && rhs) = delete;
    MissionNode &operator=(const MissionNode && rhs) = delete;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /* state */) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /* state */) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /* state */) override;

private:
    void heart_beat();
    bool validate_pose(const std::shared_ptr<const GoToPose::Goal> & goal);
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const GoToPose::Goal> & goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> & goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> & goal_handle);
    void execute(const std::shared_ptr<GoalHandle> & goal_handle);

    /*
          Need:
              action server
              action client
              publisher 10Hz
              subscriber /joint_states
              timer to track 10Hz
              state
      */
    rclcpp_action::Server<GoToPose>::SharedPtr mission_server_;
    // rclcpp_action::Client<typename ActionT>::SharedPtr mission_client;
    rclcpp_lifecycle::LifecyclePublisher<Heartbeat>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    // state needs to be std::atomic or mutex protected
    // uint8_t state_;
    std::thread exec_thread_;
    std::atomic<State> state_{State::INACTIVE};

};
} //namespace positronic_mission

#endif  // POSITRONIC_MISSION__MISSION_NODE_HPP_"
