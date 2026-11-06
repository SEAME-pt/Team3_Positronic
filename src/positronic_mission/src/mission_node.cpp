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

#include "positronic_mission/mission_node.hpp"
#include <cmath>
#include <memory>
#include <rclcpp/rate.hpp>
#include "positronic_interfaces/action/go_to_pose.hpp"

using namespace std::chrono_literals;

positronic_mission::MissionNode::MissionNode()
    : LifecycleNode("mission_node"),
    publisher_(nullptr),
    timer_(nullptr)
{
    RCLCPP_INFO(get_logger(), "IN constructor");
}

positronic_mission::MissionNode::~MissionNode()
{
    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_configure(const rclcpp_lifecycle::State & /* state */)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    RCLCPP_INFO(get_logger(), "IN on_configure");
    // Create everything here
    publisher_ = create_publisher<Heartbeat>("/mission/heartbeat", qos);

    mission_server_ = rclcpp_action::create_server<GoToPose>(
        this, "go_to_pose",
        [this](const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const GoToPose::Goal> & goal) {
            return handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandle> & goal_handle) {
            return handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandle> & goal_handle) {
            handle_accepted(goal_handle);
        });

    timer_ = create_wall_timer(100ms, [this]{heart_beat();});
    timer_->cancel();

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "IN on_cleanup");
    mission_server_ = nullptr;
    publisher_ = nullptr;
    timer_ = nullptr;
    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "IN on_activate");

    // This is a placeholder
    // I need to check if the arm is actually state
    // TODO (possible strategies):
    // can read from /joint_states
    // check move_group node
    // controller status from ros2_control
    state_ = State::IDLE;


    LifecycleNode::on_activate(state);

    timer_->reset();

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "IN on_deactivate");

    state_ = State::INACTIVE;
    timer_->cancel();
    LifecycleNode::on_deactivate(state);

    // Need a way to stop the loop here. Do I need it in shutdown as well?
    // Use std::atomic<bool>
    // Set it to true here
    // Call the thread .join() after

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "IN on_shutdown");
    mission_server_ = nullptr;
    publisher_ = nullptr;
    timer_ = nullptr;
    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }

    return CallbackReturn::SUCCESS;
}

void positronic_mission::MissionNode::heart_beat()
{
    auto msg = Heartbeat();
    msg.stamp = now();

    // Update msg state from the actual state
    msg.state = static_cast<uint8_t>(state_.load());
    msg.node_name = get_name();

    publisher_->publish(msg);
}

static bool is_finite_pose(const geometry_msgs::msg::PoseStamped & p)
{
    const auto pos = p.pose.position;
    const auto qt = p.pose.orientation;

    return (std::isfinite(pos.x) && std::isfinite(pos.y)
        && std::isfinite(pos.z) && std::isfinite(qt.x)
        && std::isfinite(qt.y) && std::isfinite(qt.z)
        && std::isfinite(qt.w));
}

bool positronic_mission::MissionNode::validate_pose(const std::shared_ptr<const GoToPose::Goal> & goal)
{
    // Can goal be null ?
    if (!goal) {
        RCLCPP_WARN(get_logger(), "no goal");
        return false;
    }

    // Only empty frames rejected for now
    // Maybe we need to create a list of accepted frames
    if (goal->target_pose.header.frame_id == "") {
        RCLCPP_WARN(get_logger(), "invalid frame");
        return false;
    }

    auto pose = goal->target_pose;
    const auto qt = pose.pose.orientation;
    const auto norm = std::sqrt(qt.x * qt.x + qt.y * qt.y + qt.z * qt.z + qt.w + qt.w);
    if (std::abs(norm - 1.0) > 1e-3) {
        RCLCPP_WARN(get_logger(), "invalid w");
        return false;
    }

    if (!is_finite_pose(pose)) {
        RCLCPP_WARN(get_logger(), "non numeric pose values");
        return false;
    }
    return true;
}

rclcpp_action::GoalResponse positronic_mission::MissionNode::handle_goal(const rclcpp_action::GoalUUID & /* uuid */, const std::shared_ptr<const GoToPose::Goal> & goal)
{
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        // Reject
        // RCLCPP_INFO(get_logger(), "Rejected, node is not activated");
        RCLCPP_WARN(get_logger(), "Rejected, node is not activated");
        return rclcpp_action::GoalResponse::REJECT;
    }
    // This is a weird one
    // compare_exchange_strong does
    // if equals to first param, change value to second param, return true
    // if different, set expected to old value, return false
    State expected = State::IDLE;
    if (!state_.compare_exchange_strong(expected, State::MOVING)) {
        RCLCPP_WARN(get_logger(), "Rejected, arm is not idle");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!validate_pose(goal)) {
        RCLCPP_WARN(get_logger(), "Rejected, invalid pose");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // My parsing goes here
    // I need:
    // Check if the robot arm is idle. If not: reject
    // Check if the Pose received makes sense. If not: reject
    // Else: accept and send to moveit
    // How will I receive the goala here?? Big question!

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse positronic_mission::MissionNode::handle_cancel(const std::shared_ptr<GoalHandle> & /* goal_handle */)
{
    // Accepting all cancelations for now
    RCLCPP_INFO(get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void positronic_mission::MissionNode::handle_accepted(const std::shared_ptr<GoalHandle> & goal_handle)
{
    // Needs to execute it in a thread to not block the node
    auto exec_function = [this, goal_handle](){ execute(goal_handle); };
    exec_thread_ = std::thread(exec_function);

    // exec_thread_ = std::thread([this, goal_handle]() { execute(goal_handle); });
}

void positronic_mission::MissionNode::execute(const std::shared_ptr<GoalHandle> & goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing");

    rclcpp::Rate hertz(1);

    // Read the received goal from the client
    auto goal = goal_handle->get_goal();

    // What do I do with the goal? Just publish it for moveit?

    // Create empty feedback and result, to be populated by me
    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();

    const int fake_work = 10;
    for (int i = 0; i < fake_work && rclcpp::ok(); i++) {
        if (goal_handle->is_canceling()) {
            result->code = GoToPose::Result::CANCELED;
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal canceled");
            return ;
        }
        // What would cause me to abort here?
        // If moveit declines the action, if would be aborted before the handle_goal no?
        // result->ERROR here ?
        // Maybe I need an abort flag

        // state_ = MOVING;
        feedback->stage = "executing";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "Working, iteration: %d", i);
        hertz.sleep();
    }
    if (rclcpp::ok()) {
        result->code = GoToPose::Result::OK;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal achieved");
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto mission_node = std::make_shared<positronic_mission::MissionNode>();
  rclcpp::spin(mission_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

/*
    Three places that use state:

    handle_goal reads it to decide whether to accept (idle) or reject
    the execution code and the cancel/stop code write it as the arm moves between states
    heartBeat() reads it to fill msg.state
*/
