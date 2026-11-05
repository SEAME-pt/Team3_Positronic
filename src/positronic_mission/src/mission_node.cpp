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
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPose::Goal> goal) {
            return handle_goal(uuid, goal);
        },
        [this](std::shared_ptr<GoalHandle> goal_handle) {
            return handle_cancel(goal_handle);
        },
        [this](std::shared_ptr<GoalHandle> goal_handle) {
            handle_accepted(std::move(goal_handle));
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
    LifecycleNode::on_activate(state);

    timer_->reset();

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "IN on_deactivate");

    timer_->cancel();
    LifecycleNode::on_deactivate(state);

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

    // Will need a new variable for it later. And to update it
    msg.state = Heartbeat::STATE_IDLE;
    msg.node_name = get_name();

    publisher_->publish(msg);
}

rclcpp_action::GoalResponse positronic_mission::MissionNode::handle_goal(const rclcpp_action::GoalUUID & /* uuid */, std::shared_ptr<const GoToPose::Goal> /* goal */)
{
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        // Reject
        RCLCPP_INFO(get_logger(), "Rejected, node is not activated");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse positronic_mission::MissionNode::handle_cancel(std::shared_ptr<GoalHandle> /* goal_handle */)
{
    // Accepting all cancelations for now
    RCLCPP_INFO(get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void positronic_mission::MissionNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
{
    // Needs to execute it in a thread to not block the node
    auto exec_function = [this, goal_handle](){
        return execute(goal_handle);
    };
    exec_thread_ = std::thread(exec_function);
}

void positronic_mission::MissionNode::execute(std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing");

    rclcpp::Rate hertz(1);

    // Read the received goal from the client
    auto goal = goal_handle->get_goal();

    // Create empty feedback and result, to be populated by me
    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();

    const int fake_work = 10;
    for (int i = 0; i < fake_work && rclcpp::ok(); i++) {
        if (goal_handle->is_canceling()) {
            result->set__state(result->CANCELED);
            feedback->stage = "canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal canceled");
            return ;
        }
        // What would cause me to abort here?
        // If moveit declines the action, if would be aborted before the handle_goal no?
        // result->ERROR here ?
        // Maybe I need an abort flag

        feedback->stage = "executing";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "Working, iteration: %d", i);
        hertz.sleep();
    }
    if (rclcpp::ok()) {
        result->set__state(result->OK);
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
