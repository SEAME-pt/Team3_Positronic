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
    : LifecycleNode("mission_node")
{
    RCLCPP_INFO(get_logger(), "IN constructor");
}

positronic_mission::MissionNode::~MissionNode()
{
    state_ = State::INACTIVE;
    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }
}

void positronic_mission::MissionNode::handle_reset(
    const std::shared_ptr<StateTrigger::Request> & /* request */,
    std::shared_ptr<StateTrigger::Response> response)
{
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        response->accepted = false;
        response->message = "Refused: Arm is not activated";
        return;
    }
    State expected = State::STOPPED;
    if (state_.compare_exchange_strong(expected, State::IDLE)) {
        response->accepted = true;
        response->message = "Reset done";
        return;
    }
    expected = State::ERROR;
    if (state_.compare_exchange_strong(expected, State::IDLE)) {
        response->accepted = true;
        response->message = "Reset done";
        return;
    }
    response->accepted = false;
    response->message = "Refused: State is not ERROR or STOPPED";
}

void positronic_mission::MissionNode::handle_emergency_stop(
    const std::shared_ptr<StateTrigger::Request> & /* request */,
    std::shared_ptr<StateTrigger::Response> response)
{
    State state_now = state_.load();
    // TODO: If the arm is INACTIVE. I'm setting it to STOPPED.
    // Need to define the behavior. Choose between INACTIVE or STOPPED.
    while (state_now != State::ERROR && !state_.compare_exchange_weak(state_now, State::STOPPED)) {
    }
    // need to send a .stop() to moveit once I have it.

    response->accepted = true;
    response->message = (state_now ==
        State::ERROR) ? "Stopped. State kept as ERROR" : "Emergency Stop activated";
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_configure(
    const rclcpp_lifecycle::State & /* state */)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    RCLCPP_INFO(get_logger(), "IN on_configure");
    // Create everything here
    publisher_ = create_publisher<Heartbeat>("/mission/heartbeat", qos);
    reset_state_ = create_service<StateTrigger>("/mission/reset",
                    [this](const std::shared_ptr<StateTrigger::Request> & request,
                        std::shared_ptr<StateTrigger::Response> response) {
                        handle_reset(request, response);
                    });
    emergency_stop_ = create_service<StateTrigger>("/mission/stop",
                        [this](const std::shared_ptr<StateTrigger::Request> & request,
                            std::shared_ptr<StateTrigger::Response> response) {
                            handle_emergency_stop(request, response);
                        });

    mission_server_ = rclcpp_action::create_server<GoToPose>(
        this, "/mission/go_to_pose",
        [this](const rclcpp_action::GoalUUID & uuid,
            const std::shared_ptr<const GoToPose::Goal> & goal) {
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

void positronic_mission::MissionNode::reset_ptrs()
{
    mission_server_ = nullptr;
    publisher_ = nullptr;
    reset_state_ = nullptr;
    emergency_stop_ = nullptr;
    timer_ = nullptr;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_cleanup(
    const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "IN on_cleanup");
    state_ = State::INACTIVE;
    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }
    reset_ptrs();

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_activate(
    const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "IN on_activate");

    // State is a placeholder
    // I need to check if the arm is actually state
    // TODO (possible strategies):
    // can read from /joint_states
    // check move_group node
    // controller status from ros2_control

    // For now. Only put into IDLE if the arm was INACTIVE.
    // Does not override STOPPED and ERROR States
    State expected = State::INACTIVE;
    state_.compare_exchange_strong(expected, State::IDLE);
    LifecycleNode::on_activate(state);

    timer_->reset();

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_deactivate(
    const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "IN on_deactivate");

    // Get the current State
    // Check if it's not ERROR. Then check if it didn't change from the last check
    // If it changed, retry with new value. If it didn't, set state_ = STOPPED and exits loop
    // Using _weak because it can be faster on loops. Don't use it for single checks since it can spuriously fail!
    State state_now = state_.load();
    while ((state_now == State::IDLE || state_now == State::MOVING)
        && !state_.compare_exchange_weak(state_now, State::INACTIVE)) {
    }
    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }

    timer_->cancel();
    LifecycleNode::on_deactivate(state);

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_shutdown(
    const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "IN on_shutdown");

    state_ = State::INACTIVE;
    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }

    reset_ptrs();

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
    const auto & pos = p.pose.position;
    const auto & qt = p.pose.orientation;

    return (std::isfinite(pos.x) && std::isfinite(pos.y)
        && std::isfinite(pos.z) && std::isfinite(qt.x)
        && std::isfinite(qt.y) && std::isfinite(qt.z)
        && std::isfinite(qt.w));
}

bool positronic_mission::MissionNode::validate_pose(
    const std::shared_ptr<const GoToPose::Goal> & goal)
{
    // Can goal be null ?
    if (!goal) {
        RCLCPP_WARN(get_logger(), "no goal");
        return false;
    }

    // We need to create a list of accepted frames
    // For now, only base_link is accepted
    if (goal->target_pose.header.frame_id != "base_link") {
        RCLCPP_WARN(get_logger(), "invalid frame");
        return false;
    }

    const auto & pose = goal->target_pose;
    if (!is_finite_pose(pose)) {
        RCLCPP_WARN(get_logger(), "non numeric pose values");
        return false;
    }

    const auto & qt = pose.pose.orientation;
    const auto norm = std::sqrt(qt.x * qt.x + qt.y * qt.y + qt.z * qt.z + qt.w * qt.w);
    if (std::abs(norm - 1.0) > 1e-3) {
        RCLCPP_WARN(get_logger(), "invalid quaternion");
        return false;
    }

    return true;
}

rclcpp_action::GoalResponse positronic_mission::MissionNode::handle_goal(
    const rclcpp_action::GoalUUID & /* uuid */,
    const std::shared_ptr<const GoToPose::Goal> & goal)
{
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(get_logger(), "Rejected, node is not activated");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!validate_pose(goal)) {
        RCLCPP_WARN(get_logger(), "Rejected, invalid pose");
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

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse positronic_mission::MissionNode::handle_cancel(
    const std::shared_ptr<GoalHandle> & /* goal_handle */)
{
    // Accepting all cancelations for now
    RCLCPP_INFO(get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void positronic_mission::MissionNode::handle_accepted(
    const std::shared_ptr<GoalHandle> & goal_handle)
{
    // Join thread if a goal was finished. Before starting a second goal
    if (exec_thread_.joinable()) {
        exec_thread_.join();
    }
    // Needs to execute it in a thread to not block the node
    auto exec_function = [this, goal_handle](){ execute(goal_handle); };
    exec_thread_ = std::thread(exec_function);
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
    bool canceled_in_loop{false};
    bool stopped_in_loop{false};

    const int fake_work = 10;
    for (int i = 0; i < fake_work && rclcpp::ok(); i++) {
        if (goal_handle->is_canceling()) {
            canceled_in_loop = true;
            break;
        }
        if (state_ != State::MOVING) {
            stopped_in_loop = true;
            break ;
        }
        // What would cause me to abort here?
        // If moveit declines the action, if would be aborted before the handle_goal no?
        // result->ERROR;
        // state_ = State::ERROR;
        // Maybe I need an abort flag

        // const auto plan_code = move_group.plan(plan);
        // if (plan_code != SUCCESS)
        //
        // const auto exec_code = move_group.execute(plan);
        // if (exec_code != SUCCESS)

        feedback->stage = "executing";
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "Working, iteration: %d", i);
        hertz.sleep();
    }

    if (!rclcpp::ok()) {
        // Protection for the lower checks. If mission_node is shutdown
        // TODO: call backend .stop() before returning
        return ;
    }

    State expected = State::MOVING;
    if (canceled_in_loop) {
        // If state is different than MOVING, don't touch it.
        state_.compare_exchange_strong(expected, State::STOPPED);
        // Cancel anyways because arm was or STOPPED or ERROR
        result->code = GoToPose::Result::CANCELED;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
    }
    else if (stopped_in_loop) {
        result->code = GoToPose::Result::STOPPED;
        goal_handle->abort(result);
        RCLCPP_INFO(get_logger(), "Force stop activated");
    } else {
        if (state_.compare_exchange_strong(expected, State::IDLE)) {
            result->success = true;
            result->code = GoToPose::Result::OK;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal achieved");
        } else {
            result->code = (expected ==
                State::ERROR ) ? GoToPose::Result::ERROR : GoToPose::Result::STOPPED;
            goal_handle->abort(result);
            RCLCPP_INFO(get_logger(), "State changed after execution");
        }
    }
}

#ifndef TEST_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto mission_node = std::make_shared<positronic_mission::MissionNode>();
  rclcpp::spin(mission_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
#endif

/*
    Three places that use state:

    handle_goal reads it to decide whether to accept (idle) or reject
    the execution code and the cancel/stop code write it as the arm moves between states
    heartBeat() reads it to fill msg.state
*/


/* This is the topic to report errors. Future idea?
 *
 * auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                .reliable()
                .transient_local();   // late joiners receive the last 10 messages
 *
 */
