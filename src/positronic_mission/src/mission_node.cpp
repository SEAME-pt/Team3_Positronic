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
#include <rclcpp/logging.hpp>
#include "positronic_interfaces/action/go_to_pose.hpp"
#include "positronic_mission/motion_backend.hpp"

using namespace std::chrono_literals;

positronic_mission::MissionNode::MissionNode(
    const rclcpp::NodeOptions & options)
    : LifecycleNode("mission_node", options)
{
    RCLCPP_INFO(get_logger(), "IN constructor");
}

positronic_mission::MissionNode::~MissionNode()
{
    state_ = State::INACTIVE;
    if (backend_) {
        backend_->stop();
    }
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
    // Set the arm to STOPPED, unless it is already in ERROR
    while (state_now != State::ERROR && !state_.compare_exchange_weak(state_now, State::STOPPED)) {
    }
    // Abort backend if it was executing
    if (backend_) {
        backend_->stop();
    }

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

    // Fake backend
    // Only uses Mock Backend for now. Need to update it to <MoveitBackend>();
    // Or a not so clean idea. Keep both
    // std::shared_ptr<MotionBackend> get_backend(const std::string & type) {
    //      if type == "real" { return std::make_shared<MoveitBackend>() };
    //      if type == "mock" { return std::make_shared<MockBackend>() };
    //      return nullptr;
    // }
    if (!backend_) {
        backend_ = std::make_shared<MockBackend>();
    }

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
    if (backend_) {
        backend_->stop();
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
    if (backend_) {
        backend_->stop();
    }
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

rclcpp_action::GoalResponse positronic_mission::MissionNode::handle_goal(
    const rclcpp_action::GoalUUID & /* uuid */,
    const std::shared_ptr<const GoToPose::Goal> & goal)
{
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(get_logger(), "Rejected, node is not activated");
        return rclcpp_action::GoalResponse::REJECT;
    }
    std::string reason = validate_pose(goal->target_pose);
    if (!reason.empty()) {
        RCLCPP_WARN(get_logger(), "Rejected, %s" ,reason.c_str());
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
    backend_->reset_stop();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse positronic_mission::MissionNode::handle_cancel(
    const std::shared_ptr<GoalHandle> & /* goal_handle */)
{
    // Accepting all cancelations for now
    RCLCPP_INFO(get_logger(), "Cancel requested");
    if (backend_) {
        backend_->stop();
    }
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

    // Read the received goal from the client
    auto goal = goal_handle->get_goal();

    // Create empty feedback and result, to be populated by me
    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();
    feedback->stage = "executing";
    goal_handle->publish_feedback(feedback);

    // Blocking call made to Moveit
    const MotionOutcome outcome = backend_->plan_and_execute(goal->target_pose);

    if (!rclcpp::ok()) {
        // Protection for the lower checks. If mission_node is shutdown
        backend_->stop();
        return ;
    }

    State expected = State::MOVING;
    // Client cancel
    if (outcome == MotionOutcome::PREEMPTED && goal_handle->is_canceling()) {
        // If state is different than MOVING, don't touch it.
        state_.compare_exchange_strong(expected, State::STOPPED);
        // Cancel anyways because arm was or STOPPED or ERROR
        result->code = GoToPose::Result::CANCELED;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return ;
    }
    // Someone else changed the State (emergency stop, deactivate mission_node)
    if (state_ != State::MOVING || outcome == MotionOutcome::PREEMPTED) {
        state_.compare_exchange_strong(expected, State::STOPPED);
        result->code = (expected ==
            State::ERROR) ? GoToPose::Result::ERROR : GoToPose::Result::STOPPED;
        goal_handle->abort(result);
        RCLCPP_INFO(get_logger(), "Force stop activated");
        return ;
    }
    // Planning failed
    if (outcome == MotionOutcome::PLANNING_FAILED) {
        state_.compare_exchange_strong(expected, State::IDLE);
        result->code = GoToPose::Result::REJECTED;
        goal_handle->abort(result);
        RCLCPP_INFO(get_logger(), "Moveit planning failed");
        return ;
    }
    // Execution failed
    if (outcome == MotionOutcome::EXECUTION_FAILED) {
        state_.compare_exchange_strong(expected, State::ERROR);
        result->code = GoToPose::Result::ERROR;
        goal_handle->abort(result);
        RCLCPP_INFO(get_logger(), "Execution failed");
        return ;
    }
    // Success
    if (state_.compare_exchange_strong(expected, State::IDLE)) {
        result->success = true;
        result->code = GoToPose::Result::OK;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal achieved");
    // Just a backup check. Shouldn't be reachable
    } else {
        result->code = GoToPose::Result::STOPPED;
        goal_handle->abort(result);
        RCLCPP_INFO(get_logger(), "State changed after execution");
    }
}

/* This is the topic to report errors. Future idea?
 *
 * auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                .reliable()
                .transient_local();   // late joiners receive the last 10 messages
 *
 */

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(positronic_mission::MissionNode)
