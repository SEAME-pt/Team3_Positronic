#include "positronic_mission/moveit_backend.hpp"
#include <memory>
#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node_options.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "positronic_mission/motion_backend.hpp"

namespace positronic_mission
{
using moveit::core::MoveItErrorCode;

MoveitBackend::MoveitBackend(const std::string & group_name) : group_name_(group_name)
{
    node_ = std::make_shared<rclcpp::Node>(
        "positronic_mission_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    executor_->add_node(node_);
    spin_thread_ = std::thread([this](){
        executor_->spin();
    });
}

MoveitBackend::~MoveitBackend()
{
    stop();
    executor_->cancel();
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
}

MotionOutcome MoveitBackend::plan_and_execute(const geometry_msgs::msg::PoseStamped & pose)
{
    // Blocking. Called from the execute thread.
    if (stop_requested_) {
        return MotionOutcome::PREEMPTED;
    }

    if (!move_group_) {
        return MotionOutcome::PLANNING_FAILED;
    }

    // Set the pose
    if (!move_group_->setPoseTarget(pose)) {
        return MotionOutcome::PLANNING_FAILED;
    }

    // Create an empty plan, plan into it
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_->plan(plan) != MoveItErrorCode::SUCCESS) {
        return MotionOutcome::PLANNING_FAILED;
    }

    // Stop happened while planning
    if (stop_requested_) {
        return MotionOutcome::PREEMPTED;
    }

    // Blocking call
    auto result = move_group_->execute(plan);
    if (stop_requested_ || result == MoveItErrorCode::PREEMPTED) {
        return MotionOutcome::PREEMPTED;
    }
    if (result != MoveItErrorCode::SUCCESS) {
        return MotionOutcome::EXECUTION_FAILED;
    }

    return MotionOutcome::OK;
}

void MoveitBackend::stop()
{
    stop_requested_ = true;
    if (move_group_) {
        move_group_->stop();
    }
}

// Clears a pending stop. Called by the node only when a new goal is accepted.
void MoveitBackend::reset_stop()
{
    stop_requested_ = false;
}

bool MoveitBackend::ready()
{
    if (move_group_) {
        return true;
    }
    try {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, group_name_, nullptr, rclcpp::Duration::from_seconds(2.0));
        return true;
    }
    catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "Moveit not ready for group '%s': %s",
            group_name_.c_str(), e.what());
        return false;
    }
}
}  // namespace positronic_mission
