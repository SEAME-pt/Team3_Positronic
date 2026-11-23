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

#include "positronic_mission/moveit_backend.hpp"
#include <chrono>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <ratio>
#include <rclcpp/duration.hpp>
#include <rclcpp/node_options.hpp>
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "positronic_mission/motion_backend.hpp"

namespace positronic_mission
{
using moveit::core::MoveItErrorCode;

MoveitBackend::MoveitBackend(std::string group_name)
: group_name_(std::move(group_name))
{
  node_ = std::make_shared<rclcpp::Node>(
    "positronic_mission_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {
        executor_->spin();
  });
  while (!executor_->is_spinning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

MoveitBackend::~MoveitBackend()
{
  MoveitBackend::stop();
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

MotionOutcome MoveitBackend::plan_and_execute_cartesian(
    const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  if (stop_requested_) {
    return MotionOutcome::PREEMPTED;
  }

  if (!move_group_) {
    return MotionOutcome::PLANNING_FAILED;
  }

  /*
    double computeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints, double eef_step,
        moveit_msgs::msg::RobotTrajectory& trajectory, bool avoid_collisions = true,
        moveit_msgs::msg::MoveItErrorCodes* error_code = nullptr);
  */

  moveit_msgs::msg::MoveItErrorCodes error_code;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double eef_step = 0.01;
  bool avoid_collisions = true;

  if (move_group_->computeCartesianPath(waypoints, eef_step, trajectory, avoid_collisions, &error_code) < 0.999f) {
    return MotionOutcome::PLANNING_FAILED;
  }
  if (error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return (MotionOutcome::PLANNING_FAILED);
  }

  if (stop_requested_) {
    return MotionOutcome::PREEMPTED;
  }

  auto result = move_group_->execute(trajectory);
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
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->get_logger(), "Moveit not ready for group '%s': %s", group_name_.c_str(), e.what());
    return false;
  }
}
}  // namespace positronic_mission
