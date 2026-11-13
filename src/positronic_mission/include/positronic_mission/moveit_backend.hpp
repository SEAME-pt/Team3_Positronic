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

#ifndef POSITRONIC_MISSION__MOVEIT_BACKEND_HPP_
#define POSITRONIC_MISSION__MOVEIT_BACKEND_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "positronic_mission/motion_backend.hpp"
#include "rclcpp/rclcpp.hpp"

// Forwarding the class to the .cpp file
namespace moveit::planning_interface
{
class MoveGroupInterface;
}

namespace positronic_mission
{
// Real backend: connected to moveit interface
class MoveitBackend : public MotionBackend
{
public:
  explicit MoveitBackend(std::string group_name);
  ~MoveitBackend() override;
  MoveitBackend(const MoveitBackend &) = delete;
  MoveitBackend & operator=(const MoveitBackend &) = delete;
  MoveitBackend(MoveitBackend &&) = delete;
  MoveitBackend & operator=(MoveitBackend &&) = delete;

  // Blocking. Called from the execute thread.
  MotionOutcome plan_and_execute(const geometry_msgs::msg::PoseStamped & pose) override;

  // Must be safe to call from any thread. Makes a running
  // plan_and_execute return PREEMPTED.
  void stop() override;

  // Clears a pending stop. Called by the node only when a new goal is accepted.
  void reset_stop() override;

  // Creates the move_group_. If it's unavailable returns false
  // To be used in on_activate()
  bool ready() override;

private:
  std::string group_name_;
  std::thread spin_thread_;
  std::shared_ptr<rclcpp::Node> node_{nullptr};
  std::atomic<bool> stop_requested_{false};
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{nullptr};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_{nullptr};
};
}  // namespace positronic_mission

#endif  // POSITRONIC_MISSION__MOVEIT_BACKEND_HPP_
