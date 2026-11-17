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

#ifndef POSITRONIC_MISSION__MOTION_BACKEND_HPP_
#define POSITRONIC_MISSION__MOTION_BACKEND_HPP_

#include <atomic>
#include <chrono>
#include <thread>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace positronic_mission
{

enum class MotionOutcome : std::uint8_t
{
  OK,
  PLANNING_FAILED,
  EXECUTION_FAILED,
  PREEMPTED
};

// What the mission node needs from "something that moves the arm".
// The MoveIt version will implement the same three functions later.
class MotionBackend
{
public:
  MotionBackend() = default;
  virtual ~MotionBackend() = default;
  MotionBackend(const MotionBackend &) = delete;
  MotionBackend & operator=(const MotionBackend &) = delete;
  MotionBackend(MotionBackend &&) = delete;
  MotionBackend & operator=(MotionBackend &&) = delete;

  // Blocking. Called from the execute thread.
  virtual MotionOutcome plan_and_execute(const geometry_msgs::msg::PoseStamped & pose) = 0;

  // Must be safe to call from any thread. Makes a running
  // plan_and_execute return PREEMPTED.
  virtual void stop() = 0;

  // Clears a pending stop. Called by the node only when a new goal is accepted.
  virtual void reset_stop() = 0;

  // Check if the move_groups is accessible to validade the backend connection
  virtual bool ready() = 0;
};
}  // namespace positronic_mission

#endif  // POSITRONIC_MISSION__MOTION_BACKEND_HPP_
