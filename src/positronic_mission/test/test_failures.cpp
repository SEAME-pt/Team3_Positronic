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

#include <chrono>

#include "mission_fixture.hpp"

using positronic_mission::GoToPose;
using positronic_mission::MotionOutcome;
using positronic_mission::State;
using rclcpp_action::ResultCode;
namespace lc = lifecycle_msgs::msg;
using positronic_mission::test::MissionNodeTest;
using positronic_mission::test::WrappedResult;

// Measures how long `fn` takes, in milliseconds
template<typename Fn>
static int64_t elapsed_ms(Fn && fn)
{
  const auto start = std::chrono::steady_clock::now();
  fn();
  return std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start)
         .count();
}

// ======================= Motion failures =======================

TEST_F(MissionNodeTest, PlanningFailedGivesRejectedAndIdle)
{
  // Arrange: the "arm" can't find a plan
  mock_->outcome = MotionOutcome::PLANNING_FAILED;

  // Act
  auto handle = send_goal(valid_pose());
  ASSERT_NE(handle, nullptr);
  auto result = wait_result(handle);
  ASSERT_TRUE(result.has_value());

  // Assert: aborted with REJECTED, arm never moved so back to IDLE
  EXPECT_EQ(result->code, ResultCode::ABORTED);
  EXPECT_EQ(result->result->code, GoToPose::Result::REJECTED);
  EXPECT_FALSE(result->result->success);
  EXPECT_TRUE(wait_for_state(State::IDLE));

  // No reset needed: the next goal is accepted straight away
  mock_->outcome = MotionOutcome::OK;
  auto next = send_goal(valid_pose());
  ASSERT_NE(next, nullptr);
  auto next_result = wait_result(next);
  ASSERT_TRUE(next_result.has_value());
  EXPECT_EQ(next_result->result->code, GoToPose::Result::OK);
}

TEST_F(MissionNodeTest, ExecutionFailedGivesErrorAndNeedsReset)
{
  // Arrange: the "arm" fails while moving
  mock_->outcome = MotionOutcome::EXECUTION_FAILED;

  // Act
  auto handle = send_goal(valid_pose());
  ASSERT_NE(handle, nullptr);
  auto result = wait_result(handle);
  ASSERT_TRUE(result.has_value());

  // Assert: aborted with ERROR, node goes to ERROR
  EXPECT_EQ(result->code, ResultCode::ABORTED);
  EXPECT_EQ(result->result->code, GoToPose::Result::ERROR);
  ASSERT_TRUE(wait_for_state(State::ERROR));

  // Goals are rejected until someone resets
  EXPECT_EQ(send_goal(valid_pose()), nullptr);
  EXPECT_EQ(mock_->calls, 1);

  auto reset = call_reset();
  ASSERT_NE(reset, nullptr);
  EXPECT_TRUE(reset->accepted);
  ASSERT_TRUE(wait_for_state(State::IDLE));

  mock_->outcome = MotionOutcome::OK;
  auto next = send_goal(valid_pose());
  ASSERT_NE(next, nullptr);
  auto next_result = wait_result(next);
  ASSERT_TRUE(next_result.has_value());
  EXPECT_EQ(next_result->result->code, GoToPose::Result::OK);
}

// ======================= E-stop race =======================

// A stop that lands right after the goal is accepted, possibly before the
// motion even started, must still stop it. This is the reset_stop() fix.
TEST_F(MissionNodeTest, EStopRightAfterAcceptNeverRunsFullMotion)
{
  mock_->duration_ms = 2000;

  auto handle = send_goal(valid_pose());
  ASSERT_NE(handle, nullptr);

  // No wait for MOVING: stop as early as possible
  std::optional<WrappedResult> result;
  const auto ms = elapsed_ms([&] {
        ASSERT_NE(call_stop(), nullptr);
        result = wait_result(handle);
  });

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->code, ResultCode::ABORTED);
  EXPECT_EQ(result->result->code, GoToPose::Result::STOPPED);
  EXPECT_LT(ms, 1000) << "The motion ran instead of being stopped";
  EXPECT_TRUE(wait_for_state(State::STOPPED));
}

// ======================= Interruptions =======================

TEST_F(MissionNodeTest, DeactivateMidGoalAbortsQuickly)
{
  // Arrange: a long motion, mid-way
  mock_->duration_ms = 5000;
  auto handle = send_goal(valid_pose());
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_for_state(State::MOVING));

  // Act: deactivate must not wait for the 5 s motion
  const auto ms = elapsed_ms([&] {
        ASSERT_EQ(node_->deactivate().id(), lc::State::PRIMARY_STATE_INACTIVE);
  });
  EXPECT_LT(ms, 1000) << "Deactivate waited for the motion to finish";
  EXPECT_GE(mock_->stops, 1);

  // The goal was aborted
  auto result = wait_result(handle);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->code, ResultCode::ABORTED);
  EXPECT_EQ(result->result->code, GoToPose::Result::STOPPED);

  // Deactivated from MOVING -> INACTIVE, so activate gives IDLE
  last_state_ = 255;
  ASSERT_EQ(node_->activate().id(), lc::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(wait_for_state(State::IDLE));
}

TEST_F(MissionNodeTest, ShutdownMidGoalExitsQuickly)
{
  mock_->duration_ms = 5000;
  auto handle = send_goal(valid_pose());
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_for_state(State::MOVING));

  const auto ms = elapsed_ms([&] {
        EXPECT_EQ(node_->shutdown().id(), lc::State::PRIMARY_STATE_FINALIZED);
  });
  EXPECT_LT(ms, 1000) << "Shutdown waited for the motion to finish";
  EXPECT_GE(mock_->stops, 1) << "Shutdown didn't stop the backend";
}

TEST_F(MissionNodeTest, DestroyMidGoalExitsQuickly)
{
  mock_->duration_ms = 5000;
  auto handle = send_goal(valid_pose());
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_for_state(State::MOVING));

  // Stop spinning first, so nothing runs while the node is destroyed
  executor_->cancel();
  spin_thread_.join();

  // Act: destroy the node (runs ~MissionNode)
  const auto ms = elapsed_ms([&] {
        node_.reset();
  });

  EXPECT_LT(ms, 1000) << "Destructor waited for the motion to finish";
  EXPECT_GE(mock_->stops, 1) << "Destructor didn't stop the backend";
}
