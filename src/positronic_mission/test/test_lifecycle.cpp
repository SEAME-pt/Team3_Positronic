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

#include <gtest/gtest.h>
#include <chrono>
#include <rclcpp_action/client.hpp>
#include "mission_fixture.hpp"
#include "positronic_interfaces/action/go_to_pose.hpp"
#include "positronic_mission/motion_backend.hpp"

using positronic_mission::GoToPose;
using positronic_mission::MotionOutcome;
using positronic_mission::State;
using rclcpp_action::ResultCode;

namespace positronic_mission::test
{

TEST_F(MissionNodeTest, LifecycleTransitions)
{
  mock_->duration_ms = 1000;

  // Make sure I can cycle the node through it's States and nothing breaks
  ASSERT_NO_FATAL_FAILURE(cycle(1));

  // Send a goal to check if the node is still corectly setup
  auto handle = send_goal(valid_pose());
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_for_state(State::MOVING));

  // Check if the previous goal succeeds
  auto result = wait_result(handle);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->code, ResultCode::SUCCEEDED);
  EXPECT_EQ(result->result->code, GoToPose::Result::OK);
  EXPECT_TRUE(wait_for_state(State::IDLE));

  // Just a sanity check, doing the cycle twice
  ASSERT_NO_FATAL_FAILURE(cycle(2));

  handle = send_goal(valid_pose());
  ASSERT_NE(handle, nullptr);
  ASSERT_TRUE(wait_for_state(State::MOVING));

  result = wait_result(handle);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->code, ResultCode::SUCCEEDED);
  EXPECT_EQ(result->result->code, GoToPose::Result::OK);
  EXPECT_TRUE(wait_for_state(State::IDLE));
}

TEST_F(MissionNodeTest, LifecycleIdle)
{
  // IDLE → deactivate → activate gives IDLE.
  // Make sure the node stats IDLE
  ASSERT_TRUE(wait_for_state(State::IDLE));

  // Deactivate -> Wait -> Activate
  ASSERT_NO_FATAL_FAILURE(deactivate_activate_cycle());

  EXPECT_TRUE(wait_for_state(State::IDLE));
}

TEST_F(MissionNodeTest, LifecycleStopped)
{
  // STOPPED (e-stop) → deactivate → activate is still STOPPED.
  auto stop = call_stop();
  ASSERT_NE(stop, nullptr);
  EXPECT_TRUE(stop->accepted);

  ASSERT_TRUE(wait_for_state(State::STOPPED));

  // Deactivate -> Wait -> Activate
  ASSERT_NO_FATAL_FAILURE(deactivate_activate_cycle());

  EXPECT_TRUE(wait_for_state(State::STOPPED));
}

TEST_F(MissionNodeTest, LifecycleError)
{
  // ERROR → deactivate → activate is still ERROR.

  // Force the mock to fail on execution
  mock_->outcome = MotionOutcome::EXECUTION_FAILED;
  auto handle = send_goal(valid_pose());
  ASSERT_NE(handle, nullptr);
  auto result = wait_result(handle);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->code, ResultCode::ABORTED);
  EXPECT_EQ(result->result->code, GoToPose::Result::ERROR);
  ASSERT_TRUE(wait_for_state(State::ERROR));

  // Deactivate -> Wait -> Activate
  ASSERT_NO_FATAL_FAILURE(deactivate_activate_cycle());

  EXPECT_TRUE(wait_for_state(State::ERROR));
}

TEST_F(MissionNodeTest, LifecycleStopWhileInactive)
{
  // An e-stop while inactive, followed by activate, gives STOPPED.

  // Start from inactive State
  ASSERT_EQ(node_->deactivate().id(), LifeState::PRIMARY_STATE_INACTIVE);
  std::this_thread::sleep_for(100ms);

  // Call Emergency Stop
  auto stop = call_stop();
  ASSERT_NE(stop, nullptr);
  EXPECT_TRUE(stop->accepted);

  // Activate and check if it's STOPPED
  last_state_ = 255;
  ASSERT_EQ(node_->activate().id(), LifeState::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(wait_for_state(State::STOPPED));
}

TEST_F(MissionNodeTest, LifecycleFullClean)
{
  // STOPPED → full cycle with cleanup gives IDLE (“reconfigure is a deliberate reset”).

  // Call Emergency Stop
  auto stop = call_stop();
  ASSERT_NE(stop, nullptr);
  EXPECT_TRUE(stop->accepted);
  ASSERT_TRUE(wait_for_state(State::STOPPED));

  ASSERT_NO_FATAL_FAILURE(cycle(1));
  // cycle() asserts IDLE at the end
}

TEST_F(MissionNodeTest, Heartbeat)
{
  heartbeat_count_ = 0;
  std::this_thread::sleep_for(1000ms);
  EXPECT_TRUE(node_name_ok_);
  EXPECT_TRUE(last_stamp_ok_);
  const int count = heartbeat_count_.load();
  EXPECT_GE(count, 20);
  EXPECT_LE(count, 30);

  // Deactivate and expect the message to stay the same
  ASSERT_EQ(node_->deactivate().id(), LifeState::PRIMARY_STATE_INACTIVE);
  std::this_thread::sleep_for(100ms);
  const int tmp_count = heartbeat_count_.load();
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(heartbeat_count_.load(), tmp_count);

  // Resumes and expect the message to increase
  ASSERT_EQ(node_->activate().id(), LifeState::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(wait_until([&] {
      return heartbeat_count_.load() > tmp_count;
  }));
}
}  // namespace positronic_mission::test
