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

// AI-generated (Claude, Anthropic).
// Prompt:
//   Test the real MoveitBackend without MoveIt running (#46, test review M1): a stop before
//   planning gives PREEMPTED, reset_stop clears it, a backend never connected to MoveIt gives
//   PLANNING_FAILED, and ready() is false, within a bounded time, when MoveIt is not there.

// The node tests use MockBackend, which has its own copy of the stop check. These tests run
// the real backend's checks, the ones that protect the real arm. Planning and executing need a
// running move_group: that is sim or HIL evidence, not a unit test.

#include <gtest/gtest.h>
#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "positronic_mission/motion_backend.hpp"
#include "rclcpp/rclcpp.hpp"

#include "positronic_mission/moveit_backend.hpp"
#include "positronic_mission/pose_validation.hpp"

namespace
{
using positronic_mission::MotionOutcome;
using positronic_mission::MoveitBackend;

geometry_msgs::msg::PoseStamped any_pose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = positronic_mission::BASE_FRAME;
  pose.pose.position.x = 0.15;
  pose.pose.position.z = 0.20;
  pose.pose.orientation.w = 1.0;
  return pose;
}

std::vector<geometry_msgs::msg::Pose> any_path()
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(2);

  geometry_msgs::msg::Pose pose_1;
  pose_1.position.x = 0.15;
  pose_1.position.z = 0.20;
  pose_1.orientation.w = 1.0;
  waypoints.push_back(pose_1);

  geometry_msgs::msg::Pose pose_2;
  pose_2.position.x = 0.30;
  pose_2.position.z = 0.205;
  pose_2.orientation.w = 1.0;
  waypoints.push_back(pose_2);

  return waypoints;
}

class MoveitBackendTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // MoveIt waits for the robot description (URDF and SRDF) on a topic before giving up; its
    // default wait is 10 s each. The backend's node takes parameters from the global arguments,
    // so short waits keep the ready() test fast. No robot description is ever published here.
    const char * argv[] = {"test", "--ros-args",
      "-p", "robot_description_timeout:=0.1",
      "-p", "robot_description_semantic_timeout:=0.1"};
    rclcpp::init(6, argv);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    backend_ = std::make_unique<MoveitBackend>("arm");
  }

  std::unique_ptr<MoveitBackend> backend_;
};
}  // namespace

// The stop that lands between "goal accepted" and "planning starts" must win.
TEST_F(MoveitBackendTest, StopBeforePlanningIsPreempted)
{
  backend_->stop();
  EXPECT_EQ(backend_->plan_and_execute(any_pose()), MotionOutcome::PREEMPTED);
}

// The stop stays until reset_stop(): a second call is preempted too.
TEST_F(MoveitBackendTest, StopStaysUntilReset)
{
  backend_->stop();
  EXPECT_EQ(backend_->plan_and_execute(any_pose()), MotionOutcome::PREEMPTED);
  EXPECT_EQ(backend_->plan_and_execute(any_pose()), MotionOutcome::PREEMPTED);
}

// After reset_stop() the stop no longer applies: the call goes on to the next check, which
// fails because the backend never connected to MoveIt.
TEST_F(MoveitBackendTest, ResetStopClearsTheStop)
{
  backend_->stop();
  backend_->reset_stop();
  EXPECT_EQ(backend_->plan_and_execute(any_pose()), MotionOutcome::PLANNING_FAILED);
}

// ready() was never called (or failed): there is no move_group, so nothing can be planned.
TEST_F(MoveitBackendTest, NotConnectedIsPlanningFailed)
{
  EXPECT_EQ(backend_->plan_and_execute(any_pose()), MotionOutcome::PLANNING_FAILED);
}

// stop() without a move_group must not crash (the node calls it on deactivate and shutdown).
TEST_F(MoveitBackendTest, StopWithoutMoveGroupIsSafe)
{
  EXPECT_NO_THROW(backend_->stop());
  EXPECT_NO_THROW(backend_->stop());
}

// No robot description: ready() returns false at its first check, so the mission node stays
// INACTIVE. It must return in bounded time, or activating the node would hang. The 5 s bound
// catches a regression where ready() starts waiting on MoveIt without the description.
// (The "move_group does not answer" path is tested in MoveitBackendReadyTest.)
TEST_F(MoveitBackendTest, ReadyIsFalseWithoutMoveIt)
{
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(backend_->ready());
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(5));

  // Still not connected afterwards: a failed ready() leaves no half-built move_group.
  EXPECT_EQ(backend_->plan_and_execute(any_pose()), MotionOutcome::PLANNING_FAILED);
}

// Destroying the backend stops its spin thread (the destructor must not hang).
TEST_F(MoveitBackendTest, DestructorReturns)
{
  const auto start = std::chrono::steady_clock::now();
  backend_.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(1));
}

// The stop stays until reset_stop(): a second call is preempted too.
TEST_F(MoveitBackendTest, StopStaysUntilResetCartesian)
{
  backend_->stop();
  EXPECT_EQ(backend_->plan_and_execute_cartesian(any_path()), MotionOutcome::PREEMPTED);
  EXPECT_EQ(backend_->plan_and_execute_cartesian(any_path()), MotionOutcome::PREEMPTED);
}

// After reset_stop() the stop no longer applies: the call goes on to the next check, which
// fails because the backend never connected to MoveIt.
TEST_F(MoveitBackendTest, ResetStopClearsTheStopCartesian)
{
  backend_->stop();
  backend_->reset_stop();
  EXPECT_EQ(backend_->plan_and_execute_cartesian(any_path()), MotionOutcome::PLANNING_FAILED);
}

// Own suite: ready() needs the robot description parameters to get past its first check, and
// global parameters are fixed at init. So this test runs its own init/shutdown, outside
// MoveitBackendTest's.
TEST(MoveitBackendReadyTest, ReadyIsFalseWhenMoveGroupDoesNotAnswer)
{
  // Dummy values are enough: the move_group wait fails before MoveGroupInterface is built,
  // so nothing ever parses them.
  const char * argv[] = {"test", "--ros-args",
    "-p", "robot_description:=dummy",
    "-p", "robot_description_semantic:=dummy"};
  rclcpp::init(static_cast<int>(std::size(argv)), argv);

  {
    MoveitBackend backend("arm");
    const auto start = std::chrono::steady_clock::now();

    EXPECT_FALSE(backend.ready());
    // Second call reuses the existing client and must fail the same way.
    EXPECT_FALSE(backend.ready());
    // Two 2 s waits, plus margin for a slow CI runner.
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(8));

    // A failed ready() leaves no half-built move_group.
    EXPECT_EQ(backend.plan_and_execute(any_pose()), MotionOutcome::PLANNING_FAILED);
  }  // Destroy the backend before shutting ROS down

  rclcpp::shutdown();
}
