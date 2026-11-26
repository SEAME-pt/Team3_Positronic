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

#include "mission_fixture.hpp"
#include "positronic_interfaces/action/go_to_pose.hpp"
#include "positronic_mission/pose_validation.hpp"

using geometry_msgs::msg::PoseStamped;
using positronic_mission::GoToPose;

// ======================= Pose Check =======================

namespace
{
geometry_msgs::msg::PoseStamped valid_pose()
{
  PoseStamped p;
  p.header.frame_id = positronic_mission::BASE_FRAME;
  p.pose.position.x = 0.15;
  p.pose.position.z = 0.20;
  p.pose.orientation.w = 1.0;
  return p;
}

std::vector<geometry_msgs::msg::Pose> valid_waypoints()
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
}  // namespace

TEST(PoseValidation, FrameValidation)
{
  auto p = valid_pose();
  auto q = valid_pose();
  auto r = valid_pose();

  p.header.frame_id = "invalid";
  q.header.frame_id = "";

  EXPECT_EQ(positronic_mission::validate_pose(p), "invalid frame_id");
  EXPECT_EQ(positronic_mission::validate_pose(q), "invalid frame_id");
  EXPECT_EQ(positronic_mission::validate_pose(r), "");

  auto goal = std::make_shared<GoToPose::Goal>();
  goal->mode = GoToPose::Goal::MODE_POSE;
  goal->target_pose.header.frame_id = "";
  EXPECT_EQ(positronic_mission::validate_input(goal), "invalid frame_id");
}

TEST(PoseValidation, QuaternionValidation)
{
  auto p = valid_pose();
  auto q = valid_pose();
  auto r = valid_pose();
  auto s = valid_pose();
  auto t = valid_pose();

  p.pose.orientation.w = 0.0;
  q.pose.orientation.x = 0.7071;
  q.pose.orientation.w = 0.7071;
  r.pose.orientation.w = 1.002;
  s.pose.orientation.w = -1.0;
  t.pose.orientation.w = 1.0005;

  EXPECT_EQ(positronic_mission::validate_pose(p), "invalid quaternion");
  EXPECT_EQ(positronic_mission::validate_pose(q), "");
  EXPECT_EQ(positronic_mission::validate_pose(r), "invalid quaternion");
  EXPECT_EQ(positronic_mission::validate_pose(s), "");
  EXPECT_EQ(positronic_mission::validate_pose(t), "");
}

TEST(PoseValidation, NonFiniteValidation)
{
  auto p = valid_pose();
  auto q = valid_pose();
  auto r = valid_pose();

  p.pose.orientation.x = NAN;
  q.pose.position.x = NAN;
  r.pose.position.z = INFINITY;

  EXPECT_EQ(positronic_mission::validate_pose(p), "non numeric pose value");
  EXPECT_EQ(positronic_mission::validate_pose(q), "non numeric pose value");
  EXPECT_EQ(positronic_mission::validate_pose(r), "non numeric pose value");
}

TEST(PoseValidation, InvalidGoal)
{
  auto goal = std::make_shared<GoToPose::Goal>();
  goal->mode = 99;

  EXPECT_EQ(positronic_mission::validate_input(goal), "invalid mode");

  goal->mode = GoToPose::Goal::MODE_CARTESIAN;
  goal->cartesian_path = std::vector<geometry_msgs::msg::Pose>();

  EXPECT_EQ(positronic_mission::validate_input(goal), "empty waypoints");
}

TEST(PoseValidation, WaypointValidation)
{
  const int i = 1;
  auto p = valid_waypoints();
  auto q = valid_waypoints();

  q[i].orientation.w = 0;

  EXPECT_EQ(positronic_mission::validate_waypoints(p), "");
  EXPECT_EQ(
    positronic_mission::validate_waypoints(q),
    "invalid quaternion on waypoint: " + std::to_string(i));
}

TEST(PoseValidation, NonFiniteValues)
{
  int i = 1;
  auto p = valid_waypoints();
  auto q = valid_waypoints();
  auto r = valid_waypoints();

  p[i].orientation.x = NAN;
  q[i].position.x = INFINITY;
  r[i].position.z = NAN;

  EXPECT_EQ(
    positronic_mission::validate_waypoints(p),
    "non numeric pose value on waypoint: " + std::to_string(i));
  EXPECT_EQ(
    positronic_mission::validate_waypoints(q),
    "non numeric pose value on waypoint: " + std::to_string(i));
  EXPECT_EQ(
    positronic_mission::validate_waypoints(r),
    "non numeric pose value on waypoint: " + std::to_string(i));
}

TEST(PoseValidation, EmptyInput)
{
  const std::vector<geometry_msgs::msg::Pose> p;

  EXPECT_EQ(positronic_mission::validate_waypoints(p), "empty waypoints");
}
