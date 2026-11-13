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

using positronic_mission::GoToPose;
using geometry_msgs::msg::PoseStamped;

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
