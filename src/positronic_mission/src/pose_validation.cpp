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

#include <cmath>

#include "positronic_interfaces/action/go_to_pose.hpp"
#include "positronic_mission/pose_validation.hpp"

namespace positronic_mission
{

static std::string check_values(const geometry_msgs::msg::Pose & p)
{
  const auto & pos = p.position;
  const auto & qt = p.orientation;

  if (!(std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z) &&
    std::isfinite(qt.x) && std::isfinite(qt.y) && std::isfinite(qt.z) && std::isfinite(qt.w)))
  {
    return "non numeric pose value";
  }

  const auto norm = std::sqrt(qt.x * qt.x + qt.y * qt.y + qt.z * qt.z + qt.w * qt.w);
  if (std::abs(norm - 1.0) > 1e-3) {
    return "invalid quaternion";
  }
  return "";
}

std::string validate_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  // We need to create a list of accepted frames
  // Only accepting g_base for now. Which is the arm base
  if (pose.header.frame_id != BASE_FRAME) {
    return "invalid frame_id";
  }

  return check_values(pose.pose);
}

std::string validate_waypoints(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  if (waypoints.empty()) {
    return "empty waypoints";
  }

  for (size_t i = 0; i < waypoints.size(); i++) {
    auto value_check = check_values(waypoints[i]);
    if (!value_check.empty()) {
      return value_check + " on waypoint: " + std::to_string(i);
    }
  }
  return "";
}

std::string validate_input(const std::shared_ptr<const GoToPose::Goal> & goal)
{
  switch (goal->mode) {
    case (GoToPose::Goal::MODE_POSE):
      return validate_pose(goal->target_pose);
    case (GoToPose::Goal::MODE_CARTESIAN):
      return validate_waypoints(goal->cartesian_path);
    default:
      return "invalid mode";
  }
}
}  // namespace positronic_mission
