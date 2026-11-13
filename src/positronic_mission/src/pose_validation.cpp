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

#include "positronic_mission/pose_validation.hpp"
#include <cmath>

namespace positronic_mission
{
static bool is_finite_pose(const geometry_msgs::msg::PoseStamped & p)
{
  const auto & pos = p.pose.position;
  const auto & qt = p.pose.orientation;

  return std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z) &&
         std::isfinite(qt.x) && std::isfinite(qt.y) && std::isfinite(qt.z) && std::isfinite(qt.w);
}

std::string validate_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  // We need to create a list of accepted frames
  // For now, only base_link is accepted
  if (pose.header.frame_id != "g_base") {
    return "invalid frame_id";
  }

  if (!is_finite_pose(pose)) {
    return "non numeric pose value";
  }

  const auto & qt = pose.pose.orientation;
  const auto norm = std::sqrt(qt.x * qt.x + qt.y * qt.y + qt.z * qt.z + qt.w * qt.w);
  if (std::abs(norm - 1.0) > 1e-3) {
    return "invalid quaternion";
  }

  return "";
}
}  // namespace positronic_mission
