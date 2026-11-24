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

#ifndef POSITRONIC_MISSION__POSE_VALIDATION_HPP_
#define POSITRONIC_MISSION__POSE_VALIDATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "positronic_interfaces/action/go_to_pose.hpp"

namespace positronic_mission
{
inline constexpr const char * BASE_FRAME = "g_base";
using positronic_interfaces::action::GoToPose;

// Empty string if valid, otherwise returns the reason
std::string validate_input(const std::shared_ptr<const GoToPose::Goal> & goal);
std::string validate_pose(const geometry_msgs::msg::PoseStamped & pose);
std::string validate_waypoints(const std::vector<geometry_msgs::msg::Pose> & waypoints);

}  // namespace positronic_mission

#endif  // POSITRONIC_MISSION__POSE_VALIDATION_HPP_
