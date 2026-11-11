#ifndef POSITRONIC_MISSION__POSE_VALIDATION_HPP_
#define POSITRONIC_MISSION__POSE_VALIDATION_HPP_

#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace positronic_mission
{
    // Empty string if valid, otherwise returns the reason
    std::string validate_pose(const geometry_msgs::msg::PoseStamped & pose);
}   // namespace positronic_mission

#endif // POSITRONIC_MISSION__POSE_VALIDATION_HPP_
