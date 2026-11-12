#include <cmath>
#include "positronic_mission/pose_validation.hpp"

namespace positronic_mission
{
    static bool is_finite_pose(const geometry_msgs::msg::PoseStamped & p)
    {
        const auto & pos = p.pose.position;
        const auto & qt = p.pose.orientation;

        return (std::isfinite(pos.x) && std::isfinite(pos.y)
            && std::isfinite(pos.z) && std::isfinite(qt.x)
            && std::isfinite(qt.y) && std::isfinite(qt.z)
            && std::isfinite(qt.w));
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
}   // namespace positronic_mission
