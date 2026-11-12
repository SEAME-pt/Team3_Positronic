#ifndef POSITRONIC_MISSION__MOVEIT_BACKEND_HPP_
#define POSITRONIC_MISSION__MOVEIT_BACKEND_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "positronic_mission/motion_backend.hpp"

// Forwarding the class to the .cpp file
namespace moveit::planning_interface { class MoveGroupInterface; }

namespace positronic_mission
{
// Real backend: connected to moveit interface
class MoveitBackend: public MotionBackend
{
public:
    explicit MoveitBackend(const std::string & group_name);
    ~MoveitBackend() override;
    MoveitBackend(const MoveitBackend &) = delete;
    MoveitBackend &operator=(const MoveitBackend &) = delete;
    MoveitBackend(MoveitBackend &&) = delete;
    MoveitBackend &operator=(MoveitBackend &&) = delete;

    // Blocking. Called from the execute thread.
    MotionOutcome plan_and_execute(const geometry_msgs::msg::PoseStamped & pose) override;

    // Must be safe to call from any thread. Makes a running
    // plan_and_execute return PREEMPTED.
    void stop() override;

    // Clears a pending stop. Called by the node only when a new goal is accepted.
    void reset_stop() override;

    // Creates the move_group_. If it's unavailable returns false
    // To be used in on_activate()
    bool ready() override;

private:
    std::string group_name_;
    std::thread spin_thread_;
    std::shared_ptr<rclcpp::Node> node_{nullptr};
    std::atomic<bool> stop_requested_{false};
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{nullptr};
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_{nullptr};
};
}  // namespace positronic_mission

#endif  // POSITRONIC_MISSION__MOVEIT_BACKEND_HPP_
