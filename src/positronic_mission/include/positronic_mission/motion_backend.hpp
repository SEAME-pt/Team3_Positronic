#ifndef POSITRONIC_MISSION__MOTION_BACKEND_HPP_
#define POSITRONIC_MISSION__MOTION_BACKEND_HPP_

#include <atomic>
#include <chrono>
#include <thread>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace positronic_mission
{

enum class MotionOutcome { OK, PLANNING_FAILED, EXECUTION_FAILED, PREEMPTED };

// What the mission node needs from "something that moves the arm".
// The MoveIt version will implement the same three functions later.
class MotionBackend
{
public:
    virtual ~MotionBackend() = default;

    // Blocking. Called from the execute thread.
    virtual MotionOutcome plan_and_execute(const geometry_msgs::msg::PoseStamped & pose) = 0;

    // Must be safe to call from any thread. Makes a running
    // plan_and_execute return PREEMPTED.
    virtual void stop() = 0;

    // Clears a pending stop. Called by the node only when a new goal is accepted.
    virtual void reset_stop() = 0;
};
}  // namespace positronic_mission

#endif  // POSITRONIC_MISSION__MOTION_BACKEND_HPP_
