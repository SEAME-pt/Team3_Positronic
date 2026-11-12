#ifndef POSITRONIC_MISSION__MOVEIT_BACKEND_HPP_
#define POSITRONIC_MISSION__MOVEIT_BACKEND_HPP_

#include <atomic>
#include <chrono>
#include <thread>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "positronic_mission/motion_backend.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

namespace positronic_mission
{
// Real backend: connected to moveit interface
class MoveitBackend: public MotionBackend
{
public:

    // Blocking. Called from the execute thread.
    MotionOutcome plan_and_execute(const geometry_msgs::msg::PoseStamped & pose)
    {
            /*
             * What MoveIt takes and gives back

             The simplest route is MoveGroupInterface:

             Your backend	                 MoveIt call	                    Returns
             plan_and_execute(pose)	    setPoseTarget(pose), then plan(plan)	MoveItErrorCode
                                        execute(plan) (blocking)               	MoveItErrorCode
             stop()	                    stop() (safe from another thread)	    nothing

             Then map the error codes to your MotionOutcome:
             plan() not SUCCESS → PLANNING_FAILED
             execute() gives PREEMPTED → PREEMPTED
             execute() gives anything else that isn’t SUCCESS → EXECUTION_FAILED
             */

            // To follow your stop contract
            // check the stop flag before planning
            // between planning and executing
            // and before returning.

            if (stop_requested_) {
                return MotionOutcome::PREEMPTED;
            }
            return MotionOutcome::OK;
    }

    // Must be safe to call from any thread. Makes a running
    // plan_and_execute return PREEMPTED.
    void stop()
    {
        stop_requested_ = true;
    }

    // Clears a pending stop. Called by the node only when a new goal is accepted.
    void reset_stop()
    {
        stop_requested_ = false;
    }

private:
    std::atomic<bool> stop_requested_{false};

};
}  // namespace positronic_mission

#endif  // POSITRONIC_MISSION__MOVEIT
