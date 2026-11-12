#ifndef POSITRONIC_MISSION__TEST__MOCK_BACKEND_HPP_
#define POSITRONIC_MISSION__TEST__MOCK_BACKEND_HPP_

#include <atomic>
#include <chrono>
#include <thread>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "positronic_mission/motion_backend.hpp"

namespace positronic_mission
{

// AI Generated
// Fake arm for tests: waits `duration`, then returns `outcome`.
class MockBackend : public MotionBackend
{
public:
    // Knobs a test can change between goals
    std::atomic<int> duration_ms{1000};
    std::atomic<MotionOutcome> outcome{MotionOutcome::OK};

    // Counters a test can check
    std::atomic<int> calls{0};
    std::atomic<int> stops{0};

    MotionOutcome plan_and_execute(const geometry_msgs::msg::PoseStamped &) override
    {
        ++calls;
        if (stop_requested_) {
            return MotionOutcome::PREEMPTED;
        }
        const auto end = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(duration_ms.load());
        while (std::chrono::steady_clock::now() < end) {
            if (stop_requested_) {
                return MotionOutcome::PREEMPTED;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return outcome.load();
    }

    void stop() override
    {
        ++stops;
        stop_requested_ = true;
    }

    void reset_stop() override
    {
        stop_requested_ = false;
    }

private:
    std::atomic<bool> stop_requested_{false};
};

}  // namespace positronic_mission

#endif  // POSITRONIC_MISSION___MOCK_BACKEND_HPP_
