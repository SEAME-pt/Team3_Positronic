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

#ifndef POSITRONIC_HARDWARE__FAKE_ARM_HPP_
#define POSITRONIC_HARDWARE__FAKE_ARM_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>

#include "positronic_hardware/arm_backend.hpp"

namespace positronic_hardware
{

/// How FakeArm moves (#122). INSTANT is at every target the moment it gets it. MEASURED moves
/// like the real arm measured in #33: it lags behind the target, the wrist starts late, and at
/// rest every joint sits a little off its command.
enum class FakeMode : std::uint8_t
{
  INSTANT,
  MEASURED
};

/// A pretend arm, so the driver runs without the robot. It never touches the serial port.
///
/// It starts in START_POSE. In INSTANT mode, the default, it is at every target as soon as
/// write_angles() gets it, with no motion in between. In MEASURED mode it imitates what the
/// firmware does with the targets, with the numbers of #33 (docs/03_verification/jtc_on_the_arm):
/// - Lag: each joint follows its target as a first-order lag with LAG_TIME_CONSTANT_S. On a
///   ramp it runs speed x LAG_TIME_CONSTANT_S behind, and the rest error below adds to that on
///   a joint moving up: J1 reads 2.2 + 0.5 = 2.7 deg behind at 10 deg/s and 4.4 + 0.5 = 4.9 at
///   20, the two numbers of #33. Moving down the rest error takes off instead.
/// - Wrist start-up: the joints of STARTS_LATE, standing still, start WRIST_START_DELAY after a
///   new target. A target that changes while they already move does not delay them again.
/// - Rest error: the pose it reports is REST_ERROR_DEG below where the joint really is, always.
///   So at rest every joint reads that far off its command, even one never commanded to move.
/// It only reads the time through `now`, so a test sets the time and never sleeps.
class FakeArm : public ArmBackend
{
public:
  using Clock = std::chrono::steady_clock;
  using Now = std::function<Clock::time_point()>;

  /// The pose the fake arm reports, in radians.
  static constexpr JointPositions START_POSE{0.0, 0.5, 0.3, 0.0, 0.0, 0.0};

  /// The servo temperatures the fake arm reports, in degC: a cool arm.
  static constexpr ServoTemperatures TEMPERATURES{30, 30, 30, 30, 30, 30};

  /// MEASURED: the time constant of the lag, in seconds, one value for every joint. #33 saw J1
  /// 2.7 deg behind at 10 deg/s and 4.9 at 20: a line of slope 0.22 s and 0.5 deg at 0 deg/s,
  /// which is J1's rest error.
  static constexpr double LAG_TIME_CONSTANT_S{0.22};

  /// MEASURED: how long a standing wrist servo takes to start after its target changes (#33,
  /// "a wrist servo takes about 0.2 s to start moving after the command changes").
  static constexpr std::chrono::milliseconds WRIST_START_DELAY{200};

  /// MEASURED: the joints that start WRIST_START_DELAY late, J4 and J5 in #33.
  static constexpr std::array<bool, NUM_JOINTS> STARTS_LATE{false, false, false, true, true, false};

  /// MEASURED: how far below the joint the reported pose is, in degrees, J1 first. In the 0.5 to
  /// 1.5 deg of #33, without the gripper: J1 0.5 (the lag line at 0 deg/s), J4 0.7 and J5 1.4
  /// (at rest, never commanded), the others the middle of the range.
  static constexpr std::array<double, NUM_JOINTS> REST_ERROR_DEG{0.5, 1.0, 1.0, 0.7, 1.4, 1.0};

  /// MEASURED: a joint this close to its target, in radians, is standing still (0.006 deg).
  static constexpr double SETTLED_RAD{1e-4};

  /// INSTANT, the arm of the existing tests and of backend:=fake.
  FakeArm() = default;

  /// `mode`, with the time read from `now`. An empty `now` reads the steady clock.
  FakeArm(FakeMode mode, Now now)
  : mode_{mode},
    now_{std::move(now)}
  {
    if (!now_) {
      now_ = &Clock::now;
    }
  }

  /// Where the joints are now, as the arm reports them.
  [[nodiscard]] JointPositions read_angles() const override
  {
    if (mode_ == FakeMode::INSTANT) {
      return angles_;
    }
    const Clock::time_point now{now_()};
    JointPositions angles{};
    for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
      angles.at(joint) = position(joint, now) - REST_ERROR_DEG.at(joint) * DEG_TO_RAD;
    }
    return angles;
  }

  /// Where the joints will be. In MEASURED mode a joint whose target is NaN or infinite keeps
  /// going to its previous one: the plugin refuses such a target before it gets here (#34), and
  /// a pose that turned NaN would never recover.
  void write_angles(const JointPositions & angles) override
  {
    target_ = angles;
    if (mode_ == FakeMode::INSTANT) {
      angles_ = angles;
      return;
    }
    const Clock::time_point now{now_()};
    for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
      const double target{angles.at(joint)};
      Motion & motion{motion_.at(joint)};
      if (!std::isfinite(target)) {
        continue;
      }
      if (now < motion.start) {
        // A wrist joint still waiting to start: it has not moved, only its target changes.
        motion.to = target;
        continue;
      }
      const double here{position(joint, now)};
      const bool standing{std::abs(motion.to - here) <= SETTLED_RAD};
      motion.from = here;
      motion.start = now;
      if (STARTS_LATE.at(joint) && standing && target != motion.to) {
        motion.start = now + WRIST_START_DELAY;
      }
      motion.to = target;
    }
  }

  void write_gripper(std::uint8_t value) override
  {
    gripper_ = value;
  }

  [[nodiscard]] std::optional<std::uint8_t> read_gripper() const override
  {
    return gripper_;
  }

  /// INSTANT has nothing to interrupt. MEASURED stops every joint where it is.
  [[nodiscard]] bool halt() override
  {
    request_halt();
    return true;
  }

  /// The same stop as halt().
  void request_halt() override
  {
    target_.reset();
    if (mode_ == FakeMode::INSTANT) {
      return;
    }
    const Clock::time_point now{now_()};
    for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
      const double here{position(joint, now)};
      motion_.at(joint) = Motion{here, here, now};
    }
  }

  /// The fake arm has no servos to power.
  [[nodiscard]] bool power_on() override
  {
    return true;
  }

  /// The fake pose is known from the start.
  [[nodiscard]] bool has_sample() const override
  {
    return true;
  }

  /// There is no serial line to go silent, so the fake link never times out.
  [[nodiscard]] bool link_timed_out() const override
  {
    return false;
  }

  /// The fake servos never warm up.
  [[nodiscard]] std::optional<ServoTemperatures> read_temperatures() const override
  {
    return TEMPERATURES;
  }

  /// So they never overheat either.
  [[nodiscard]] std::optional<Overheat> overheat() const override
  {
    return std::nullopt;
  }

  /// And their temperature is always known.
  [[nodiscard]] bool temperature_stale() const override
  {
    return false;
  }

  /// No serial line, so no send waits for a sync-write.
  [[nodiscard]] std::uint64_t echo_timeouts() const override
  {
    return 0;
  }

  /// The last write_angles(), gone with a halt, as the serial backend's last target.
  [[nodiscard]] std::optional<JointPositions> last_target() const override
  {
    return target_;
  }

private:
  static constexpr double DEG_TO_RAD{3.14159265358979323846 / 180.0};

  /// MEASURED, one joint: from `from` at `start` towards `to`.
  struct Motion
  {
    double from{0.0};
    double to{0.0};
    Clock::time_point start;
  };

  /// Where `joint` really is at `now`, before the rest error. Before `start` (a late wrist, or a
  /// clock that went back) it has not left `from`.
  [[nodiscard]] double position(std::size_t joint, Clock::time_point now) const
  {
    const Motion & motion{motion_.at(joint)};
    if (now <= motion.start) {
      return motion.from;
    }
    const double elapsed{std::chrono::duration<double>(now - motion.start).count()};
    return motion.to + (motion.from - motion.to) * std::exp(-elapsed / LAG_TIME_CONSTANT_S);
  }

  /// Every joint standing at START_POSE.
  static std::array<Motion, NUM_JOINTS> standing_at_start()
  {
    std::array<Motion, NUM_JOINTS> motion{};
    for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
      motion.at(joint) = Motion{START_POSE.at(joint), START_POSE.at(joint), Clock::time_point{}};
    }
    return motion;
  }

  FakeMode mode_{FakeMode::INSTANT};
  Now now_{&Clock::now};
  JointPositions angles_{START_POSE};
  std::array<Motion, NUM_JOINTS> motion_{standing_at_start()};
  std::optional<JointPositions> target_;
  std::uint8_t gripper_{GRIPPER_OPEN_VALUE};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__FAKE_ARM_HPP_
