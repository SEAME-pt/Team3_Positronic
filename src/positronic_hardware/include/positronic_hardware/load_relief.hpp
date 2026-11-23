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

#ifndef POSITRONIC_HARDWARE__LOAD_RELIEF_HPP_
#define POSITRONIC_HARDWARE__LOAD_RELIEF_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/joint_limit_check.hpp"
#include "positronic_hardware/tracking_check.hpp"
#include "positronic_protocol/decode.hpp"
#include "positronic_protocol/types.hpp"
#include "positronic_protocol/units.hpp"

namespace positronic_hardware
{

/// How far a still joint's position may wobble between frames, servo steps (the STS jitters one
/// step at rest). Further, the joint moved or the frame lied: the count starts over from there.
inline constexpr int LOAD_RELIEF_STILL_STEPS{2};

/// The limits of the check: `enabled` from the plugin's load_relief parameter, the rest fixed
/// (the tests shorten `hold`).
struct LoadReliefLimits
{
  bool enabled{true};
  /// How far from its target a still joint must stand. J1 on 20 Nov 2026: load 0 on target, 3
  /// to 13 at 0.5 deg short, about 30 more per degree.
  double residual_rad{0.5 * TRACKING_DEG_TO_RAD};
  /// The load it must report too, 0 to 1000 in 0.1 % of the drive, sign dropped.
  int load{20};
  /// How long both must hold. A joint settles within this on its way to a target.
  std::chrono::milliseconds hold{2000};
};

/// Decides when a still joint pushing short of its target is sent its own position, so its servo
/// stops pushing and heating. A joint is counted while the controller's command for it stays the
/// same and its frames show it still (speed 0, not moving, within LOAD_RELIEF_STILL_STEPS of
/// where the count started), at least `residual_rad` from that command, with at least `load`
/// either way. At `hold` it is relieved to where it stands, or refused outside the URDF limits,
/// and left alone until the controller moves it. Pure: no port, no clock, no thread.
class LoadReliefCheck
{
public:
  using Clock = std::chrono::steady_clock;
  /// The status frames found in one poll, J1 first; nullopt for a joint with no good frame
  /// (missing, or bridged from an older poll). A nullopt frame is skipped.
  using Statuses =
    std::array<std::optional<positronic_protocol::ServoStatus>, positronic_protocol::NUM_JOINTS>;

  /// `range` is the URDF joint limits: a relief outside them is refused, with the tolerance of
  /// every command (#34).
  LoadReliefCheck(LoadReliefLimits limits, const PositionLimits & range)
  : limits_{limits},
    range_{range}
  {
    reset();
  }

  /// One poll after a send: its `frames` and the time, against the command of the last wire(),
  /// the one that send carried. Returns the relief or the refusal to log, if any: one per call,
  /// the joints after it skip this poll as if their frames were missing. Not called while the
  /// arm is held; reset() then.
  [[nodiscard]] std::optional<ReliefEvent> update(const Statuses & frames, Clock::time_point now)
  {
    std::optional<ReliefEvent> event;
    if (!limits_.enabled) {
      return event;
    }
    // Stops at the event: a later joint's nullopt would also erase it.
    for (std::size_t joint{0}; joint < NUM_JOINTS && !event.has_value(); ++joint) {
      const std::optional<positronic_protocol::ServoStatus> & frame{frames.at(joint)};
      if (frame.has_value() && !joints_.at(joint).done) {
        event = watch(joint, *frame, now);
      }
    }
    return event;
  }

  /// What to send for the controller's `command`, kept as the target of the next update(): the
  /// command, with each relieved joint at the position it was relieved to. A joint whose command
  /// changes (the controller moved it, or a write landed during the relief's poll) starts over
  /// and goes out as commanded. Called on every command before it is sent.
  [[nodiscard]] JointPositions wire(const JointPositions & command)
  {
    JointPositions sent{command};
    for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
      JointState & state{joints_.at(joint)};
      if (command.at(joint) != command_.at(joint)) {
        state = JointState{};
      }
      sent.at(joint) = state.relief.value_or(command.at(joint));
    }
    command_ = command;
    return sent;
  }

  /// A stop, a cut or a power on: every joint starts over, nothing is relieved and the command is
  /// unknown until the next wire(). The event numbers go on.
  void reset()
  {
    joints_.fill(JointState{});
    command_.fill(std::numeric_limits<double>::quiet_NaN());
  }

private:
  struct JointState
  {
    /// Relieved or refused: nothing more until the controller moves the joint.
    bool done{false};
    /// The end of the hold; nullopt while not counting.
    std::optional<Clock::time_point> due;
    /// Where the count started, servo steps.
    std::uint16_t still_position{0};
    /// The position sent in place of the command.
    std::optional<double> relief;
  };

  [[nodiscard]] std::optional<ReliefEvent> watch(
    std::size_t joint, const positronic_protocol::ServoStatus & frame, Clock::time_point now)
  {
    namespace pp = positronic_protocol;
    JointState & state{joints_.at(joint)};
    const double position{pp::to_radians(pp::angle_from_steps(frame.position, joint))};
    const double residual{command_.at(joint) - position};
    const int load{std::abs(static_cast<int>(frame.load))};  // a joint pushes up or down
    // A non-finite residual (an unknown command) is never pushing.
    if (
      frame.speed != 0 || frame.moving || !std::isfinite(residual) ||
      std::abs(residual) < limits_.residual_rad || load < limits_.load)
    {
      state = JointState{};
      return std::nullopt;
    }
    const int moved{
      std::abs(static_cast<int>(frame.position) - static_cast<int>(state.still_position))};
    if (!state.due.has_value() || moved > LOAD_RELIEF_STILL_STEPS) {
      state.due = now + limits_.hold;
      state.still_position = frame.position;
      return std::nullopt;
    }
    if (now < state.due.value_or(now)) {
      return std::nullopt;
    }
    state.done = true;
    // Where it stands now, not where the count started: the Atom rounds a goal down, and one
    // step of creep toward the command would become three of push (J4, 21 Nov 2026).
    const PositionLimit & limit{range_.at(joint)};
    const bool inside{
      position >= limit.lower - LIMIT_TOLERANCE && position <= limit.upper + LIMIT_TOLERANCE};
    if (inside) {
      state.relief = position;
    }
    ++events_;
    return ReliefEvent{events_, joint, load, residual, frame.temperature, !inside};
  }

  LoadReliefLimits limits_;
  PositionLimits range_;
  std::array<JointState, NUM_JOINTS> joints_{};
  /// The controller's command of the last wire(); NaN until the first.
  JointPositions command_{};
  std::uint64_t events_{0};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__LOAD_RELIEF_HPP_
