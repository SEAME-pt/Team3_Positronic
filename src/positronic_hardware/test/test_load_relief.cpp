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

// AI-generated (Claude, Anthropic).
// Prompt:
//   Write a GoogleTest suite for load_relief.hpp (#183), the pure check that sends a still joint
//   pushing short of its target its own position: no port, no thread, each turn is wire() of the
//   command then update() of the poll's frames at a time the test sets. Cover: relieved exactly
//   at the hold and not 1 ms before, with the values of the frame, and kept until the controller
//   changes that joint's command (not when its load falls, nor when another joint moves, nor when
//   the old command comes back); speed, the moving flag or a light load restart the count, and
//   so do a command that changes during the hold and a move of three steps (two are a wobble;
//   the relief goes where the joint stands at the end); the residual and the load exactly at
//   their limits and just under, either sign; a NaN or infinite command, and polls before the
//   first wire(); a missing frame skipped; disabled; J3, the inverted joint, refused outside its
//   URDF limit with the command tolerance, and counted again once moved; two reliefs due
//   together come one per update, and reset() clears both and keeps the numbers.
// Follow-up prompt (#183, the review):
//   J1 at its upper URDF limit, with the command tolerance, and refused past it, as J3 at its
//   lower one.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/joint_limit_check.hpp"
#include "positronic_hardware/load_relief.hpp"
#include "positronic_hardware/tracking_check.hpp"
#include "positronic_protocol/decode.hpp"
#include "positronic_protocol/types.hpp"
#include "positronic_protocol/units.hpp"

namespace
{
using positronic_hardware::JointPositions;
using positronic_hardware::LoadReliefCheck;
using positronic_hardware::LoadReliefLimits;
using positronic_hardware::PositionLimits;
using positronic_hardware::ReliefEvent;
using positronic_hardware::TRACKING_DEG_TO_RAD;
using positronic_protocol::NUM_JOINTS;
using positronic_protocol::ServoStatus;
using std::chrono::milliseconds;
using Clock = LoadReliefCheck::Clock;
using Frames = LoadReliefCheck::Statuses;

constexpr double ONE_DEG{TRACKING_DEG_TO_RAD};
constexpr JointPositions ZERO{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
/// The servo steps of 0 rad on every joint.
constexpr std::uint16_t CENTER{2048};
constexpr std::size_t J3{2};
constexpr milliseconds HOLD{100};
const Clock::time_point T0{Clock::time_point{} + milliseconds{1000}};

/// Six still servos at 0 rad with load 0, at 35 degC.
Frames at_rest()
{
  Frames frames{};
  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    ServoStatus frame{};
    frame.position = CENTER;
    frame.temperature = 35;
    frames.at(joint) = frame;
  }
  return frames;
}

/// `frames` with `joint` at `steps` reporting `load`.
Frames loaded(std::size_t joint, int load, std::uint16_t steps = CENTER, Frames frames = at_rest())
{
  ServoStatus frame{frames.at(joint).value_or(ServoStatus{})};
  frame.load = static_cast<std::int16_t>(load);
  frame.position = steps;
  frames.at(joint) = frame;
  return frames;
}

/// `frames` with `joint` turning at `speed` or flagged moving.
Frames turning(Frames frames, std::size_t joint, std::int16_t speed, bool moving)
{
  ServoStatus frame{frames.at(joint).value_or(ServoStatus{})};
  frame.speed = speed;
  frame.moving = moving;
  frames.at(joint) = frame;
  return frames;
}

/// ZERO with `joint` at `rad`.
JointPositions with(std::size_t joint, double rad)
{
  JointPositions positions{ZERO};
  positions.at(joint) = rad;
  return positions;
}

/// `steps` of `joint` in radians, through the Atom's own conversion.
double rad_of(std::uint16_t steps, std::size_t joint)
{
  return positronic_protocol::to_radians(positronic_protocol::angle_from_steps(steps, joint));
}

/// Short limits so the tests count milliseconds, not seconds.
LoadReliefLimits short_limits()
{
  LoadReliefLimits limits{};
  limits.hold = HOLD;
  return limits;
}

/// +-3 rad on every joint, wider than every relief here, J3's lower end at `j3_lower`.
PositionLimits range(double j3_lower = -3.0)
{
  PositionLimits limits{};
  for (positronic_hardware::PositionLimit & limit : limits) {
    limit.lower = -3.0;
    limit.upper = 3.0;
  }
  limits.at(J3).lower = j3_lower;
  return limits;
}

/// One turn of the serial thread: `command` through wire() to the arm, then the poll's `frames`.
std::optional<ReliefEvent> turn(
  LoadReliefCheck & check, const JointPositions & command, const Frames & frames,
  Clock::time_point at)
{
  static_cast<void>(check.wire(command));
  return check.update(frames, at);
}

/// The turn at `start`, which starts the count, and the one at `start` + HOLD, which ends it.
std::optional<ReliefEvent> after_hold(
  LoadReliefCheck & check, const JointPositions & command, const Frames & frames,
  Clock::time_point start = T0)
{
  static_cast<void>(turn(check, command, frames, start));
  return turn(check, command, frames, start + HOLD);
}
}  // namespace

TEST(LoadRelief, AStillLoadedJointIsRelievedAtTheHoldUntilItsCommandChanges)
{
  LoadReliefCheck check{short_limits(), range()};
  const JointPositions command{with(0, ONE_DEG)};  // J1 stands at 0, 1 deg short
  static_cast<void>(turn(check, command, loaded(0, 30), T0));
  EXPECT_FALSE(turn(check, command, loaded(0, 30), T0 + HOLD - milliseconds{1}).has_value());
  EXPECT_EQ(check.wire(command), command);
  const ReliefEvent event{turn(check, command, loaded(0, 30), T0 + HOLD).value_or(ReliefEvent{})};
  EXPECT_EQ(event.number, 1u);
  EXPECT_EQ(event.joint, 0u);
  EXPECT_EQ(event.load, 30);
  EXPECT_NEAR(event.residual_rad, ONE_DEG, 1e-12);
  EXPECT_EQ(event.celsius, 35);
  EXPECT_FALSE(event.refused);
  const JointPositions sent{check.wire(command)};
  EXPECT_DOUBLE_EQ(sent.at(0), 0.0);
  // Its load falls, or it sags 12 steps and pushes on (a weight): the relief stays, also while
  // the controller moves the other joints.
  EXPECT_FALSE(turn(check, command, loaded(0, 0), T0 + 2 * HOLD).has_value());
  EXPECT_FALSE(turn(check, command, loaded(0, 30, 2060), T0 + 10 * HOLD).has_value());
  EXPECT_EQ(check.wire(command), sent);
  const JointPositions others{ONE_DEG, 0.2, -0.3, 0.4, -0.5, 0.6};
  EXPECT_EQ(check.wire(others), (JointPositions{0.0, 0.2, -0.3, 0.4, -0.5, 0.6}));
  // A different command for J1 (moved, or written during the relief's poll) goes out as is,
  // and the relief is over, even when the old command comes back.
  const JointPositions moved{with(0, ONE_DEG + 1e-9)};
  EXPECT_EQ(check.wire(moved), moved);
  EXPECT_EQ(check.wire(command), command);
}

TEST(LoadRelief, SpeedTheMovingFlagOrALightLoadBreaksTheCount)
{
  const JointPositions command{with(0, ONE_DEG)};
  const Frames pushing{loaded(0, 30)};
  for (const Frames & broken :
    {turning(pushing, 0, 12, false), turning(pushing, 0, -12, false),
      turning(pushing, 0, 0, true), loaded(0, 19)})
  {
    LoadReliefCheck check{short_limits(), range()};
    static_cast<void>(turn(check, command, loaded(0, 30), T0));
    static_cast<void>(turn(check, command, broken, T0 + milliseconds{50}));
    // HOLD after the first frame, but the count started over at 60.
    static_cast<void>(turn(check, command, loaded(0, 30), T0 + milliseconds{60}));
    EXPECT_FALSE(turn(check, command, loaded(0, 30), T0 + HOLD).has_value());
    EXPECT_TRUE(turn(check, command, loaded(0, 30), T0 + milliseconds{60} + HOLD).has_value());
  }
}

TEST(LoadRelief, ACommandThatChangesDuringTheHoldRestartsTheCount)
{
  LoadReliefCheck check{short_limits(), range()};
  static_cast<void>(turn(check, with(0, ONE_DEG), loaded(0, 30), T0));
  // A slow trajectory: the command creeps on while the joint stays where it is.
  const JointPositions crept{with(0, 1.1 * ONE_DEG)};
  static_cast<void>(turn(check, crept, loaded(0, 30), T0 + milliseconds{50}));
  EXPECT_FALSE(turn(check, crept, loaded(0, 30), T0 + HOLD).has_value());
  EXPECT_TRUE(turn(check, crept, loaded(0, 30), T0 + milliseconds{50} + HOLD).has_value());
}

TEST(LoadRelief, TheResidualAndTheLoadCountExactlyAtTheirLimitsEitherWay)
{
  const LoadReliefLimits limits{short_limits()};
  // J4 holding the gripper reports a negative load: it counts, without its sign.
  for (const int load : {limits.load, -limits.load}) {
    LoadReliefCheck check{limits, range()};
    const std::optional<ReliefEvent> relief{after_hold(check, with(0, ONE_DEG), loaded(0, load))};
    EXPECT_EQ(relief.value_or(ReliefEvent{}).load, limits.load) << load;
  }
  for (const double residual : {limits.residual_rad, -limits.residual_rad}) {
    LoadReliefCheck check{limits, range()};
    EXPECT_TRUE(after_hold(check, with(0, residual), loaded(0, 30)).has_value()) << residual;
  }
  LoadReliefCheck light{limits, range()};
  EXPECT_FALSE(after_hold(light, with(0, ONE_DEG), loaded(0, limits.load - 1)).has_value());
  LoadReliefCheck close{limits, range()};
  EXPECT_FALSE(after_hold(close, with(0, limits.residual_rad - 1e-6), loaded(0, 30)).has_value());
}

TEST(LoadRelief, DisabledDecidesNothing)
{
  LoadReliefLimits limits{short_limits()};
  limits.enabled = false;
  LoadReliefCheck check{limits, range()};
  const JointPositions command{with(0, ONE_DEG)};
  EXPECT_FALSE(after_hold(check, command, loaded(0, 60)).has_value());
  EXPECT_EQ(check.wire(command), command);
}

TEST(LoadRelief, ANonFiniteCommandIsNeverPushing)
{
  for (const double bad :
    {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()})
  {
    LoadReliefCheck check{short_limits(), range()};
    const JointPositions command{with(0, bad)};
    EXPECT_FALSE(after_hold(check, command, loaded(0, 60)).has_value()) << bad;
    EXPECT_FALSE(std::isfinite(check.wire(command).at(0))) << bad;
  }
  // Before the first wire() the command is unknown too: J1 4.2 deg from 0 is not counted.
  LoadReliefCheck fresh{short_limits(), range()};
  static_cast<void>(fresh.update(loaded(0, 60, 2000), T0));
  EXPECT_FALSE(fresh.update(loaded(0, 60, 2000), T0 + HOLD).has_value());
  EXPECT_EQ(fresh.wire(ZERO), ZERO);
}

TEST(LoadRelief, TheReliefGoesToWhereTheJointStandsAtTheEndOfTheHold)
{
  // 2000 steps is 4.2 deg from the command 0.
  LoadReliefCheck moved{short_limits(), range()};
  static_cast<void>(turn(moved, ZERO, loaded(0, 30, 2000), T0));
  static_cast<void>(turn(moved, ZERO, loaded(0, 30, 1998), T0 + milliseconds{50}));
  // Three steps under where the count started: a move, or a corrupt frame. It starts over there.
  static_cast<void>(turn(moved, ZERO, loaded(0, 30, 1997), T0 + milliseconds{90}));
  EXPECT_FALSE(turn(moved, ZERO, loaded(0, 30, 1997), T0 + HOLD).has_value());
  static_cast<void>(turn(moved, ZERO, loaded(0, 30, 1997), T0 + milliseconds{90} + HOLD));
  EXPECT_DOUBLE_EQ(moved.wire(ZERO).at(0), rad_of(1997, 0));

  // Two steps is a wobble, not a move: the count goes on, and the relief goes where the joint
  // stands at its end.
  LoadReliefCheck wobbly{short_limits(), range()};
  static_cast<void>(turn(wobbly, ZERO, loaded(0, 30, 2000), T0));
  static_cast<void>(turn(wobbly, ZERO, loaded(0, 30, 2002), T0 + milliseconds{50}));
  static_cast<void>(turn(wobbly, ZERO, loaded(0, 30, 2001), T0 + HOLD));
  EXPECT_DOUBLE_EQ(wobbly.wire(ZERO).at(0), rad_of(2001, 0));
}

TEST(LoadRelief, AMissingFrameIsSkipped)
{
  LoadReliefCheck check{short_limits(), range()};
  const JointPositions command{with(0, ONE_DEG)};
  Frames missing{loaded(0, 30)};
  missing.at(0).reset();
  static_cast<void>(turn(check, command, loaded(0, 30), T0));
  // No frame of J1 at the end of the hold: no relief, and no restart; the next frame relieves.
  EXPECT_FALSE(turn(check, command, missing, T0 + HOLD).has_value());
  EXPECT_TRUE(turn(check, command, loaded(0, 30), T0 + HOLD + milliseconds{10}).has_value());
}

TEST(LoadRelief, AReliefOutsideTheUrdfLimitsIsRefusedWithTheCommandTolerance)
{
  // J3, the inverted joint, still at 2024 steps (-2.1 deg), its command 1 deg further down.
  constexpr std::uint16_t STEPS{2024};
  const double position{rad_of(STEPS, J3)};
  const JointPositions command{with(J3, position - ONE_DEG)};
  for (const double lower : {position, position + 1e-7}) {
    LoadReliefCheck check{short_limits(), range(lower)};
    const ReliefEvent relief{
      after_hold(check, command, loaded(J3, 30, STEPS)).value_or(ReliefEvent{})};
    EXPECT_FALSE(relief.refused) << lower;
    EXPECT_DOUBLE_EQ(check.wire(command).at(J3), position) << lower;
  }
  // J1 still at 2072 steps, its command 1 deg further up: the same tolerance at the upper limit.
  const double top{rad_of(2072, 0)};
  const JointPositions up{with(0, top + ONE_DEG)};
  for (const double upper : {top, top - 1e-7}) {
    PositionLimits limits{range()};
    limits.at(0).upper = upper;
    LoadReliefCheck check{short_limits(), limits};
    static_cast<void>(after_hold(check, up, loaded(0, 30, 2072)));
    EXPECT_DOUBLE_EQ(check.wire(up).at(0), top) << upper;
  }
  PositionLimits under{range()};
  under.at(0).upper = top - 1e-5;
  LoadReliefCheck past{short_limits(), under};
  EXPECT_TRUE(after_hold(past, up, loaded(0, 30, 2072)).value_or(ReliefEvent{}).refused);
  EXPECT_EQ(past.wire(up), up);
  LoadReliefCheck check{short_limits(), range(position + 1e-5)};
  const ReliefEvent refused{
    after_hold(check, command, loaded(J3, 30, STEPS)).value_or(ReliefEvent{})};
  EXPECT_TRUE(refused.refused);
  EXPECT_EQ(refused.joint, J3);
  EXPECT_NEAR(refused.residual_rad, -ONE_DEG, 1e-12);
  // Nothing sent, and nothing more decided until the controller moves J3.
  EXPECT_FALSE(turn(check, command, loaded(J3, 30, STEPS), T0 + 10 * HOLD).has_value());
  EXPECT_EQ(check.wire(command), command);
  const JointPositions moved{with(J3, position - 2.0 * ONE_DEG)};
  EXPECT_TRUE(after_hold(check, moved, loaded(J3, 30, STEPS), T0 + 20 * HOLD).has_value());
}

TEST(LoadRelief, TwoReliefsDueTogetherComeOnePerUpdateAndResetClearsBoth)
{
  LoadReliefCheck check{short_limits(), range()};
  JointPositions command{with(0, ONE_DEG)};
  command.at(4) = -ONE_DEG;
  const Frames both{loaded(4, 40, CENTER, loaded(0, 30))};
  const ReliefEvent first{after_hold(check, command, both).value_or(ReliefEvent{})};
  EXPECT_EQ(first.number, 1u);
  EXPECT_EQ(first.joint, 0u);
  EXPECT_DOUBLE_EQ(check.wire(command).at(4), -ONE_DEG);  // J5 skipped that poll
  const ReliefEvent second{
    turn(check, command, both, T0 + HOLD + milliseconds{10}).value_or(ReliefEvent{})};
  EXPECT_EQ(second.number, 2u);
  EXPECT_EQ(second.joint, 4u);
  ASSERT_EQ(check.wire(command), ZERO);
  check.reset();
  EXPECT_EQ(check.wire(command), command);
  // A whole hold again, and the numbers go on.
  EXPECT_EQ(after_hold(check, command, both, T0 + 20 * HOLD).value_or(ReliefEvent{}).number, 3u);
}
