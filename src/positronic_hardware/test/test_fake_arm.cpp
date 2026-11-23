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
//   Unit tests for the MEASURED mode of fake_arm.hpp (#122 part 1), with a clock the test
//   sets, never a sleep. INSTANT stays at every target at once. MEASURED: a step follows the
//   first-order lag (exactly 1 - 1/e of the step after one time constant), a ramp on J1 reads
//   2.7 deg behind at 10 deg/s and 4.9 at 20 (within 0.5 deg, #33), the wrist (J4, J5) starts
//   exactly 200 ms late and not again while it moves, every joint rests REST_ERROR_DEG off its
//   command, a halt freezes the pose and cancels a late start, NaN and inf targets are ignored,
//   a clock that stands still or goes back moves nothing.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/fake_arm.hpp"

namespace
{

using positronic_hardware::FakeArm;
using positronic_hardware::FakeMode;
using positronic_hardware::JointPositions;
using positronic_hardware::NUM_JOINTS;
using std::chrono::milliseconds;

constexpr double DEG{3.14159265358979323846 / 180.0};
constexpr JointPositions START{FakeArm::START_POSE};
constexpr std::size_t J1{0};
constexpr std::size_t J4{3};
constexpr std::size_t J5{4};

/// Doubles that went through exp() and a few sums.
constexpr double EXACT{1e-12};

/// What the arm reports for `joint` standing at `position`.
double reported(std::size_t joint, double position)
{
  return position - FakeArm::REST_ERROR_DEG.at(joint) * DEG;
}

/// START with `joint` moved by `rad`.
JointPositions start_with(std::size_t joint, double rad)
{
  JointPositions pose{START};
  pose.at(joint) += rad;
  return pose;
}

class MeasuredFakeArm : public ::testing::Test
{
protected:
  /// Any time will do: the arm only looks at differences.
  const FakeArm::Clock::time_point t0_{std::chrono::seconds{100}};
  FakeArm::Clock::time_point now_{t0_};
  FakeArm arm_{FakeMode::MEASURED, [this]() {
      return now_;
    }};

  void at_ms(int ms)
  {
    now_ = t0_ + milliseconds{ms};
  }

  double angle(std::size_t joint) const
  {
    return arm_.read_angles().at(joint);
  }

  /// Drives `joint` from START at `deg_per_s` (negative goes down) for two seconds, one target
  /// every 10 ms like the control loop, and returns the last command minus the reported pose,
  /// in degrees. Two seconds are nine time constants: the lag has settled.
  double ramp_error_deg(std::size_t joint, double deg_per_s)
  {
    JointPositions target{START};
    for (int step{1}; step <= 200; ++step) {
      at_ms(10 * step);
      target.at(joint) = START.at(joint) + deg_per_s * DEG * 0.01 * step;
      arm_.write_angles(target);
    }
    return (target.at(joint) - angle(joint)) / DEG;
  }
};

}  // namespace

TEST(InstantFakeArm, IsAtTheTargetAtOnce)
{
  FakeArm arm{};
  EXPECT_EQ(arm.read_angles(), START);
  EXPECT_FALSE(arm.last_target().has_value());

  const JointPositions target{start_with(J1, 0.25)};
  arm.write_angles(target);

  EXPECT_EQ(arm.read_angles(), target);
  EXPECT_EQ(arm.last_target(), target);
  EXPECT_TRUE(arm.halt());
  EXPECT_FALSE(arm.last_target().has_value());
  EXPECT_EQ(arm.read_angles(), target);
}

// Without a clock the arm reads the steady clock: it still works, it is only not testable.
TEST(MeasuredFakeArmClock, AnEmptyClockReadsTheSteadyClock)
{
  FakeArm arm{FakeMode::MEASURED, nullptr};
  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    EXPECT_NEAR(arm.read_angles().at(joint), reported(joint, START.at(joint)), EXACT) << joint;
  }
}

TEST_F(MeasuredFakeArm, StartsAtTheStartPoseOffByTheRestError)
{
  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    EXPECT_NEAR(angle(joint), reported(joint, START.at(joint)), EXACT) << joint;
  }
}

// The rest errors are the range #33 measured, without the gripper.
TEST_F(MeasuredFakeArm, EveryRestErrorIsInTheMeasuredRange)
{
  for (const double error : FakeArm::REST_ERROR_DEG) {
    EXPECT_GE(error, 0.5);
    EXPECT_LE(error, 1.5);
  }
}

// A first-order lag: after one time constant it has done 1 - 1/e of the step, after none
// nothing.
TEST_F(MeasuredFakeArm, AStepFollowsTheFirstOrderLag)
{
  const double step{0.1};
  arm_.write_angles(start_with(J1, step));
  EXPECT_NEAR(angle(J1), reported(J1, START.at(J1)), EXACT);
  EXPECT_EQ(arm_.last_target(), start_with(J1, step));

  now_ = t0_ + std::chrono::duration_cast<FakeArm::Clock::duration>(
                 std::chrono::duration<double>{FakeArm::LAG_TIME_CONSTANT_S});
  EXPECT_NEAR(angle(J1), reported(J1, START.at(J1) + step * (1.0 - std::exp(-1.0))), 1e-9);
}

// #33 on J1: 2.7 deg behind at 10 deg/s, 4.9 at 20, within 0.5 deg. The model gives 2.75 and
// 5.0: the 0.22 s lag, half a 10 ms control step, and the 0.5 deg rest error.
TEST_F(MeasuredFakeArm, ARampUpLagsAsTheArmAt10DegPerSecond)
{
  EXPECT_NEAR(ramp_error_deg(J1, 10.0), 2.7, 0.5);
}

TEST_F(MeasuredFakeArm, ARampUpLagsAsTheArmAt20DegPerSecond)
{
  EXPECT_NEAR(ramp_error_deg(J1, 20.0), 4.9, 0.5);
}

// Moving down, the pose reads below the joint, so the rest error takes off from the lag.
TEST_F(MeasuredFakeArm, ARampDownLagsByTheRestErrorLess)
{
  const double lag_deg{10.0 * FakeArm::LAG_TIME_CONSTANT_S};
  EXPECT_NEAR(-ramp_error_deg(J1, -10.0), lag_deg - FakeArm::REST_ERROR_DEG.at(J1), 0.1);
}

// #33: a standing wrist servo starts 0.2 s after its target changes; J1 starts at once.
TEST_F(MeasuredFakeArm, TheWristStartsExactly200MillisecondsLate)
{
  JointPositions target{START};
  target.at(J1) += 0.1;
  target.at(J4) += 0.1;
  target.at(J5) -= 0.1;
  arm_.write_angles(target);

  at_ms(200);
  EXPECT_GT(angle(J1), reported(J1, START.at(J1)));
  EXPECT_NEAR(angle(J4), reported(J4, START.at(J4)), EXACT);
  EXPECT_NEAR(angle(J5), reported(J5, START.at(J5)), EXACT);

  at_ms(201);
  EXPECT_GT(angle(J4), reported(J4, START.at(J4)));
  EXPECT_LT(angle(J5), reported(J5, START.at(J5)));
}

// The delay is the start from standing: a new target while moving does not stop the wrist.
TEST_F(MeasuredFakeArm, TheWristDoesNotWaitAgainWhileItMoves)
{
  arm_.write_angles(start_with(J4, 0.1));
  at_ms(300);
  const double moving{angle(J4)};
  arm_.write_angles(start_with(J4, 0.2));

  at_ms(310);
  EXPECT_GT(angle(J4), moving);
}

// Standing again, the next start waits again.
TEST_F(MeasuredFakeArm, TheWristWaitsAgainOnceItStands)
{
  arm_.write_angles(start_with(J4, 0.1));
  at_ms(5000);
  const double standing{angle(J4)};
  EXPECT_NEAR(standing, reported(J4, START.at(J4) + 0.1), 1e-6);
  arm_.write_angles(start_with(J4, 0.2));

  at_ms(5200);
  EXPECT_DOUBLE_EQ(angle(J4), standing);
  at_ms(5210);
  EXPECT_GT(angle(J4), standing);
}

// The controller holding a pose sends the same target every cycle: that is no start, and the
// delay counts from the cycle that changes the target.
TEST_F(MeasuredFakeArm, ResendingTheSameTargetIsNoStart)
{
  arm_.write_angles(START);
  at_ms(50);
  arm_.write_angles(start_with(J4, 0.1));

  at_ms(250);
  EXPECT_NEAR(angle(J4), reported(J4, START.at(J4)), EXACT);
  at_ms(251);
  EXPECT_GT(angle(J4), reported(J4, START.at(J4)));
}

// A target that changes while the wrist waits moves the goal, not the start.
TEST_F(MeasuredFakeArm, ATargetChangedWhileTheWristWaitsKeepsTheStart)
{
  arm_.write_angles(start_with(J4, 0.1));
  at_ms(100);
  arm_.write_angles(start_with(J4, -0.1));

  at_ms(200);
  EXPECT_NEAR(angle(J4), reported(J4, START.at(J4)), EXACT);
  at_ms(201);
  EXPECT_LT(angle(J4), reported(J4, START.at(J4)));
}

// #33: every joint settles off its command by its rest error.
TEST_F(MeasuredFakeArm, EveryJointRestsOffItsCommandByItsRestError)
{
  JointPositions target{START};
  for (double & angle : target) {
    angle += 0.2;
  }
  arm_.write_angles(target);
  at_ms(5000);

  const JointPositions pose{arm_.read_angles()};
  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    EXPECT_NEAR((target.at(joint) - pose.at(joint)) / DEG, FakeArm::REST_ERROR_DEG.at(joint), 1e-6)
      << joint;
  }
}

// #33, joint 4 "never commanded": the controller holds the pose it read, and the arm settles
// one rest error below that too.
TEST_F(MeasuredFakeArm, HoldingTheReadPoseRestsOneRestErrorBelowIt)
{
  const JointPositions held{arm_.read_angles()};
  arm_.write_angles(held);
  at_ms(5000);

  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    EXPECT_NEAR((held.at(joint) - angle(joint)) / DEG, FakeArm::REST_ERROR_DEG.at(joint), 1e-6)
      << joint;
  }
}

TEST_F(MeasuredFakeArm, AHaltStopsEveryJointWhereItIs)
{
  arm_.write_angles(start_with(J1, 0.1));
  at_ms(100);
  const double stopped{angle(J1)};
  EXPECT_GT(stopped, reported(J1, START.at(J1)));

  EXPECT_TRUE(arm_.halt());
  EXPECT_FALSE(arm_.last_target().has_value());
  at_ms(2000);
  EXPECT_DOUBLE_EQ(angle(J1), stopped);
}

// A wrist still waiting to start never starts once the arm is stopped.
TEST_F(MeasuredFakeArm, AHaltCancelsAWaitingWrist)
{
  arm_.write_angles(start_with(J4, 0.1));
  at_ms(100);
  arm_.request_halt();

  at_ms(2000);
  EXPECT_NEAR(angle(J4), reported(J4, START.at(J4)), EXACT);
  EXPECT_FALSE(arm_.last_target().has_value());
}

// The plugin refuses these before they get here (#34). If one did, the joint keeps going to
// its previous target and the others move: the pose never turns NaN.
TEST_F(MeasuredFakeArm, ANonFiniteTargetLeavesItsJointGoingWhereItWent)
{
  JointPositions target{start_with(J4, 0.1)};
  target.at(0) = std::numeric_limits<double>::quiet_NaN();
  target.at(1) = std::numeric_limits<double>::infinity();
  target.at(2) = -std::numeric_limits<double>::infinity();
  arm_.write_angles(target);
  at_ms(5000);

  const JointPositions pose{arm_.read_angles()};
  for (std::size_t joint{0}; joint < 3; ++joint) {
    EXPECT_NEAR(pose.at(joint), reported(joint, START.at(joint)), EXACT) << joint;
  }
  EXPECT_NEAR(pose.at(J4), reported(J4, START.at(J4) + 0.1), 1e-6);
}

TEST_F(MeasuredFakeArm, AClockThatStandsStillMovesNothing)
{
  arm_.write_angles(start_with(J1, 0.1));
  EXPECT_NEAR(angle(J1), reported(J1, START.at(J1)), EXACT);
  EXPECT_NEAR(angle(J1), reported(J1, START.at(J1)), EXACT);
}

TEST_F(MeasuredFakeArm, AClockThatGoesBackMovesNothing)
{
  at_ms(1000);
  arm_.write_angles(start_with(J1, 0.1));
  at_ms(0);
  EXPECT_NEAR(angle(J1), reported(J1, START.at(J1)), EXACT);
  EXPECT_TRUE(std::isfinite(angle(J1)));
}
