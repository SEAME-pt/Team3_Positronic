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
//   Write a GoogleTest suite for velocity_estimator.hpp. The estimator differences joint
//   positions over dt and low-pass filters the result. Check that: the first sample returns
//   zero (no baseline yet); a constant position gives zero velocity; alpha = 1 gives the exact
//   finite difference; alpha < 1 rises monotonically towards a constant rate without
//   overshooting; a dt at or below MIN_DT holds the previous estimate instead of dividing by
//   near-zero; a dt above max_dt (silent link) restarts the estimate at zero and rebases; and
//   reset() clears the history so the next sample is a fresh zero baseline.

#include <gtest/gtest.h>

#include <cstddef>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/velocity_estimator.hpp"

namespace
{
using positronic_hardware::JointPositions;
using positronic_hardware::NUM_JOINTS;
using positronic_hardware::VelocityEstimator;

/// Every joint set to the same angle, to keep the arithmetic easy to read.
JointPositions uniform(double value)
{
  JointPositions out{};
  out.fill(value);
  return out;
}

TEST(VelocityEstimator, FirstSampleHasNoVelocity)
{
  VelocityEstimator estimator{};
  // Nothing to difference against yet: the first sample only sets the baseline.
  const JointPositions velocity{estimator.update(uniform(1.0), 0.01)};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_DOUBLE_EQ(velocity.at(i), 0.0) << i;
  }
}

TEST(VelocityEstimator, ConstantPositionGivesZeroVelocity)
{
  VelocityEstimator estimator{};
  estimator.update(uniform(0.5), 0.01);
  for (int step{0}; step < 10; ++step) {
    const JointPositions velocity{estimator.update(uniform(0.5), 0.01)};
    for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
      EXPECT_DOUBLE_EQ(velocity.at(i), 0.0) << i;
    }
  }
}

TEST(VelocityEstimator, ConvergesToTheConstantRate)
{
  // alpha = 1: no smoothing, so one step already gives the exact finite difference.
  VelocityEstimator estimator{1.0};
  constexpr double DT{0.02};
  constexpr double STEP{0.1};  // rad per tick -> 0.1 / 0.02 = 5 rad/s
  estimator.update(uniform(0.0), DT);
  const JointPositions velocity{estimator.update(uniform(STEP), DT)};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_DOUBLE_EQ(velocity.at(i), STEP / DT) << i;
  }
}

TEST(VelocityEstimator, SmoothingLagsThenCatchesUp)
{
  // alpha < 1: the filtered value rises towards the true rate over several ticks, never past it.
  VelocityEstimator estimator{0.5};
  constexpr double DT{0.02};
  constexpr double STEP{0.1};
  constexpr double TRUE_RATE{STEP / DT};
  estimator.update(uniform(0.0), DT);

  double previous{0.0};
  double current{0.0};
  for (int step{1}; step <= 8; ++step) {
    const JointPositions velocity{estimator.update(uniform(STEP * step), DT)};
    current = velocity.at(0);
    EXPECT_GT(current, previous);            // monotonically rising
    EXPECT_LT(current, TRUE_RATE + 1e-9);    // but never overshoots a constant rate
    previous = current;
  }
  EXPECT_NEAR(current, TRUE_RATE, 0.2);      // and has nearly arrived after a few ticks
}

TEST(VelocityEstimator, ZeroPeriodHoldsTheEstimate)
{
  VelocityEstimator estimator{1.0};
  constexpr double DT{0.02};
  estimator.update(uniform(0.0), DT);
  const JointPositions moving{estimator.update(uniform(0.1), DT)};
  // A second read() in the same tick (dt = 0) must not divide by zero; it holds the last value.
  const JointPositions held{estimator.update(uniform(0.2), 0.0)};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_DOUBLE_EQ(held.at(i), moving.at(i)) << i;
  }
}

TEST(VelocityEstimator, LongGapRestartsAtZero)
{
  // alpha = 1 and a steady rate, then a dt well above the default max_dt (0.5 s): the silent
  // link must restart the estimate at zero rather than report a huge false jump.
  VelocityEstimator estimator{1.0};
  constexpr double DT{0.02};
  estimator.update(uniform(0.0), DT);
  estimator.update(uniform(0.1), DT);  // now carries a non-zero velocity

  const JointPositions after_gap{estimator.update(uniform(5.0), 1.0)};  // 1.0 s > max_dt
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_DOUBLE_EQ(after_gap.at(i), 0.0) << i;
  }

  // And it rebased on the post-gap sample: a normal step from there differences cleanly.
  const JointPositions resumed{estimator.update(uniform(5.1), DT)};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_DOUBLE_EQ(resumed.at(i), 0.1 / DT) << i;
  }
}

TEST(VelocityEstimator, ResetClearsHistory)
{
  VelocityEstimator estimator{1.0};
  constexpr double DT{0.02};
  estimator.update(uniform(0.0), DT);
  estimator.update(uniform(0.1), DT);  // now carries a non-zero velocity and a baseline

  estimator.reset();
  // After reset the next sample is a fresh baseline again: no velocity from the old history.
  const JointPositions velocity{estimator.update(uniform(5.0), DT)};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_DOUBLE_EQ(velocity.at(i), 0.0) << i;
  }
}

}  // namespace