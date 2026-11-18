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
//   Unit tests for tracking_check.hpp (#103 part 2), without a thread or a clock: the test
//   passes the time in. Below the threshold never trips, over it trips at the time and not
//   before, coming back inside forgets the clock, the joint and the signed error are reported,
//   a NaN pose does not count, reset forgets everything.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/tracking_check.hpp"

namespace
{

using positronic_hardware::JointPositions;
using positronic_hardware::TRACKING_DEG_TO_RAD;
using positronic_hardware::TrackingCheck;
using positronic_hardware::TrackingError;
using positronic_hardware::TrackingLimits;
using std::chrono::milliseconds;

constexpr double DEG{TRACKING_DEG_TO_RAD};
constexpr JointPositions POSE{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

/// POSE with joint `index` (0 = J1) moved by `deg`.
JointPositions pose_with(std::size_t index, double deg)
{
  JointPositions out{POSE};
  out.at(index) += deg * DEG;
  return out;
}

TrackingCheck::Clock::time_point at(int ms)
{
  return TrackingCheck::Clock::time_point{} + milliseconds{ms};
}

}  // namespace

TEST(TrackingCheck, DefaultsAreEightDegreesForThreeHundredMilliseconds)
{
  const TrackingCheck check{};
  EXPECT_NEAR(check.limits().threshold_rad, 8.0 * DEG, 1e-12);
  EXPECT_EQ(check.limits().time, milliseconds{300});
}

TEST(TrackingCheck, BelowTheThresholdNeverTrips)
{
  TrackingCheck check{};
  for (int ms{0}; ms <= 5000; ms += 33) {
    EXPECT_FALSE(check.update(pose_with(0, 2.0), POSE, at(ms)).has_value()) << ms;
  }
  // Exactly at the threshold is still inside.
  for (int ms{0}; ms <= 1000; ms += 33) {
    EXPECT_FALSE(check.update(pose_with(0, 8.0), POSE, at(ms)).has_value()) << ms;
  }
}

TEST(TrackingCheck, TripsOnceTheJointHasBeenOverForTheTime)
{
  TrackingCheck check{};
  const JointPositions target{pose_with(0, 9.0)};
  EXPECT_FALSE(check.update(target, POSE, at(0)).has_value());    // the clock starts
  EXPECT_FALSE(check.update(target, POSE, at(299)).has_value());  // not yet
  const std::optional<TrackingError> error{check.update(target, POSE, at(300))};
  ASSERT_TRUE(error.has_value());
  const TrackingError got{error.value_or(TrackingError{})};
  EXPECT_EQ(got.joint, 0U);
  EXPECT_NEAR(got.error_rad, 9.0 * DEG, 1e-12);
}

TEST(TrackingCheck, ComingBackInsideForgetsTheClock)
{
  TrackingCheck check{};
  const JointPositions target{pose_with(2, 9.0)};
  EXPECT_FALSE(check.update(target, POSE, at(0)).has_value());
  EXPECT_FALSE(check.update(target, POSE, at(100)).has_value());
  // Inside for one cycle: the 100 ms are forgotten.
  EXPECT_FALSE(check.update(POSE, POSE, at(133)).has_value());
  EXPECT_FALSE(check.update(target, POSE, at(166)).has_value());
  EXPECT_FALSE(check.update(target, POSE, at(166 + 299)).has_value());
  EXPECT_TRUE(check.update(target, POSE, at(166 + 300)).has_value());
}

TEST(TrackingCheck, ReportsTheJointAndTheSignedError)
{
  TrackingCheck check{TrackingLimits{5.0 * DEG, milliseconds{100}}};
  const JointPositions target{pose_with(4, -6.0)};  // J5 asked to go 6 deg down, did not
  EXPECT_FALSE(check.update(target, POSE, at(0)).has_value());
  const std::optional<TrackingError> error{check.update(target, POSE, at(100))};
  ASSERT_TRUE(error.has_value());
  const TrackingError got{error.value_or(TrackingError{})};
  EXPECT_EQ(got.joint, 4U);
  EXPECT_NEAR(got.error_rad, -6.0 * DEG, 1e-12);
}

TEST(TrackingCheck, AnUnknownPoseDoesNotCount)
{
  TrackingCheck check{};
  JointPositions unknown{POSE};
  unknown.at(0) = std::nan("");
  const JointPositions target{pose_with(0, 20.0)};
  for (int ms{0}; ms <= 1000; ms += 33) {
    EXPECT_FALSE(check.update(target, unknown, at(ms)).has_value()) << ms;
  }
  // Infinity on either side is unknown too, and so is a NaN target.
  JointPositions infinite{POSE};
  infinite.at(0) = std::numeric_limits<double>::infinity();
  JointPositions nan_target{target};
  nan_target.at(0) = std::nan("");
  for (int ms{2000}; ms <= 3000; ms += 33) {
    EXPECT_FALSE(check.update(target, infinite, at(ms)).has_value()) << ms;
    EXPECT_FALSE(check.update(nan_target, POSE, at(ms)).has_value()) << ms;
  }
  // The pose arrives: the clock starts now, not at 0.
  EXPECT_FALSE(check.update(target, POSE, at(4000)).has_value());
  EXPECT_FALSE(check.update(target, POSE, at(4000 + 299)).has_value());
  EXPECT_TRUE(check.update(target, POSE, at(4000 + 300)).has_value());
}

TEST(TrackingCheck, ResetForgetsEveryClock)
{
  TrackingCheck check{};
  const JointPositions target{pose_with(1, 9.0)};
  EXPECT_FALSE(check.update(target, POSE, at(0)).has_value());
  EXPECT_FALSE(check.update(target, POSE, at(250)).has_value());
  check.reset();
  EXPECT_FALSE(check.update(target, POSE, at(300)).has_value());
  EXPECT_FALSE(check.update(target, POSE, at(599)).has_value());
  EXPECT_TRUE(check.update(target, POSE, at(600)).has_value());
}
