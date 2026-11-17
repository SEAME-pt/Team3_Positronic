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
//   Write a GoogleTest suite for servo_temperature.hpp, the pure logic of the servo
//   over-temperature cut: no port, no thread, no ROS. Check the constants (the cut at 60 degC,
//   the lowest limit the plugin accepts, the activation cool-down); plausible() at 0, 1, 99 and
//   100 degC; hottest() picks the hottest servo and the J1 side on a tie;
//   cool_enough_to_activate() refuses from limit - 5 degC; and OverheatDetector: readings under
//   the limit never trip, one hot reading followed by a cool one does not trip, two hot readings
//   in a row trip with the hottest servo of the second, a reading exactly at the limit counts,
//   an implausible reading neither counts nor resets the count, the trip stays after the servos
//   cool down, and a lowered limit trips earlier.

#include <gtest/gtest.h>

#include <optional>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/servo_temperature.hpp"

namespace
{
using positronic_hardware::ACTIVATE_COOLDOWN_C;
using positronic_hardware::cool_enough_to_activate;
using positronic_hardware::hottest;
using positronic_hardware::MIN_SERVO_TEMP_LIMIT_C;
using positronic_hardware::Overheat;
using positronic_hardware::OverheatDetector;
using positronic_hardware::plausible;
using positronic_hardware::SERVO_TEMP_LIMIT_C;
using positronic_hardware::ServoTemperatures;

constexpr ServoTemperatures COOL{31, 33, 35, 30, 29, 28};
/// J3 at the limit, the rest cool.
constexpr ServoTemperatures J3_AT_LIMIT{31, 33, 60, 30, 29, 28};
/// J2 above the limit, J3 at it: J2 is the hottest.
constexpr ServoTemperatures J2_HOTTEST{31, 62, 60, 30, 29, 28};
/// One slot reads 0: what an unanswered servo looks like.
constexpr ServoTemperatures ONE_ZERO{31, 0, 35, 30, 29, 28};
}  // namespace

TEST(ServoTemperature, Constants)
{
  EXPECT_EQ(SERVO_TEMP_LIMIT_C, 60);
  EXPECT_LT(MIN_SERVO_TEMP_LIMIT_C, SERVO_TEMP_LIMIT_C);
  EXPECT_EQ(ACTIVATE_COOLDOWN_C, 5);
}

TEST(ServoTemperature, PlausibleRange)
{
  EXPECT_TRUE(plausible(COOL));
  EXPECT_TRUE(plausible(ServoTemperatures{1, 1, 1, 1, 1, 1}));
  EXPECT_TRUE(plausible(ServoTemperatures{99, 99, 99, 99, 99, 99}));
  EXPECT_FALSE(plausible(ONE_ZERO));
  EXPECT_FALSE(plausible(ServoTemperatures{31, 33, 100, 30, 29, 28}));
}

TEST(ServoTemperature, HottestServo)
{
  const Overheat hot{hottest(J2_HOTTEST)};
  EXPECT_EQ(hot.joint, 1U);
  EXPECT_EQ(hot.celsius, 62);
  // A tie goes to the J1 side.
  const Overheat tie{hottest(ServoTemperatures{40, 45, 45, 30, 29, 28})};
  EXPECT_EQ(tie.joint, 1U);
  EXPECT_EQ(hottest(ServoTemperatures{50, 30, 30, 30, 30, 30}).joint, 0U);
}

TEST(ServoTemperature, ActivationNeedsTheCoolDownMargin)
{
  EXPECT_TRUE(cool_enough_to_activate(ServoTemperatures{54, 30, 30, 30, 30, 30}, 60));
  EXPECT_FALSE(cool_enough_to_activate(ServoTemperatures{30, 30, 30, 30, 30, 55}, 60));
  EXPECT_FALSE(cool_enough_to_activate(J3_AT_LIMIT, 60));
  // With a lowered limit the margin moves with it.
  EXPECT_FALSE(cool_enough_to_activate(COOL, 40));
}

TEST(OverheatDetector, CoolReadingsNeverTrip)
{
  OverheatDetector detector{SERVO_TEMP_LIMIT_C};
  for (int i{0}; i < 100; ++i) {
    EXPECT_FALSE(detector.update(COOL).has_value());
  }
}

TEST(OverheatDetector, OneHotReadingIsNotEnough)
{
  OverheatDetector detector{SERVO_TEMP_LIMIT_C};
  EXPECT_FALSE(detector.update(J3_AT_LIMIT).has_value());
  EXPECT_FALSE(detector.update(COOL).has_value());
  EXPECT_FALSE(detector.update(J3_AT_LIMIT).has_value());  // the count started again
  EXPECT_FALSE(detector.tripped().has_value());
}

TEST(OverheatDetector, TwoHotReadingsInARowTrip)
{
  OverheatDetector detector{SERVO_TEMP_LIMIT_C};
  EXPECT_FALSE(detector.update(J3_AT_LIMIT).has_value());
  // Exactly at the limit counts; the trip names the hottest servo of the second reading.
  const std::optional<Overheat> trip{detector.update(J2_HOTTEST)};
  ASSERT_TRUE(trip.has_value());
  EXPECT_EQ(trip.value_or(Overheat{}).joint, 1U);
  EXPECT_EQ(trip.value_or(Overheat{}).celsius, 62);
}

TEST(OverheatDetector, AnImplausibleReadingNeitherCountsNorResets)
{
  OverheatDetector detector{SERVO_TEMP_LIMIT_C};
  EXPECT_FALSE(detector.update(J3_AT_LIMIT).has_value());
  EXPECT_FALSE(detector.update(ONE_ZERO).has_value());
  EXPECT_TRUE(detector.update(J3_AT_LIMIT).has_value());

  // And an implausible hot-looking reading alone never trips.
  OverheatDetector other{SERVO_TEMP_LIMIT_C};
  const ServoTemperatures garbage{200, 200, 200, 200, 200, 200};
  EXPECT_FALSE(other.update(garbage).has_value());
  EXPECT_FALSE(other.update(garbage).has_value());
}

TEST(OverheatDetector, TheTripStaysAfterTheServosCool)
{
  OverheatDetector detector{SERVO_TEMP_LIMIT_C};
  static_cast<void>(detector.update(J3_AT_LIMIT));
  ASSERT_TRUE(detector.update(J3_AT_LIMIT).has_value());
  const std::optional<Overheat> later{detector.update(COOL)};
  ASSERT_TRUE(later.has_value());
  EXPECT_EQ(later.value_or(Overheat{}).joint, 2U);
  EXPECT_EQ(later.value_or(Overheat{}).celsius, 60);
}

TEST(OverheatDetector, ALoweredLimitTripsEarlier)
{
  // How the hardware test cuts the arm without heating anything: limit just above the room.
  OverheatDetector detector{34};
  EXPECT_FALSE(detector.update(COOL).has_value());  // J3 at 35: first hot reading
  EXPECT_TRUE(detector.update(COOL).has_value());
  EXPECT_EQ(detector.limit_c(), 34);
}
