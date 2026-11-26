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
//
// Follow-up prompt (#170, the cut by time):
//   With the poll's frames a reading comes every 33 ms, and J1 gives false hot ones, each one
//   alone. Cut by time: a servo at or above the limit for HOT_FOR (1 s). Check: constantly hot
//   polls trip at the first one 1 s after the first hot one, and a cool reading starts the clock
//   again; two requests a second apart still trip; with no wait one reading alone does not. A
//   reading one degree under the limit neither starts nor stops the clock (59 and 60 in turn
//   trip on a 60 after 1 s, a 58 stops it). One clock per servo: J4 held at 59 does not keep
//   alive a clock J1's false readings start, the trip names J4; J1 and J4 together name J1.
//
// Follow-up prompt (cut at 65 degC):
//   The default cut goes from 60 to 65 degC. The detector cases keep their readings around 60,
//   now as an explicit 60 degC limit (LIMIT_60): they test the detector, not the default.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/servo_temperature.hpp"

namespace
{
using positronic_hardware::ACTIVATE_COOLDOWN_C;
using positronic_hardware::cool_enough_to_activate;
using positronic_hardware::HOT_FOR;
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
/// The limit the detector cases are written for: the old default, a lowered limit now.
constexpr int LIMIT_60{60};
/// J2 above the limit, J3 at it: J2 is the hottest.
constexpr ServoTemperatures J2_HOTTEST{31, 62, 60, 30, 29, 28};
/// One slot reads 0: what an unanswered servo looks like.
constexpr ServoTemperatures ONE_ZERO{31, 0, 35, 30, 29, 28};
/// The poll's cadence while moving (#170): a reading with every send.
constexpr std::int64_t POLL_MS{33};

/// The time of a reading `ms` after the first one.
OverheatDetector::Clock::time_point at_ms(std::int64_t ms)
{
  return OverheatDetector::Clock::time_point{} + std::chrono::milliseconds{ms};
}
}  // namespace

TEST(ServoTemperature, Constants)
{
  EXPECT_EQ(SERVO_TEMP_LIMIT_C, 65);
  EXPECT_LT(MIN_SERVO_TEMP_LIMIT_C, SERVO_TEMP_LIMIT_C);
  EXPECT_EQ(ACTIVATE_COOLDOWN_C, 5);
  EXPECT_EQ(HOT_FOR, std::chrono::milliseconds{1000});
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
  OverheatDetector detector{LIMIT_60};
  for (std::int64_t i{0}; i < 100; ++i) {
    EXPECT_FALSE(detector.update(COOL, at_ms(i * 1000)).has_value());
  }
}

TEST(OverheatDetector, OneHotReadingIsNotEnough)
{
  OverheatDetector detector{LIMIT_60};
  EXPECT_FALSE(detector.update(J3_AT_LIMIT, at_ms(0)).has_value());
  EXPECT_FALSE(detector.update(COOL, at_ms(1000)).has_value());
  // 2 s after the first hot reading, but the cool one started the count again.
  EXPECT_FALSE(detector.update(J3_AT_LIMIT, at_ms(2000)).has_value());
  EXPECT_FALSE(detector.tripped().has_value());
}

TEST(OverheatDetector, TwoRequestsASecondApartTrip)
{
  // At rest the thread asks once a second: two hot readings in a row, as before the frames.
  OverheatDetector detector{LIMIT_60};
  EXPECT_FALSE(detector.update(J3_AT_LIMIT, at_ms(0)).has_value());
  // Exactly HOT_FOR later counts. The trip names J3, whose clock ran out, not J2, hotter in this
  // reading but at the limit only since it.
  const std::optional<Overheat> trip{detector.update(J2_HOTTEST, at_ms(1000))};
  ASSERT_TRUE(trip.has_value());
  EXPECT_EQ(trip.value_or(Overheat{}).joint, 2U);
  EXPECT_EQ(trip.value_or(Overheat{}).celsius, 60);
}

TEST(OverheatDetector, ConstantlyHotPollsTripAtOneSecond)
{
  OverheatDetector detector{LIMIT_60};
  // Polls 0 to 30 are 0 to 990 ms after the first: not yet. Poll 31, at 1023 ms, trips.
  for (std::int64_t i{0}; i <= 30; ++i) {
    ASSERT_FALSE(detector.update(J3_AT_LIMIT, at_ms(i * POLL_MS)).has_value()) << i;
  }
  EXPECT_TRUE(detector.update(J3_AT_LIMIT, at_ms(31 * POLL_MS)).has_value());
}

TEST(OverheatDetector, OneDegreeUnderTheLimitNeitherStartsNorStopsTheClock)
{
  // A servo crossing the limit reads 59 and 60 in turn for seconds (J4 and J6 on the arm).
  constexpr ServoTemperatures J3_UNDER{31, 33, 59, 30, 29, 28};
  OverheatDetector detector{LIMIT_60};
  EXPECT_FALSE(detector.update(J3_UNDER, at_ms(0)).has_value());        // does not start it
  EXPECT_FALSE(detector.update(J3_AT_LIMIT, at_ms(500)).has_value());   // starts it
  EXPECT_FALSE(detector.update(J3_UNDER, at_ms(1000)).has_value());     // 1 s after the 59
  EXPECT_FALSE(detector.update(J3_AT_LIMIT, at_ms(1100)).has_value());  // 600 ms of clock
  EXPECT_FALSE(detector.update(J3_UNDER, at_ms(1500)).has_value());     // past it, but a 59
  const std::optional<Overheat> trip{detector.update(J3_AT_LIMIT, at_ms(1533))};
  ASSERT_TRUE(trip.has_value());
  EXPECT_EQ(trip.value_or(Overheat{}).celsius, 60);

  // 58 is more than the band under the limit: it stops the clock.
  OverheatDetector edge{LIMIT_60};
  EXPECT_FALSE(edge.update(J3_AT_LIMIT, at_ms(0)).has_value());
  EXPECT_FALSE(edge.update(ServoTemperatures{31, 33, 58, 30, 29, 28}, at_ms(500)).has_value());
  EXPECT_FALSE(edge.update(J3_AT_LIMIT, at_ms(1000)).has_value());
}

TEST(OverheatDetector, EachServoHasItsOwnClock)
{
  // J4 held at 59, in the band, while J1 gives two false readings 2 s apart. With one clock for
  // the hottest servo, J4's 59s never stopped the one J1 started, and the second false reading
  // cut the arm as "J1 at 76" (replayed on the arm's recordings).
  constexpr ServoTemperatures J4_59_J1_FALSE{76, 34, 32, 59, 32, 35};
  constexpr ServoTemperatures J4_59{41, 34, 32, 59, 32, 35};
  constexpr ServoTemperatures J4_60{41, 34, 32, 60, 32, 35};
  OverheatDetector detector{LIMIT_60};
  EXPECT_FALSE(detector.update(J4_59_J1_FALSE, at_ms(0)).has_value());
  for (std::int64_t ms{POLL_MS}; ms < 2000; ms += POLL_MS) {
    ASSERT_FALSE(detector.update(J4_59, at_ms(ms)).has_value()) << ms;
  }
  EXPECT_FALSE(detector.update(J4_59_J1_FALSE, at_ms(2000)).has_value());
  // J4 itself at the limit for 1 s: the cut, naming J4.
  EXPECT_FALSE(detector.update(J4_60, at_ms(2100)).has_value());
  const std::optional<Overheat> trip{detector.update(J4_60, at_ms(3100))};
  ASSERT_TRUE(trip.has_value());
  EXPECT_EQ(trip.value_or(Overheat{}).joint, 3U);
  EXPECT_EQ(trip.value_or(Overheat{}).celsius, 60);
}

TEST(OverheatDetector, J1IsWatchedAndNamedFirst)
{
  // J1 is joint 0, the first of the loop. At the limit as long as J4, the trip names J1: the
  // J1 side first.
  constexpr ServoTemperatures J1_AND_J4_AT_LIMIT{60, 33, 35, 61, 29, 28};
  OverheatDetector detector{LIMIT_60};
  EXPECT_FALSE(detector.update(J1_AND_J4_AT_LIMIT, at_ms(0)).has_value());
  const std::optional<Overheat> trip{detector.update(J1_AND_J4_AT_LIMIT, at_ms(1000))};
  ASSERT_TRUE(trip.has_value());
  EXPECT_EQ(trip.value_or(Overheat{}).joint, 0U);
  EXPECT_EQ(trip.value_or(Overheat{}).celsius, 60);
}

TEST(OverheatDetector, WithNoWaitOneReadingAloneStillDoesNotTrip)
{
  OverheatDetector detector{LIMIT_60, std::chrono::milliseconds{0}};
  EXPECT_FALSE(detector.update(J3_AT_LIMIT, at_ms(0)).has_value());
  EXPECT_TRUE(detector.update(J3_AT_LIMIT, at_ms(0)).has_value());
}

TEST(OverheatDetector, AnImplausibleReadingNeitherCountsNorResets)
{
  OverheatDetector detector{LIMIT_60};
  EXPECT_FALSE(detector.update(J3_AT_LIMIT, at_ms(0)).has_value());
  EXPECT_FALSE(detector.update(ONE_ZERO, at_ms(500)).has_value());
  EXPECT_TRUE(detector.update(J3_AT_LIMIT, at_ms(1000)).has_value());

  // And an implausible hot-looking reading alone never trips.
  OverheatDetector other{LIMIT_60};
  const ServoTemperatures garbage{200, 200, 200, 200, 200, 200};
  EXPECT_FALSE(other.update(garbage, at_ms(0)).has_value());
  EXPECT_FALSE(other.update(garbage, at_ms(1000)).has_value());
}

TEST(OverheatDetector, TheTripStaysAfterTheServosCool)
{
  OverheatDetector detector{LIMIT_60};
  static_cast<void>(detector.update(J3_AT_LIMIT, at_ms(0)));
  ASSERT_TRUE(detector.update(J3_AT_LIMIT, at_ms(1000)).has_value());
  const std::optional<Overheat> later{detector.update(COOL, at_ms(2000))};
  ASSERT_TRUE(later.has_value());
  EXPECT_EQ(later.value_or(Overheat{}).joint, 2U);
  EXPECT_EQ(later.value_or(Overheat{}).celsius, 60);
}

TEST(OverheatDetector, ALoweredLimitTripsEarlier)
{
  // How the hardware test cuts the arm without heating anything: limit just above the room.
  OverheatDetector detector{34};
  EXPECT_FALSE(detector.update(COOL, at_ms(0)).has_value());  // J3 at 35: first hot reading
  EXPECT_TRUE(detector.update(COOL, at_ms(1000)).has_value());
  EXPECT_EQ(detector.limit_c(), 34);
}
