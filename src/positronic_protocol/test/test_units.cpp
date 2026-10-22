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
//   Write a GoogleTest suite for units.hpp. Check known angles, rounding (0.29 deg -> 29,
//   not 28 like pymycobot), to_radians, a round-trip over every wire value, a round-trip
//   error below half a centidegree, rejection of NaN / infinity / out-of-range, and the
//   six-joint array versions (all-or-nothing on failure).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>

#include "positronic_protocol/types.hpp"
#include "positronic_protocol/units.hpp"

namespace
{

namespace pp = positronic_protocol;

constexpr double PI{3.14159265358979323846};

double deg(double degrees)
{
  return degrees * PI / 180.0;
}

/// to_centidegrees() for a value in degrees, or a sentinel when it fails.
int centi(double degrees)
{
  const std::optional<pp::CentiDegrees> value{pp::to_centidegrees(deg(degrees))};
  return value.has_value() ? int{*value} : std::numeric_limits<int>::min();
}

}  // namespace

TEST(Units, KnownAngles)
{
  EXPECT_EQ(centi(0.0), 0);
  EXPECT_EQ(centi(90.0), 9000);
  EXPECT_EQ(centi(-90.0), -9000);
  EXPECT_EQ(centi(180.0), 18000);
  EXPECT_EQ(centi(-0.61), -61);
  EXPECT_EQ(pp::to_centidegrees(PI / 2).value_or(0), 9000);
}

TEST(Units, RoundsInsteadOfTruncating)
{
  // pymycobot: int(0.29 * 100) == 28. We round to the nearest 0.01 deg.
  EXPECT_EQ(centi(0.29), 29);
  EXPECT_EQ(centi(0.004), 0);
  EXPECT_EQ(centi(0.006), 1);
  EXPECT_EQ(centi(-0.006), -1);
}

TEST(Units, ToRadians)
{
  EXPECT_DOUBLE_EQ(pp::to_radians(pp::CentiDegrees{0}), 0.0);
  EXPECT_DOUBLE_EQ(pp::to_radians(pp::CentiDegrees{9000}), PI / 2);
  EXPECT_DOUBLE_EQ(pp::to_radians(pp::CentiDegrees{-18000}), -PI);
}

TEST(Units, EveryWireValueSurvivesTheRoundTrip)
{
  for (int value{-pp::MAX_CENTIDEGREES}; value <= pp::MAX_CENTIDEGREES; ++value) {
    const auto wire{static_cast<pp::CentiDegrees>(value)};
    const auto back{pp::to_centidegrees(pp::to_radians(wire))};
    ASSERT_TRUE(back.has_value()) << value;
    ASSERT_EQ(back.value_or(0), wire);
  }
}

TEST(Units, RadiansRoundTripErrorIsHalfACentidegree)
{
  const double max_error{deg(0.005) + 1e-12};
  // Integer loop counter: adding 0.0137 to a double again and again accumulates error.
  for (int step{-23868}; step <= 23868; ++step) {
    const double degrees{step * 0.0137};
    const auto wire{pp::to_centidegrees(deg(degrees))};
    ASSERT_TRUE(wire.has_value()) << degrees;
    EXPECT_LE(std::abs(pp::to_radians(wire.value_or(0)) - deg(degrees)), max_error) << degrees;
  }
}

TEST(Units, RejectsWhatTheWireCannotCarry)
{
  EXPECT_FALSE(pp::to_centidegrees(std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_FALSE(pp::to_centidegrees(std::numeric_limits<double>::infinity()).has_value());
  EXPECT_FALSE(pp::to_centidegrees(-std::numeric_limits<double>::infinity()).has_value());
  EXPECT_FALSE(pp::to_centidegrees(1e300).has_value());
  EXPECT_EQ(centi(327.67), 32767);
  EXPECT_EQ(centi(-327.67), -32767);
  EXPECT_EQ(centi(327.68), std::numeric_limits<int>::min());
  EXPECT_EQ(centi(-327.68), std::numeric_limits<int>::min());
}

TEST(Units, JointArrayToCentidegrees)
{
  const pp::JointRadians radians{deg(0.87), 0.0, deg(-0.61), deg(-1.23), deg(-0.08), PI / 2};
  const auto angles{pp::to_centidegrees(radians)};
  ASSERT_TRUE(angles.has_value());
  EXPECT_EQ(angles.value_or(pp::JointAngles{}), (pp::JointAngles{87, 0, -61, -123, -8, 9000}));
}

TEST(Units, JointArrayFailsIfAnyJointFails)
{
  pp::JointRadians radians{};
  radians.back() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(pp::to_centidegrees(radians).has_value());
  radians.back() = 0.0;
  radians.front() = deg(400.0);
  EXPECT_FALSE(pp::to_centidegrees(radians).has_value());
}

TEST(Units, JointArrayToRadians)
{
  const pp::JointRadians radians{pp::to_radians(pp::JointAngles{9000, -9000, 0, 18000, 1, -1})};
  EXPECT_DOUBLE_EQ(radians.at(0), PI / 2);
  EXPECT_DOUBLE_EQ(radians.at(1), -PI / 2);
  EXPECT_DOUBLE_EQ(radians.at(2), 0.0);
  EXPECT_DOUBLE_EQ(radians.at(3), PI);
  EXPECT_DOUBLE_EQ(radians.at(4), deg(0.01));
  EXPECT_DOUBLE_EQ(radians.at(5), -deg(0.01));
}
