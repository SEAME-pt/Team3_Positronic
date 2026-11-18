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

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <unordered_map>

#include "positronic_hardware/parameters.hpp"

namespace
{

using positronic_hardware::Backend;
using positronic_hardware::describe;
using positronic_hardware::HardwareConfig;
using positronic_hardware::parse_hardware_parameters;
using positronic_hardware::ParsedParameters;
using Parameters = std::unordered_map<std::string, std::string>;
using std::chrono::milliseconds;

/// True when `parameters` give a config.
bool accepted(const Parameters & parameters)
{
  return parse_hardware_parameters(parameters).config.has_value();
}

/// The reason one parameter set to `value` is refused, every other at its default.
std::string refusal(const std::string & name, const std::string & value)
{
  return parse_hardware_parameters(Parameters{{name, value}}).error;
}

}  // namespace

TEST(Parameters, NothingSetGivesTheDefaults)
{
  const ParsedParameters parsed{parse_hardware_parameters(Parameters{})};
  ASSERT_TRUE(parsed.config.has_value());
  EXPECT_TRUE(parsed.error.empty());
  const HardwareConfig & config{parsed.config.value()};
  const HardwareConfig defaults{};
  EXPECT_EQ(config.backend, defaults.backend);
  EXPECT_EQ(config.port, defaults.port);
  EXPECT_EQ(config.baud, defaults.baud);
  EXPECT_DOUBLE_EQ(config.velocity_filter_alpha, defaults.velocity_filter_alpha);
  EXPECT_EQ(config.timing.moving_period, defaults.timing.moving_period);
  EXPECT_EQ(config.timing.send_settle, defaults.timing.send_settle);
  EXPECT_EQ(config.servo_temp_limit_c, defaults.servo_temp_limit_c);
  EXPECT_DOUBLE_EQ(config.tracking_error_deg, defaults.tracking_error_deg);
  EXPECT_EQ(config.tracking_error_ms, defaults.tracking_error_ms);
}

TEST(Parameters, EveryParameterReachesItsField)
{
  const ParsedParameters parsed{parse_hardware_parameters(Parameters{
      {"backend", "fake"},
      {"port", "/dev/null"},
      {"baud", "115200"},
      {"velocity_filter_alpha", "0.5"},
      {"moving_period_ms", "40"},
      {"send_settle_ms", "25"},
      {"servo_temp_limit_c", "45"},
      {"tracking_error_deg", "6.5"},
      {"tracking_error_ms", "200"}})};
  ASSERT_TRUE(parsed.config.has_value()) << parsed.error;
  const HardwareConfig & config{parsed.config.value()};
  EXPECT_EQ(config.backend, Backend::FAKE);
  EXPECT_EQ(config.port, "/dev/null");
  EXPECT_EQ(config.baud, 115200);
  EXPECT_DOUBLE_EQ(config.velocity_filter_alpha, 0.5);
  EXPECT_EQ(config.timing.moving_period, milliseconds{40});
  EXPECT_EQ(config.timing.send_settle, milliseconds{25});
  EXPECT_EQ(config.servo_temp_limit_c, 45);
  EXPECT_DOUBLE_EQ(config.tracking_error_deg, 6.5);
  EXPECT_EQ(config.tracking_error_ms, 200);
}

// The list in the message comes from the tables: a parameter added there shows up here.
TEST(Parameters, AnUnknownNameIsRefusedWithEveryKnownName)
{
  EXPECT_EQ(
    refusal("moving_period", "40"),
    "Unknown hardware parameter 'moving_period'. Known: backend, port, baud, "
    "velocity_filter_alpha, moving_period_ms, send_settle_ms, servo_temp_limit_c, "
    "tracking_error_deg, tracking_error_ms.");
}

// Every unknown name in one message, in alphabetical order: one run shows every typo.
TEST(Parameters, EveryUnknownNameIsReportedInOrder)
{
  const ParsedParameters parsed{parse_hardware_parameters(
    Parameters{{"tracking_eror_ms", "300"}, {"backend", "fake"}, {"baudrate", "115200"}})};
  EXPECT_FALSE(parsed.config.has_value());
  EXPECT_EQ(
    parsed.error.substr(0, parsed.error.find(". Known:")),
    "Unknown hardware parameters 'baudrate', 'tracking_eror_ms'");
}

// Each end of each range is a valid value. The shortest period needs a shorter settle than the
// 30 ms default, or the settle rule refuses it.
TEST(Parameters, TheEndsOfEveryRangeAreAccepted)
{
  EXPECT_TRUE(accepted(Parameters{{"moving_period_ms", "20"}, {"send_settle_ms", "10"}}));
  EXPECT_TRUE(accepted(Parameters{{"moving_period_ms", "100"}}));
  EXPECT_TRUE(accepted(Parameters{{"send_settle_ms", "0"}}));
  EXPECT_TRUE(accepted(Parameters{{"moving_period_ms", "100"}, {"send_settle_ms", "99"}}));
  EXPECT_TRUE(accepted(Parameters{{"servo_temp_limit_c", "30"}}));
  EXPECT_TRUE(accepted(Parameters{{"servo_temp_limit_c", "60"}}));
  EXPECT_TRUE(accepted(Parameters{{"tracking_error_deg", "1"}}));
  EXPECT_TRUE(accepted(Parameters{{"tracking_error_deg", "45"}}));
  EXPECT_TRUE(accepted(Parameters{{"tracking_error_ms", "50"}}));
  EXPECT_TRUE(accepted(Parameters{{"tracking_error_ms", "5000"}}));
}

// One step past each end is refused, and the message gives the range.
TEST(Parameters, OneStepOutsideARangeIsRefusedWithTheRange)
{
  EXPECT_EQ(
    refusal("moving_period_ms", "19"),
    "Parameter 'moving_period_ms' is '19'; use a whole number in [20, 100].");
  EXPECT_EQ(
    refusal("moving_period_ms", "101"),
    "Parameter 'moving_period_ms' is '101'; use a whole number in [20, 100].");
  EXPECT_EQ(
    refusal("send_settle_ms", "-1"),
    "Parameter 'send_settle_ms' is '-1'; use a whole number in [0, 99].");
  EXPECT_EQ(
    refusal("send_settle_ms", "100"),
    "Parameter 'send_settle_ms' is '100'; use a whole number in [0, 99].");
  EXPECT_EQ(
    refusal("servo_temp_limit_c", "29"),
    "Parameter 'servo_temp_limit_c' is '29'; use a whole number in [30, 60].");
  EXPECT_EQ(
    refusal("servo_temp_limit_c", "61"),
    "Parameter 'servo_temp_limit_c' is '61'; use a whole number in [30, 60].");
  EXPECT_EQ(
    refusal("tracking_error_deg", "0.9"),
    "Parameter 'tracking_error_deg' is '0.9'; use a number in [1, 45].");
  EXPECT_EQ(
    refusal("tracking_error_deg", "45.1"),
    "Parameter 'tracking_error_deg' is '45.1'; use a number in [1, 45].");
  EXPECT_EQ(
    refusal("tracking_error_ms", "49"),
    "Parameter 'tracking_error_ms' is '49'; use a whole number in [50, 5000].");
  EXPECT_EQ(
    refusal("tracking_error_ms", "5001"),
    "Parameter 'tracking_error_ms' is '5001'; use a whole number in [50, 5000].");
}

TEST(Parameters, AFractionWhereAWholeNumberIsExpectedIsRefused)
{
  EXPECT_FALSE(accepted(Parameters{{"tracking_error_ms", "300.5"}}));
  EXPECT_FALSE(accepted(Parameters{{"servo_temp_limit_c", "45.0"}}));
  EXPECT_FALSE(accepted(Parameters{{"moving_period_ms", "40ms"}}));
}

// from_chars reads "nan", "inf" and "-inf" as numbers: the range check is what refuses them.
TEST(Parameters, NotANumberIsRefused)
{
  EXPECT_FALSE(accepted(Parameters{{"tracking_error_deg", "nan"}}));
  EXPECT_FALSE(accepted(Parameters{{"velocity_filter_alpha", "nan"}}));
  EXPECT_FALSE(accepted(Parameters{{"tracking_error_deg", "inf"}}));
  EXPECT_FALSE(accepted(Parameters{{"tracking_error_deg", "-inf"}}));
  EXPECT_FALSE(accepted(Parameters{{"velocity_filter_alpha", "inf"}}));
  EXPECT_FALSE(accepted(Parameters{{"velocity_filter_alpha", "-inf"}}));
  EXPECT_FALSE(accepted(Parameters{{"tracking_error_ms", "abc"}}));
  EXPECT_FALSE(accepted(Parameters{{"tracking_error_deg", ""}}));
}

// The settle is part of a turn, so it must stay below the period, the 50 ms default here.
TEST(Parameters, TheSettleMustStayBelowThePeriod)
{
  EXPECT_TRUE(accepted(Parameters{{"send_settle_ms", "49"}}));
  EXPECT_EQ(
    refusal("send_settle_ms", "50"), "send_settle_ms (50) must be below moving_period_ms (50).");
}

TEST(Parameters, TheSpecialParametersKeepTheirRules)
{
  EXPECT_EQ(refusal("backend", "real"), "Parameter 'backend' is 'real'; use 'serial' or 'fake'.");
  EXPECT_EQ(refusal("port", ""), "Parameter 'port' is empty.");
  EXPECT_EQ(refusal("baud", "9600"), "Parameter 'baud' is '9600'; use 115200 or 1000000.");
  EXPECT_EQ(
    refusal("velocity_filter_alpha", "0"),
    "Parameter 'velocity_filter_alpha' is '0'; use a number in (0, 1].");
  EXPECT_FALSE(accepted(Parameters{{"velocity_filter_alpha", "1.5"}}));
  EXPECT_TRUE(accepted(Parameters{{"velocity_filter_alpha", "1"}}));
}

// The INFO line of on_init, the same text as before #163.
TEST(Parameters, TheInfoLineHasEveryValue)
{
  EXPECT_EQ(
    describe(HardwareConfig{}),
    "Backend 'serial', port /dev/ttyTHS1 at 1000000 baud, velocity filter alpha 0.300, moving "
    "period 50 ms, send settle 30 ms, servo temperature limit 60 degC, tracking error 8.0 deg "
    "for 300 ms.");
}
