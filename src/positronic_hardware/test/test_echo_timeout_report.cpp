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
//   Write a GoogleTest suite for echo_timeout_report.hpp (#150): the plugin's decision of when
//   read() warns about sends that got no sync-write. It takes the backend's count and the
//   current time. Check that: nothing new is never due; the first new count is due at once,
//   with all of it; within the period nothing more is due, after it the sends since the last
//   warning are; exactly the period is due; reset() makes the next count due at once, as for
//   a new backend; without reset() a new backend's small count stays hidden; the default
//   period is 5 s.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>

#include "positronic_hardware/echo_timeout_report.hpp"

namespace
{

using positronic_hardware::EchoTimeoutReport;
using std::chrono::milliseconds;

const EchoTimeoutReport::Clock::time_point T0{};
const milliseconds PERIOD{5000};

}  // namespace

TEST(EchoTimeoutReport, NothingNewIsNeverDue)
{
  EchoTimeoutReport report{PERIOD};
  EXPECT_FALSE(report.due(0, T0).has_value());
  EXPECT_FALSE(report.due(0, T0 + 3 * PERIOD).has_value());
}

TEST(EchoTimeoutReport, TheFirstCountIsDueAtOnce)
{
  EchoTimeoutReport report{PERIOD};
  EXPECT_EQ(report.due(3, T0), std::optional<std::uint64_t>{3});
}

TEST(EchoTimeoutReport, WithinThePeriodNothingMoreIsDue)
{
  EchoTimeoutReport report{PERIOD};
  ASSERT_TRUE(report.due(3, T0).has_value());
  EXPECT_FALSE(report.due(10, T0 + milliseconds{4999}).has_value());
}

TEST(EchoTimeoutReport, AfterThePeriodTheSendsSinceTheLastWarningAreDue)
{
  EchoTimeoutReport report{PERIOD};
  ASSERT_TRUE(report.due(3, T0).has_value());
  EXPECT_FALSE(report.due(7, T0 + milliseconds{1000}).has_value());
  // Exactly the period is due, and with the 7 sends since the warning of 3.
  EXPECT_EQ(report.due(10, T0 + PERIOD), std::optional<std::uint64_t>{7});
}

TEST(EchoTimeoutReport, AfterTheResetANewBackendIsDueAtOnce)
{
  EchoTimeoutReport report{PERIOD};
  ASSERT_TRUE(report.due(10, T0).has_value());
  report.reset();
  EXPECT_EQ(report.due(1, T0 + milliseconds{1}), std::optional<std::uint64_t>{1});
}

TEST(EchoTimeoutReport, WithoutTheResetANewBackendStaysHidden)
{
  // What on_configure's reset() prevents: the new backend counts from zero.
  EchoTimeoutReport report{PERIOD};
  ASSERT_TRUE(report.due(10, T0).has_value());
  EXPECT_FALSE(report.due(1, T0 + 2 * PERIOD).has_value());
}

TEST(EchoTimeoutReport, TheDefaultPeriodIsFiveSeconds)
{
  EXPECT_EQ(EchoTimeoutReport::DEFAULT_PERIOD, milliseconds{5000});
}
