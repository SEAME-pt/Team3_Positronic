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
//   Write a GoogleTest suite for link_watchdog.hpp (#35). The watchdog takes the current time
//   as an argument. Check that: it is not timed out right after reset(); it times out when no
//   reply comes for longer than the timeout; exactly the timeout is not yet a fault; a reply
//   restarts the count; it does not latch (a late reply clears the timeout); silence() reports
//   the time since the last reply; and the default timeout is the 200 ms of #35.

#include <gtest/gtest.h>

#include <chrono>

#include "positronic_hardware/link_watchdog.hpp"

namespace
{
using positronic_hardware::LinkWatchdog;
using std::chrono::milliseconds;

// Any fixed point works: the watchdog only looks at differences between time points.
const LinkWatchdog::Clock::time_point T0{std::chrono::seconds{100}};
}  // namespace

TEST(LinkWatchdog, DefaultTimeoutIs200Ms)
{
  const LinkWatchdog watchdog{};
  EXPECT_EQ(watchdog.timeout(), milliseconds{200});
}

TEST(LinkWatchdog, NotTimedOutRightAfterReset)
{
  LinkWatchdog watchdog{milliseconds{200}};
  watchdog.reset(T0);
  EXPECT_FALSE(watchdog.timed_out(T0));
  EXPECT_FALSE(watchdog.timed_out(T0 + milliseconds{199}));
}

TEST(LinkWatchdog, ExactlyTheTimeoutIsNotYetAFault)
{
  LinkWatchdog watchdog{milliseconds{200}};
  watchdog.reset(T0);
  EXPECT_FALSE(watchdog.timed_out(T0 + milliseconds{200}));
  EXPECT_TRUE(watchdog.timed_out(T0 + milliseconds{201}));
}

TEST(LinkWatchdog, ArmThatNeverAnswersTimesOut)
{
  // reset() at activation, then no reply at all.
  LinkWatchdog watchdog{milliseconds{200}};
  watchdog.reset(T0);
  EXPECT_TRUE(watchdog.timed_out(T0 + milliseconds{500}));
}

TEST(LinkWatchdog, ReplyRestartsTheCount)
{
  LinkWatchdog watchdog{milliseconds{200}};
  watchdog.reset(T0);
  watchdog.on_reply(T0 + milliseconds{150});
  EXPECT_FALSE(watchdog.timed_out(T0 + milliseconds{300}));
  EXPECT_TRUE(watchdog.timed_out(T0 + milliseconds{351}));
}

TEST(LinkWatchdog, DoesNotLatch)
{
  // Latching is the plugin's decision, not the watchdog's.
  LinkWatchdog watchdog{milliseconds{200}};
  watchdog.reset(T0);
  ASSERT_TRUE(watchdog.timed_out(T0 + milliseconds{300}));
  watchdog.on_reply(T0 + milliseconds{300});
  EXPECT_FALSE(watchdog.timed_out(T0 + milliseconds{310}));
}

TEST(LinkWatchdog, SilenceIsTimeSinceLastReply)
{
  LinkWatchdog watchdog{milliseconds{200}};
  watchdog.reset(T0);
  watchdog.on_reply(T0 + milliseconds{40});
  EXPECT_EQ(watchdog.silence(T0 + milliseconds{100}), milliseconds{60});
}
