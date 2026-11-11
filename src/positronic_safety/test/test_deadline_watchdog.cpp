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
//   Write a GoogleTest suite for deadline_watchdog.hpp (issue #45), in the style of
//   test_link_watchdog.cpp. DeadlineWatchdog takes the current time as an argument, so use
//   made-up time points, no sleeps. Check that: a watchdog that was never fed is expired; it is
//   not expired right after a feed nor exactly at the timeout, and is expired 1 ms later; a
//   feed restarts the count; it does not latch; clear() forgets the feeds; silence() is the
//   time since the last feed and empty when there was none; a watchdog can be replaced by one
//   with another timeout (the node does that when it reads its parameters).

#include <gtest/gtest.h>

#include <chrono>

#include "positronic_safety/deadline_watchdog.hpp"

namespace
{
using positronic_safety::DeadlineWatchdog;
using std::chrono::milliseconds;

// Any fixed point works: the watchdog only looks at differences between time points.
const DeadlineWatchdog::Clock::time_point T0{std::chrono::seconds{100}};
}  // namespace

TEST(DeadlineWatchdog, NeverFedIsExpired)
{
  // A heartbeat that never came is as bad as one that stopped.
  const DeadlineWatchdog watchdog{milliseconds{200}};
  EXPECT_TRUE(watchdog.expired(T0));
}

TEST(DeadlineWatchdog, NotExpiredRightAfterAFeed)
{
  DeadlineWatchdog watchdog{milliseconds{200}};
  watchdog.feed(T0);
  EXPECT_FALSE(watchdog.expired(T0));
  EXPECT_FALSE(watchdog.expired(T0 + milliseconds{199}));
}

TEST(DeadlineWatchdog, ExactlyTheTimeoutIsNotYetExpired)
{
  DeadlineWatchdog watchdog{milliseconds{200}};
  watchdog.feed(T0);
  EXPECT_FALSE(watchdog.expired(T0 + milliseconds{200}));
  EXPECT_TRUE(watchdog.expired(T0 + milliseconds{201}));
}

TEST(DeadlineWatchdog, FeedRestartsTheCount)
{
  DeadlineWatchdog watchdog{milliseconds{200}};
  watchdog.feed(T0);
  watchdog.feed(T0 + milliseconds{150});
  EXPECT_FALSE(watchdog.expired(T0 + milliseconds{300}));
  EXPECT_TRUE(watchdog.expired(T0 + milliseconds{351}));
}

TEST(DeadlineWatchdog, DoesNotLatch)
{
  // Staying stopped is the supervisor's decision (SupervisorLogic), not the watchdog's.
  DeadlineWatchdog watchdog{milliseconds{200}};
  watchdog.feed(T0);
  ASSERT_TRUE(watchdog.expired(T0 + milliseconds{300}));
  watchdog.feed(T0 + milliseconds{300});
  EXPECT_FALSE(watchdog.expired(T0 + milliseconds{310}));
}

TEST(DeadlineWatchdog, ClearForgetsTheFeeds)
{
  DeadlineWatchdog watchdog{milliseconds{200}};
  watchdog.feed(T0);
  watchdog.clear();
  EXPECT_TRUE(watchdog.expired(T0));
  EXPECT_FALSE(watchdog.silence(T0).has_value());
}

TEST(DeadlineWatchdog, SilenceIsTimeSinceTheLastFeed)
{
  DeadlineWatchdog watchdog{milliseconds{200}};
  watchdog.feed(T0);
  watchdog.feed(T0 + milliseconds{40});
  // An optional equals a value only when it holds one and that one is equal.
  EXPECT_TRUE(watchdog.silence(T0 + milliseconds{100}) == milliseconds{60});
}

TEST(DeadlineWatchdog, SilenceIsEmptyWhenNeverFed)
{
  const DeadlineWatchdog watchdog{milliseconds{200}};
  EXPECT_FALSE(watchdog.silence(T0).has_value());
}

TEST(DeadlineWatchdog, CanBeReplacedWithAnotherTimeout)
{
  // The node builds its watchdogs again in on_configure, from its parameters.
  DeadlineWatchdog watchdog{milliseconds{200}};
  watchdog.feed(T0);
  watchdog = DeadlineWatchdog{milliseconds{50}};
  EXPECT_EQ(watchdog.timeout(), milliseconds{50});
  EXPECT_TRUE(watchdog.expired(T0));  // the feed went with the old one
  watchdog.feed(T0);
  EXPECT_FALSE(watchdog.expired(T0 + milliseconds{50}));
  EXPECT_TRUE(watchdog.expired(T0 + milliseconds{51}));
}
