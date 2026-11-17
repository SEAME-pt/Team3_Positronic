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

#ifndef POSITRONIC_SAFETY__DEADLINE_WATCHDOG_HPP_
#define POSITRONIC_SAFETY__DEADLINE_WATCHDOG_HPP_

#include <chrono>
#include <optional>

namespace positronic_safety
{

/// Tells when something that must keep happening has stopped (#45). The supervisor has three:
/// the mission heartbeat, the driver's fault data, and the answer to the request in flight.
///
/// feed() records that it happened. expired() is true when it has not happened for longer than
/// the timeout, and also when it has never happened: a heartbeat that never came is as bad as
/// one that stopped.
///
/// The caller passes the current time into every method instead of the watchdog reading the
/// clock itself (the same idea as LinkWatchdog in positronic_hardware). That keeps it pure
/// logic: a unit test hands it made-up time points and needs no sleeps and no fake clock.
///
/// Not thread-safe on purpose: the supervisor runs every callback on one thread.
class DeadlineWatchdog
{
public:
  /// A clock that only moves forward. The system clock can jump (NTP, someone setting the
  /// date), and a jump must neither raise a false timeout nor hide a real one.
  using Clock = std::chrono::steady_clock;

  explicit DeadlineWatchdog(std::chrono::milliseconds timeout)
  : timeout_{timeout}
  {
  }

  /// It happened at `now`.
  void feed(Clock::time_point now)
  {
    last_ = now;
  }

  /// Forgets every feed(): expired() stays true until the next one.
  void clear()
  {
    last_.reset();
  }

  /// True when it has not happened for longer than the timeout at `now`, or never happened.
  [[nodiscard]] bool expired(Clock::time_point now) const
  {
    return !last_.has_value() || now - *last_ > timeout_;
  }

  /// How long ago it last happened at `now`. Empty when it never happened.
  [[nodiscard]] std::optional<Clock::duration> silence(Clock::time_point now) const
  {
    if (!last_.has_value()) {
      return std::nullopt;
    }
    return now - *last_;
  }

  [[nodiscard]] std::chrono::milliseconds timeout() const
  {
    return timeout_;
  }

private:
  std::chrono::milliseconds timeout_;
  std::optional<Clock::time_point> last_;
};

}  // namespace positronic_safety

#endif  // POSITRONIC_SAFETY__DEADLINE_WATCHDOG_HPP_
