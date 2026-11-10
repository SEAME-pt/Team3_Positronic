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

#ifndef POSITRONIC_HARDWARE__LINK_WATCHDOG_HPP_
#define POSITRONIC_HARDWARE__LINK_WATCHDOG_HPP_

#include <atomic>
#include <chrono>

namespace positronic_hardware
{

/// Detects a silent serial link (#35): no valid reply from the arm for longer than a timeout.
///
/// Two threads use it. The serial thread calls on_reply() each time it decodes a reply, and the
/// control loop calls timed_out() on every read(). The time of the last reply is one atomic
/// integer, so neither side ever waits for the other.
///
/// The caller passes the current time into every method instead of the watchdog reading the
/// clock itself. That keeps it pure logic: a unit test hands it made-up time points and needs no
/// sleeps and no fake clock.
///
/// It does not latch. timed_out() turns false again once a reply arrives; the plugin decides
/// whether a timeout stays a fault until someone re-activates the driver.
class LinkWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  /// No valid reply for this long is a fault (#35). One lost reply already means about 160-170 ms
  /// of silence while the arm moves: the rest of the turn after the last reply (up to the 50 ms
  /// moving period), a turn with no reply, which ends only when its read gives up after VTIME
  /// (100 ms, see PosixSerialPort), then the next reply (8 ms typical, 20 ms worst seen). So
  /// 200 ms tolerates one lost reply and faults on the second one in a row.
  static constexpr std::chrono::milliseconds DEFAULT_TIMEOUT{200};

  explicit LinkWatchdog(std::chrono::milliseconds timeout = DEFAULT_TIMEOUT)
  : timeout_{timeout}
  {
  }

  /// Starts counting from `now`, as if a reply had just arrived. Call it when the link is
  /// (re)started, so an arm that never answers also times out after `timeout`.
  void reset(Clock::time_point now)
  {
    on_reply(now);
  }

  /// A valid reply arrived at `now`. Called from the serial thread.
  void on_reply(Clock::time_point now)
  {
    last_reply_.store(now.time_since_epoch().count());
  }

  /// How long the link has been silent at `now`. Called from the control loop.
  [[nodiscard]] Clock::duration silence(Clock::time_point now) const
  {
    const Clock::time_point last{Clock::duration{last_reply_.load()}};
    return now - last;
  }

  /// True when the link has been silent for longer than the timeout at `now`.
  [[nodiscard]] bool timed_out(Clock::time_point now) const
  {
    return silence(now) > timeout_;
  }

  [[nodiscard]] std::chrono::milliseconds timeout() const
  {
    return timeout_;
  }

private:
  // The control loop must never block on a mutex, so the counter must be a real atomic, not one
  // the compiler emulates with a lock.
  static_assert(std::atomic<Clock::rep>::is_always_lock_free);

  const std::chrono::milliseconds timeout_;
  std::atomic<Clock::rep> last_reply_{0};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__LINK_WATCHDOG_HPP_
