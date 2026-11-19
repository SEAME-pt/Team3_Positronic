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

#ifndef POSITRONIC_HARDWARE__ECHO_TIMEOUT_REPORT_HPP_
#define POSITRONIC_HARDWARE__ECHO_TIMEOUT_REPORT_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace positronic_hardware
{

/// Decides when read() warns about the sends that got no sync-write from the arm (#150).
///
/// The backend only counts them (ArmBackend::echo_timeouts()). A target the arm refuses comes
/// back every turn, so one warning per turn would flood the log: this keeps the count already
/// reported and the time of the last warning, and says when a new one is due, at most one per
/// period, with how many sends since the last one.
///
/// The caller passes the current time, as with LinkWatchdog, so a unit test hands it made-up
/// time points and needs no sleeps.
class EchoTimeoutReport
{
public:
  using Clock = std::chrono::steady_clock;

  /// At most one warning every 5 s.
  static constexpr std::chrono::milliseconds DEFAULT_PERIOD{5000};

  explicit EchoTimeoutReport(std::chrono::milliseconds period = DEFAULT_PERIOD)
  : period_{period}
  {
  }

  /// How many sends to warn about now, given the backend's count: nullopt when none is new, or
  /// when the last warning is younger than the period. A due report is taken: the next one
  /// counts from here.
  [[nodiscard]] std::optional<std::uint64_t> due(std::uint64_t count, Clock::time_point now)
  {
    if (count <= reported_) {
      return std::nullopt;
    }
    if (next_.has_value() && now < next_.value()) {
      return std::nullopt;
    }
    const std::uint64_t fresh{count - reported_};
    reported_ = count;
    next_ = now + period_;
    return fresh;
  }

  /// Forgets what was reported, for a new backend: its count starts at zero again, and without
  /// this every warning would stay hidden until it passed the old backend's count.
  void reset()
  {
    reported_ = 0;
    next_.reset();
  }

private:
  std::chrono::milliseconds period_;
  std::uint64_t reported_{0};
  std::optional<Clock::time_point> next_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__ECHO_TIMEOUT_REPORT_HPP_
