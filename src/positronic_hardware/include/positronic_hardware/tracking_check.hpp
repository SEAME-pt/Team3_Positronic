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

#ifndef POSITRONIC_HARDWARE__TRACKING_CHECK_HPP_
#define POSITRONIC_HARDWARE__TRACKING_CHECK_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>

#include "positronic_hardware/arm_backend.hpp"

namespace positronic_hardware
{

/// Defaults of the unfollowed-command fault (#103): a joint this far from its target for this
/// long. 8 deg is wider than the controller's 5 deg trajectory tolerance and than the lag seen
/// on the arm (4.9 deg at 20 deg/s), so a healthy move never trips it and the JTC aborts first.
inline constexpr double TRACKING_ERROR_DEG{8.0};
inline constexpr int TRACKING_ERROR_MS{300};

/// What the plugin accepts for tracking_error_deg and tracking_error_ms.
inline constexpr double MIN_TRACKING_ERROR_DEG{1.0};
inline constexpr double MAX_TRACKING_ERROR_DEG{45.0};
inline constexpr int MIN_TRACKING_ERROR_MS{50};
inline constexpr int MAX_TRACKING_ERROR_MS{5000};

inline constexpr double TRACKING_DEG_TO_RAD{3.14159265358979323846 / 180.0};

/// The threshold and the time of the check, in the units the check works in.
struct TrackingLimits
{
  double threshold_rad{TRACKING_ERROR_DEG * TRACKING_DEG_TO_RAD};
  std::chrono::milliseconds time{TRACKING_ERROR_MS};
};

/// The joint that tripped (0 = J1) and how far it was, target minus measured, radians.
struct TrackingError
{
  std::size_t joint{0};
  double error_rad{0.0};
};

/// Watches, per joint, how long the measured pose has been further than the threshold from the
/// last target on the wire. One call per control cycle; the first joint over the time wins.
///
/// A joint back inside the threshold forgets its clock: a lag that comes and goes under the
/// time is the normal lag of a move, not a fault. A NaN on either side counts as inside: a pose
/// that is not known yet cannot be a pose that was not followed.
class TrackingCheck
{
public:
  using Clock = std::chrono::steady_clock;

  TrackingCheck() = default;
  explicit TrackingCheck(TrackingLimits limits)
  : limits_{limits}
  {
  }

  /// `target` is the last send_angles that reached the serial line, `measured` the pose read
  /// this cycle, `now` the time of the cycle. The error once a joint has been over the threshold
  /// for the time, nullopt otherwise.
  [[nodiscard]] std::optional<TrackingError> update(
    const JointPositions & target, const JointPositions & measured, Clock::time_point now)
  {
    for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
      const double error{target.at(joint) - measured.at(joint)};
      std::optional<Clock::time_point> & since{since_.at(joint)};
      if (!std::isfinite(error) || std::abs(error) <= limits_.threshold_rad) {
        since.reset();
        continue;
      }
      if (!since.has_value()) {
        since = now;
      } else if (now - *since >= limits_.time) {
        return TrackingError{joint, error};
      }
    }
    return std::nullopt;
  }

  /// Forgets every clock: after an activate, or when there is no target on the wire.
  void reset()
  {
    since_.fill(std::nullopt);
  }

  [[nodiscard]] const TrackingLimits & limits() const
  {
    return limits_;
  }

private:
  TrackingLimits limits_{};
  std::array<std::optional<Clock::time_point>, NUM_JOINTS> since_{};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__TRACKING_CHECK_HPP_
