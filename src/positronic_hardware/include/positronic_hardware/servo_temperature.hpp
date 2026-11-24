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

#ifndef POSITRONIC_HARDWARE__SERVO_TEMPERATURE_HPP_
#define POSITRONIC_HARDWARE__SERVO_TEMPERATURE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "positronic_hardware/arm_backend.hpp"

namespace positronic_hardware
{

/// A servo at or above this temperature stops the arm and cuts its power: the servos of the 280
/// should never get there in normal use, so reaching it means something is badly wrong (a joint
/// pushing against an obstacle or holding a load it cannot). 10 degC under the 70 degC at which
/// Feetech-family servos turn their own torque off, so the arm is cut by the driver, not by
/// one servo giving up on its own. The plugin may lower it (servo_temp_limit_c), never raise it.
inline constexpr int SERVO_TEMP_LIMIT_C{60};

/// The lowest limit the plugin accepts. Below it a cool arm would be cut at once.
inline constexpr int MIN_SERVO_TEMP_LIMIT_C{30};

/// Activation is refused while any servo is within this many degC of the limit: an arm that
/// just cooled to one degree under it would be cut again a minute after the power on.
inline constexpr int ACTIVATE_COOLDOWN_C{5};

/// Plausible readings, degC. 0 is what an unanswered slot looks like, and nothing above 99 can
/// come from a servo that still answers: a reading outside this range is ignored, never acted on.
inline constexpr int MIN_PLAUSIBLE_C{1};
inline constexpr int MAX_PLAUSIBLE_C{99};

/// How long a servo must stay at or above the limit, with no reading more than LIMIT_BAND_C
/// under it, before the cut. J1 gives false hot readings, each one alone between normal ones:
/// up to 98 degC in the frames of the poll (#170, docs/03_verification/write_sweep), 45 between
/// two of 34 when asked (2026-11-13). The first hot reading only starts the clock, so one alone
/// never cuts; at rest, one request a second, the cut is two hot readings in a row. Under load
/// J4 climbed 10 degC in about a minute (20 and 21 Nov 2026): one second is about 0.2 degC.
inline constexpr std::chrono::milliseconds HOT_FOR{1000};

/// A servo crossing a degree reads n and n + 1 in turn for seconds, so a reading this far under
/// the limit neither starts nor stops the clock: only a lower one stops it. With every reading
/// under the limit stopping it, J6 crossing 38 to 39 (20 Nov 2026) would have been cut 19.3 s
/// after its first 39; with this, 1.5 s. A servo just under the limit may be cut up to this
/// much early.
inline constexpr int LIMIT_BAND_C{1};

/// True when one reading can be a servo temperature.
[[nodiscard]] inline bool plausible_celsius(std::uint8_t celsius)
{
  return celsius >= MIN_PLAUSIBLE_C && celsius <= MAX_PLAUSIBLE_C;
}

/// True when all six readings are plausible temperatures.
[[nodiscard]] inline bool plausible(const ServoTemperatures & temperatures)
{
  return std::all_of(temperatures.begin(), temperatures.end(), plausible_celsius);
}

/// The hottest servo: the first one, J1 side, on a tie.
[[nodiscard]] inline Overheat hottest(const ServoTemperatures & temperatures)
{
  Overheat hot{};
  std::size_t joint{0};
  for (const std::uint8_t celsius : temperatures) {
    if (joint == 0 || celsius > hot.celsius) {
      hot = Overheat{joint, celsius};
    }
    ++joint;
  }
  return hot;
}

/// True when every servo is more than ACTIVATE_COOLDOWN_C under `limit_c`.
[[nodiscard]] inline bool cool_enough_to_activate(
  const ServoTemperatures & temperatures, int limit_c)
{
  return hottest(temperatures).celsius < limit_c - ACTIVATE_COOLDOWN_C;
}

/// Decides when the servo temperatures must cut the arm.
///
/// Fed one reading at a time by the serial thread, with the time it was taken. Each servo has
/// its own clock: its first reading at the limit starts it, one more than LIMIT_BAND_C under the
/// limit stops it, and a reading at the limit `hot_for` after the start trips. One clock for the
/// hottest servo would let a servo held in the band keep alive a clock that another servo's
/// false readings started. It stays tripped: cooling down does not undo a cut, only a new
/// backend (configure) does. An implausible reading changes nothing, neither towards the cut nor
/// away from it. Pure logic: no port, no clock of its own, no thread.
class OverheatDetector
{
public:
  using Clock = std::chrono::steady_clock;

  explicit OverheatDetector(int limit_c, std::chrono::milliseconds hot_for = HOT_FOR)
  : limit_c_{limit_c},
    hot_for_{hot_for}
  {
  }

  /// One reading, taken at `at`. The trip (the servo whose clock ran out, J1 side first, at its
  /// reading then) once tripped, nullopt before.
  std::optional<Overheat> update(const ServoTemperatures & temperatures, Clock::time_point at)
  {
    if (trip_.has_value() || !plausible(temperatures)) {
      return trip_;
    }
    for (std::size_t joint{0}; joint < temperatures.size(); ++joint) {
      const int celsius{temperatures.at(joint)};
      std::optional<Clock::time_point> & since{hot_since_.at(joint)};
      // A reading in the band changes nothing: a servo on the limit toggles between the two.
      const bool at_limit{celsius >= limit_c_};
      if (celsius < limit_c_ - LIMIT_BAND_C) {
        since.reset();
      } else if (at_limit && since.has_value()) {
        if (at - since.value() >= hot_for_ && !trip_.has_value()) {
          trip_ = Overheat{joint, celsius};
        }
      } else if (at_limit) {
        since = at;
      }
    }
    return trip_;
  }

  [[nodiscard]] std::optional<Overheat> tripped() const
  {
    return trip_;
  }

  [[nodiscard]] int limit_c() const
  {
    return limit_c_;
  }

private:
  int limit_c_;
  std::chrono::milliseconds hot_for_;
  // Per servo, when its first reading at the limit came, since its last one under the band.
  std::array<std::optional<Clock::time_point>, NUM_JOINTS> hot_since_{};
  std::optional<Overheat> trip_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__SERVO_TEMPERATURE_HPP_
