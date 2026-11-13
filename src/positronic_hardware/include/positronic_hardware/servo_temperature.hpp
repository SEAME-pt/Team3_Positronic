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

/// Readings in a row at or above the limit before the cut, so one corrupt byte cannot cut the
/// arm. At one reading a second that is two seconds at most, on a temperature that takes
/// minutes to climb.
inline constexpr int READINGS_TO_TRIP{2};

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
/// Fed one reading at a time by the serial thread. It trips once READINGS_TO_TRIP plausible
/// readings in a row have a servo at or above the limit, and stays tripped: cooling down does
/// not undo a cut, only a new backend (configure) does. An implausible reading changes nothing,
/// neither towards the cut nor away from it. Pure logic: no port, no clock, no thread.
class OverheatDetector
{
public:
  explicit OverheatDetector(int limit_c)
  : limit_c_{limit_c}
  {
  }

  /// One reading. The trip (hottest servo of the reading that completed it) once tripped,
  /// nullopt before.
  std::optional<Overheat> update(const ServoTemperatures & temperatures)
  {
    if (trip_.has_value() || !plausible(temperatures)) {
      return trip_;
    }
    const Overheat hot{hottest(temperatures)};
    hot_in_a_row_ = hot.celsius >= limit_c_ ? hot_in_a_row_ + 1 : 0;
    if (hot_in_a_row_ >= READINGS_TO_TRIP) {
      trip_ = hot;
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
  int hot_in_a_row_{0};
  std::optional<Overheat> trip_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__SERVO_TEMPERATURE_HPP_
