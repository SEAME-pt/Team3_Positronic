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

#ifndef POSITRONIC_HARDWARE__PARAMETERS_HPP_
#define POSITRONIC_HARDWARE__PARAMETERS_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include "positronic_hardware/fake_arm.hpp"
#include "positronic_hardware/load_relief.hpp"
#include "positronic_hardware/serial_arm.hpp"
#include "positronic_hardware/servo_temperature.hpp"
#include "positronic_hardware/tracking_check.hpp"
#include "positronic_hardware/velocity_estimator.hpp"

namespace positronic_hardware
{

/// Where the joint angles come from: the arm on the serial port, or FakeArm.
enum class Backend : std::uint8_t
{
  SERIAL,
  FAKE
};

/// The dead-man check (#136): no heartbeat from the safety supervisor for this long while active
/// halts the arm (FaultCode::SUPERVISOR_LOST). 500 ms is 25 heartbeats at the supervisor's 50 Hz,
/// both nodes on the Jetson. 0 turns the check off, for a run without the supervisor.
inline constexpr int SUPERVISOR_TIMEOUT_MS{500};
inline constexpr int MAX_SUPERVISOR_TIMEOUT_MS{5000};

/// The <param> values of the <hardware> tag, checked in on_init. A parameter the URDF does not
/// set keeps its default.
struct HardwareConfig
{
  Backend backend{Backend::SERIAL};
  /// How FakeArm moves (#122). Only the fake backend reads it.
  FakeMode fake_mode{FakeMode::INSTANT};
  std::string port{"/dev/ttyTHS1"};
  int baud{1000000};
  double velocity_filter_alpha{VelocityEstimator::DEFAULT_ALPHA};
  SerialArm::Timing timing{};
  int servo_temp_limit_c{SERVO_TEMP_LIMIT_C};
  double tracking_error_deg{TRACKING_ERROR_DEG};
  int tracking_error_ms{TRACKING_ERROR_MS};
  int supervisor_timeout_ms{SUPERVISOR_TIMEOUT_MS};
  LoadReliefLimits relief{};
  /// Speed byte of every send_angles, 1 to 100: a speed cap, see ADR 0008.
  int send_speed{SerialArm::DEFAULT_SEND_SPEED};
};

/// The config read from the URDF, or the reason it was refused: exactly one of the two is set.
struct ParsedParameters
{
  std::optional<HardwareConfig> config;
  std::string error;
};

/// Reads the <param> values of the <hardware> tag into a HardwareConfig (#163). Refused on a
/// name the driver does not know, or on a value out of its range: a typo in the URDF must stop
/// the driver, not be ignored. Pure, no logging: the plugin logs the result.
[[nodiscard]] ParsedParameters parse_hardware_parameters(
  const std::unordered_map<std::string, std::string> & parameters);

/// One line with every value of `config`, for the INFO log of on_init.
[[nodiscard]] std::string describe(const HardwareConfig & config);

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__PARAMETERS_HPP_
