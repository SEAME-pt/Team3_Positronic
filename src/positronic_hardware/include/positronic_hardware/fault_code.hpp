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

#ifndef POSITRONIC_HARDWARE__FAULT_CODE_HPP_
#define POSITRONIC_HARDWARE__FAULT_CODE_HPP_

#include <cstdint>
#include <string_view>

namespace positronic_hardware
{

/// What the driver reports in its fault state interface (#35), as a whole number in a double.
/// The supervisor (#45) stops the arm on any value other than NONE, so a new fault needs no
/// change there. The values are part of the interface: add new ones at the end, never reuse or
/// renumber one, and give each its line in describe_fault().
enum class FaultCode : std::uint8_t
{
  /// No fault: the link is alive (or the component is not active).
  NONE = 0,
  /// No valid reply from the arm for LinkWatchdog::DEFAULT_TIMEOUT while active. Latched until
  /// the next activate.
  LINK_TIMEOUT = 1,
  /// A joint further than tracking_error_deg from the last target on the wire for longer than
  /// tracking_error_ms while active (#103): the arm did not follow the command. Latched until
  /// the next activate, targets held meanwhile, like the link timeout.
  TRACKING_ERROR = 2,
  /// A servo stayed at or above servo_temp_limit_c for 1 s, no reading of it more than 1 degC
  /// under: the serial thread has already stopped the arm and powered its servos off. read()
  /// also returns ERROR. Latched until the next activate.
  OVERHEAT = 3,
  /// No heartbeat from the safety supervisor for supervisor_timeout_ms while active (#136): it
  /// died, hangs or was killed, and nobody would stop the arm. Targets held, like the link
  /// timeout, and read() returns ERROR, like the overheat: with no supervisor left to act on
  /// the fault, the controller manager takes the controllers and the component out of ACTIVE.
  /// Latched until the next activate.
  SUPERVISOR_LOST = 4,
};

/// The one table of the fault codes, as /diagnostics shows them (#121): the name and why the
/// arm stopped. No default: a new FaultCode without its line here does not compile. A value
/// outside the enum is "UNKNOWN", never dropped.
[[nodiscard]] constexpr std::string_view describe_fault(FaultCode code) noexcept
{
  switch (code) {
    case FaultCode::NONE:
      return "NONE: no fault";
    case FaultCode::LINK_TIMEOUT:
      return "LINK_TIMEOUT: no valid reply from the arm; targets held";
    case FaultCode::TRACKING_ERROR:
      return "TRACKING_ERROR: a joint did not follow its target; targets held";
    case FaultCode::OVERHEAT:
      return "OVERHEAT: a servo stayed at its temperature limit for 1 s; the arm was stopped and "
             "powered off";
    case FaultCode::SUPERVISOR_LOST:
      return "SUPERVISOR_LOST: no heartbeat from the safety supervisor; targets held";
  }
  return "UNKNOWN: unknown fault code";
}

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__FAULT_CODE_HPP_
