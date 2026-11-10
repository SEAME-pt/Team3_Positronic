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

#ifndef POSITRONIC_HARDWARE__JOINT_LIMIT_CHECK_HPP_
#define POSITRONIC_HARDWARE__JOINT_LIMIT_CHECK_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "positronic_hardware/arm_backend.hpp"

namespace positronic_hardware
{

/// Position limits of one joint, in radians. Both ends are allowed (inclusive).
struct PositionLimit
{
  double lower{0.0};
  double upper{0.0};
};

/// Position limits of the six joints, J1 (base) to J6 (flange), same order as JointPositions.
using PositionLimits = std::array<PositionLimit, NUM_JOINTS>;

/// Why a command was refused. NONE means the command may be sent to the arm.
enum class CommandFault : std::uint8_t
{
  NONE,
  WRONG_COUNT,
  NOT_FINITE,
  BELOW_LOWER,
  ABOVE_UPPER
};

/// Result of check_command(). When the command is refused, `joint` is the index (0 = J1) of the
/// first bad joint and `value` the bad value, so the caller can log "joint name + value".
/// For WRONG_COUNT, `joint` is 0 and `value` is the count that was received.
struct CommandCheck
{
  CommandFault fault{CommandFault::NONE};
  std::size_t joint{0};
  double value{0.0};

  [[nodiscard]] bool ok() const noexcept
  {
    return fault == CommandFault::NONE;
  }
};

/// A read-only view of a command: `size` doubles starting at `data`. C++17 has no std::span,
/// and a struct keeps the pointer and the size from being swapped by mistake at the call site.
struct CommandView
{
  const double * data{nullptr};
  std::size_t size{0};
};

/// Slack added to both ends of every limit, in radians. It only absorbs floating point noise
/// (a limit converted deg -> rad -> deg), it is NOT a safety margin: 1e-6 rad is about 6e-5 deg,
/// far below the 0.01 deg resolution of the protocol.
inline constexpr double LIMIT_TOLERANCE{1e-6};

/// Checks a full set of six position commands against the joint limits, before anything is
/// sent to the arm.
///
/// For each joint, J1 to J6, in that order, and the first failure wins: the value is finite
/// (NaN and +-inf are refused), then value >= lower - tolerance, then value <= upper + tolerance.
/// The finite check comes first because every comparison with NaN is false: a plain
/// "value < lower || value > upper" would let a NaN through.
///
/// It never clamps: a bad command is reported, not "fixed". Clamping silently would hide a bug
/// upstream (a planner or a controller asking for an impossible pose). Pure, ROS-free,
/// no allocation, no exception, so it is safe to call from write() on every cycle.
[[nodiscard]] inline CommandCheck check_command(
  const JointPositions & command, const PositionLimits & limits,
  double tolerance = LIMIT_TOLERANCE) noexcept
{
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    const double value{command.at(i)};
    const PositionLimit & limit{limits.at(i)};
    if (!std::isfinite(value)) {
      return CommandCheck{CommandFault::NOT_FINITE, i, value};
    }
    if (value < limit.lower - tolerance) {
      return CommandCheck{CommandFault::BELOW_LOWER, i, value};
    }
    if (value > limit.upper + tolerance) {
      return CommandCheck{CommandFault::ABOVE_UPPER, i, value};
    }
  }
  return CommandCheck{};
}

/// Same check for a command of unknown length (the "wrong joint count" case of the issue).
/// The count is checked first: anything but exactly NUM_JOINTS values, or a null pointer, is
/// WRONG_COUNT and no value is read. Otherwise the six values are checked as above.
[[nodiscard]] inline CommandCheck check_command(
  CommandView command, const PositionLimits & limits, double tolerance = LIMIT_TOLERANCE) noexcept
{
  if (command.size != NUM_JOINTS || command.data == nullptr) {
    return CommandCheck{CommandFault::WRONG_COUNT, 0, static_cast<double>(command.size)};
  }
  JointPositions values{};
  std::copy_n(command.data, NUM_JOINTS, values.begin());
  return check_command(values, limits, tolerance);
}

/// True when one limit is usable: both ends finite and lower < upper.
[[nodiscard]] inline bool limit_valid(const PositionLimit & limit) noexcept
{
  const bool finite{std::isfinite(limit.lower) && std::isfinite(limit.upper)};
  return finite && limit.lower < limit.upper;
}

/// True when every limit is usable (see limit_valid). Call it once in on_init: a missing or
/// swapped <limit> in the URDF must stop the driver, not let it run with a check that refuses
/// everything (or nothing).
[[nodiscard]] inline bool limits_valid(const PositionLimits & limits) noexcept
{
  return std::all_of(limits.begin(), limits.end(), limit_valid);
}

/// Short, stable text for a fault, for the log line.
[[nodiscard]] inline const char * to_string(CommandFault fault) noexcept
{
  switch (fault) {
    case CommandFault::NONE:
      return "ok";
    case CommandFault::WRONG_COUNT:
      return "wrong joint count";
    case CommandFault::NOT_FINITE:
      return "not a finite number";
    case CommandFault::BELOW_LOWER:
      return "below lower limit";
    case CommandFault::ABOVE_UPPER:
      return "above upper limit";
  }
  return "unknown fault";
}

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__JOINT_LIMIT_CHECK_HPP_
