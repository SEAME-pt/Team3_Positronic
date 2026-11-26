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

#ifndef POSITRONIC_SAFETY__FAULT_CODE_HPP_
#define POSITRONIC_SAFETY__FAULT_CODE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace positronic_safety
{

/// Finds the driver's fault code (#35) in a /dynamic_joint_states message: the value of the
/// state interface `interface` of the entry `gpio` ("driver" and "fault_code" on our arm).
///
/// Empty when the message does not carry a usable code: the gpio or the interface is not
/// there, the lists do not match in length, or the value is NaN or infinite. The supervisor
/// treats "no usable code" as bad data: it does not count as news from the driver, so when it
/// lasts longer than the fault timeout the arm is halted.
[[nodiscard]] inline std::optional<double> find_fault_code(
  const control_msgs::msg::DynamicJointState & message, std::string_view gpio,
  std::string_view interface)
{
  // joint_names[i] goes with interface_values[i]; never read past the shorter list.
  const std::size_t entries = std::min(message.joint_names.size(), message.interface_values.size());
  for (std::size_t i = 0; i < entries; ++i) {
    if (message.joint_names[i] != gpio) {
      continue;
    }
    const auto & entry = message.interface_values[i];
    const std::size_t count = std::min(entry.interface_names.size(), entry.values.size());
    for (std::size_t j = 0; j < count; ++j) {
      if (entry.interface_names[j] == interface && std::isfinite(entry.values[j])) {
        return entry.values[j];
      }
    }
  }
  return std::nullopt;
}

/// Why the driver faulted, from a /diagnostics message (#121): the message of the status of the
/// hardware component `hardware_id` when its level is not OK, and "" when it is OK. Empty when
/// the message has no status of that component: the controller manager publishes there too, as
/// "ros2_control", and a starting status has no hardware id.
[[nodiscard]] inline std::optional<std::string> find_fault_reason(
  const diagnostic_msgs::msg::DiagnosticArray & message, std::string_view hardware_id)
{
  for (const auto & status : message.status) {
    if (status.hardware_id != hardware_id) {
      continue;
    }
    if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      return std::string{};
    }
    return status.message;
  }
  return std::nullopt;
}

}  // namespace positronic_safety

#endif  // POSITRONIC_SAFETY__FAULT_CODE_HPP_
