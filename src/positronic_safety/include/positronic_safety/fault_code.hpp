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
#include <string_view>

#include "control_msgs/msg/dynamic_joint_state.hpp"

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

}  // namespace positronic_safety

#endif  // POSITRONIC_SAFETY__FAULT_CODE_HPP_
