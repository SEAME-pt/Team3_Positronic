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

#ifndef POSITRONIC_PROTOCOL__ENCODE_HPP_
#define POSITRONIC_PROTOCOL__ENCODE_HPP_

#include <cstdint>
#include <optional>

#include "positronic_protocol/types.hpp"

namespace positronic_protocol
{

/// Request frame for power_on: `FE FE 02 10 FA`.
[[nodiscard]] Frame encode_power_on() noexcept;

/// Request frame for power_off: `FE FE 02 11 FA`.
[[nodiscard]] Frame encode_power_off() noexcept;

/// Request frame for stop: `FE FE 02 29 FA`.
[[nodiscard]] Frame encode_stop() noexcept;

/// Request frame for get_angles: `FE FE 02 20 FA`.
[[nodiscard]] Frame encode_get_angles() noexcept;

/// Request frame for send_angles: `FE FE 0F 22` + 6 x int16 big-endian + speed + `FA`.
///
/// @param angles Target joint angles in 0.01 deg.
/// @param speed Percent of the firmware maximum, from MIN_SPEED to MAX_SPEED.
/// @return The frame, or std::nullopt when `speed` is out of range.
[[nodiscard]] std::optional<Frame> encode_send_angles(
  const JointAngles & angles, std::uint8_t speed) noexcept;

}  // namespace positronic_protocol

#endif  // POSITRONIC_PROTOCOL__ENCODE_HPP_
