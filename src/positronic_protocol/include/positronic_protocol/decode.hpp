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

#ifndef POSITRONIC_PROTOCOL__DECODE_HPP_
#define POSITRONIC_PROTOCOL__DECODE_HPP_

#include <cstdint>

#include "positronic_protocol/types.hpp"

namespace positronic_protocol
{

/// Finds the get_angles reply (`FE FE 0E 20` + 6 x int16 big-endian + `FA`) in `rx`.
///
/// `rx` may start with noise (the servo bus traffic shares the UART) and may end in the
/// middle of the reply. The search moves one byte at a time and never reads outside `rx`.
///
/// - FOUND: `value` holds the six angles in 0.01 deg; `consumed` points past the reply.
/// - NEED_MORE_DATA: `rx` ends with the start of a reply; `consumed` drops the noise before it.
/// - NOT_FOUND: no reply and no partial reply; `consumed` equals `rx.size`.
[[nodiscard]] Decoded<JointAngles> decode_get_angles_reply(ByteView rx) noexcept;

/// Finds the one-byte reply to `command` (`FE FE 03 CMD STATUS FA`) in `rx`.
///
/// Used for power_on, power_off, send_angles and stop. `value` is the status byte
/// (1 in every capture so far). Same search and `consumed` rules as decode_get_angles_reply().
[[nodiscard]] Decoded<std::uint8_t> decode_ack(ByteView rx, CommandId command) noexcept;

}  // namespace positronic_protocol

#endif  // POSITRONIC_PROTOCOL__DECODE_HPP_
