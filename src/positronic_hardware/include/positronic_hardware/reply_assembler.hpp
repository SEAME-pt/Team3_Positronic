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

#ifndef POSITRONIC_HARDWARE__REPLY_ASSEMBLER_HPP_
#define POSITRONIC_HARDWARE__REPLY_ASSEMBLER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_protocol/decode.hpp"
#include "positronic_protocol/types.hpp"
#include "positronic_protocol/units.hpp"

namespace positronic_hardware
{

/// Turns a stream of received bytes into the latest get_angles reply.
///
/// The UART is shared with the servo bus, so bytes arrive mixed with noise and a reply can be
/// split across several reads (see the capture: a single get_angles reply comes in over three
/// RXT bursts). This class owns the rolling buffer and the decode loop so the serial backend
/// does not: feed it whatever the port handed you with push(), and it returns the newest set of
/// joint angles whenever one completes.
///
/// It is pure: no port, no thread, no ROS. That is what lets the serial framing be unit-tested
/// against bytes recorded on the real arm, with no hardware. It runs in the serial thread, never
/// in the control loop, so a std::vector buffer is fine here.
class ReplyAssembler
{
public:
  /// Hard cap on the buffer. One get_angles reply plus a burst of servo-bus noise is well under
  /// this; past it we drop the oldest bytes rather than grow without bound on a silent link.
  static constexpr std::size_t MAX_BUFFER{512};

  /// Feeds `count` freshly received bytes. Returns the latest complete pose (radians) if one or
  /// more replies finished in this buffer, or std::nullopt when nothing completed yet.
  ///
  /// If several replies are already buffered, the most recent one wins: the control loop wants
  /// the freshest angles, not a backlog.
  std::optional<JointPositions> push(const std::uint8_t * data, std::size_t count)
  {
    buffer_.insert(buffer_.end(), data, data + count);

    std::optional<JointPositions> latest{};
    while (true) {
      const positronic_protocol::Decoded<positronic_protocol::JointAngles> decoded{
        positronic_protocol::decode_get_angles_reply(
          positronic_protocol::ByteView{buffer_.data(), buffer_.size()})};

      drop_front(decoded.consumed);

      if (decoded.status == positronic_protocol::DecodeStatus::FOUND) {
        latest = positronic_protocol::to_radians(decoded.value);
        continue;
      }
      break;
    }

    if (buffer_.size() > MAX_BUFFER) {
      drop_front(buffer_.size() - MAX_BUFFER);
    }
    return latest;
  }

  void clear()
  {
    buffer_.clear();
  }

private:
  void drop_front(std::size_t count)
  {
    const std::size_t drop{count < buffer_.size() ? count : buffer_.size()};
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(drop));
  }

  std::vector<std::uint8_t> buffer_{};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__REPLY_ASSEMBLER_HPP_
