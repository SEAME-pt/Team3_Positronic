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

#include "positronic_protocol/encode.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>

#include "positronic_protocol/types.hpp"

namespace positronic_protocol
{

namespace
{

/// Bytes around the data: FE FE LEN CMD before it, FA after it.
constexpr std::size_t FRAME_OVERHEAD{5};

/// Bytes that LEN does not count: FE FE LEN.
constexpr std::size_t BYTES_NOT_IN_LEN{3};

/// Where the data starts: after FE FE LEN CMD. Signed because std::next takes a distance.
constexpr std::ptrdiff_t DATA_OFFSET{4};

/// Data of send_angles: 6 x int16 + 1 speed byte.
using SendAnglesData = std::array<std::uint8_t, 2 * NUM_JOINTS + 1>;

/// Builds `FE FE LEN CMD data FA`.
///
/// N is known at compile time, so the static_assert proves the frame fits in Frame::bytes
/// and std::get<> checks every fixed index. Nothing can fail at run time, so there is no
/// error path to return (and no dead branch to cover).
template<std::size_t N>
Frame build_frame(CommandId command, const std::array<std::uint8_t, N> & data) noexcept
{
  constexpr std::size_t FRAME_SIZE{N + FRAME_OVERHEAD};
  static_assert(FRAME_SIZE <= MAX_FRAME_SIZE, "the frame does not fit in Frame::bytes");

  Frame frame{};
  std::get<0>(frame.bytes) = HEADER;
  std::get<1>(frame.bytes) = HEADER;
  std::get<2>(frame.bytes) = static_cast<std::uint8_t>(FRAME_SIZE - BYTES_NOT_IN_LEN);
  std::get<3>(frame.bytes) = static_cast<std::uint8_t>(command);
  std::copy(data.begin(), data.end(), std::next(frame.bytes.begin(), DATA_OFFSET));
  std::get<FRAME_SIZE - 1>(frame.bytes) = FOOTER;
  frame.size = FRAME_SIZE;
  return frame;
}

/// Six angles as int16 big-endian (high byte first), then the speed.
///
/// No memcpy: the Jetson (ARM) and the PC (x86) are little-endian, so copying the
/// memory would put the low byte first. Shifts give the same bytes on every CPU.
SendAnglesData send_angles_data(const JointAngles & angles, std::uint8_t speed) noexcept
{
  static_assert(std::tuple_size<SendAnglesData>::value == 2 * NUM_JOINTS + 1);
  SendAnglesData data{};
  // Exactly 2 bytes per joint, and the array has room for 2 per joint plus the speed,
  // so the iterator stops before the last byte.
  SendAnglesData::iterator out{data.begin()};
  for (const auto angle : angles) {
    // Through uint16_t: -61 becomes 0xFFC3 (two's complement) without shifting a
    // negative number, which is undefined behaviour before C++20.
    const auto raw{static_cast<std::uint16_t>(angle)};
    *out = static_cast<std::uint8_t>(raw >> 8U);
    ++out;
    *out = static_cast<std::uint8_t>(raw & 0xFFU);
    ++out;
  }
  std::get<2 * NUM_JOINTS>(data) = speed;
  return data;
}

}  // namespace

Frame encode_power_on() noexcept
{
  return build_frame(CommandId::POWER_ON, std::array<std::uint8_t, 0>{});
}

Frame encode_power_off() noexcept
{
  return build_frame(CommandId::POWER_OFF, std::array<std::uint8_t, 0>{});
}

Frame encode_release_all_servos() noexcept
{
  return build_frame(CommandId::RELEASE_ALL_SERVOS, std::array<std::uint8_t, 0>{});
}

Frame encode_stop() noexcept
{
  return build_frame(CommandId::STOP, std::array<std::uint8_t, 0>{});
}

Frame encode_get_angles() noexcept
{
  return build_frame(CommandId::GET_ANGLES, std::array<std::uint8_t, 0>{});
}

std::optional<Frame> encode_send_angles(const JointAngles & angles, std::uint8_t speed) noexcept
{
  if (speed < MIN_SPEED || speed > MAX_SPEED) {
    return std::nullopt;
  }
  return build_frame(CommandId::SEND_ANGLES, send_angles_data(angles, speed));
}

}  // namespace positronic_protocol
