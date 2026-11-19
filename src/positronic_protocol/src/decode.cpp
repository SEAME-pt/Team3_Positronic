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

#include "positronic_protocol/decode.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

#include "positronic_protocol/detail/byte_access.hpp"
#include "positronic_protocol/types.hpp"

namespace positronic_protocol
{

namespace
{

/// Byte offsets inside a frame: `FE FE LEN CMD DATA...`.
constexpr std::size_t LEN_OFFSET{2};
constexpr std::size_t CMD_OFFSET{3};
constexpr std::size_t DATA_OFFSET{4};

/// A frame has LEN + 3 bytes: LEN counts CMD, data and footer, but not FE FE LEN.
constexpr std::size_t BYTES_NOT_IN_LEN{3};

/// LEN of the get_angles reply: CMD + 12 data bytes + footer = 0x0E.
constexpr std::uint8_t GET_ANGLES_LEN{static_cast<std::uint8_t>(2 * NUM_JOINTS + 2)};

/// LEN of the get_servo_temps reply: CMD + 6 data bytes + footer = 0x08.
constexpr std::uint8_t SERVO_TEMPS_LEN{static_cast<std::uint8_t>(NUM_JOINTS + 2)};

/// LEN of an ack: CMD + 1 status byte + footer = 0x03.
constexpr std::uint8_t ACK_LEN{3};

/// The exact start of the reply we are waiting for: `FE FE LEN CMD`.
///
/// There is no checksum, so matching all four bytes (plus the footer at the right
/// distance) is what makes the search reliable inside the servo noise. It also rejects
/// the echo of our own request, whose LEN is 0x02.
struct Signature
{
  std::uint8_t len{0};
  CommandId command{CommandId::GET_ANGLES};
};

/// Where the reply is, before parsing its data.
struct Match
{
  DecodeStatus status{DecodeStatus::NOT_FOUND};
  std::size_t start{0};  // index of the first FE (FOUND only)
  std::size_t consumed{0};
  std::size_t rejected{0};
};

/// Expected byte at `offset` of a frame with `signature`.
std::uint8_t expected_byte(Signature signature, std::size_t offset) noexcept
{
  switch (offset) {
    case 0:
    case 1:
      return HEADER;
    case LEN_OFFSET:
      return signature.len;
    default:
      return static_cast<std::uint8_t>(signature.command);
  }
}

/// Scans `rx` for a frame with `signature`, one start position at a time.
///
/// At each position it compares the 4 signature bytes and then the footer. A mismatch
/// after FE FE counts as a rejected candidate. If the buffer ends while everything still
/// matches, the frame may be arriving in pieces: report NEED_MORE_DATA and keep it.
Match find_frame(ByteView rx, Signature signature) noexcept
{
  Match match{};
  const std::size_t frame_size{signature.len + BYTES_NOT_IN_LEN};
  const std::size_t footer_offset{frame_size - 1};

  for (std::size_t start{0}; start < rx.size; ++start) {
    bool mismatch{false};
    bool truncated{false};
    for (std::size_t offset{0}; offset <= CMD_OFFSET && !mismatch && !truncated; ++offset) {
      if (start + offset >= rx.size) {
        truncated = true;
      } else if (detail::read_byte(rx, start + offset) != expected_byte(signature, offset)) {
        mismatch = true;
        if (offset > 1) {
          ++match.rejected;
        }
      }
    }
    if (!mismatch && !truncated) {
      if (start + footer_offset >= rx.size) {
        truncated = true;
      } else if (detail::read_byte(rx, start + footer_offset) != FOOTER) {
        mismatch = true;
        ++match.rejected;
      }
    }
    if (truncated) {
      match.status = DecodeStatus::NEED_MORE_DATA;
      match.consumed = start;
      return match;
    }
    if (!mismatch) {
      match.status = DecodeStatus::FOUND;
      match.start = start;
      match.consumed = start + frame_size;
      return match;
    }
  }
  match.consumed = rx.size;
  return match;
}

/// Reads a signed int16 stored big-endian at `index`.
CentiDegrees read_int16_be(ByteView rx, std::size_t index) noexcept
{
  const auto high{static_cast<std::uint16_t>(detail::read_byte(rx, index))};
  const auto low{static_cast<std::uint16_t>(detail::read_byte(rx, index + 1))};
  const auto raw{static_cast<std::uint16_t>((high << 8U) | low)};
  return static_cast<CentiDegrees>(raw);
}

/// The start of the Atom's sync-write to the six servos: header FF FF, broadcast id FE, LEN
/// 0x34, SYNC_WRITE (0x83), first register 0x29, 7 data bytes per servo.
constexpr std::array<std::uint8_t, 7> SYNC_WRITE_START{0xFF, 0xFF, 0xFE, 0x34, 0x83, 0x29, 0x07};

/// One servo in it: the id and the 7 data bytes.
constexpr std::size_t SYNC_WRITE_BYTES_PER_SERVO{8};

/// The whole frame: the start, six servos and the checksum, 56 bytes.
constexpr std::size_t SYNC_WRITE_SIZE{
  SYNC_WRITE_START.size() + NUM_JOINTS * SYNC_WRITE_BYTES_PER_SERVO + 1};

/// The checksum covers the frame from the id, right after FF FF, to the last data byte.
constexpr std::size_t SYNC_WRITE_ID_OFFSET{2};

/// True when the sync-write at `start`, whole in `rx`, carries the checksum of its bytes.
bool sync_write_checksum_ok(ByteView rx, std::size_t start) noexcept
{
  const std::size_t checksum_index{start + SYNC_WRITE_SIZE - 1};
  unsigned int sum{0};
  for (std::size_t index{start + SYNC_WRITE_ID_OFFSET}; index < checksum_index; ++index) {
    sum += detail::read_byte(rx, index);
  }
  return static_cast<std::uint8_t>(~sum) == detail::read_byte(rx, checksum_index);
}

/// find_frame() for the sync-write: the 7 start bytes, then the checksum. A mismatch after
/// FF FF, or a bad checksum, counts as a rejected candidate. A buffer that ends while
/// everything still matches is NEED_MORE_DATA, so a frame arriving in pieces is kept.
Match find_sync_write(ByteView rx) noexcept
{
  Match match{};
  for (std::size_t start{0}; start < rx.size; ++start) {
    std::size_t matched{0};
    while (matched < SYNC_WRITE_START.size() && start + matched < rx.size &&
      detail::read_byte(rx, start + matched) == SYNC_WRITE_START.at(matched))
    {
      ++matched;
    }
    const bool start_matched{matched == SYNC_WRITE_START.size()};
    const bool cut_in_start{!start_matched && start + matched == rx.size};
    const bool cut_in_body{start_matched && start + SYNC_WRITE_SIZE > rx.size};
    if (cut_in_start || cut_in_body) {
      match.status = DecodeStatus::NEED_MORE_DATA;
      match.consumed = start;
      return match;
    }
    if (start_matched && sync_write_checksum_ok(rx, start)) {
      match.status = DecodeStatus::FOUND;
      match.start = start;
      match.consumed = start + SYNC_WRITE_SIZE;
      return match;
    }
    if (matched > 1) {
      ++match.rejected;
    }
  }
  match.consumed = rx.size;
  return match;
}

/// Copies the bookkeeping of `match` into a Decoded result.
template<typename T>
Decoded<T> from_match(const Match & match) noexcept
{
  Decoded<T> result{};
  result.status = match.status;
  result.consumed = match.consumed;
  result.rejected = match.rejected;
  return result;
}

}  // namespace

Decoded<JointAngles> decode_get_angles_reply(ByteView rx) noexcept
{
  const Match match{find_frame(rx, Signature{GET_ANGLES_LEN, CommandId::GET_ANGLES})};
  auto result{from_match<JointAngles>(match)};
  if (match.status == DecodeStatus::FOUND) {
    std::size_t index{match.start + DATA_OFFSET};
    for (auto & angle : result.value) {
      angle = read_int16_be(rx, index);
      index += 2;
    }
  }
  return result;
}

Decoded<ServoTemperatures> decode_servo_temps_reply(ByteView rx) noexcept
{
  const Match match{find_frame(rx, Signature{SERVO_TEMPS_LEN, CommandId::GET_SERVO_TEMPS})};
  auto result{from_match<ServoTemperatures>(match)};
  if (match.status == DecodeStatus::FOUND) {
    std::size_t index{match.start + DATA_OFFSET};
    for (auto & celsius : result.value) {
      celsius = detail::read_byte(rx, index);
      ++index;
    }
  }
  return result;
}

Decoded<std::uint8_t> decode_ack(ByteView rx, CommandId command) noexcept
{
  const Match match{find_frame(rx, Signature{ACK_LEN, command})};
  auto result{from_match<std::uint8_t>(match)};
  if (match.status == DecodeStatus::FOUND) {
    result.value = detail::read_byte(rx, match.start + DATA_OFFSET);
  }
  return result;
}

Decoded<std::uint8_t> decode_sync_write_echo(ByteView rx) noexcept
{
  const Match match{find_sync_write(rx)};
  auto result{from_match<std::uint8_t>(match)};
  if (match.status == DecodeStatus::FOUND) {
    result.value = static_cast<std::uint8_t>(NUM_JOINTS);
  }
  return result;
}

}  // namespace positronic_protocol
