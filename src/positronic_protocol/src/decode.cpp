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

/// Every servo-bus frame (`FF FF id ...`) carries the same checksum: the inverted low byte of
/// the sum from the id, right after FF FF, to the last data byte, compared with the last byte.
constexpr std::size_t SERVO_BUS_ID_OFFSET{2};

/// True when the servo-bus frame at `start`, whole in `rx` with `size` bytes, carries the
/// checksum of its bytes: the sync-write and the servo status frames alike.
bool servo_checksum_ok(ByteView rx, std::size_t start, std::size_t size) noexcept
{
  const std::size_t checksum_index{start + size - 1};
  unsigned int sum{0};
  for (std::size_t index{start + SERVO_BUS_ID_OFFSET}; index < checksum_index; ++index) {
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
    if (start_matched && servo_checksum_ok(rx, start, SYNC_WRITE_SIZE)) {
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

/// A servo status frame: FF FF id LEN(0x11) err + 15 data bytes + checksum, 21 bytes (#170).
constexpr std::uint8_t SERVO_STATUS_LEN{0x11};
constexpr std::size_t SERVO_STATUS_SIZE{21};
constexpr std::size_t SERVO_STATUS_LEN_OFFSET{3};
constexpr std::size_t SERVO_STATUS_ERROR_OFFSET{4};
constexpr std::size_t SERVO_STATUS_DATA_OFFSET{5};
/// The highest position a servo reports: 12 bits, one turn of 4096 steps.
constexpr std::uint16_t MAX_SERVO_POSITION{4095};

/// Reads an unsigned int16 stored little-endian at `index`.
std::uint16_t read_uint16_le(ByteView rx, std::size_t index) noexcept
{
  const auto low{static_cast<std::uint16_t>(detail::read_byte(rx, index))};
  const auto high{static_cast<std::uint16_t>(detail::read_byte(rx, index + 1))};
  return static_cast<std::uint16_t>(low | (high << 8U));
}

/// find_frame() for a servo status frame: FF FF, an id 1 to 7, LEN 0x11, then the checksum 17
/// bytes later, and a position a servo can report. A mismatch after FF FF, a bad checksum or an
/// impossible position counts as a rejected candidate; a buffer that ends while everything
/// still matches is NEED_MORE_DATA.
Match find_servo_status(ByteView rx) noexcept
{
  Match match{};
  for (std::size_t start{0}; start < rx.size; ++start) {
    if (detail::read_byte(rx, start) != 0xFF) {
      continue;
    }
    if (start + 1 >= rx.size) {
      match.status = DecodeStatus::NEED_MORE_DATA;
      match.consumed = start;
      return match;
    }
    if (detail::read_byte(rx, start + 1) != 0xFF) {
      continue;
    }
    if (start + SERVO_STATUS_LEN_OFFSET >= rx.size) {
      match.status = DecodeStatus::NEED_MORE_DATA;
      match.consumed = start;
      return match;
    }
    const std::uint8_t id{detail::read_byte(rx, start + SERVO_BUS_ID_OFFSET)};
    const bool id_ok{id >= 1 && id <= GRIPPER_SERVO_ID};
    const bool len_ok{detail::read_byte(rx, start + SERVO_STATUS_LEN_OFFSET) == SERVO_STATUS_LEN};
    if (!id_ok || !len_ok) {
      ++match.rejected;
      continue;
    }
    if (start + SERVO_STATUS_SIZE > rx.size) {
      match.status = DecodeStatus::NEED_MORE_DATA;
      match.consumed = start;
      return match;
    }
    // A position past one turn cannot come from a servo: a corrupt frame whose checksum
    // happened to match, rejected like one that did not.
    const bool position_ok{
      read_uint16_le(rx, start + SERVO_STATUS_DATA_OFFSET) <= MAX_SERVO_POSITION};
    if (!servo_checksum_ok(rx, start, SERVO_STATUS_SIZE) || !position_ok) {
      ++match.rejected;
      continue;
    }
    match.status = DecodeStatus::FOUND;
    match.start = start;
    match.consumed = start + SERVO_STATUS_SIZE;
    return match;
  }
  match.consumed = rx.size;
  return match;
}

/// A sign-magnitude value: bit `sign_bit` is the sign, the bits below it the magnitude.
std::int16_t sign_magnitude(std::uint16_t raw, unsigned int sign_bit) noexcept
{
  const auto magnitude{static_cast<std::int16_t>(raw & ((1U << sign_bit) - 1U))};
  if ((raw & (1U << sign_bit)) != 0U) {
    return static_cast<std::int16_t>(-magnitude);
  }
  return magnitude;
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

Decoded<ServoStatus> decode_servo_status(ByteView rx) noexcept
{
  // Register map of the 15 data bytes read at register 56 (STS servos): position 56-57,
  // speed 58-59, load 60-61, voltage 62, temperature 63, 64 async flag, 65 status, 66 moving.
  constexpr std::size_t POSITION{0};
  constexpr std::size_t SPEED{2};
  constexpr std::size_t LOAD{4};
  constexpr std::size_t VOLTAGE{6};
  constexpr std::size_t TEMPERATURE{7};
  constexpr std::size_t MOVING{10};
  constexpr unsigned int SPEED_SIGN_BIT{15};
  constexpr unsigned int LOAD_SIGN_BIT{10};
  const Match match{find_servo_status(rx)};
  auto result{from_match<ServoStatus>(match)};
  if (match.status == DecodeStatus::FOUND) {
    const std::size_t data{match.start + SERVO_STATUS_DATA_OFFSET};
    result.value.id = detail::read_byte(rx, match.start + SERVO_BUS_ID_OFFSET);
    result.value.error = detail::read_byte(rx, match.start + SERVO_STATUS_ERROR_OFFSET);
    result.value.position = read_uint16_le(rx, data + POSITION);
    result.value.speed = sign_magnitude(read_uint16_le(rx, data + SPEED), SPEED_SIGN_BIT);
    result.value.load = sign_magnitude(read_uint16_le(rx, data + LOAD), LOAD_SIGN_BIT);
    result.value.voltage = detail::read_byte(rx, data + VOLTAGE);
    result.value.temperature = detail::read_byte(rx, data + TEMPERATURE);
    result.value.moving = detail::read_byte(rx, data + MOVING) != 0;
  }
  return result;
}

CentiDegrees angle_from_steps(std::uint16_t position, std::size_t joint) noexcept
{
  constexpr std::int64_t STEPS_PER_TURN{4096};
  constexpr std::int64_t CENTER_STEPS{2048};
  constexpr std::int64_t CDEG_PER_TURN{36000};
  constexpr std::size_t INVERTED_JOINT{2};  // J3
  // Integer division truncates toward zero, as the Atom does: 2053 steps is -43, not -44.
  std::int64_t cdeg{(CENTER_STEPS - position) * CDEG_PER_TURN / STEPS_PER_TURN};
  if (joint == INVERTED_JOINT) {
    cdeg = -cdeg;
  }
  return static_cast<CentiDegrees>(cdeg);
}

}  // namespace positronic_protocol
