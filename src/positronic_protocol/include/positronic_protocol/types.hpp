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

#ifndef POSITRONIC_PROTOCOL__TYPES_HPP_
#define POSITRONIC_PROTOCOL__TYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

/// Encode and decode the myCobot 280 serial frames.
///
/// Frame layout: `FE FE LEN CMD [DATA...] FA`, where `LEN = N + 2` for N data bytes.
/// There is no checksum. Angles travel as signed big-endian int16 in 0.01 deg.
namespace positronic_protocol
{

/// First two bytes of every frame.
inline constexpr std::uint8_t HEADER{0xFE};

/// Last byte of every frame.
inline constexpr std::uint8_t FOOTER{0xFA};

/// Number of joints of the myCobot 280.
inline constexpr std::size_t NUM_JOINTS{6};

/// Capacity of a Frame. The largest frame we build (send_angles) has 18 bytes.
inline constexpr std::size_t MAX_FRAME_SIZE{32};

/// Lowest speed accepted by send_angles (percent of the firmware maximum).
inline constexpr std::uint8_t MIN_SPEED{1};

/// Highest speed accepted by send_angles (percent of the firmware maximum).
inline constexpr std::uint8_t MAX_SPEED{100};

/// Command ids of the Elephant serial protocol (myCobot 280).
enum class CommandId : std::uint8_t
{
  POWER_ON = 0x10,            ///< Power the servos on.
  POWER_OFF = 0x11,           ///< Power the servos off.
  RELEASE_ALL_SERVOS = 0x13,  ///< Torque off: limp arm, still answers.
  GET_ANGLES = 0x20,          ///< Read the six joint angles.
  SEND_ANGLES = 0x22,         ///< Move the six joints to the given angles.
  STOP = 0x29,                ///< Stop the current motion.
};

/// Joint angle in the wire unit: 0.01 deg, range +-327.67 deg.
using CentiDegrees = std::int16_t;

/// Six joint angles in the wire unit.
using JointAngles = std::array<CentiDegrees, NUM_JOINTS>;

/// Six joint angles in radians (REP 103), as seen outside the driver.
using JointRadians = std::array<double, NUM_JOINTS>;

/// A complete frame ready to be written to the serial port.
struct Frame
{
  /// Frame bytes. Only the first `size` are valid.
  std::array<std::uint8_t, MAX_FRAME_SIZE> bytes{};
  /// Number of valid bytes in `bytes`.
  std::size_t size{0};
};

/// Read-only view of received bytes (C++17 stand-in for std::span).
///
/// It does not own the bytes: the buffer must outlive the view.
struct ByteView
{
  /// First byte, or nullptr when empty.
  const std::uint8_t * data{nullptr};
  /// Number of bytes readable from `data`.
  std::size_t size{0};
};

/// Outcome of a decode call.
enum class DecodeStatus : std::uint8_t
{
  FOUND,           ///< Reply found and parsed into `Decoded::value`.
  NEED_MORE_DATA,  ///< The buffer ends with a valid prefix of the reply.
  NOT_FOUND,       ///< Nothing usable in the buffer.
};

/// Result of a decode call. The caller drops `consumed` bytes from the front of its buffer.
template<typename T>
struct Decoded
{
  /// What the decoder found.
  DecodeStatus status{DecodeStatus::NOT_FOUND};
  /// Bytes the caller may drop from the front of its buffer.
  std::size_t consumed{0};
  /// Candidates skipped on the way (bad footer, wrong length).
  std::size_t rejected{0};
  /// Parsed reply. Only meaningful when `status` is FOUND.
  T value{};
};

}  // namespace positronic_protocol

#endif  // POSITRONIC_PROTOCOL__TYPES_HPP_
