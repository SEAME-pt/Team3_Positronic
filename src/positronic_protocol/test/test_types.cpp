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

// AI-generated (Claude, Anthropic).
// Prompt:
//   Write a GoogleTest suite for types.hpp and detail/byte_access.hpp. Check the frame
//   markers (FE/FA) and the CommandId values, that every command id also appears on the
//   capture's TX lines, that the largest frame (send_angles) fits MAX_FRAME_SIZE, the
//   speed range and the wire-unit size, that Frame/ByteView/Decoded default to empty, and
//   that read_byte and view_of never read out of bounds, including at compile time.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "positronic_protocol/detail/byte_access.hpp"
#include "positronic_protocol/types.hpp"

namespace
{

namespace pp = positronic_protocol;

constexpr const char * CAPTURE_FILE{TEST_DATA_DIR "/frames_20261016_124447.txt"};

std::uint8_t id(pp::CommandId command)
{
  return static_cast<std::uint8_t>(command);
}

/// Command ids of every `FE FE LEN CMD ... FA` frame found on the TX lines of a capture.
/// A TX line may hold more than one frame (stop_during_motion sends send_angles, then stop).
std::set<std::uint8_t> tx_command_ids(const std::string & path)
{
  std::set<std::uint8_t> ids{};
  std::ifstream file{path};
  std::string line{};
  while (std::getline(file, line)) {
    if (line.rfind("TX ", 0) != 0) {
      continue;
    }
    std::istringstream hex{line.substr(3)};
    std::vector<std::uint8_t> bytes{};
    std::string token{};
    while (hex >> token) {
      bytes.push_back(static_cast<std::uint8_t>(std::stoul(token, nullptr, 16)));
    }
    std::size_t i{0};
    while (i + 3 < bytes.size()) {
      const bool is_header{bytes.at(i) == pp::HEADER && bytes.at(i + 1) == pp::HEADER};
      const std::size_t footer_index{i + 2 + bytes.at(i + 2)};
      if (is_header && footer_index < bytes.size() && bytes.at(footer_index) == pp::FOOTER) {
        ids.insert(bytes.at(i + 3));
        i = footer_index + 1;
      } else {
        ++i;
      }
    }
  }
  return ids;
}

// The helpers are constexpr, so the bounds check also works at compile time.
constexpr std::array<std::uint8_t, 2> COMPILE_TIME_RX{0xFE, 0xFA};
static_assert(
  pp::detail::read_byte(pp::ByteView{COMPILE_TIME_RX.data(), COMPILE_TIME_RX.size()}, 1) == 0xFA);
static_assert(
  pp::detail::read_byte(pp::ByteView{COMPILE_TIME_RX.data(), COMPILE_TIME_RX.size()}, 2) == 0);

}  // namespace

TEST(Types, FrameMarkers)
{
  EXPECT_EQ(pp::HEADER, 0xFE);
  EXPECT_EQ(pp::FOOTER, 0xFA);
}

TEST(Types, CommandIdValues)
{
  EXPECT_EQ(id(pp::CommandId::POWER_ON), 0x10);
  EXPECT_EQ(id(pp::CommandId::POWER_OFF), 0x11);
  EXPECT_EQ(id(pp::CommandId::RELEASE_ALL_SERVOS), 0x13);
  EXPECT_EQ(id(pp::CommandId::GET_ANGLES), 0x20);
  EXPECT_EQ(id(pp::CommandId::SEND_ANGLES), 0x22);
  EXPECT_EQ(id(pp::CommandId::STOP), 0x29);
}

TEST(Types, CommandIdsAppearInCapture)
{
  const std::set<std::uint8_t> sent{tx_command_ids(CAPTURE_FILE)};
  ASSERT_FALSE(sent.empty()) << "could not read " << CAPTURE_FILE;

  const std::array<pp::CommandId, 6> commands{
    pp::CommandId::POWER_ON, pp::CommandId::POWER_OFF, pp::CommandId::RELEASE_ALL_SERVOS,
    pp::CommandId::GET_ANGLES, pp::CommandId::SEND_ANGLES, pp::CommandId::STOP};
  for (const auto command : commands) {
    EXPECT_EQ(sent.count(id(command)), 1U) << "command 0x" << std::hex << int{id(command)};
  }
}

TEST(Types, LargestFrameFits)
{
  // send_angles: FE FE LEN CMD + 6 x int16 + speed + FA
  const std::size_t send_angles_size{4 + 2 * pp::NUM_JOINTS + 1 + 1};
  EXPECT_EQ(send_angles_size, 18U);
  EXPECT_LE(send_angles_size, pp::MAX_FRAME_SIZE);
}

TEST(Types, SpeedRange)
{
  EXPECT_EQ(pp::MIN_SPEED, 1);
  EXPECT_EQ(pp::MAX_SPEED, 100);
}

TEST(Types, WireUnitHoldsFullCircle)
{
  // +-180 deg in 0.01 deg must fit in the wire type.
  EXPECT_EQ(sizeof(pp::CentiDegrees), 2U);
  EXPECT_LE(18000, std::numeric_limits<pp::CentiDegrees>::max());
  EXPECT_GE(-18000, std::numeric_limits<pp::CentiDegrees>::min());
}

TEST(Types, DefaultsAreEmpty)
{
  const pp::Frame frame{};
  EXPECT_EQ(frame.size, 0U);
  for (const auto byte : frame.bytes) {
    EXPECT_EQ(byte, 0);
  }

  const pp::ByteView view{};
  EXPECT_EQ(view.data, nullptr);
  EXPECT_EQ(view.size, 0U);

  const pp::Decoded<pp::JointAngles> decoded{};
  EXPECT_EQ(decoded.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(decoded.consumed, 0U);
  EXPECT_EQ(decoded.rejected, 0U);
  for (const auto angle : decoded.value) {
    EXPECT_EQ(angle, 0);
  }
}

TEST(ByteAccess, ReadInsideView)
{
  const std::array<std::uint8_t, 5> rx{0xFE, 0xFE, 0x02, 0x10, 0xFA};
  const pp::ByteView view{rx.data(), rx.size()};
  EXPECT_EQ(pp::detail::read_byte(view, 0), 0xFE);
  EXPECT_EQ(pp::detail::read_byte(view, 3), 0x10);
  EXPECT_EQ(pp::detail::read_byte(view, 4), 0xFA);
}

TEST(ByteAccess, ReadOutsideViewReturnsZero)
{
  const std::array<std::uint8_t, 3> rx{0xFE, 0xFE, 0x02};
  // The view covers only the first 2 bytes, so index 2 is out of range even though
  // the array has a third byte.
  const pp::ByteView view{rx.data(), 2};
  EXPECT_EQ(pp::detail::read_byte(view, 2), 0);
  EXPECT_EQ(pp::detail::read_byte(view, 1000), 0);
}

TEST(ByteAccess, ReadEmptyViewReturnsZero)
{
  EXPECT_EQ(pp::detail::read_byte(pp::ByteView{}, 0), 0);
  // A non-zero size with a null pointer must not be dereferenced.
  EXPECT_EQ(pp::detail::read_byte(pp::ByteView{nullptr, 10}, 0), 0);
}

TEST(ByteAccess, ViewOfFrameCoversValidBytes)
{
  pp::Frame frame{};
  frame.bytes.front() = pp::HEADER;
  frame.size = 2;

  const pp::ByteView view{pp::detail::view_of(frame)};
  EXPECT_EQ(view.size, 2U);
  EXPECT_EQ(pp::detail::read_byte(view, 0), pp::HEADER);
  EXPECT_EQ(pp::detail::read_byte(view, 2), 0);  // outside the valid bytes
}

TEST(ByteAccess, ViewOfFrameClampsBadSize)
{
  pp::Frame frame{};
  frame.size = pp::MAX_FRAME_SIZE + 100;
  EXPECT_EQ(pp::detail::view_of(frame).size, pp::MAX_FRAME_SIZE);
}
