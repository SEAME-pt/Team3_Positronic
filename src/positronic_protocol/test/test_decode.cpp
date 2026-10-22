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
//   Write a GoogleTest suite for decode.hpp. Cover decode_ack happy paths plus
//   wrong-command and echo rejection; decode_get_angles_reply on clean frames, large
//   angles, int16 limits and payloads that contain header-like bytes; servo noise and
//   byte-by-byte resync; and split or multiple frames, cutting the buffer at every
//   position and streaming it in chunks of every size, the way the UART delivers it.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "positronic_protocol/decode.hpp"
#include "positronic_protocol/types.hpp"
#include "test_helpers.hpp"

namespace
{

namespace pp = positronic_protocol;
using test_helpers::Bytes;
using test_helpers::concat;
using test_helpers::parse_hex;
using test_helpers::prefix_of;
using test_helpers::view_of;

/// get_angles reply from the capture: [0.87, 0, -0.61, -1.23, -0.08, 0] deg.
const Bytes ANGLES_FRAME{parse_hex("fe fe 0e 20 00 57 00 00 ff c3 ff 85 ff f8 00 00 fa")};
const pp::JointAngles ANGLES_VALUE{87, 0, -61, -123, -8, 0};

struct AckCase
{
  pp::CommandId command{pp::CommandId::POWER_ON};
  const char * frame{nullptr};
};

/// Servo bus traffic that arrived before the get_angles reply (capture, get_angles_00).
/// Feetech style frames (FF FF id len ...), with a lone FE inside.
const Bytes SERVO_NOISE{parse_hex(
  "ff ff 07 02 01 f5 ff ff fe 0a 82 38 0f 01 02 03 04 05 06 19 ff ff 01 11 00 01 08 00 00 00 00 79 "
  "26 00 00 00 01 08 00 00 3c ff ff 02 11 00 02 08 00 00 00 00 79 23 00 00 00 02 08 00 00 3c ff ff "
  "03 11 00 f9 07 00 00 00 00 7a 21 00 00 00 f9 07 00 00 50 ff ff 04 11 00 0e 08 00 00 00 00 4a 21 "
  "00 00 00 0e 08 00 00 53 ff ff 05 11 00 fb 07 00 00 00 00 4a 20 00 00 00 fb 07 00 00 7b ff ff 06 "
  "11 00 f9 07 00 00 00 00 49 23 00 00 00 f9 07 00 00 7c")};

}  // namespace

// ---------------------------------------------------------------------------
// T6: decode_ack, happy path
// ---------------------------------------------------------------------------

TEST(DecodeAck, AllAckedCommands)
{
  const std::array<AckCase, 5> cases{{
    {pp::CommandId::POWER_ON, "fe fe 03 10 01 fa"},
    {pp::CommandId::POWER_OFF, "fe fe 03 11 01 fa"},
    {pp::CommandId::RELEASE_ALL_SERVOS, "fe fe 03 13 01 fa"},
    {pp::CommandId::SEND_ANGLES, "fe fe 03 22 01 fa"},
    {pp::CommandId::STOP, "fe fe 03 29 01 fa"},
  }};
  for (const auto & c : cases) {
    const Bytes rx{parse_hex(c.frame)};
    const auto result{pp::decode_ack(view_of(rx), c.command)};
    EXPECT_EQ(result.status, pp::DecodeStatus::FOUND) << c.frame;
    EXPECT_EQ(result.value, 1) << c.frame;
    EXPECT_EQ(result.consumed, 6U) << c.frame;
    EXPECT_EQ(result.rejected, 0U) << c.frame;
  }
}

TEST(DecodeAck, StatusBytePassesThrough)
{
  const Bytes rx{parse_hex("fe fe 03 22 00 fa")};
  const auto result{pp::decode_ack(view_of(rx), pp::CommandId::SEND_ANGLES)};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, 0);
}

TEST(DecodeAck, IgnoresAckOfAnotherCommand)
{
  // stop_during_motion: the send_angles ack is in the buffer, we wait for the stop ack.
  const Bytes rx{parse_hex("fe fe 03 22 01 fa")};
  const auto result{pp::decode_ack(view_of(rx), pp::CommandId::STOP)};
  EXPECT_EQ(result.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(result.consumed, 6U);
  EXPECT_EQ(result.rejected, 1U);
}

TEST(DecodeAck, NoiseBeforeAckFromCapture)
{
  // power_on: the servo bus says something before the ack.
  const Bytes rx{parse_hex("ff ff fe 04 03 28 01 d1 fe fe 03 10 01 fa")};
  const auto result{pp::decode_ack(view_of(rx), pp::CommandId::POWER_ON)};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, 1);
  EXPECT_EQ(result.consumed, rx.size());
}

// ---------------------------------------------------------------------------
// T7: decode_get_angles_reply, happy path
// ---------------------------------------------------------------------------

TEST(DecodeAngles, CleanFrame)
{
  const auto result{pp::decode_get_angles_reply(view_of(ANGLES_FRAME))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, ANGLES_VALUE);
  EXPECT_EQ(result.consumed, 17U);
}

TEST(DecodeAngles, LargeAnglesFromCapture)
{
  // get_angles_j1_out_0: J1 at 29.09 deg.
  const Bytes j1_out{parse_hex("fe fe 0e 20 0b 5d ff ef ff c3 ff 85 00 2b 00 3d fa")};
  const auto out{pp::decode_get_angles_reply(view_of(j1_out))};
  ASSERT_EQ(out.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(out.value, (pp::JointAngles{2909, -17, -61, -123, 43, 61}));

  // get_angles_released_moved_0: J4 and J6 moved by hand.
  const Bytes moved{parse_hex("fe fe 0e 20 00 4f ff ef ff c3 f7 6b 00 2b 1a d9 fa")};
  const auto by_hand{pp::decode_get_angles_reply(view_of(moved))};
  ASSERT_EQ(by_hand.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(by_hand.value, (pp::JointAngles{79, -17, -61, -2197, 43, 6873}));
}

TEST(DecodeAngles, Int16Limits)
{
  const Bytes rx{parse_hex("fe fe 0e 20 7f ff 80 00 ff ff 00 01 80 01 00 00 fa")};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, (pp::JointAngles{32767, -32768, -1, 1, -32767, 0}));
}

TEST(DecodeAngles, DataMayContainHeaderBytes)
{
  // -258 is FE FE on the wire. It must not confuse the search.
  const Bytes rx{parse_hex("fe fe 0e 20 fe fe fe fe 00 00 00 00 00 00 00 00 fa")};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, (pp::JointAngles{-258, -258, 0, 0, 0, 0}));
}

// ---------------------------------------------------------------------------
// T8: noise before the header and resynchronisation
// ---------------------------------------------------------------------------

TEST(DecodeNoise, RealServoNoise)
{
  const Bytes rx{concat({SERVO_NOISE, ANGLES_FRAME})};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, ANGLES_VALUE);
  EXPECT_EQ(result.consumed, rx.size());
}

TEST(DecodeNoise, LoneHeaderBytes)
{
  const Bytes rx{concat({parse_hex("fe 00 fe 12 fe 34"), ANGLES_FRAME})};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, ANGLES_VALUE);
  EXPECT_EQ(result.rejected, 0U);  // a lone FE is not a candidate
}

TEST(DecodeNoise, ManyHeadersInARow)
{
  const Bytes rx{concat({parse_hex("fe fe fe fe"), ANGLES_FRAME})};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, ANGLES_VALUE);
  EXPECT_EQ(result.consumed, rx.size());
}

TEST(DecodeNoise, EchoOfRequestIsNotAReply)
{
  const Bytes echo{parse_hex("fe fe 02 20 fa")};
  const auto alone{pp::decode_get_angles_reply(view_of(echo))};
  EXPECT_EQ(alone.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(alone.consumed, echo.size());
  EXPECT_EQ(alone.rejected, 1U);

  const Bytes rx{concat({echo, ANGLES_FRAME})};
  const auto with_reply{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(with_reply.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(with_reply.value, ANGLES_VALUE);
}

TEST(DecodeNoise, EchoOfAckRequestIsNotAnAck)
{
  const Bytes echo{parse_hex("fe fe 02 10 fa")};
  EXPECT_EQ(
    pp::decode_ack(view_of(echo), pp::CommandId::POWER_ON).status, pp::DecodeStatus::NOT_FOUND);
}

TEST(DecodeNoise, BadFooterThenGoodFrame)
{
  Bytes corrupted{ANGLES_FRAME};
  corrupted.back() = 0x00;
  const Bytes rx{concat({corrupted, ANGLES_FRAME})};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, ANGLES_VALUE);
  EXPECT_GE(result.rejected, 1U);
  EXPECT_EQ(result.consumed, rx.size());
}

TEST(DecodeNoise, WrongLengthIsRejected)
{
  // Right header and command, wrong LEN: a corrupted frame, not our reply.
  const Bytes rx{parse_hex("fe fe 0d 20 00 00 00 00 00 00 00 00 00 00 00 fa")};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  EXPECT_EQ(result.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(result.rejected, 1U);
}

TEST(DecodeNoise, WrongLengthHeaderBeforeReply)
{
  // A false FE FE with a large LEN right before the real reply. A parser that trusts
  // LEN and never backtracks (like the one in libmyCobotCpp) swallows the reply as data.
  const Bytes rx{concat({parse_hex("fe fe 30 01 02 03"), ANGLES_FRAME})};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, ANGLES_VALUE);
  EXPECT_EQ(result.rejected, 1U);
  EXPECT_EQ(result.consumed, rx.size());
}

TEST(DecodeNoise, AckIsNotAnAnglesReply)
{
  const Bytes rx{parse_hex("fe fe 03 20 01 fa")};
  EXPECT_EQ(pp::decode_get_angles_reply(view_of(rx)).status, pp::DecodeStatus::NOT_FOUND);
}

TEST(DecodeNoise, NullViewIsSafe)
{
  // A broken view (nullptr with a size) must not be dereferenced.
  const auto result{pp::decode_get_angles_reply(pp::ByteView{nullptr, 100})};
  EXPECT_NE(result.status, pp::DecodeStatus::FOUND);
}

// ---------------------------------------------------------------------------
// T9: split and multiple frames
// ---------------------------------------------------------------------------

TEST(DecodeSplit, EmptyBuffer)
{
  const auto result{pp::decode_get_angles_reply(pp::ByteView{})};
  EXPECT_EQ(result.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(result.consumed, 0U);
}

TEST(DecodeSplit, EndsWithOneHeaderByte)
{
  const Bytes rx{parse_hex("00 11 22 fe")};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  EXPECT_EQ(result.status, pp::DecodeStatus::NEED_MORE_DATA);
  EXPECT_EQ(result.consumed, 3U);  // drop the noise, keep the FE
}

TEST(DecodeSplit, EndsWithTwoHeaderBytes)
{
  const Bytes rx{parse_hex("00 11 fe fe")};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  EXPECT_EQ(result.status, pp::DecodeStatus::NEED_MORE_DATA);
  EXPECT_EQ(result.consumed, 2U);
}

TEST(DecodeSplit, EndsInsideTheData)
{
  const Bytes rx{concat({SERVO_NOISE, parse_hex("fe fe 0e 20 00 57 00")})};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  EXPECT_EQ(result.status, pp::DecodeStatus::NEED_MORE_DATA);
  EXPECT_EQ(result.consumed, SERVO_NOISE.size());
}

TEST(DecodeSplit, TwoFramesInARow)
{
  const Bytes second_frame{parse_hex("fe fe 0e 20 0b 5d ff ef ff c3 ff 85 00 2b 00 3d fa")};
  const Bytes rx{concat({ANGLES_FRAME, second_frame})};

  const auto first{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(first.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(first.value, ANGLES_VALUE);
  EXPECT_EQ(first.consumed, ANGLES_FRAME.size());

  const pp::ByteView rest{&rx.at(first.consumed), rx.size() - first.consumed};
  const auto second{pp::decode_get_angles_reply(rest)};
  ASSERT_EQ(second.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(second.value, (pp::JointAngles{2909, -17, -61, -123, 43, 61}));
}

TEST(DecodeSplit, FalseStartInsideNoise)
{
  // A candidate that looks right for 4 bytes and then fails the footer.
  const Bytes fake{parse_hex("fe fe 0e 20 01 02 03")};
  const Bytes rx{concat({fake, SERVO_NOISE, ANGLES_FRAME})};
  const auto result{pp::decode_get_angles_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, ANGLES_VALUE);
  EXPECT_GE(result.rejected, 1U);
}

TEST(DecodeSplit, EveryCutPosition)
{
  // The UART hands out bytes in arbitrary pieces (like Serial.available() on an ESP32).
  // Whatever the cut, the decoder never invents a reply and never drops past the cut.
  const Bytes rx{concat({SERVO_NOISE, ANGLES_FRAME})};
  for (std::size_t cut{0}; cut <= rx.size(); ++cut) {
    const auto result{pp::decode_get_angles_reply(prefix_of(rx, cut))};
    ASSERT_LE(result.consumed, cut) << "cut " << cut;
    if (cut == rx.size()) {
      ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
      ASSERT_EQ(result.value, ANGLES_VALUE);
    } else {
      ASSERT_NE(result.status, pp::DecodeStatus::FOUND) << "cut " << cut;
    }
    if (cut > SERVO_NOISE.size() && cut < rx.size()) {
      // The reply has started: wait for the rest and keep its first byte.
      ASSERT_EQ(result.status, pp::DecodeStatus::NEED_MORE_DATA) << "cut " << cut;
      ASSERT_EQ(result.consumed, SERVO_NOISE.size()) << "cut " << cut;
    }
  }
}

TEST(DecodeSplit, StreamingInChunksOfEverySize)
{
  // Simulates the #22 read loop: append what arrived, decode, drop `consumed`.
  const Bytes stream{concat({SERVO_NOISE, ANGLES_FRAME, SERVO_NOISE})};
  for (std::size_t chunk{1}; chunk <= stream.size(); ++chunk) {
    Bytes buffer{};
    int found{0};
    for (std::size_t sent{0}; sent < stream.size(); sent += chunk) {
      const std::size_t end{std::min(sent + chunk, stream.size())};
      buffer.insert(
        buffer.end(), stream.begin() + static_cast<std::ptrdiff_t>(sent),
        stream.begin() + static_cast<std::ptrdiff_t>(end));
      const auto result{pp::decode_get_angles_reply(view_of(buffer))};
      ASSERT_LE(result.consumed, buffer.size());
      if (result.status == pp::DecodeStatus::FOUND) {
        ASSERT_EQ(result.value, ANGLES_VALUE) << "chunk " << chunk;
        ++found;
      }
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(result.consumed));
    }
    EXPECT_EQ(found, 1) << "chunk " << chunk;
  }
}
