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
// Follow-up prompt (servo over-temperature):
//   decode_servo_temps_reply finds FE FE 08 E5 + six bytes + FA: a clean frame, one behind
//   recorded servo noise, a frame cut at every position (NEED_MORE_DATA, then FOUND), the
//   echo of the request and a get_angles reply are not temperatures, a temperatures reply is
//   not a get_angles reply, and a wrong LEN is rejected.
// Follow-up prompt (#150, the sync-write echo):
//   decode_sync_write_echo finds the Atom's sync-write to the six servos: the recorded frame,
//   the same frame at the end of everything the Atom sent after one recorded send_angles, a
//   frame cut at every position, a bad checksum, the other servo-bus frames, and our own replies.
//   Then: an empty buffer, and a frame with a bad checksum followed by a good one, which is found
//   with the bad start counted as rejected.
// Follow-up prompt (#170, the servo status frames):
//   decode_servo_status finds one servo status frame (FF FF id 11 err + 15 bytes + checksum):
//   the recorded frame of servo 1 with its fields, the six frames of the recorded burst in order
//   and nothing else in it, a frame cut at every position, a bad checksum alone and before a good
//   one, the gripper (id 7), the error byte and the sign-magnitude speed and load, the sync-write
//   and our replies are not it, an empty buffer. angle_from_steps against values pymycobot
//   reported.

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

/// get_servo_temps reply in the format pymycobot 4.0.4 decodes: J1..J6 = 31, 33, 35, 30, 29,
/// 28 degC. Synthetic, one distinct value per servo; RECORDED_TEMPS_FRAME below is a real one.
const Bytes TEMPS_FRAME{parse_hex("fe fe 08 e5 1f 21 23 1e 1d 1c fa")};
const pp::ServoTemperatures TEMPS_VALUE{31, 33, 35, 30, 29, 28};

/// A real get_servo_temps reply, recorded from the arm on 2026-11-13: J1..J6 = 34, 32, 31, 32,
/// 30, 34 degC, the same values pymycobot read in the same session.
const Bytes RECORDED_TEMPS_FRAME{parse_hex("fe fe 08 e5 22 20 1f 20 1e 22 fa")};
const pp::ServoTemperatures RECORDED_TEMPS_VALUE{34, 32, 31, 32, 30, 34};

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

/// The Atom's sync-write to servos 1 to 6, recorded on the arm on 2026-11-13 (tap of #115).
const Bytes RECORDED_SYNC_WRITE{parse_hex(
  "ff ff fe 34 83 29 07 01 32 e8 07 00 00 f1 01 02 32 01 08 00 00 32 01 03 32 4a 08 00 00 2c 01 "
  "04 32 3f 08 00 00 2c 01 05 32 13 08 00 00 9e 01 06 32 de 08 00 00 fe 01 2a")};

/// Everything the Atom put on the line after one send_angles, same tap: the ack, a ping of
/// servo 7, the sync-read and the six servo replies, and the sync-write last. 208 bytes.
const Bytes RECORDED_AFTER_SEND{parse_hex(
  "fe fe 03 22 01 fa ff ff 07 02 01 f5 ff ff fe 0a 82 38 0f 01 02 03 04 05 06 19 ff ff 01 11 00 e8 "
  "07 00 00 00 00 79 23 00 00 00 e8 07 00 00 73 ff ff 02 11 00 02 08 00 00 00 00 79 20 00 00 00 01 "
  "08 00 00 40 ff ff 03 11 00 4b 08 00 00 00 00 7a 1f 00 00 00 4a 08 00 00 ad ff ff 04 11 00 40 08 "
  "00 00 00 00 4a 20 00 00 00 3f 08 00 00 f1 ff ff 05 11 00 14 08 00 00 00 00 4a 1e 00 00 00 13 08 "
  "00 00 4a ff ff 06 11 00 df 08 00 00 00 00 49 22 00 00 00 de 08 00 00 b0 ff ff fe 34 83 29 07 01 "
  "32 db 07 00 00 fe 01 02 32 01 08 00 00 2c 01 03 32 4a 08 00 00 2c 01 04 32 3f 08 00 00 2c 01 05 "
  "32 13 08 00 00 2c 01 06 32 de 08 00 00 2c 01 74")};

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

// ---------------------------------------------------------------------------
// decode_servo_temps_reply (servo over-temperature)
// ---------------------------------------------------------------------------

TEST(DecodeTemps, CleanFrame)
{
  const auto result{pp::decode_servo_temps_reply(view_of(TEMPS_FRAME))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, TEMPS_VALUE);
  EXPECT_EQ(result.consumed, TEMPS_FRAME.size());
  EXPECT_EQ(result.rejected, 0U);
}

TEST(DecodeTemps, RecordedFrameFromArm)
{
  // The frame as it came off the arm on 2026-11-13, not a synthetic one.
  const auto result{pp::decode_servo_temps_reply(view_of(RECORDED_TEMPS_FRAME))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, RECORDED_TEMPS_VALUE);
  EXPECT_EQ(result.consumed, RECORDED_TEMPS_FRAME.size());
  EXPECT_EQ(result.rejected, 0U);
}

TEST(DecodeTemps, BehindServoNoise)
{
  const Bytes rx{concat({SERVO_NOISE, TEMPS_FRAME, SERVO_NOISE})};
  const auto result{pp::decode_servo_temps_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, TEMPS_VALUE);
  EXPECT_EQ(result.consumed, SERVO_NOISE.size() + TEMPS_FRAME.size());
}

TEST(DecodeTemps, EveryCutPosition)
{
  // A reply split by the UART: every prefix waits for more, the whole frame is found.
  for (std::size_t cut{1}; cut < TEMPS_FRAME.size(); ++cut) {
    const Bytes head{TEMPS_FRAME.begin(), TEMPS_FRAME.begin() + static_cast<std::ptrdiff_t>(cut)};
    const auto result{pp::decode_servo_temps_reply(view_of(head))};
    EXPECT_EQ(result.status, pp::DecodeStatus::NEED_MORE_DATA) << "cut " << cut;
    EXPECT_EQ(result.consumed, 0U) << "cut " << cut;
  }
}

TEST(DecodeTemps, EchoOfRequestIsNotAReply)
{
  const Bytes rx{concat({parse_hex("fe fe 02 e5 fa"), TEMPS_FRAME})};
  const auto result{pp::decode_servo_temps_reply(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, TEMPS_VALUE);
  EXPECT_EQ(result.rejected, 1U);
}

TEST(DecodeTemps, AnglesAndTemperaturesAreNotMixedUp)
{
  // Both replies can sit in the same buffer: each decoder finds only its own.
  const Bytes rx{concat({ANGLES_FRAME, TEMPS_FRAME})};
  const auto temps{pp::decode_servo_temps_reply(view_of(rx))};
  ASSERT_EQ(temps.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(temps.value, TEMPS_VALUE);
  EXPECT_EQ(pp::decode_get_angles_reply(view_of(TEMPS_FRAME)).status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(
    pp::decode_servo_temps_reply(view_of(ANGLES_FRAME)).status, pp::DecodeStatus::NOT_FOUND);
}

TEST(DecodeTemps, WrongLengthIsRejected)
{
  // Twelve data bytes (int16 per servo) is not the format we decode: rejected, not misread.
  const Bytes rx{parse_hex("fe fe 0e e5 00 1f 00 21 00 23 00 1e 00 1d 00 1c fa")};
  const auto result{pp::decode_servo_temps_reply(view_of(rx))};
  EXPECT_EQ(result.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(result.rejected, 1U);
}

// ---------------------------------------------------------------------------
// decode_sync_write_echo (#150): the last frame of the Atom after a send_angles
// ---------------------------------------------------------------------------

TEST(DecodeSyncWrite, RecordedFrameFromArm)
{
  const auto result{pp::decode_sync_write_echo(view_of(RECORDED_SYNC_WRITE))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value, pp::NUM_JOINTS);
  EXPECT_EQ(result.consumed, RECORDED_SYNC_WRITE.size());
  EXPECT_EQ(result.consumed, 56U);
  EXPECT_EQ(result.rejected, 0U);
}

TEST(DecodeSyncWrite, LastFrameOfEverythingAfterASend)
{
  // The ack, the ping, the sync-read and the six servo replies all start with FE FE or FF FF:
  // none is taken for the sync-write, which ends the recorded sequence.
  const auto result{pp::decode_sync_write_echo(view_of(RECORDED_AFTER_SEND))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.consumed, RECORDED_AFTER_SEND.size());
  EXPECT_EQ(RECORDED_AFTER_SEND.size(), 208U);
}

TEST(DecodeSyncWrite, EveryCutPosition)
{
  // A sync-write split by the UART: every prefix waits for more, nothing of it is dropped.
  for (std::size_t cut{1}; cut < RECORDED_SYNC_WRITE.size(); ++cut) {
    const Bytes head{
      RECORDED_SYNC_WRITE.begin(), RECORDED_SYNC_WRITE.begin() + static_cast<std::ptrdiff_t>(cut)};
    const auto result{pp::decode_sync_write_echo(view_of(head))};
    EXPECT_EQ(result.status, pp::DecodeStatus::NEED_MORE_DATA) << "cut " << cut;
    EXPECT_EQ(result.consumed, 0U) << "cut " << cut;
  }
}

TEST(DecodeSyncWrite, EveryCutPositionBehindThePoll)
{
  // The same, with the servo replies in front: they are dropped, the start of the frame is kept.
  const std::size_t before{RECORDED_AFTER_SEND.size() - RECORDED_SYNC_WRITE.size()};
  for (std::size_t cut{before + 1}; cut < RECORDED_AFTER_SEND.size(); ++cut) {
    const Bytes head{
      RECORDED_AFTER_SEND.begin(), RECORDED_AFTER_SEND.begin() + static_cast<std::ptrdiff_t>(cut)};
    const auto result{pp::decode_sync_write_echo(view_of(head))};
    EXPECT_EQ(result.status, pp::DecodeStatus::NEED_MORE_DATA) << "cut " << cut;
    EXPECT_EQ(result.consumed, before) << "cut " << cut;
  }
}

TEST(DecodeSyncWrite, BadChecksumIsRejected)
{
  Bytes rx{RECORDED_SYNC_WRITE};
  rx.back() = static_cast<std::uint8_t>(rx.back() + 1);
  const auto result{pp::decode_sync_write_echo(view_of(rx))};
  EXPECT_EQ(result.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(result.consumed, rx.size());
  EXPECT_EQ(result.rejected, 1U);
}

TEST(DecodeSyncWrite, OtherServoFramesAreNotIt)
{
  // The recorded poll: a ping, the sync-read (FF FF FE 0A 82, same broadcast id) and six
  // replies. Nothing found, everything dropped.
  const auto result{pp::decode_sync_write_echo(view_of(SERVO_NOISE))};
  EXPECT_EQ(result.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(result.consumed, SERVO_NOISE.size());
}

TEST(DecodeSyncWrite, OurRepliesAreNotIt)
{
  // Nor are the Atom's replies to us, and the sync-write is no reply either.
  EXPECT_EQ(pp::decode_sync_write_echo(view_of(ANGLES_FRAME)).status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(pp::decode_sync_write_echo(view_of(TEMPS_FRAME)).status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(
    pp::decode_get_angles_reply(view_of(RECORDED_SYNC_WRITE)).status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(
    pp::decode_ack(view_of(RECORDED_SYNC_WRITE), pp::CommandId::SEND_ANGLES).status,
    pp::DecodeStatus::NOT_FOUND);
}

TEST(DecodeSyncWrite, AnEmptyBufferFindsNothing)
{
  const auto result{pp::decode_sync_write_echo(pp::ByteView{})};
  EXPECT_EQ(result.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(result.consumed, 0U);
  EXPECT_EQ(result.rejected, 0U);
}

// On the wire the thread must step over a damaged frame and still find the next good one.
TEST(DecodeSyncWrite, AGoodFrameAfterABadChecksumIsFound)
{
  Bytes rx{RECORDED_SYNC_WRITE};
  rx.back() = static_cast<std::uint8_t>(rx.back() + 1);
  rx.insert(rx.end(), RECORDED_SYNC_WRITE.begin(), RECORDED_SYNC_WRITE.end());
  const auto result{pp::decode_sync_write_echo(view_of(rx))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.consumed, rx.size());
  EXPECT_EQ(result.rejected, 1U);
}

// ---------------------------------------------------------------------------
// #170: decode_servo_status, the frames of the Atom's poll
// ---------------------------------------------------------------------------

namespace
{

/// The servo-bus checksum: the inverted low byte of the sum from the id to the last data byte.
Bytes with_servo_checksum(Bytes frame)
{
  unsigned int sum{0};
  for (std::size_t i{2}; i + 1 < frame.size(); ++i) {
    sum += frame.at(i);
  }
  frame.back() = static_cast<std::uint8_t>(~sum);
  return frame;
}

/// The status frame of servo 1 in RECORDED_AFTER_SEND: position 2024, 12.1 V, 35 degC.
const Bytes STATUS_FRAME_1{
  parse_hex("ff ff 01 11 00 e8 07 00 00 00 00 79 23 00 00 00 e8 07 00 00 73")};

}  // namespace

TEST(DecodeServoStatus, RecordedFrameFromArm)
{
  const auto result{pp::decode_servo_status(view_of(STATUS_FRAME_1))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.consumed, STATUS_FRAME_1.size());
  EXPECT_EQ(result.rejected, 0U);
  EXPECT_EQ(result.value.id, 1);
  EXPECT_EQ(result.value.error, 0);
  EXPECT_EQ(result.value.position, 2024);
  EXPECT_EQ(result.value.speed, 0);
  EXPECT_EQ(result.value.load, 0);
  EXPECT_EQ(result.value.voltage, 121);
  EXPECT_EQ(result.value.temperature, 35);
  EXPECT_FALSE(result.value.moving);
}

TEST(DecodeServoStatus, TheSixFramesOfTheRecordedBurstInOrderAndNothingElse)
{
  // The ack, the ping of servo 7, the sync-read and the sync-write in the burst are not status
  // frames: six frames come out, ids 1 to 6, then NOT_FOUND with the rest consumed.
  constexpr std::array<std::uint16_t, 6> POSITIONS{2024, 2050, 2123, 2112, 2068, 2271};
  Bytes rx{RECORDED_AFTER_SEND};
  for (std::size_t joint{0}; joint < 6; ++joint) {
    const auto result{pp::decode_servo_status(view_of(rx))};
    ASSERT_EQ(result.status, pp::DecodeStatus::FOUND) << joint;
    EXPECT_EQ(result.value.id, joint + 1);
    EXPECT_EQ(result.value.position, POSITIONS.at(joint));
    EXPECT_EQ(result.value.error, 0);
    rx.erase(rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(result.consumed));
  }
  const auto rest{pp::decode_servo_status(view_of(rx))};
  EXPECT_EQ(rest.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(rest.consumed, rx.size());
}

TEST(DecodeServoStatus, EveryCutPosition)
{
  for (std::size_t cut{1}; cut < STATUS_FRAME_1.size(); ++cut) {
    const auto result{pp::decode_servo_status(prefix_of(STATUS_FRAME_1, cut))};
    EXPECT_EQ(result.status, pp::DecodeStatus::NEED_MORE_DATA) << "cut " << cut;
    EXPECT_EQ(result.consumed, 0U) << "cut " << cut;
  }
}

TEST(DecodeServoStatus, BadChecksumIsRejectedAndTheNextGoodFrameIsFound)
{
  Bytes bad{STATUS_FRAME_1};
  bad.back() = static_cast<std::uint8_t>(bad.back() + 1);
  const auto alone{pp::decode_servo_status(view_of(bad))};
  EXPECT_EQ(alone.status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(alone.consumed, bad.size());
  EXPECT_EQ(alone.rejected, 1U);

  const Bytes both{concat({bad, STATUS_FRAME_1})};
  const auto result{pp::decode_servo_status(view_of(both))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.consumed, both.size());
  EXPECT_EQ(result.rejected, 1U);
  EXPECT_EQ(result.value.position, 2024);
}

TEST(DecodeServoStatus, TheGripperAnswersToo)
{
  Bytes frame{STATUS_FRAME_1};
  frame.at(2) = pp::GRIPPER_SERVO_ID;
  frame = with_servo_checksum(frame);
  const auto result{pp::decode_servo_status(view_of(frame))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value.id, pp::GRIPPER_SERVO_ID);
}

TEST(DecodeServoStatus, ErrorByteSpeedLoadAndMovingAreDecoded)
{
  // Servo 5 pushing backwards: error byte 0x20, speed -100 steps/s (0x8064), load -5.6 %
  // (0x0438: sign bit 10), moving.
  Bytes frame{parse_hex("ff ff 05 11 20 0c 08 64 80 38 04 4b 30 00 00 80 0c 08 00 00 00")};
  frame = with_servo_checksum(frame);
  const auto result{pp::decode_servo_status(view_of(frame))};
  ASSERT_EQ(result.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(result.value.id, 5);
  EXPECT_EQ(result.value.error, 0x20);
  EXPECT_EQ(result.value.position, 2060);
  EXPECT_EQ(result.value.speed, -100);
  EXPECT_EQ(result.value.load, -56);
  EXPECT_EQ(result.value.voltage, 75);
  EXPECT_EQ(result.value.temperature, 48);
  EXPECT_TRUE(result.value.moving);
}

TEST(DecodeServoStatus, OtherFramesAreNotIt)
{
  EXPECT_EQ(
    pp::decode_servo_status(view_of(RECORDED_SYNC_WRITE)).status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(pp::decode_servo_status(view_of(ANGLES_FRAME)).status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(pp::decode_servo_status(view_of(TEMPS_FRAME)).status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(
    pp::decode_sync_write_echo(view_of(STATUS_FRAME_1)).status, pp::DecodeStatus::NOT_FOUND);
  EXPECT_EQ(pp::decode_servo_status(pp::ByteView{}).status, pp::DecodeStatus::NOT_FOUND);
}

// The Atom's conversion, against what pymycobot reported for the same servo positions on
// 19 and 20 Nov 2026: 1994 steps was 4.75 deg on J1, 2060 was -1.05 on J5, 3758 was 150.29 on J3.
TEST(DecodeServoStatus, AngleFromSteps)
{
  EXPECT_EQ(pp::angle_from_steps(2048, 0), 0);
  EXPECT_EQ(pp::angle_from_steps(1994, 0), 475);
  EXPECT_EQ(pp::angle_from_steps(2060, 4), -105);
  EXPECT_EQ(pp::angle_from_steps(2053, 1), -44);
  EXPECT_EQ(pp::angle_from_steps(3758, 2), 15029);
  EXPECT_EQ(pp::angle_from_steps(1623, 3), 3735);
}
