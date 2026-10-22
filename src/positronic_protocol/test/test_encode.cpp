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
//   Write a GoogleTest suite for encode.hpp. Check that each no-payload command encodes
//   byte-for-byte to its TX line in the 2026-10-16 capture, that LEN counts CMD + data +
//   footer, and for send_angles: the two capture frames, the frame shape, int16 big-endian
//   signed joints, the joint order, and that an out-of-range speed returns nullopt.

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

#include "positronic_protocol/encode.hpp"
#include "positronic_protocol/types.hpp"
#include "test_helpers.hpp"

namespace
{

namespace pp = positronic_protocol;
using test_helpers::Bytes;
using test_helpers::bytes_of;
using test_helpers::parse_hex;
using test_helpers::to_hex;

/// Encodes a single joint value (J1) and returns its two bytes on the wire.
Bytes j1_bytes(pp::CentiDegrees value)
{
  const std::optional<pp::Frame> frame{pp::encode_send_angles({value, 0, 0, 0, 0, 0}, 20)};
  if (!frame.has_value()) {
    return {};
  }
  const Bytes bytes{bytes_of(frame.value_or(pp::Frame{}))};
  return Bytes{bytes.at(4), bytes.at(5)};
}

}  // namespace

// T3: the commands without data. Expected bytes are the TX lines of the
// 2026-10-16 capture (test/data/frames_20261016_124447.txt).

TEST(Encode, PowerOn)
{
  EXPECT_EQ(to_hex(bytes_of(pp::encode_power_on())), "fe fe 02 10 fa");
}

TEST(Encode, PowerOff)
{
  EXPECT_EQ(to_hex(bytes_of(pp::encode_power_off())), "fe fe 02 11 fa");
}

TEST(Encode, Stop)
{
  EXPECT_EQ(to_hex(bytes_of(pp::encode_stop())), "fe fe 02 29 fa");
}

TEST(Encode, ReleaseAllServos)
{
  EXPECT_EQ(to_hex(bytes_of(pp::encode_release_all_servos())), "fe fe 02 13 fa");
}

TEST(Encode, GetAngles)
{
  EXPECT_EQ(to_hex(bytes_of(pp::encode_get_angles())), "fe fe 02 20 fa");
}

TEST(Encode, LenCountsCmdDataAndFooter)
{
  // LEN = frame size - 3 (FE FE LEN are not counted).
  for (const auto & frame :
    {pp::encode_power_on(), pp::encode_power_off(), pp::encode_release_all_servos(),
      pp::encode_stop(), pp::encode_get_angles()})
  {
    ASSERT_EQ(frame.size, 5U);
    EXPECT_EQ(frame.bytes.at(2), frame.size - 3);
  }
}

// T4: send_angles.

TEST(EncodeSendAngles, MatchesCaptureJ1Out)
{
  // send_angles_j1_out: J1 at 29.92 deg, the rest at the start pose, speed 20.
  const auto frame{pp::encode_send_angles({2992, -17, -61, -123, 43, 61}, 20)};
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(
    to_hex(bytes_of(frame.value_or(pp::Frame{}))),
    "fe fe 0f 22 0b b0 ff ef ff c3 ff 85 00 2b 00 3d 14 fa");
}

TEST(EncodeSendAngles, MatchesCaptureStopTest)
{
  // stop_during_motion: back to the start pose at speed 10.
  const auto frame{pp::encode_send_angles({-8, -17, -61, -123, 43, 61}, 10)};
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(
    to_hex(bytes_of(frame.value_or(pp::Frame{}))),
    "fe fe 0f 22 ff f8 ff ef ff c3 ff 85 00 2b 00 3d 0a fa");
}

TEST(EncodeSendAngles, FrameShape)
{
  const auto frame{pp::encode_send_angles({}, 50)};
  ASSERT_TRUE(frame.has_value());
  const pp::Frame built{frame.value_or(pp::Frame{})};
  ASSERT_EQ(built.size, 18U);
  EXPECT_EQ(built.bytes.at(0), pp::HEADER);
  EXPECT_EQ(built.bytes.at(1), pp::HEADER);
  EXPECT_EQ(built.bytes.at(2), 0x0F);
  EXPECT_EQ(built.bytes.at(3), 0x22);
  EXPECT_EQ(built.bytes.at(16), 50);  // speed
  EXPECT_EQ(built.bytes.at(17), pp::FOOTER);
}

TEST(EncodeSendAngles, BigEndianSigned)
{
  EXPECT_EQ(j1_bytes(3000), parse_hex("0b b8"));
  EXPECT_EQ(j1_bytes(-3000), parse_hex("f4 48"));
  EXPECT_EQ(j1_bytes(9000), parse_hex("23 28"));
  EXPECT_EQ(j1_bytes(-61), parse_hex("ff c3"));
  EXPECT_EQ(j1_bytes(-1), parse_hex("ff ff"));
  EXPECT_EQ(j1_bytes(32767), parse_hex("7f ff"));
  EXPECT_EQ(j1_bytes(-32768), parse_hex("80 00"));
}

TEST(EncodeSendAngles, JointOrder)
{
  const auto frame{pp::encode_send_angles({1, 2, 3, 4, 5, 6}, 20)};
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(
    to_hex(bytes_of(frame.value_or(pp::Frame{}))),
    "fe fe 0f 22 00 01 00 02 00 03 00 04 00 05 00 06 14 fa");
}

TEST(EncodeSendAngles, SpeedLimits)
{
  EXPECT_FALSE(pp::encode_send_angles({}, 0).has_value());
  EXPECT_FALSE(pp::encode_send_angles({}, 101).has_value());
  EXPECT_FALSE(pp::encode_send_angles({}, 255).has_value());
  EXPECT_TRUE(pp::encode_send_angles({}, pp::MIN_SPEED).has_value());
  EXPECT_TRUE(pp::encode_send_angles({}, pp::MAX_SPEED).has_value());
}
