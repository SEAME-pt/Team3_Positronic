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
//   Write a GoogleTest suite for reply_assembler.hpp, which turns serial bytes into the newest
//   get_angles reply. Build replies from known angles (FE FE 0E 20, six big-endian int16 in
//   0.01 deg, FA) and check: a whole reply decodes to radians; a reply split over three pushes
//   completes only on the last; servo-bus noise and the echo of our own request before a reply
//   are skipped; of two replies in one push the newest wins; noise alone gives nothing and does
//   not hide a later reply; clear() drops a partial reply.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/reply_assembler.hpp"

namespace
{
using positronic_hardware::JointPositions;
using positronic_hardware::NUM_JOINTS;
using positronic_hardware::ReplyAssembler;

using Centidegrees = std::array<std::int16_t, NUM_JOINTS>;

constexpr double PI{3.14159265358979323846};

/// Positive, negative and zero angles, up to the arm's joint limit of about 165 deg.
constexpr Centidegrees ANGLES{1000, -500, 0, 9000, -16500, 12345};
constexpr Centidegrees OTHER_ANGLES{-10, 20, -30, 40, -50, 60};

/// A complete get_angles reply: FE FE 0E 20, six big-endian int16 in 0.01 deg, FA.
std::vector<std::uint8_t> reply(const Centidegrees & angles)
{
  std::vector<std::uint8_t> bytes{0xFE, 0xFE, 0x0E, 0x20};
  for (const std::int16_t angle : angles) {
    const auto raw{static_cast<std::uint16_t>(angle)};
    bytes.push_back(static_cast<std::uint8_t>(raw >> 8U));
    bytes.push_back(static_cast<std::uint8_t>(raw & 0xFFU));
  }
  bytes.push_back(0xFA);
  return bytes;
}

/// Servo-bus traffic, as seen on the shared UART in the capture of #21.
const std::vector<std::uint8_t> SERVO_NOISE{0xFF, 0xFF, 0x01, 0x11, 0x00, 0x01, 0x08,
  0x00, 0x00, 0x00, 0x00, 0x79, 0x26};

/// The echo of our own get_angles request, which the arm's UART sends back.
const std::vector<std::uint8_t> REQUEST_ECHO{0xFE, 0xFE, 0x02, 0x20, 0xFA};

std::vector<std::uint8_t> join(
  std::vector<std::uint8_t> first, const std::vector<std::uint8_t> & second)
{
  first.insert(first.end(), second.begin(), second.end());
  return first;
}

std::optional<JointPositions> push(
  ReplyAssembler & assembler, const std::vector<std::uint8_t> & bytes)
{
  return assembler.push(bytes.data(), bytes.size());
}

void expect_angles(const std::optional<JointPositions> & pose, const Centidegrees & angles)
{
  ASSERT_TRUE(pose.has_value());
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    const double expected{angles.at(i) / 100.0 * PI / 180.0};
    EXPECT_NEAR(pose.value_or(JointPositions{}).at(i), expected, 1e-9) << i;
  }
}

}  // namespace

TEST(ReplyAssembler, DecodesAWholeReply)
{
  ReplyAssembler assembler;
  expect_angles(push(assembler, reply(ANGLES)), ANGLES);
}

TEST(ReplyAssembler, CompletesAReplySplitOverThreeReads)
{
  // In the capture, one reply arrived in three bursts. Only the last one completes it.
  const std::vector<std::uint8_t> bytes{reply(ANGLES)};
  const std::vector<std::uint8_t> first(bytes.begin(), bytes.begin() + 3);
  const std::vector<std::uint8_t> second(bytes.begin() + 3, bytes.begin() + 10);
  const std::vector<std::uint8_t> third(bytes.begin() + 10, bytes.end());

  ReplyAssembler assembler;
  EXPECT_FALSE(push(assembler, first).has_value());
  EXPECT_FALSE(push(assembler, second).has_value());
  expect_angles(push(assembler, third), ANGLES);
}

TEST(ReplyAssembler, SkipsNoiseAndTheRequestEchoBeforeAReply)
{
  ReplyAssembler assembler;
  expect_angles(push(assembler, join(join(SERVO_NOISE, REQUEST_ECHO), reply(ANGLES))), ANGLES);
}

TEST(ReplyAssembler, NewestOfTwoRepliesWins)
{
  // The control loop wants the freshest pose, not a backlog.
  ReplyAssembler assembler;
  expect_angles(push(assembler, join(reply(ANGLES), reply(OTHER_ANGLES))), OTHER_ANGLES);
}

TEST(ReplyAssembler, NoiseAloneGivesNothingAndDoesNotHideALaterReply)
{
  ReplyAssembler assembler;
  // Far more noise than MAX_BUFFER, so the cap is reached many times over.
  for (int i{0}; i < 100; ++i) {
    EXPECT_FALSE(push(assembler, SERVO_NOISE).has_value());
  }
  expect_angles(push(assembler, reply(ANGLES)), ANGLES);
}

TEST(ReplyAssembler, ClearDropsAPartialReply)
{
  const std::vector<std::uint8_t> bytes{reply(ANGLES)};
  const std::vector<std::uint8_t> head(bytes.begin(), bytes.begin() + 8);
  const std::vector<std::uint8_t> tail(bytes.begin() + 8, bytes.end());

  ReplyAssembler assembler;
  EXPECT_FALSE(push(assembler, head).has_value());
  assembler.clear();
  // The tail alone is not a reply: the head it belonged to is gone.
  EXPECT_FALSE(push(assembler, tail).has_value());
}
