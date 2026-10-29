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
//   Write the GoogleTest "judge" that runs the library against the recorded capture: each
//   TX frame must equal the encoder output, each get_angles RX must decode to the value
//   pymycobot returned, the RX line must equal its RXT bursts glued together, send_angles
//   is acked at once, the stop-during-motion sequence works, and a replay fed in the real
//   UART bursts yields exactly one reply per request.

// The "judge": the library against bytes recorded on the real arm.
//
// For every section of the capture:
//   - each frame we sent must equal what our encoder builds;
//   - our decoder must find, in the raw RX, the same value pymycobot returned (RESULT).
// If the C++ disagrees with both the bytes and RESULT, the bug is ours. If the bytes and
// RESULT disagree with each other, the capture itself is broken.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "capture_parser.hpp"
#include "positronic_protocol/decode.hpp"
#include "positronic_protocol/encode.hpp"
#include "positronic_protocol/types.hpp"
#include "test_helpers.hpp"

namespace
{

namespace pp = positronic_protocol;
using capture_parser::Section;
using test_helpers::Bytes;
using test_helpers::bytes_of;
using test_helpers::to_hex;
using test_helpers::view_of;

constexpr const char * CAPTURE_FILE{TEST_DATA_DIR "/frames_20261016_124447.txt"};

/// Section whose RESULT is the start pose of the motions (the last of the 20 reads).
constexpr const char * BASE_POSE_SECTION{"get_angles_19"};

/// The motions of capture_frames.py (S5 and S6), relative to the start pose.
struct Motion
{
  const char * section{nullptr};
  int j1_offset{0};       // 0.01 deg; +30 deg because the start J1 was negative
  std::uint8_t speed{0};  // SPEED = 20, STOP_TEST_SPEED = 10 in the script
};

constexpr std::array<Motion, 3> MOTIONS{{
  {"send_angles_j1_out", 3000, 20},
  {"stop_during_motion", 0, 10},
  {"send_angles_j1_back", 0, 20},
}};

const std::vector<Section> & capture()
{
  static const std::vector<Section> sections{capture_parser::parse_capture(CAPTURE_FILE)};
  return sections;
}

/// A copy of the section called `name`, or an empty section (empty name) when there is none.
Section section_named(const std::string & name)
{
  const auto & sections{capture()};
  const auto it{std::find_if(sections.begin(), sections.end(), [&name](const Section & s) {
        return s.name == name;
  })};
  return it == sections.end() ? Section{} : *it;
}

std::optional<pp::JointAngles> as_joint_angles(const std::vector<int> & values)
{
  if (values.size() != pp::NUM_JOINTS) {
    return std::nullopt;
  }
  pp::JointAngles angles{};
  std::transform(values.begin(), values.end(), angles.begin(), [](int value) {
      return static_cast<pp::CentiDegrees>(value);
  });
  return angles;
}

/// What the encoder should have produced for a send_angles frame of `section`.
std::optional<Bytes> expected_send_angles(const std::string & section)
{
  const Section base{section_named(BASE_POSE_SECTION)};
  const auto values{capture_parser::centidegrees_from_result(base.result)};
  if (!values.has_value()) {
    return std::nullopt;
  }
  for (const auto & motion : MOTIONS) {
    if (section == motion.section) {
      std::vector<int> target{*values};
      target.front() += motion.j1_offset;
      const auto angles{as_joint_angles(target)};
      if (!angles.has_value()) {
        return std::nullopt;
      }
      const auto frame{pp::encode_send_angles(*angles, motion.speed)};
      return frame.has_value() ? std::optional<Bytes>{bytes_of(*frame)} : std::nullopt;
    }
  }
  return std::nullopt;
}

/// Decodes the reply expected for `command` and returns {status, consumed}.
struct Outcome
{
  pp::DecodeStatus status{pp::DecodeStatus::NOT_FOUND};
  std::size_t consumed{0};
};

Outcome decode_reply(std::uint8_t command, const Bytes & rx)
{
  if (command == static_cast<std::uint8_t>(pp::CommandId::GET_ANGLES)) {
    const auto result{pp::decode_get_angles_reply(view_of(rx))};
    return Outcome{result.status, result.consumed};
  }
  const auto result{pp::decode_ack(view_of(rx), static_cast<pp::CommandId>(command))};
  return Outcome{result.status, result.consumed};
}

bool is_one_of_our_commands(std::uint8_t command)
{
  const std::array<pp::CommandId, 6> ours{
    pp::CommandId::POWER_ON, pp::CommandId::POWER_OFF, pp::CommandId::RELEASE_ALL_SERVOS,
    pp::CommandId::GET_ANGLES, pp::CommandId::SEND_ANGLES, pp::CommandId::STOP};
  return std::any_of(ours.begin(), ours.end(), [command](pp::CommandId id) {
             return command == static_cast<std::uint8_t>(id);
  });
}

}  // namespace

TEST(RecordedFrames, CaptureLoads)
{
  ASSERT_GE(capture().size(), 40U) << "could not read " << CAPTURE_FILE;
  for (const char * name :
    {"power_on", "get_angles_00", "send_angles_j1_out", "stop_during_motion", "power_off",
      BASE_POSE_SECTION})
  {
    EXPECT_EQ(section_named(name).name, name);
  }
}

TEST(RecordedFrames, RxEqualsItsBursts)
{
  // Integrity of the judge: the RX line must be exactly the RXT bursts glued together.
  for (const auto & section : capture()) {
    Bytes glued{};
    for (const auto & burst : section.rx_bursts) {
      glued.insert(glued.end(), burst.begin(), burst.end());
    }
    EXPECT_EQ(glued, section.rx) << section.name;
  }
}

TEST(RecordedFrames, TxMatchesEncoder)
{
  std::size_t checked{0};
  for (const auto & section : capture()) {
    for (const auto & sent : capture_parser::split_frames(section.tx)) {
      const auto command{static_cast<pp::CommandId>(sent.at(3))};
      std::optional<Bytes> expected{};
      switch (command) {
        case pp::CommandId::POWER_ON:
          expected = bytes_of(pp::encode_power_on());
          break;
        case pp::CommandId::POWER_OFF:
          expected = bytes_of(pp::encode_power_off());
          break;
        case pp::CommandId::GET_ANGLES:
          expected = bytes_of(pp::encode_get_angles());
          break;
        case pp::CommandId::RELEASE_ALL_SERVOS:
          expected = bytes_of(pp::encode_release_all_servos());
          break;
        case pp::CommandId::STOP:
          expected = bytes_of(pp::encode_stop());
          break;
        case pp::CommandId::SEND_ANGLES:
          expected = expected_send_angles(section.name);
          ASSERT_TRUE(expected.has_value()) << section.name << ": not in MOTIONS";
          break;
        default:
          continue;  // a pymycobot command the library does not implement
      }
      EXPECT_EQ(to_hex(sent), to_hex(expected.value_or(Bytes{}))) << section.name;
      ++checked;
    }
  }
  // 1 power_on + 1 power_off + 34 get_angles (31 reads + 3 retries with the arm off)
  // + 3 send_angles + 1 stop + 1 release_all_servos. A lower count means a section
  // stopped being checked.
  EXPECT_EQ(checked, 41U);
}

TEST(RecordedFrames, AnglesMatchResult)
{
  std::size_t checked{0};
  for (const auto & section : capture()) {
    if (capture_parser::first_command(section) != std::uint8_t{0x20}) {
      continue;
    }
    const auto result{pp::decode_get_angles_reply(view_of(section.rx))};
    const auto values{capture_parser::centidegrees_from_result(section.result)};
    if (!values.has_value()) {
      // pymycobot gave up (RESULT -1, power_off): there must be no reply to find.
      EXPECT_NE(result.status, pp::DecodeStatus::FOUND) << section.name;
      continue;
    }
    const auto expected{as_joint_angles(*values)};
    ASSERT_TRUE(expected.has_value()) << section.name;
    ASSERT_EQ(result.status, pp::DecodeStatus::FOUND) << section.name;
    EXPECT_EQ(result.value, expected.value_or(pp::JointAngles{})) << section.name;
    EXPECT_EQ(result.consumed, section.rx.size()) << section.name;
    ++checked;
  }
  EXPECT_EQ(checked, 31U);  // 32 get_angles sections, minus the one with the arm off
}

TEST(RecordedFrames, TorqueAcksMatchResult)
{
  for (const auto & [name, command] :
    {std::pair{"power_on", pp::CommandId::POWER_ON},
      std::pair{"power_off", pp::CommandId::POWER_OFF},
      std::pair{"release_all_servos", pp::CommandId::RELEASE_ALL_SERVOS}})
  {
    const Section section{section_named(name)};
    ASSERT_EQ(section.name, name);
    const auto result{pp::decode_ack(view_of(section.rx), command)};
    ASSERT_EQ(result.status, pp::DecodeStatus::FOUND) << name;
    EXPECT_EQ(std::to_string(result.value), section.result) << name;
  }
}

TEST(RecordedFrames, SendAnglesIsAckedRightAway)
{
  // Q2: the ack arrives ~3 ms after the request, not at the end of the motion.
  for (const char * name : {"send_angles_j1_out", "send_angles_j1_back"}) {
    const Section section{section_named(name)};
    ASSERT_FALSE(section.rx_bursts.empty()) << name;
    const auto in_first_burst{
      pp::decode_ack(view_of(section.rx_bursts.front()), pp::CommandId::SEND_ANGLES)};
    ASSERT_EQ(in_first_burst.status, pp::DecodeStatus::FOUND) << name;
    EXPECT_EQ(in_first_burst.value, 1) << name;
  }
}

TEST(RecordedFrames, StopDuringMotion)
{
  const Bytes rx{section_named("stop_during_motion").rx};
  ASSERT_FALSE(rx.empty());

  const auto send_ack{pp::decode_ack(view_of(rx), pp::CommandId::SEND_ANGLES)};
  ASSERT_EQ(send_ack.status, pp::DecodeStatus::FOUND);

  // After the send_angles ack: ~500 ms of servo traffic, then the stop ack.
  const Bytes rest{rx.begin() + static_cast<std::ptrdiff_t>(send_ack.consumed), rx.end()};
  const auto stop_ack{pp::decode_ack(view_of(rest), pp::CommandId::STOP)};
  ASSERT_EQ(stop_ack.status, pp::DecodeStatus::FOUND);
  EXPECT_EQ(stop_ack.value, 1);
  EXPECT_EQ(stop_ack.consumed, rest.size());
}

TEST(RecordedFrames, ReplayInTheRealBursts)
{
  // Feed the bytes in the pieces the UART really delivered, the way #22 will:
  // append, decode, drop `consumed`. Each request must get exactly one reply.
  std::size_t replayed{0};
  for (const auto & section : capture()) {
    const std::uint8_t command{capture_parser::first_command(section).value_or(0)};
    if (
      !is_one_of_our_commands(command) ||
      decode_reply(command, section.rx).status != pp::DecodeStatus::FOUND)
    {
      continue;
    }
    Bytes buffer{};
    int found{0};
    for (const auto & burst : section.rx_bursts) {
      buffer.insert(buffer.end(), burst.begin(), burst.end());
      const Outcome outcome{decode_reply(command, buffer)};
      ASSERT_LE(outcome.consumed, buffer.size()) << section.name;
      found += (outcome.status == pp::DecodeStatus::FOUND) ? 1 : 0;
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(outcome.consumed));
    }
    EXPECT_EQ(found, 1) << section.name;
    ++replayed;
  }
  // power_on, power_off, release_all_servos, 31 get_angles, 3 send_angles
  EXPECT_EQ(replayed, 37U);
}
