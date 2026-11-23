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

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <filesystem>
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

constexpr std::uint8_t GRIPPER_SPEED{20};
constexpr std::uint8_t GET_GRIPPER_VALUE{0x65};
constexpr std::uint8_t SET_GRIPPER_VALUE{0x67};

struct Target
{
  const char * section{nullptr};
  std::uint8_t value{0};
};

constexpr std::array<Target, 4> TARGETS{{
  {"set_gripper_open", 100},
  {"set_gripper_half", 50},
  {"set_gripper_closed", 0},
  {"set_gripper_open_again", 100},
}};

std::string capture_file()
{
  std::vector<std::string> files{};
  for (const auto & entry : std::filesystem::directory_iterator{TEST_DATA_DIR}) {
    const std::string name{entry.path().filename().string()};
    if (name.rfind("frames_gripper_", 0) == 0 && entry.path().extension() == ".txt") {
      files.push_back(entry.path().string());
    }
  }
  std::sort(files.begin(), files.end());
  return files.empty() ? std::string{} : files.back();
}

const std::vector<Section> & capture()
{
  static const std::vector<Section> sections{
    capture_file().empty() ? std::vector<Section>{} :
    capture_parser::parse_capture(capture_file())};
  return sections;
}

bool has_capture()
{
  return !capture_file().empty();
}

std::optional<int> result_value(const std::string & result)
{
  if (result.empty() || result.size() > 3) {
    return std::nullopt;
  }
  for (const char character : result) {
    if (std::isdigit(static_cast<unsigned char>(character)) == 0) {
      return std::nullopt;
    }
  }
  return std::stoi(result);
}

std::optional<std::uint8_t> target_of(const std::string & section)
{
  for (const auto & target : TARGETS) {
    if (section == target.section) {
      return target.value;
    }
  }
  return std::nullopt;
}

bool is_gripper_read(const Section & section)
{
  return capture_parser::first_command(section) == std::optional<std::uint8_t>{GET_GRIPPER_VALUE};
}

}  // namespace

TEST(RecordedGripper, CaptureLoads)
{
  if (!has_capture()) {
    GTEST_SKIP() << "no frames_gripper_*.txt in " << TEST_DATA_DIR;
  }
  for (const auto & target : TARGETS) {
    const bool found{std::any_of(
        capture().begin(), capture().end(),
        [&target](const Section & section) {return section.name == target.section;})};
    EXPECT_TRUE(found) << target.section;
  }
  EXPECT_TRUE(std::any_of(capture().begin(), capture().end(), is_gripper_read));
}

TEST(RecordedGripper, RxEqualsItsBursts)
{
  if (!has_capture()) {
    GTEST_SKIP() << "no frames_gripper_*.txt in " << TEST_DATA_DIR;
  }
  for (const auto & section : capture()) {
    Bytes glued{};
    for (const auto & burst : section.rx_bursts) {
      glued.insert(glued.end(), burst.begin(), burst.end());
    }
    EXPECT_EQ(glued, section.rx) << section.name;
  }
}

TEST(RecordedGripper, TxMatchesEncoder)
{
  if (!has_capture()) {
    GTEST_SKIP() << "no frames_gripper_*.txt in " << TEST_DATA_DIR;
  }
  std::size_t reads{0};
  std::size_t sets{0};
  for (const auto & section : capture()) {
    for (const auto & sent : capture_parser::split_frames(section.tx)) {
      if (sent.at(3) == GET_GRIPPER_VALUE) {
        EXPECT_EQ(to_hex(sent), to_hex(bytes_of(pp::encode_get_gripper_value()))) << section.name;
        ++reads;
      } else if (sent.at(3) == SET_GRIPPER_VALUE) {
        const auto value{target_of(section.name)};
        ASSERT_TRUE(value.has_value()) << section.name << ": not in TARGETS";
        const auto expected{pp::encode_set_gripper_value(value.value_or(0), GRIPPER_SPEED)};
        ASSERT_TRUE(expected.has_value()) << section.name;
        EXPECT_EQ(to_hex(sent), to_hex(bytes_of(expected.value_or(pp::Frame{})))) << section.name;
        ++sets;
      }
    }
  }
  EXPECT_GE(reads, 1U);
  EXPECT_GE(sets, TARGETS.size());
}

TEST(RecordedGripper, GripperValueMatchesResult)
{
  if (!has_capture()) {
    GTEST_SKIP() << "no frames_gripper_*.txt in " << TEST_DATA_DIR;
  }
  std::size_t checked{0};
  for (const auto & section : capture()) {
    if (!is_gripper_read(section)) {
      continue;
    }
    const auto result{pp::decode_ack(view_of(section.rx), pp::CommandId::GET_GRIPPER_VALUE)};
    const auto expected{result_value(section.result)};
    if (!expected.has_value()) {
      EXPECT_NE(result.status, pp::DecodeStatus::FOUND) << section.name;
      continue;
    }
    ASSERT_EQ(result.status, pp::DecodeStatus::FOUND) << section.name;
    EXPECT_EQ(int{result.value}, expected.value_or(-1)) << section.name;
    ++checked;
  }
  EXPECT_GE(checked, 1U);
}

TEST(RecordedGripper, ReplayInTheRealBursts)
{
  if (!has_capture()) {
    GTEST_SKIP() << "no frames_gripper_*.txt in " << TEST_DATA_DIR;
  }
  std::size_t replayed{0};
  for (const auto & section : capture()) {
    if (
      !is_gripper_read(section) ||
      pp::decode_ack(view_of(section.rx), pp::CommandId::GET_GRIPPER_VALUE).status !=
      pp::DecodeStatus::FOUND)
    {
      continue;
    }
    Bytes buffer{};
    int found{0};
    for (const auto & burst : section.rx_bursts) {
      buffer.insert(buffer.end(), burst.begin(), burst.end());
      const auto outcome{pp::decode_ack(view_of(buffer), pp::CommandId::GET_GRIPPER_VALUE)};
      ASSERT_LE(outcome.consumed, buffer.size()) << section.name;
      found += (outcome.status == pp::DecodeStatus::FOUND) ? 1 : 0;
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(outcome.consumed));
    }
    EXPECT_EQ(found, 1) << section.name;
    ++replayed;
  }
  EXPECT_GE(replayed, 1U);
}
