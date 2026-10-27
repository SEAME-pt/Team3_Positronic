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
//   Write a GoogleTest suite for serial_arm.hpp. SerialArm implements ArmBackend by running a
//   dedicated thread that polls the arm over a SerialTransport, assembles get_angles replies and
//   publishes the latest pose through a RealtimeBuffer; read_angles() only copies that buffer.
//   Drive it with a fake transport (no hardware) that replays the exact three UART bursts
//   recorded on the real arm in the get_angles_19 section of frames_20261016_124447.txt. Check
//   that: read_angles() returns the start pose (all zeros, never NaN) before any reply arrives;
//   after the recorded bursts it returns the six angles pymycobot reported for that section
//   (RESULT, in radians); and the destructor stops the thread and returns promptly even while
//   the transport is still streaming. The device I/O in PosixSerialPort is out of scope here and
//   is verified on the Jetson.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/serial_arm.hpp"
#include "positronic_hardware/serial_transport.hpp"

namespace
{
using positronic_hardware::JointPositions;
using positronic_hardware::NUM_JOINTS;
using positronic_hardware::SerialArm;
using positronic_hardware::SerialTransport;

/// A transport that answers every request with a fixed sequence of byte bursts, looping. It is
/// thread-safe because SerialArm reads it from its poll thread.
class ReplayTransport : public SerialTransport
{
public:
  explicit ReplayTransport(std::vector<std::vector<std::uint8_t>> bursts)
  : bursts_{std::move(bursts)}
  {
  }

  bool write(const std::uint8_t * /*data*/, std::size_t /*count*/) override
  {
    return true;  // the request is accepted; the reply is scripted below.
  }

  std::size_t read(std::uint8_t * dst, std::size_t max) override
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    if (bursts_.empty()) {
      return 0;
    }
    const std::vector<std::uint8_t> & burst{bursts_.at(index_)};
    index_ = (index_ + 1) % bursts_.size();
    const std::size_t n{std::min(max, burst.size())};
    std::copy_n(burst.begin(), n, dst);
    return n;
  }

private:
  std::vector<std::vector<std::uint8_t>> bursts_;
  std::size_t index_{0};
  std::mutex mutex_{};
};

/// Parses a space-separated hex string ("ff fe 0a") into bytes. Test-only.
std::vector<std::uint8_t> parse_hex(std::string_view hex)
{
  std::vector<std::uint8_t> bytes{};
  for (std::size_t i{0}; i + 1 < hex.size(); i += 3) {
    bytes.push_back(
      static_cast<std::uint8_t>(std::stoul(std::string{hex.substr(i, 2)}, nullptr, 16)));
  }
  return bytes;
}

/// The three RXT bursts of get_angles_19, verbatim, as hex. Kept as text so the formatters leave
/// the data alone: a 150-byte brace-init list of integers is wrapped differently by each tool.
const std::vector<std::vector<std::uint8_t>> & recorded_bursts()
{
  static const std::vector<std::vector<std::uint8_t>> bursts{
    parse_hex("ff ff"), parse_hex("07 02 01 f5 ff ff fe 0a 82"),
    parse_hex(
      "38 0f 01 02 03 04 05 06 19 ff ff 01 11 00 01 08 00 00 00 00 79 26 00 00 00 01 08 00 "
      "00 3c ff ff 02 11 00 02 08 00 00 00 00 79 23 00 00 00 02 08 00 00 3c ff ff 03 11 00 "
      "f9 07 00 00 00 00 7b 21 00 00 00 f9 07 00 00 4f ff ff 04 11 00 0e 08 00 00 00 00 4a "
      "21 00 00 00 0e 08 00 00 53 ff ff 05 11 00 fb 07 00 00 00 00 49 20 00 00 00 fb 07 00 "
      "00 7c ff ff 06 11 00 f9 07 00 00 00 00 49 23 00 00 00 f9 07 00 00 7c fe fe 0e 20 ff "
      "f8 ff ef ff c3 ff 85 00 2b 00 3d fa")};
  return bursts;
}

/// RESULT [-0.08, -0.17, -0.61, -1.23, 0.43, 0.61] in radians.
JointPositions expected_pose()
{
  constexpr double DEG2RAD{M_PI / 180.0};
  JointPositions pose{-0.08, -0.17, -0.61, -1.23, 0.43, 0.61};
  for (double & angle : pose) {
    angle *= DEG2RAD;
  }
  return pose;
}

/// Polls read_angles() until it leaves the start pose, or the timeout elapses.
JointPositions wait_for_reply(const SerialArm & arm)
{
  for (int tries{0}; tries < 200; ++tries) {
    const JointPositions pose{arm.read_angles()};
    bool moved{false};
    for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
      if (pose.at(i) != 0.0) {
        moved = true;
      }
    }
    if (moved) {
      return pose;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  return arm.read_angles();
}

}  // namespace

TEST(SerialArm, StartsAtStartPoseBeforeAnyReply)
{
  // A transport that never answers: read_angles() must give the start pose, never NaN.
  const std::vector<std::vector<std::uint8_t>> no_reply{};
  SerialArm arm{std::make_unique<ReplayTransport>(no_reply)};
  const JointPositions pose{arm.read_angles()};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_DOUBLE_EQ(pose.at(i), 0.0) << i;
  }
}

TEST(SerialArm, DecodesRecordedReply)
{
  SerialArm arm{std::make_unique<ReplayTransport>(recorded_bursts())};
  const JointPositions pose{wait_for_reply(arm)};
  const JointPositions expected{expected_pose()};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_NEAR(pose.at(i), expected.at(i), 1e-9) << i;
  }
}

TEST(SerialArm, StopsCleanly)
{
  // The destructor must stop the thread and return promptly, even mid-stream.
  const auto start{std::chrono::steady_clock::now()};
  {
    SerialArm arm{std::make_unique<ReplayTransport>(recorded_bursts())};
    std::this_thread::sleep_for(std::chrono::milliseconds{20});
  }
  const auto elapsed{std::chrono::steady_clock::now() - start};
  EXPECT_LT(elapsed, std::chrono::seconds{2});
}
