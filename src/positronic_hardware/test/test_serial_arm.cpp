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
// Follow-up prompt:
//   PosixSerialPort takes the port with TIOCEXCL. On a pseudo-terminal, check that a second
//   PosixSerialPort on the same device is refused while the first one is open.
// Follow-up prompt (#36):
//   SerialArm now has halt() and power_on(). Each leaves a request that the poll thread writes
//   at the start of its next turn, before the get_angles, and waits up to REQUEST_TIMEOUT for
//   it. halt() also holds every target until power_on(), and the stop forgets the last target
//   sent. Check: the stop goes out and no send_angles follows it while held, even for a command
//   stored before the stop; power_on() resumes sending and its frame goes out once; the target
//   sent before the stop goes out again after power_on(); halt() fails within the deadline on a
//   port that refuses writes; and halt() returns in time on a silent arm.
// Follow-up prompt (#36):
//   request_halt() is halt() without the wait, for on_error on the control loop. Check: it
//   returns at once, the stop still goes out, and targets are held until power_on().
// Follow-up prompt (#35):
//   Check that link_timed_out() is false while replies arrive, true once the line has been
//   silent past the timeout, false again when replies resume, and true when the arm never
//   answers at all.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/serial_arm.hpp"
#include "positronic_hardware/serial_transport.hpp"
#include "positronic_protocol/types.hpp"

namespace
{
using positronic_hardware::JointPositions;
using positronic_hardware::NUM_JOINTS;
using positronic_hardware::PosixSerialPort;
using positronic_hardware::SerialArm;
using positronic_hardware::SerialTransport;

constexpr std::uint8_t STOP_ID{static_cast<std::uint8_t>(positronic_protocol::CommandId::STOP)};
constexpr std::uint8_t POWER_ON_ID{
  static_cast<std::uint8_t>(positronic_protocol::CommandId::POWER_ON)};

/// A transport that answers every request with a fixed sequence of byte bursts, looping. It is
/// thread-safe because SerialArm reads it from its poll thread.
class ReplayTransport : public SerialTransport
{
public:
  explicit ReplayTransport(std::vector<std::vector<std::uint8_t>> bursts)
  : bursts_{std::move(bursts)}
  {
  }

  bool write(const std::uint8_t * data, std::size_t count) override
  {
    // A stop or power on is refused while reject_requests_ is set, as a busy port would. Every
    // other frame is accepted; the reply is scripted below. Accepted frames are kept so a test
    // can check what the thread put on the wire.
    if (reject_requests_.load() && count == 5) {
      const std::uint8_t id{*std::next(data, 3)};
      if (id == STOP_ID || id == POWER_ON_ID) {
        return false;
      }
    }
    const std::lock_guard<std::mutex> lock{mutex_};
    written_.emplace_back(data, std::next(data, static_cast<std::ptrdiff_t>(count)));
    return true;
  }

  /// A port that refuses the stop and power on frames only, until set_reject_requests(false).
  void set_reject_requests(bool reject)
  {
    reject_requests_.store(reject);
  }

  /// Every frame written so far, oldest first.
  std::vector<std::vector<std::uint8_t>> written()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return written_;
  }

  /// A silent arm: read() hands out nothing until set_silent(false).
  void set_silent(bool silent)
  {
    silent_.store(silent);
  }

  std::size_t read(std::uint8_t * dst, std::size_t max) override
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    if (silent_.load() || bursts_.empty()) {
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
  std::vector<std::vector<std::uint8_t>> written_;
  std::size_t index_{0};
  std::atomic<bool> silent_{false};
  std::atomic<bool> reject_requests_{false};
  std::mutex mutex_;
};

/// A port that refuses every write and never answers, as a cable pulled mid-session would.
class RejectingTransport : public SerialTransport
{
public:
  bool write(const std::uint8_t * /*data*/, std::size_t /*count*/) override
  {
    return false;
  }

  std::size_t read(std::uint8_t * /*dst*/, std::size_t /*max*/) override
  {
    return 0;
  }
};

/// The send_angles frames among `frames`: `FE FE 0F 22` + 12 bytes of angles + speed + `FA`.
std::vector<std::vector<std::uint8_t>> send_angles_frames(
  const std::vector<std::vector<std::uint8_t>> & frames)
{
  std::vector<std::vector<std::uint8_t>> found{};
  for (const auto & frame : frames) {
    if (frame.size() == 18 && frame[0] == 0xFE && frame[1] == 0xFE && frame[2] == 0x0F &&
      frame[3] == 0x22)
    {
      found.push_back(frame);
    }
  }
  return found;
}

/// The 5-byte request frames with command `id` among `frames`: `FE FE 02 id FA`.
std::vector<std::vector<std::uint8_t>> frames_with_id(
  const std::vector<std::vector<std::uint8_t>> & frames, std::uint8_t id)
{
  std::vector<std::vector<std::uint8_t>> found{};
  for (const auto & frame : frames) {
    if (frame.size() == 5 && frame[0] == 0xFE && frame[1] == 0xFE && frame[2] == 0x02 &&
      frame[3] == id && frame[4] == 0xFA)
    {
      found.push_back(frame);
    }
  }
  return found;
}

/// Waits until the thread has written `count` send_angles frames, or the timeout elapses.
std::vector<std::vector<std::uint8_t>> wait_for_sends(
  ReplayTransport & transport, std::size_t count)
{
  const auto deadline{std::chrono::steady_clock::now() + std::chrono::seconds{2}};
  while (std::chrono::steady_clock::now() < deadline) {
    const auto sends{send_angles_frames(transport.written())};
    if (sends.size() >= count) {
      return sends;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  return send_angles_frames(transport.written());
}

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
    parse_hex("38 0f 01 02 03 04 05 06 19 ff ff 01 11 00 01 08 00 00 00 00 79 26 00 00 00 01 08 00 "
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
  const SerialArm arm{std::make_unique<ReplayTransport>(no_reply)};
  EXPECT_FALSE(arm.has_sample());  // so the plugin knows this pose is not a measurement
  const JointPositions pose{arm.read_angles()};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_DOUBLE_EQ(pose.at(i), 0.0) << i;
  }
}

TEST(SerialArm, DecodesRecordedReply)
{
  const SerialArm arm{std::make_unique<ReplayTransport>(recorded_bursts())};
  const JointPositions pose{wait_for_reply(arm)};
  EXPECT_TRUE(arm.has_sample());
  const JointPositions expected{expected_pose()};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    EXPECT_NEAR(pose.at(i), expected.at(i), 1e-9) << i;
  }
}

TEST(SerialArm, SendsNothingBeforeTheFirstCommand)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  const SerialArm arm{std::move(transport)};
  wait_for_reply(arm);
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);

  // Only get_angles requests went out: no controller asked for anything yet.
  EXPECT_TRUE(send_angles_frames(port.written()).empty());
  EXPECT_FALSE(port.written().empty());
}

TEST(SerialArm, SendsTheCommandOnceAsASendAnglesFrame)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};

  // J1 = 0.1 rad = 572.96 -> 573 centidegrees = 0x023D big-endian; the other joints 0.
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  const auto sends{wait_for_sends(port, 1)};
  ASSERT_EQ(sends.size(), 1u);
  const std::vector<std::uint8_t> expected{0xFE, 0xFE, 0x0F, 0x22, 0x02, 0x3D, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, SerialArm::SEND_SPEED, 0xFA};
  EXPECT_EQ(sends.front(), expected);

  // The same command is not sent again: the arm already has it.
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(send_angles_frames(port.written()).size(), 1u);

  // A new command goes out once more.
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(wait_for_sends(port, 2).size(), 2u);
}

TEST(SerialArm, SkipsACommandItCannotEncode)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};

  arm.write_angles(JointPositions{std::nan(""), 0.0, 0.0, 0.0, 0.0, 0.0});
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_TRUE(send_angles_frames(port.written()).empty());
}

TEST(SerialArm, StopsCleanly)
{
  // The destructor must stop the thread and return promptly, even mid-stream.
  const auto start{std::chrono::steady_clock::now()};
  {
    const SerialArm arm{std::make_unique<ReplayTransport>(recorded_bursts())};
    std::this_thread::sleep_for(std::chrono::milliseconds{20});
  }
  const auto elapsed{std::chrono::steady_clock::now() - start};
  EXPECT_LT(elapsed, std::chrono::seconds{2});
}

TEST(SerialArm, HaltSendsAStopAndHoldsEveryTargetUntilPowerOn)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);

  EXPECT_TRUE(arm.halt());
  EXPECT_EQ(frames_with_id(port.written(), STOP_ID).size(), 1u);

  // A controller that keeps publishing while the component is inactive changes nothing.
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(send_angles_frames(port.written()).size(), 1u);

  // power_on() drops the target held during the stop: nothing goes out on its own.
  EXPECT_TRUE(arm.power_on());
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(send_angles_frames(port.written()).size(), 1u);

  // The next write_angles(), as the plugin's "stay where you are" after activate, is the
  // first to go out.
  arm.write_angles(JointPositions{0.3, 0.0, 0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(wait_for_sends(port, 2).size(), 2u);
}

TEST(SerialArm, ACommandStoredBeforeTheStopNeverGoesOut)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_silent(true);  // no reply yet, so the thread stores the command and sends nothing
  SerialArm arm{std::move(transport)};
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::this_thread::sleep_for(3 * SerialArm::REPLY_TIMEOUT);
  ASSERT_TRUE(send_angles_frames(port.written()).empty());

  EXPECT_TRUE(arm.halt());
  // The arm answers again: the stored command is held, not sent.
  port.set_silent(false);
  wait_for_reply(arm);
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(frames_with_id(port.written(), STOP_ID).size(), 1u);
  EXPECT_TRUE(send_angles_frames(port.written()).empty());
}

TEST(SerialArm, TheTargetSentBeforeTheStopGoesOutAgainAfterPowerOn)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  const JointPositions target{0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  arm.write_angles(target);
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);

  ASSERT_TRUE(arm.halt());
  ASSERT_TRUE(arm.power_on());
  // The stop interrupted that motion, so the same target is no longer "already there": the
  // thread sends it again even though it equals the last one it sent.
  arm.write_angles(target);
  EXPECT_EQ(wait_for_sends(port, 2).size(), 2u);
}

TEST(SerialArm, PowerOnSendsItsFrameOnce)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  wait_for_reply(arm);

  EXPECT_TRUE(arm.power_on());
  // The request is cleared once it is out: the next turns send no second one.
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(frames_with_id(port.written(), POWER_ON_ID).size(), 1u);
}

TEST(SerialArm, TheHoldStaysUntilThePowerOnIsOnTheWire)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  wait_for_reply(arm);
  ASSERT_TRUE(arm.halt());

  // The port refuses the power on: power_on() times out, and the request stays pending.
  port.set_reject_requests(true);
  EXPECT_FALSE(arm.power_on());

  // A target written while the power on is still pending must not get ahead of it: the hold is
  // released by the thread when the frame goes out, not by power_on() itself.
  arm.write_angles(JointPositions{0.3, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(send_angles_frames(port.written()).size(), 0u);
  EXPECT_EQ(frames_with_id(port.written(), POWER_ON_ID).size(), 0u);

  // The port takes the frame again: the pending power on goes out on its own, then the target.
  port.set_reject_requests(false);
  EXPECT_EQ(wait_for_sends(port, 1).size(), 1u);
  const auto frames{port.written()};
  const auto power_on_at{std::find_if(frames.begin(), frames.end(), [](const auto & frame) {
        return frame.size() == 5 && frame[3] == POWER_ON_ID;
  })};
  ASSERT_NE(power_on_at, frames.end());
  const std::vector<std::vector<std::uint8_t>> after_power_on{power_on_at, frames.end()};
  EXPECT_EQ(send_angles_frames(after_power_on).size(), 1u);
}

TEST(SerialArm, RequestHaltSendsTheStopWithoutWaiting)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);

  // No wait: well under one poll period, while halt() waits for the thread's next turn.
  const auto start{std::chrono::steady_clock::now()};
  arm.request_halt();
  EXPECT_LT(std::chrono::steady_clock::now() - start, SerialArm::POLL_PERIOD);

  // The stop still goes out, and a target after it is held.
  const auto deadline{std::chrono::steady_clock::now() + SerialArm::REQUEST_TIMEOUT};
  while (frames_with_id(port.written(), STOP_ID).empty() &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  EXPECT_EQ(frames_with_id(port.written(), STOP_ID).size(), 1u);
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(send_angles_frames(port.written()).size(), 1u);
}

TEST(SerialArm, HaltFailsWithinTheDeadlineWhenTheStopCannotBeWritten)
{
  SerialArm arm{std::make_unique<RejectingTransport>()};
  const auto start{std::chrono::steady_clock::now()};
  EXPECT_FALSE(arm.halt());
  const auto elapsed{std::chrono::steady_clock::now() - start};
  EXPECT_GE(elapsed, SerialArm::REQUEST_TIMEOUT);
  EXPECT_LT(elapsed, 2 * SerialArm::REQUEST_TIMEOUT);
}

TEST(SerialArm, HaltReachesASilentArmInTime)
{
  // No reply ever: a turn lasts REPLY_TIMEOUT, and the stop still goes out at the start of the
  // next one, well inside REQUEST_TIMEOUT.
  auto transport{std::make_unique<ReplayTransport>(std::vector<std::vector<std::uint8_t>>{})};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  const auto start{std::chrono::steady_clock::now()};
  EXPECT_TRUE(arm.halt());
  EXPECT_LT(std::chrono::steady_clock::now() - start, SerialArm::REQUEST_TIMEOUT);
  EXPECT_EQ(frames_with_id(port.written(), STOP_ID).size(), 1u);
}

// #35. SerialArm reads the real clock, so these tests must sleep. 300 ms is the 200 ms timeout
// plus a margin for a busy CI machine; the exact boundary is tested in test_link_watchdog.cpp.
constexpr std::chrono::milliseconds PAST_THE_TIMEOUT{300};

TEST(SerialArm, ReportsATimeoutWhileTheArmIsSilentAndClearsItWhenRepliesResume)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  const SerialArm arm{std::move(transport)};
  wait_for_reply(arm);
  ASSERT_TRUE(arm.has_sample());
  EXPECT_FALSE(arm.link_timed_out());  // replies keep arriving every turn

  port.set_silent(true);
  std::this_thread::sleep_for(PAST_THE_TIMEOUT);
  EXPECT_TRUE(arm.link_timed_out());

  // The backend does not latch: keeping the fault is the plugin's job.
  // With this transport a silent turn lasts about REPLY_TIMEOUT (50 ms), because read() returns
  // at once instead of waiting VTIME like the real port; 200 ms leaves room for the next reply.
  port.set_silent(false);
  std::this_thread::sleep_for(std::chrono::milliseconds{200});
  EXPECT_FALSE(arm.link_timed_out());
}

TEST(SerialArm, ReportsATimeoutWhenTheArmNeverAnswers)
{
  // The poll thread resets the watchdog when it starts, so silence from the first turn on
  // still times out.
  const SerialArm arm{std::make_unique<ReplayTransport>(std::vector<std::vector<std::uint8_t>>{})};
  std::this_thread::sleep_for(PAST_THE_TIMEOUT);
  EXPECT_FALSE(arm.has_sample());
  EXPECT_TRUE(arm.link_timed_out());
}

TEST(PosixSerialPort, SecondOpenIsRefusedWhileTheFirstHoldsThePort)
{
  // A pseudo-terminal opens and configures like the real UART, so no arm is needed.
  const int controller{::posix_openpt(O_RDWR | O_NOCTTY)};
  ASSERT_GE(controller, 0);
  ASSERT_EQ(::grantpt(controller), 0);
  ASSERT_EQ(::unlockpt(controller), 0);
  std::array<char, 128> port{};
  ASSERT_EQ(::ptsname_r(controller, port.data(), port.size()), 0);

  {
    const PosixSerialPort first{port.data(), 1000000};
    // TIOCEXCL: another program (or a second driver) must not share the port.
    EXPECT_THROW(PosixSerialPort(port.data(), 1000000), std::runtime_error);
  }
  // Closed again: the port is free.
  EXPECT_NO_THROW(PosixSerialPort(port.data(), 1000000));

  ::close(controller);
}
