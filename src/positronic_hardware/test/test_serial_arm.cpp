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
// Follow-up prompt (servo over-temperature):
//   The thread now asks for the servo temperatures (get_servo_temps, 0xE5) once a period, after
//   the get_angles reply of a turn that sends no target, or before the send once the reading is
//   older than max_age, and never while a stop or power on is pending. Two plausible readings in
//   a row at or above the limit make the thread itself send a stop and then a power off, hold
//   every target, refuse power_on() and send the power off again every period. With a transport
//   that answers each request like the arm (get_angles with the recorded reply, 0xE5 with
//   temperatures the test sets) and short guard times, check: the temperatures are read while
//   idle; no temperature request goes out in a stream of turns that send, until it is overdue,
//   and then it goes right after a get_angles, never after a send_angles; the cut (stop, then
//   power off, the joint and value reported, power_on() refused with no frame, no target after
//   it, the power off repeated); the cut while halted; implausible readings are ignored; and
//   temperature_stale() when the arm never answers 0xE5 while the pose still updates.
// Follow-up prompt (#150):
//   After a send_angles the thread now waits for the Atom's sync-write to the servos instead of
//   a fixed settle, with echo_timeout as the longest wait. With an echo_timeout far above the
//   replayed arm's 20 ms, check: the next get_angles never goes out before the sync-write is
//   whole on the line, and goes out long before the echo_timeout; an arm that never sync-writes
//   makes the wait last the whole echo_timeout.
// Follow-up prompt (#150):
//   A turn that asks the temperatures (the reading overdue) no longer sends in the same turn:
//   the target goes out next turn. Check that the frame after a temperature request in a
//   stream is a get_angles, never a send_angles.
// Follow-up prompt (#170):
//   With pose_from_poll (the default) the replayed arm may put the six recorded servo status
//   frames on the line before its sync-write. Check: the pose then equals those frames (the
//   Atom's conversion from steps) and no get_angles follows the send; a write wakes the thread
//   and the send follows it within a few ms, not at the next period; without writes the thread
//   polls get_angles again; a frame missing, or one with a bad checksum, brings the get_angles
//   back and the pose comes from its reply; with pose_from_poll false the #150 turn is kept, a
//   get_angles after every send even with the frames on the line.
// Follow-up prompt (#170, after the 10 min of 20 Nov):
//   The turn after a temperature request asked a get_angles (200 in 10 min): the reply to the
//   temperatures now frees the next send. The replayed arm answers get_servo_temps; with the
//   pose from the poll check that the frame after a temperature request in a stream is a
//   send_angles. The #150 test of the same case keeps its get_angles with pose_from_poll off.
//   A write that lands while the thread reads the sync-write must be sent right after it, not
//   at the two-period deadline.
// Follow-up prompt (#170, after the gripper run of 20 Nov):
//   A hot servo's corrupt frame passed the 8-bit checksum four times in 2 min and put J4 at
//   180 deg on /joint_states. Check: a frame with a good checksum whose position is 20 deg away
//   from the last pose, or whose voltage or temperature cannot be a servo's, is left out and the
//   joint keeps its last good position; a joint missing from one poll is bridged the same way,
//   with no get_angles; missing for longer than the bridge age brings the get_angles back.
// Follow-up prompt (#170, the review of 23 Nov):
//   With #174's gripper, at rest a due gripper takes the turn of the repeated send: check it
//   is served while the controller writes the same pose every 33 ms. A halt must end the
//   turn's wait at once (the stop within 10 ms), a stop still pending without its ack must not
//   spin the thread, and a target whose send got no sync-write is not sent again on every
//   write. The tests that check a frame was left out now move a marker joint and wait for it,
//   so they cannot pass without the poll being read.
//
// Follow-up prompt (#170, part 3: the servo state from the frames, the turn made robust):
//   With the pose from the poll, a poll whose six frames are all fresh is a temperature
//   reading: in a stream the temperatures equal the frames' and no get_servo_temps goes out;
//   polls that each miss a different joint are published but are no reading, so the asks go
//   on. Hot polls cut only hot_for after the first send (fast_guard: 100 ms, the period, so two
//   requests in a row still cut), the power off after the stop's ack (acked 60 ms late here),
//   nothing sent after it; a power off without its ack goes out again next turn. A poll that is
//   not published still keeps its good frames for the next bridge. A write that lands during a
//   get_angles counts for its turn (a 40 ms get_angles; count the sends after the first gripper
//   read). An unanswered send goes out again REPEAT_RETRY later, not before; a refused target
//   once a second. An answer 2 ms past its window, an ack 6 ms past: nothing is written while
//   the arm still answers, and the sends go on. Pin the default times. In the earlier tests:
//   wait for the stored last target; a stale timeout over max_age where the requests are the
//   only readings; check the fallback within two periods of the send and the off-poll pose
//   10 ms after the sync-write; drop the tests whose mutants others kill.
// Follow-up prompt (#120):
//   read_pose() returns the pose of read_angles() with the time since the thread decoded the
//   reply that carried it, nullopt before the first one. With ScriptedArm, which now can lose
//   its get_angles replies and records when it hands each one out, check: no age before the
//   first reply; replies at known times give the age since the last one, within the decode
//   margin; lost replies make the age grow by the time that passed and keep the pose; the next
//   reply brings it back down to the time since that reply. With the pose from the poll, the
//   age counts from the sync-write that closed the poll.
// Follow-up prompt (#120, CI):
//   The checks failed on a loaded CI runner: they assumed the thread decodes a reply within
//   15 ms. Make them independent of how fast the thread runs: wait for the thread to write a
//   frame after the last reply (it publishes before it writes), bound the age only from above by
//   the time since the reply went out, and prove growth and drop by comparing two reads of the
//   same reply, or the age against the time since a newer reply.
// Follow-up prompt (#133):
//   The speed byte is the constructor's send_speed: check it on the wire at 50 and 70. The
//   thread sends set_fresh_mode 1 right before every power_on and counts it; an arm that does
//   not ack the 0x16 still gets its power_on, and the count says so.
// Follow-up prompt (#133, review):
//   A halt that lands during the fresh mode wait keeps the power off: no power_on frame, the
//   stop goes out, power_on() is false. A power_on resent because the arm did not ack it sends
//   its fresh mode once, and the next request its own.

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
#include <future>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/serial_arm.hpp"
#include "positronic_hardware/serial_transport.hpp"
#include "positronic_protocol/decode.hpp"
#include "positronic_protocol/types.hpp"
#include "positronic_protocol/units.hpp"

namespace
{
using positronic_hardware::JointPositions;
using positronic_hardware::NUM_JOINTS;
using positronic_hardware::PosixSerialPort;
using positronic_hardware::SerialArm;
using positronic_hardware::SerialTransport;

constexpr std::uint8_t STOP_ID{static_cast<std::uint8_t>(positronic_protocol::CommandId::STOP)};
constexpr std::uint8_t FRESH_MODE_ID{
  static_cast<std::uint8_t>(positronic_protocol::CommandId::SET_FRESH_MODE)};
constexpr std::uint8_t POWER_ON_ID{
  static_cast<std::uint8_t>(positronic_protocol::CommandId::POWER_ON)};
constexpr std::uint8_t POWER_OFF_ID{
  static_cast<std::uint8_t>(positronic_protocol::CommandId::POWER_OFF)};
constexpr std::uint8_t GET_ANGLES_ID{
  static_cast<std::uint8_t>(positronic_protocol::CommandId::GET_ANGLES)};
constexpr std::uint8_t TEMPS_ID{
  static_cast<std::uint8_t>(positronic_protocol::CommandId::GET_SERVO_TEMPS)};
constexpr std::uint8_t GET_GRIPPER_ID{
  static_cast<std::uint8_t>(positronic_protocol::CommandId::GET_GRIPPER_VALUE)};
constexpr std::uint8_t SET_GRIPPER_ID{
  static_cast<std::uint8_t>(positronic_protocol::CommandId::SET_GRIPPER_VALUE)};
/// What the replayed arm answers to get_gripper_value when set_answer_gripper(true).
constexpr std::uint8_t GRIPPER_REPLY_VALUE{50};

/// The Atom's sync-write to the six servos, recorded on the arm on 13 Nov 2026 right after a
/// send_angles: `FF FF FE 34 83 29 07` + per servo (id, acc, goal, time, speed) + checksum. It
/// is what the arm puts on the line ~20 ms after a send_angles, once the target reached the bus.
constexpr std::string_view RECORDED_SYNC_WRITE{
  "ff ff fe 34 83 29 07 01 32 e8 07 00 00 f1 01 02 32 01 08 00 00 32 01 03 32 4a 08 00 00 2c 01 "
  "04 32 3f 08 00 00 2c 01 05 32 13 08 00 00 9e 01 06 32 de 08 00 00 fe 01 2a"};
constexpr std::chrono::milliseconds SYNC_WRITE_DELAY{20};
/// The longest a test gives the thread, after the replayed sync-write lands, to decode the poll
/// and publish the pose: the work takes well under a millisecond, the rest is the 2 ms read
/// backoff and the scheduler of a loaded test machine.
constexpr std::chrono::milliseconds DECODE_MARGIN{15};
/// How soon a frame must follow what wakes the thread (a write, a halt): it wakes, reads and
/// writes in well under a millisecond, the rest is the scheduler.
constexpr std::chrono::milliseconds WAKE_MARGIN{10};

/// The six servo status frames of the same recording, the Atom's poll right before that
/// sync-write (#170): `FF FF id 11 err` + 15 bytes + checksum each, positions 2024, 2050, 2123,
/// 2112, 2068 and 2271 steps.
constexpr std::string_view RECORDED_POLL_FRAMES{
  "ff ff 01 11 00 e8 07 00 00 00 00 79 23 00 00 00 e8 07 00 00 73 "
  "ff ff 02 11 00 02 08 00 00 00 00 79 20 00 00 00 01 08 00 00 40 "
  "ff ff 03 11 00 4b 08 00 00 00 00 7a 1f 00 00 00 4a 08 00 00 ad "
  "ff ff 04 11 00 40 08 00 00 00 00 4a 20 00 00 00 3f 08 00 00 f1 "
  "ff ff 05 11 00 14 08 00 00 00 00 4a 1e 00 00 00 13 08 00 00 4a "
  "ff ff 06 11 00 df 08 00 00 00 00 49 22 00 00 00 de 08 00 00 b0"};
constexpr std::array<std::uint16_t, 6> RECORDED_POLL_STEPS{2024, 2050, 2123, 2112, 2068, 2271};
/// The temperatures in those frames (byte 7 of each frame's data), degC.
constexpr std::array<std::uint8_t, 6> RECORDED_POLL_TEMPS{35, 32, 31, 32, 30, 34};

std::vector<std::uint8_t> parse_hex(std::string_view hex);

/// A transport that answers every request with a fixed sequence of byte bursts, looping, and
/// acknowledges requests and send_angles like the arm (the sync-write follows a send_angles).
/// It is thread-safe because SerialArm reads it from its poll thread.
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
    const std::lock_guard<std::mutex> lock{mutex_};
    const auto now{std::chrono::steady_clock::now()};
    // The Atom drops a frame that comes while it still answers a request (24 Nov 2026).
    if (!queued_.empty() && queued_.front().silences && now < queued_.front().due) {
      ++written_while_busy_;
    }
    if (count == 5) {
      const std::uint8_t id{*std::next(data, 3)};
      if (id == STOP_ID || id == POWER_ON_ID || id == POWER_OFF_ID) {
        if (reject_requests_.load()) {
          return false;
        }
        // The arm acknowledges a request with FE FE 03 <id> 01 FA, 1.8 ms after it on the
        // bench, 58 ms once for a power_on. The ack is handed out by read() when due, and the
        // line is silent until then, as the arm answers nothing else while it handles the
        // request. With ack_requests_ off the request is taken and never answered: the case
        // #103 must survive.
        if (ack_requests_.load() && (id != POWER_OFF_ID || ack_power_off_.load())) {
          queued_.push_back({now + ack_delay_.load(), {0xFE, 0xFE, 0x03, id, 0x01, 0xFA}, true});
        }
      } else if (id == GET_ANGLES_ID && angles_delay_.load().count() > 0) {
        // The reply comes this late, as on the arm (6.8 ms p50, 23 Nov 2026): nothing before.
        queued_.push_back({now + angles_delay_.load(), {}, true});
      } else if (id == GET_GRIPPER_ID && answer_gripper_.load()) {
        queued_.push_back({now, {0xFE, 0xFE, 0x03, 0x65, GRIPPER_REPLY_VALUE, 0xFA}, true});
      } else if (id == TEMPS_ID) {
        // The temperatures, cool, at once: the turn after them must show that the reply frees
        // the next send (#170).
        queued_.push_back({now, {0xFE, 0xFE, 0x08, 0xE5, 31, 33, 35, 30, 29, 28, 0xFA}, true});
      }
    } else if (count == 6 && *std::next(data, 3) == FRESH_MODE_ID) {
      // #133: set_fresh_mode. Whether the Atom acks it is open; ack_fresh_mode_ picks.
      if (ack_fresh_mode_.load()) {
        queued_.push_back({now, {0xFE, 0xFE, 0x03, FRESH_MODE_ID, 0x01, 0xFA}, true});
      }
    } else if (count == 18 && *std::next(data, 3) == 0x22) {
      // A send_angles: the ack at once, the sync-write to the servos ~20 ms later, with the
      // poses still flowing in between. The serial thread waits for the sync-write (#150);
      // with sync_write_ off the arm never sends it, and the wait must last the echo_timeout.
      queued_.push_back({now, {0xFE, 0xFE, 0x03, 0x22, 0x01, 0xFA}, false});
      if (sync_write_.load()) {
        // The poll frames, when the test set them, come right before the sync-write (#170).
        std::vector<std::uint8_t> burst{poll_frames_};
        const std::vector<std::uint8_t> sync_write{parse_hex(RECORDED_SYNC_WRITE)};
        burst.insert(burst.end(), sync_write.begin(), sync_write.end());
        // quiet_after_send_: the line carries nothing else until the sync-write is due, as
        // the real arm, which answers nothing while it handles a send_angles (#170 tests).
        queued_.push_back({now + SYNC_WRITE_DELAY, burst, quiet_after_send_.load()});
      }
    }
    written_.emplace_back(data, std::next(data, static_cast<std::ptrdiff_t>(count)));
    written_at_.push_back(now);
    return true;
  }

  /// When each frame of written() was written, same order.
  std::vector<std::chrono::steady_clock::time_point> written_at()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return written_at_;
  }

  /// When each sync-write was handed to the thread, whole, by read().
  std::vector<std::chrono::steady_clock::time_point> sync_write_times()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return sync_write_times_;
  }

  /// An arm that acknowledges a send_angles but never sync-writes it (default: it does).
  void set_sync_write(bool sync_write)
  {
    sync_write_.store(sync_write);
  }

  /// The servo status frames the arm puts on the line before each sync-write (#170). Empty by
  /// default: the recorded bursts keep flowing instead, with the status frames they carry.
  void set_poll_frames(std::vector<std::uint8_t> frames)
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    poll_frames_ = std::move(frames);
  }

  /// An arm whose line is silent between a send_angles and its sync-write, like the real one
  /// (default: the recorded bursts keep flowing, as the tests before #170 were written).
  void set_quiet_after_send(bool quiet)
  {
    quiet_after_send_.store(quiet);
  }

  /// How many sync-write frames the arm has put on the line so far.
  std::size_t sync_writes_sent()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return sync_writes_sent_;
  }

  /// #133: an arm that never acks set_fresh_mode (default: it does).
  void set_ack_fresh_mode(bool ack)
  {
    ack_fresh_mode_.store(ack);
  }

  /// An arm that never acknowledges a stop or power on (default: it does).
  void set_ack_requests(bool ack)
  {
    ack_requests_.store(ack);
  }

  /// A get_angles reply that comes `delay` after the request, the line silent until then
  /// (default: at once, from the recorded bursts).
  void set_angles_delay(std::chrono::milliseconds delay)
  {
    angles_delay_.store(delay);
  }

  /// An arm that answers get_gripper_value with GRIPPER_REPLY_VALUE (default: no answer).
  void set_answer_gripper(bool answer)
  {
    answer_gripper_.store(answer);
  }

  /// How long the arm takes to acknowledge a request (default 0).
  void set_ack_delay(std::chrono::milliseconds delay)
  {
    ack_delay_.store(delay);
  }

  /// An arm that takes a power off without acking it (default: it acks it).
  void set_ack_power_off(bool ack)
  {
    ack_power_off_.store(ack);
  }

  /// How many frames were written while a request was still unanswered (its answer not due).
  std::size_t written_while_busy()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return written_while_busy_;
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
    if (!queued_.empty()) {
      // The arm's own frames go out whole when due. A request ack silences the line until it
      // is due; the frames after a send_angles do not, the poses keep coming meanwhile.
      const Queued & next{queued_.front()};
      const auto now{std::chrono::steady_clock::now()};
      if (now >= next.due) {
        const std::size_t n{std::min(max, next.bytes.size())};
        std::copy_n(next.bytes.begin(), n, dst);
        // A request ack is 6 bytes and the temperatures 11; the sync-write alone is 56.
        if (next.bytes.size() >= 56) {
          ++sync_writes_sent_;
          sync_write_times_.push_back(now);
        }
        queued_.erase(queued_.begin());
        return n;
      }
      if (next.silences) {
        return 0;
      }
    }
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
  std::vector<std::chrono::steady_clock::time_point> written_at_;
  std::vector<std::chrono::steady_clock::time_point> sync_write_times_;
  std::atomic<bool> sync_write_{true};
  std::size_t index_{0};
  std::atomic<bool> silent_{false};
  std::atomic<bool> reject_requests_{false};
  std::atomic<bool> ack_requests_{true};
  std::atomic<bool> ack_power_off_{true};
  std::size_t written_while_busy_{0};
  std::atomic<bool> ack_fresh_mode_{true};
  std::atomic<std::chrono::milliseconds> ack_delay_{std::chrono::milliseconds{0}};
  std::atomic<std::chrono::milliseconds> angles_delay_{std::chrono::milliseconds{0}};
  std::atomic<bool> answer_gripper_{false};
  /// A frame the arm will put on the line at `due`. `silences`: nothing else until then.
  struct Queued
  {
    std::chrono::steady_clock::time_point due;
    std::vector<std::uint8_t> bytes;
    bool silences;
  };
  std::vector<Queued> queued_;
  std::size_t sync_writes_sent_{0};
  std::vector<std::uint8_t> poll_frames_;
  std::atomic<bool> quiet_after_send_{false};
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
    if (
      frame.size() == 18 && frame[0] == 0xFE && frame[1] == 0xFE && frame[2] == 0x0F &&
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
    if (
      frame.size() == 5 && frame[0] == 0xFE && frame[1] == 0xFE && frame[2] == 0x02 &&
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

/// Waits until the thread has written `count` request frames with command `id`, or 2 s pass.
std::vector<std::vector<std::uint8_t>> wait_for_requests(
  ReplayTransport & transport, std::uint8_t id, std::size_t count)
{
  const auto deadline{std::chrono::steady_clock::now() + std::chrono::seconds{2}};
  while (std::chrono::steady_clock::now() < deadline) {
    const auto found{frames_with_id(transport.written(), id)};
    if (found.size() >= count) {
      return found;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  return frames_with_id(transport.written(), id);
}

/// The first send_angles the thread wrote and the first get_angles after it, with the time of
/// each. `found` stays false until both are on the wire.
struct SendThenPoll
{
  bool found{false};
  std::chrono::steady_clock::time_point send{};
  std::chrono::steady_clock::time_point poll{};
};

SendThenPoll first_send_then_poll(ReplayTransport & transport)
{
  const auto frames{transport.written()};
  // Read after the frames, so it has a time for each of them.
  const auto times{transport.written_at()};
  const auto is_send{[](const std::vector<std::uint8_t> & frame) {
      return frame.size() == 18 && frame.at(3) == 0x22;
    }};
  const auto is_poll{[](const std::vector<std::uint8_t> & frame) {
      return frame.size() == 5 && frame.at(3) == GET_ANGLES_ID;
    }};
  SendThenPoll result{};
  std::size_t send{0};
  while (send < frames.size() && !is_send(frames.at(send))) {
    ++send;
  }
  for (std::size_t poll{send + 1}; poll < frames.size(); ++poll) {
    if (is_poll(frames.at(poll))) {
      result.found = true;
      result.send = times.at(send);
      result.poll = times.at(poll);
      return result;
    }
  }
  return result;
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
  const std::vector<std::uint8_t> expected{
    0xFE,
    0xFE,
    0x0F,
    0x22,
    0x02,
    0x3D,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    SerialArm::DEFAULT_SEND_SPEED,
    0xFA};
  EXPECT_EQ(sends.front(), expected);

  // The same command is not sent again: the arm already has it.
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(send_angles_frames(port.written()).size(), 1u);

  // A new command goes out once more.
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(wait_for_sends(port, 2).size(), 2u);
}

// #133: the speed byte, second to last, is the one the backend was built with.
TEST(SerialArm, TheSendSpeedIsTheByteOnTheWire)
{
  for (const std::uint8_t speed : {std::uint8_t{50}, std::uint8_t{70}}) {
    auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
    ReplayTransport & port{*transport};
    SerialArm arm{
      std::move(transport),
      SerialArm::Timing{},
      positronic_hardware::TemperatureGuard{},
      false,
      positronic_hardware::LoadReliefCheck{
        positronic_hardware::LoadReliefLimits{false}, positronic_hardware::PositionLimits{}},
      speed};
    arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
    const auto sends{wait_for_sends(port, 1)};
    ASSERT_EQ(sends.size(), 1u) << "speed " << int{speed};
    ASSERT_EQ(sends.front().size(), 18u);
    EXPECT_EQ(sends.front().at(16), speed);
  }
}

// #133 step zero: fresh mode 1 goes out right before every power_on, nothing between them.
TEST(SerialArm, FreshModeOneGoesOutRightBeforeEveryPowerOn)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  wait_for_reply(arm);

  ASSERT_TRUE(arm.power_on());
  ASSERT_TRUE(arm.halt());
  ASSERT_TRUE(arm.power_on());

  const std::vector<std::uint8_t> fresh_one{0xFE, 0xFE, 0x03, FRESH_MODE_ID, 0x01, 0xFA};
  const auto frames{port.written()};
  std::size_t power_ons{0};
  for (std::size_t i{0}; i < frames.size(); ++i) {
    if (frames.at(i).size() == 5 && frames.at(i).at(3) == POWER_ON_ID) {
      ++power_ons;
      ASSERT_GT(i, 0u);
      EXPECT_EQ(frames.at(i - 1), fresh_one) << "power_on " << power_ons;
    }
  }
  EXPECT_EQ(power_ons, 2u);
  EXPECT_EQ(std::count(frames.begin(), frames.end(), fresh_one), 2);
  EXPECT_EQ(arm.fresh_mode_count().sent, 2u);
  EXPECT_EQ(arm.fresh_mode_count().acked, 2u);
}

// #133: an Atom that does not ack 0x16 costs the power_on one ACK_TIMEOUT, not the activation.
TEST(SerialArm, PowerOnStillGoesOutWhenFreshModeIsNotAcknowledged)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_ack_fresh_mode(false);
  SerialArm arm{std::move(transport)};
  wait_for_reply(arm);

  const auto start{std::chrono::steady_clock::now()};
  EXPECT_TRUE(arm.power_on());
  // Lower bound only: the thread waits the whole ACK_TIMEOUT for the missing ack, so it cannot
  // be shorter. No upper bound: a loaded CI runner can stretch it.
  EXPECT_GE(std::chrono::steady_clock::now() - start, positronic_hardware::ACK_TIMEOUT);
  EXPECT_EQ(frames_with_id(port.written(), POWER_ON_ID).size(), 1u);
  EXPECT_EQ(arm.fresh_mode_count().sent, 1u);
  EXPECT_EQ(arm.fresh_mode_count().acked, 0u);
}

// #133: a halt that lands while the thread waits for the fresh mode ack wins: no power_on goes
// out, the stop does, and power_on() reports false with the hold on.
TEST(SerialArm, AHaltDuringTheFreshModeWaitKeepsThePowerOff)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_ack_fresh_mode(false);  // the wait lasts the whole ACK_TIMEOUT (80 ms)
  SerialArm arm{std::move(transport)};
  static_cast<void>(wait_for_reply(arm));

  const std::vector<std::uint8_t> fresh_one{0xFE, 0xFE, 0x03, FRESH_MODE_ID, 0x01, 0xFA};
  auto powered{std::async(std::launch::async, [&arm] {
        return arm.power_on();
  })};
  // Wait for the fresh mode frame on the wire, then halt inside the 80 ms wait for its ack.
  // power_on() false and the stop would hold even after that window (the restore of the hold
  // catches a later halt); "no power_on frame" is the one check that needs the halt inside it,
  // and 1 ms polling leaves ~79 ms of margin. Repeated 30 times on a fully loaded machine: no
  // miss.
  const auto deadline{std::chrono::steady_clock::now() + SerialArm::REQUEST_TIMEOUT};
  bool fresh_sent{false};
  while (!fresh_sent && std::chrono::steady_clock::now() < deadline) {
    const auto frames{port.written()};
    fresh_sent = std::find(frames.begin(), frames.end(), fresh_one) != frames.end();
    if (!fresh_sent) {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
  }
  ASSERT_TRUE(fresh_sent);
  arm.request_halt();

  EXPECT_FALSE(powered.get());
  const auto frames{port.written()};
  EXPECT_TRUE(frames_with_id(frames, POWER_ON_ID).empty());
  EXPECT_FALSE(frames_with_id(frames, STOP_ID).empty());
  EXPECT_EQ(arm.fresh_mode_count().sent, 1u);
}

// #133: a power_on the arm does not ack goes out again every turn, but its fresh mode only once;
// the next power_on request gets its own.
TEST(SerialArm, FreshModeGoesOutOncePerPowerOnRequestNotPerRetry)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_ack_requests(false);
  SerialArm arm{std::move(transport)};
  static_cast<void>(wait_for_reply(arm));

  EXPECT_FALSE(arm.power_on());  // never acked: gives up at REQUEST_TIMEOUT, still pending
  // A retry costs one ACK_TIMEOUT (80 ms) plus a turn, so REQUEST_TIMEOUT (300 ms) holds about
  // three: asking for two leaves room for a slow runner.
  EXPECT_GE(frames_with_id(port.written(), POWER_ON_ID).size(), 2u);
  EXPECT_EQ(arm.fresh_mode_count().sent, 1u);

  port.set_ack_requests(true);
  ASSERT_TRUE(arm.halt());  // the stop replaces the pending power_on
  ASSERT_TRUE(arm.power_on());
  EXPECT_EQ(arm.fresh_mode_count().sent, 2u);
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

TEST(SerialArm, TheFixtureArmSyncWritesAfterEverySendAngles)
{
  // Not a driver property, a fixture property the #150 tests rely on: after each send_angles
  // the replayed arm puts the recorded sync-write on the line ~20 ms later, like the real one.
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(port.sync_writes_sent(), 1u);
}

// #150. After a send_angles the thread waits for the Atom's sync-write to the servos, and for
// echo_timeout at most. The echo_timeout here is far above the replayed arm's 20 ms and the
// period is short, so a wait ended by the sync-write and one that lasts the echo_timeout cannot
// be confused, even on a loaded runner.
SerialArm::Timing echo_timing()
{
  SerialArm::Timing timing{};
  timing.moving_period = std::chrono::milliseconds{30};
  timing.echo_timeout = std::chrono::milliseconds{90};
  // These tests describe the #150 turn, a get_angles after every send. The recorded bursts
  // carry real servo status frames, so with the pose from the poll (#170) the thread would take
  // its pose from them and ask nothing for two periods.
  timing.pose_from_poll = false;
  return timing;
}
const SerialArm::Timing ECHO_TIMING{echo_timing()};

TEST(SerialArm, TheSyncWriteEndsTheWaitAfterASend)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), ECHO_TIMING};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);
  std::this_thread::sleep_for(2 * ECHO_TIMING.echo_timeout);

  const SendThenPoll turn{first_send_then_poll(port)};
  ASSERT_TRUE(turn.found);
  const auto sync_writes{port.sync_write_times()};
  ASSERT_FALSE(sync_writes.empty());
  // The next get_angles never goes out before the sync-write is whole on the line...
  EXPECT_GE(turn.poll, sync_writes.front());
  // ...and goes out long before the echo_timeout would have ended.
  EXPECT_LT(turn.poll - turn.send, ECHO_TIMING.echo_timeout - std::chrono::milliseconds{30});
  EXPECT_EQ(arm.echo_timeouts(), 0u);
}

TEST(SerialArm, WithoutTheSyncWriteTheWaitLastsTheEchoTimeout)
{
  // An arm that never sync-writes: the thread waits the whole echo_timeout.
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_sync_write(false);
  SerialArm arm{std::move(transport), ECHO_TIMING};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);
  std::this_thread::sleep_for(2 * ECHO_TIMING.echo_timeout);

  const SendThenPoll turn{first_send_then_poll(port)};
  ASSERT_TRUE(turn.found);
  EXPECT_EQ(port.sync_writes_sent(), 0u);
  EXPECT_GE(turn.poll - turn.send, ECHO_TIMING.echo_timeout);
  // Counted, so the plugin can say why the turns got slow.
  EXPECT_EQ(arm.echo_timeouts(), 1u);
}

// #103. halt() and power_on() report SUCCESS only once the arm acknowledged the request frame.
// The transport acknowledges like the arm (1.8 ms on the bench, 58 ms seen once), with the
// delay chosen per test so the wait is visible.

TEST(SerialArm, HaltReturnsTrueOnlyAfterTheStopIsAcknowledged)
{
  // The ack comes 50 ms after the stop, inside ACK_TIMEOUT: halt() must not return before it.
  // Three in a row, so a late clear of one request cannot be mistaken for the next. The poses
  // keep flowing between them, so the link watchdog never trips: the reply window must count
  // from the get_angles write, not from the start of a turn that spent 50 ms in a request.
  constexpr std::chrono::milliseconds ACK_DELAY{50};
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_ack_delay(ACK_DELAY);
  SerialArm arm{std::move(transport)};
  static_cast<void>(wait_for_reply(arm));

  for (int i{1}; i <= 3; ++i) {
    const auto start{std::chrono::steady_clock::now()};
    EXPECT_TRUE(arm.halt()) << "halt " << i;
    const auto elapsed{std::chrono::steady_clock::now() - start};
    EXPECT_GE(elapsed, ACK_DELAY) << "halt " << i << " returned before the ack";
    EXPECT_LT(elapsed, SerialArm::REQUEST_TIMEOUT) << "halt " << i;
    EXPECT_EQ(frames_with_id(port.written(), STOP_ID).size(), static_cast<std::size_t>(i));
    EXPECT_TRUE(arm.power_on());
  }
  EXPECT_FALSE(arm.link_timed_out());
}

TEST(SerialArm, HaltFailsWithinTheDeadlineWhenTheArmDoesNotAcknowledge)
{
  // The arm takes every stop and never answers it, while poses keep coming (the line is alive,
  // the request is lost). halt() must give up at REQUEST_TIMEOUT, not before and not much
  // after, and the thread must keep resending the stop every turn until it is acknowledged:
  // a stop that was never confirmed is not a stop.
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_ack_requests(false);
  SerialArm arm{std::move(transport)};
  static_cast<void>(wait_for_reply(arm));

  const auto start{std::chrono::steady_clock::now()};
  EXPECT_FALSE(arm.halt());
  const auto elapsed{std::chrono::steady_clock::now() - start};
  EXPECT_GE(elapsed, SerialArm::REQUEST_TIMEOUT);
  EXPECT_LT(elapsed, 2 * SerialArm::REQUEST_TIMEOUT);

  // The request stays pending: more stops go out after halt() returned.
  const std::size_t stops_at_return{frames_with_id(port.written(), STOP_ID).size()};
  EXPECT_GE(stops_at_return, 1u);
  std::this_thread::sleep_for(std::chrono::milliseconds{250});
  const auto frames{port.written()};
  EXPECT_GE(frames_with_id(frames, STOP_ID).size(), stops_at_return + 1);

  // Between two stops exactly one get_angles and no send_angles: one request per turn, and the
  // hold set by halt() keeps every target off the wire.
  std::size_t first{frames.size()};
  std::size_t second{frames.size()};
  for (std::size_t i{0}; i < frames.size(); ++i) {
    if (frames[i].size() == 5 && frames[i][3] == STOP_ID) {
      if (first == frames.size()) {
        first = i;
      } else {
        second = i;
        break;
      }
    }
  }
  ASSERT_LT(second, frames.size());
  std::size_t get_angles{0};
  std::size_t send_angles{0};
  for (std::size_t i{first + 1}; i < second; ++i) {
    get_angles += (frames[i].size() == 5 && frames[i][3] == 0x20) ? 1u : 0u;
    send_angles += (frames[i].size() == 18) ? 1u : 0u;
  }
  EXPECT_EQ(get_angles, 1u);
  EXPECT_EQ(send_angles, 0u);
  // The poses never stopped, so this is not a link fault.
  EXPECT_FALSE(arm.link_timed_out());
}

TEST(SerialArm, ARequestHaltDuringThePowerOnWaitKeepsTheHold)
{
  // power_on() is waiting for its ack (50 ms) when the plugin's on_error calls request_halt().
  // The hold the power_on released must come back: power_on() reports failure, a stop goes out
  // after the power_on, and no target leaves until the next power_on. Without the re-assert the
  // late clear of the power_on request erases the halt and the arm is live with nobody knowing.
  constexpr std::chrono::milliseconds ACK_DELAY{50};
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_ack_delay(ACK_DELAY);
  SerialArm arm{std::move(transport)};
  static_cast<void>(wait_for_reply(arm));
  ASSERT_TRUE(arm.halt());
  ASSERT_EQ(frames_with_id(port.written(), STOP_ID).size(), 1u);

  std::future<bool> powered{std::async(std::launch::async, [&arm]() {
        return arm.power_on();
  })};
  // Once the power_on frame is on the wire the thread is inside the ack wait.
  ASSERT_EQ(wait_for_requests(port, POWER_ON_ID, 1).size(), 1u);
  arm.request_halt();
  EXPECT_FALSE(powered.get());

  // The second stop leaves after the power_on, and the target written meanwhile never does.
  EXPECT_EQ(wait_for_requests(port, STOP_ID, 2).size(), 2u);
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::this_thread::sleep_for(3 * SerialArm::Timing{}.moving_period);
  EXPECT_EQ(send_angles_frames(port.written()).size(), 0u);
  EXPECT_EQ(frames_with_id(port.written(), POWER_ON_ID).size(), 1u);
  const auto frames{port.written()};
  std::size_t last_power_on{0};
  std::size_t last_stop{0};
  for (std::size_t i{0}; i < frames.size(); ++i) {
    if (frames[i].size() == 5 && frames[i][3] == POWER_ON_ID) {
      last_power_on = i;
    }
    if (frames[i].size() == 5 && frames[i][3] == STOP_ID) {
      last_stop = i;
    }
  }
  EXPECT_GT(last_stop, last_power_on);
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
  // A silent turn lasts about REPLY_TIMEOUT (50 ms): this transport, like the real port with
  // VTIME = 0, returns at once. 200 ms leaves room for the next reply.
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

// ---------------------------------------------------------------------------
// Servo over-temperature
// ---------------------------------------------------------------------------

namespace
{
using positronic_hardware::Overheat;
using positronic_hardware::ServoTemperatures;
using positronic_hardware::TemperatureGuard;

/// The guard of the tests: the same rules, faster, so a test lasts a second, not a minute.
TemperatureGuard fast_guard()
{
  TemperatureGuard guard{};
  guard.period = std::chrono::milliseconds{100};
  guard.max_age = std::chrono::milliseconds{300};
  guard.stale_timeout = std::chrono::milliseconds{300};
  // The period, as on the arm (1 s each): two requests in a row still cut.
  guard.hot_for = std::chrono::milliseconds{100};
  return guard;
}

constexpr ServoTemperatures COOL{31, 33, 35, 30, 29, 28};
constexpr ServoTemperatures J2_HOT{31, 61, 35, 30, 29, 28};

/// A transport that answers each request like the arm: a get_angles with the recorded reply
/// (servo noise included), a get_servo_temps with the temperatures the test sets, a stop or
/// power_on with its ack (#103), anything else with nothing. Every frame written is kept, in
/// order.
class ScriptedArm : public SerialTransport
{
public:
  bool write(const std::uint8_t * data, std::size_t count) override
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    written_.emplace_back(data, std::next(data, static_cast<std::ptrdiff_t>(count)));
    const std::uint8_t id{count == 5 ? *std::next(data, 3) : std::uint8_t{0}};
    if (id == GET_ANGLES_ID) {
      if (!answer_angles_) {
        return true;  // a lost reply (#120): the request went out, nothing comes back
      }
      for (const auto & burst : recorded_bursts()) {
        pending_.insert(pending_.end(), burst.begin(), burst.end());
      }
      angles_pending_ = true;
    } else if (id == TEMPS_ID && answer_temperatures_) {
      pending_.insert(pending_.end(), {0xFE, 0xFE, 0x08, 0xE5});
      pending_.insert(pending_.end(), temperatures_.begin(), temperatures_.end());
      pending_.push_back(0xFA);
    } else if (id == STOP_ID || id == POWER_ON_ID || id == POWER_OFF_ID) {
      pending_.insert(pending_.end(), {0xFE, 0xFE, 0x03, id, 0x01, 0xFA});
    } else if (id == GET_GRIPPER_ID) {
      pending_.insert(pending_.end(), {0xFE, 0xFE, 0x03, GET_GRIPPER_ID, gripper_, 0xFA});
    } else if (count == 6 && *std::next(data, 3) == FRESH_MODE_ID) {
      pending_.insert(pending_.end(), {0xFE, 0xFE, 0x03, FRESH_MODE_ID, 0x01, 0xFA});
    }
    return true;
  }

  void set_gripper(std::uint8_t value)
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    gripper_ = value;
  }

  std::size_t read(std::uint8_t * dst, std::size_t max) override
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    const std::size_t n{std::min(max, pending_.size())};
    std::copy_n(pending_.begin(), n, dst);
    pending_.erase(pending_.begin(), std::next(pending_.begin(), static_cast<std::ptrdiff_t>(n)));
    // The get_angles reply is complete once nothing of it is left to hand out (#120).
    if (angles_pending_ && pending_.empty()) {
      angles_pending_ = false;
      angles_handed_.push_back(std::chrono::steady_clock::now());
      written_at_handed_ = written_.size();
    }
    return n;
  }

  /// An arm whose get_angles replies are lost while `answer` is false (default: it answers).
  void set_answer_angles(bool answer)
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    answer_angles_ = answer;
  }

  /// When read() handed out the last byte of each get_angles reply, oldest first (#120).
  std::vector<std::chrono::steady_clock::time_point> angles_handed()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return angles_handed_;
  }

  /// True once the last get_angles reply is out and the thread has written a frame since (#120).
  /// The thread publishes a reply before it writes anything else, so the pose is in the buffer:
  /// a test waits on this instead of a time that a loaded machine can overrun.
  bool last_reply_published()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return !angles_pending_ && !angles_handed_.empty() && written_.size() > written_at_handed_;
  }

  void set_temperatures(const ServoTemperatures & temperatures)
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    temperatures_ = temperatures;
  }

  /// An arm whose firmware does not answer 0xE5.
  void set_answer_temperatures(bool answer)
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    answer_temperatures_ = answer;
  }

  std::vector<std::vector<std::uint8_t>> written()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return written_;
  }

private:
  std::vector<std::vector<std::uint8_t>> written_;
  std::vector<std::uint8_t> pending_;
  ServoTemperatures temperatures_{COOL};
  bool answer_temperatures_{true};
  bool answer_angles_{true};
  bool angles_pending_{false};
  std::vector<std::chrono::steady_clock::time_point> angles_handed_;
  std::size_t written_at_handed_{0};
  std::uint8_t gripper_{100};
  std::mutex mutex_;
};

/// The 5-byte request id of `frame`, or 0 for any other frame (send_angles).
std::uint8_t request_id(const std::vector<std::uint8_t> & frame)
{
  return frame.size() == 5 ? frame[3] : std::uint8_t{0};
}

bool is_gripper_set(const std::vector<std::uint8_t> & frame)
{
  return frame.size() == 7 && frame[3] == SET_GRIPPER_ID;
}

bool is_gripper_frame(const std::vector<std::uint8_t> & frame)
{
  return is_gripper_set(frame) || request_id(frame) == GET_GRIPPER_ID;
}

template<typename Port>
std::size_t gripper_sets(Port & port)
{
  const auto frames{port.written()};
  return static_cast<std::size_t>(std::count_if(frames.begin(), frames.end(), is_gripper_set));
}

template<typename Port>
std::size_t gripper_frames(Port & port)
{
  const auto frames{port.written()};
  return static_cast<std::size_t>(std::count_if(frames.begin(), frames.end(), is_gripper_frame));
}

/// Waits until `done()` is true, or two seconds pass. Returns `done()`.
template<typename Condition>
bool wait_until(Condition done)
{
  const auto deadline{std::chrono::steady_clock::now() + std::chrono::seconds{2}};
  while (!done() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  return done();
}

/// Writes a new target every 5 ms for `duration`, so every turn of the thread has one to send.
void stream_targets(SerialArm & arm, std::chrono::milliseconds duration)
{
  const auto end{std::chrono::steady_clock::now() + duration};
  double j1{0.0};
  while (std::chrono::steady_clock::now() < end) {
    j1 += 0.001;
    arm.write_angles(JointPositions{j1, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
}

/// The frames between the first and the last send_angles of `frames`.
std::vector<std::vector<std::uint8_t>> between_sends(
  const std::vector<std::vector<std::uint8_t>> & frames)
{
  const auto is_send{[](const auto & frame) {
      return frame.size() == 18;
    }};
  const auto first{std::find_if(frames.begin(), frames.end(), is_send)};
  const auto last{std::find_if(frames.rbegin(), frames.rend(), is_send)};
  if (first == frames.end()) {
    return {};
  }
  return {first, last.base()};
}
}  // namespace

TEST(SerialArmTemperature, TheDefaultTimesFitTogether)
{
  // At rest the guard asks once a period: two hot replies in a row are hot_for apart, and cut,
  // only while the period is at least hot_for. And the guard turns blind only when no reading
  // came for longer than the oldest one it lets stand, max_age.
  EXPECT_GE(TemperatureGuard{}.period, TemperatureGuard{}.hot_for);
  EXPECT_GT(TemperatureGuard{}.stale_timeout, TemperatureGuard{}.max_age);
}

TEST(SerialArmTemperature, ReadsTheTemperaturesWhileIdle)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  const SerialArm arm{std::move(transport), SerialArm::Timing{}, fast_guard()};

  ASSERT_TRUE(wait_until([&arm]() {
      return arm.read_temperatures().has_value();
  }));
  EXPECT_EQ(arm.read_temperatures().value_or(ServoTemperatures{}), COOL);
  EXPECT_FALSE(arm.temperature_stale());
  EXPECT_FALSE(arm.overheat().has_value());
  // Each temperature request follows a get_angles reply: the arm's input is free.
  const auto frames{port.written()};
  for (std::size_t i{1}; i < frames.size(); ++i) {
    if (request_id(frames[i]) == TEMPS_ID) {
      EXPECT_EQ(request_id(frames[i - 1]), GET_ANGLES_ID) << i;
    }
  }
}

TEST(SerialArmTemperature, NoTemperatureRequestInTurnsThatSendUntilItIsOverdue)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  TemperatureGuard guard{fast_guard()};
  guard.max_age = std::chrono::seconds{5};  // never overdue during the stream
  SerialArm arm{std::move(transport), SerialArm::Timing{}, guard};
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.read_temperatures().has_value();
  }));

  stream_targets(arm, std::chrono::milliseconds{600});
  const auto during{between_sends(port.written())};
  ASSERT_FALSE(during.empty());
  EXPECT_EQ(frames_with_id(during, TEMPS_ID).size(), 0U);
}

TEST(SerialArmTemperature, AnOverdueReadingTakesTheTurnAndTheSendWaitsForTheNext)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  // The #150 turn: a get_angles every turn, so the one after the temperatures asks it too. With
  // the pose from the poll that turn sends at once (SerialArmPollPose below). The requests are
  // the only readings: a stale timeout over max_age, as on the arm (10 s against 3 s), or the
  // guard turns stale between two of them.
  TemperatureGuard guard{fast_guard()};
  guard.stale_timeout = std::chrono::milliseconds{1000};
  SerialArm arm{std::move(transport), ECHO_TIMING, guard};
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.read_temperatures().has_value();
  }));

  stream_targets(arm, std::chrono::milliseconds{1200});
  const auto during{between_sends(port.written())};
  // max_age 300 ms over a 1.2 s stream: a few readings, each right after a get_angles, so
  // never between a send and its sync-write. The turn that asked sends nothing: the frame after
  // the request is the next turn's get_angles, and the target goes out after that one.
  EXPECT_GE(frames_with_id(during, TEMPS_ID).size(), 2U);
  for (std::size_t i{1}; i + 1 < during.size(); ++i) {
    if (request_id(during[i]) == TEMPS_ID) {
      EXPECT_EQ(request_id(during[i - 1]), GET_ANGLES_ID) << i;
      EXPECT_EQ(request_id(during[i + 1]), GET_ANGLES_ID) << i;
    }
  }
  EXPECT_FALSE(arm.temperature_stale());
}

TEST(SerialArmTemperature, TwoHotReadingsStopTheArmAndCutItsPower)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  SerialArm arm{std::move(transport), SerialArm::Timing{}, fast_guard()};
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_TRUE(wait_until([&port]() {
      return !send_angles_frames(port.written()).empty();
  }));

  port.set_temperatures(J2_HOT);
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.overheat().has_value();
  }));
  const Overheat trip{arm.overheat().value_or(Overheat{})};
  EXPECT_EQ(trip.joint, 1U);
  EXPECT_EQ(trip.celsius, 61);

  // On the wire: a stop, then a power off, in that order. The trip is recorded once the stop is
  // out, before the power off and its ack.
  ASSERT_TRUE(wait_until([&port]() {
      return !frames_with_id(port.written(), POWER_OFF_ID).empty();
  }));
  const auto frames{port.written()};
  const auto stop_at{std::find_if(frames.begin(), frames.end(), [](const auto & frame) {
        return request_id(frame) == STOP_ID;
  })};
  ASSERT_NE(stop_at, frames.end());
  const auto off_at{std::find_if(stop_at, frames.end(), [](const auto & frame) {
        return request_id(frame) == POWER_OFF_ID;
  })};
  EXPECT_NE(off_at, frames.end());

  // Nothing brings the servos back: power_on() fails at once without a frame, and a target
  // from a controller still publishing never goes out.
  const std::size_t sends_at_cut{send_angles_frames(port.written()).size()};
  const auto start{std::chrono::steady_clock::now()};
  EXPECT_FALSE(arm.power_on());
  EXPECT_LT(std::chrono::steady_clock::now() - start, SerialArm::POLL_PERIOD);
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::this_thread::sleep_for(std::chrono::milliseconds{500});
  EXPECT_EQ(frames_with_id(port.written(), POWER_ON_ID).size(), 0U);
  EXPECT_EQ(send_angles_frames(port.written()).size(), sends_at_cut);
  // The power off is repeated every period (100 ms here).
  EXPECT_GE(frames_with_id(port.written(), POWER_OFF_ID).size(), 3U);
  // Cooling down does not undo the cut.
  port.set_temperatures(COOL);
  std::this_thread::sleep_for(std::chrono::milliseconds{300});
  EXPECT_TRUE(arm.overheat().has_value());
  EXPECT_FALSE(arm.power_on());
}

TEST(SerialArmTemperature, TheCutWorksWhileHalted)
{
  // An inactive component: halted, the servos still holding the pose, and heating.
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  SerialArm arm{std::move(transport), SerialArm::Timing{}, fast_guard()};
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.read_temperatures().has_value();
  }));
  ASSERT_TRUE(arm.halt());

  port.set_temperatures(J2_HOT);
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.overheat().has_value();
  }));
  EXPECT_GE(frames_with_id(port.written(), POWER_OFF_ID).size(), 1U);
}

TEST(SerialArmTemperature, ImplausibleReadingsAreIgnored)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  const SerialArm arm{std::move(transport), SerialArm::Timing{}, fast_guard()};
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.read_temperatures().has_value();
  }));

  // 0 and 200 degC are not temperatures: no cut, and the last good reading stays.
  port.set_temperatures(ServoTemperatures{0, 200, 200, 200, 200, 200});
  std::this_thread::sleep_for(std::chrono::milliseconds{500});
  EXPECT_FALSE(arm.overheat().has_value());
  EXPECT_EQ(arm.read_temperatures().value_or(ServoTemperatures{}), COOL);
  EXPECT_EQ(frames_with_id(port.written(), POWER_OFF_ID).size(), 0U);
  // And no good reading for longer than the stale timeout: the guard reports it is blind.
  EXPECT_TRUE(arm.temperature_stale());
}

TEST(SerialArmTemperature, ReportsStaleWhenTheArmNeverAnswersTheTemperature)
{
  auto transport{std::make_unique<ScriptedArm>()};
  transport->set_answer_temperatures(false);
  const SerialArm arm{std::move(transport), SerialArm::Timing{}, fast_guard()};

  std::this_thread::sleep_for(std::chrono::milliseconds{500});
  EXPECT_TRUE(arm.has_sample());  // the pose still updates
  EXPECT_FALSE(arm.read_temperatures().has_value());
  EXPECT_TRUE(arm.temperature_stale());
  EXPECT_FALSE(arm.overheat().has_value());
}

TEST(SerialArmGripper, ReadsTheGripperWhileIdle)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  port.set_gripper(42);
  const SerialArm arm{std::move(transport), SerialArm::Timing{}, fast_guard(), true};

  ASSERT_TRUE(wait_until([&arm]() {
      return arm.read_gripper().has_value();
  }));
  EXPECT_EQ(arm.read_gripper().value_or(0), 42);
  const auto frames{port.written()};
  for (std::size_t i{1}; i < frames.size(); ++i) {
    if (request_id(frames[i]) == GET_GRIPPER_ID) {
      EXPECT_EQ(request_id(frames[i - 1]), GET_ANGLES_ID) << i;
    }
  }
}

TEST(SerialArmGripper, SendsANewTargetOnceAfterAGetAnglesReply)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  SerialArm arm{std::move(transport), SerialArm::Timing{}, fast_guard(), true};
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.has_sample();
  }));

  arm.write_gripper(30);
  ASSERT_TRUE(wait_until([&port]() {
      return gripper_sets(port) == 1;
  }));
  std::this_thread::sleep_for(std::chrono::milliseconds{200});
  EXPECT_EQ(gripper_sets(port), 1U);

  const auto frames{port.written()};
  const auto set{std::find_if(frames.begin(), frames.end(), is_gripper_set)};
  ASSERT_NE(set, frames.end());
  EXPECT_EQ(
    *set, (std::vector<std::uint8_t>{0xFE, 0xFE, 0x04, 0x67, 30, SerialArm::GRIPPER_SPEED, 0xFA}));
  ASSERT_NE(set, frames.begin());
  EXPECT_EQ(request_id(*std::prev(set)), GET_ANGLES_ID);
}

TEST(SerialArmGripper, WaitsWhileTheArmHasTargetsToSend)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  TemperatureGuard guard{fast_guard()};
  guard.max_age = std::chrono::seconds{5};
  SerialArm arm{std::move(transport), SerialArm::Timing{}, guard, true};
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.has_sample();
  }));

  arm.write_angles(JointPositions{0.0005, 0.0, 0.0, 0.0, 0.0, 0.0});
  arm.write_gripper(10);
  stream_targets(arm, std::chrono::milliseconds{400});
  for (const auto & frame : between_sends(port.written())) {
    EXPECT_FALSE(is_gripper_frame(frame));
  }
  EXPECT_TRUE(wait_until([&port]() {
      return gripper_sets(port) == 1;
  }));
}

TEST(SerialArmGripper, AHaltHoldsTheGripperTargetAndPowerOnDropsIt)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  SerialArm arm{std::move(transport), SerialArm::Timing{}, fast_guard(), true};
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.has_sample();
  }));

  ASSERT_TRUE(arm.halt());
  arm.write_gripper(10);
  std::this_thread::sleep_for(std::chrono::milliseconds{300});
  EXPECT_EQ(gripper_sets(port), 0U);
  EXPECT_TRUE(arm.read_gripper().has_value());

  ASSERT_TRUE(arm.power_on());
  std::this_thread::sleep_for(std::chrono::milliseconds{200});
  EXPECT_EQ(gripper_sets(port), 0U);
  arm.write_gripper(20);
  EXPECT_TRUE(wait_until([&port]() {
      return gripper_sets(port) == 1;
  }));
}

TEST(SerialArmGripper, SendsNoGripperFrameWithoutAGripper)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  SerialArm arm{std::move(transport), SerialArm::Timing{}, fast_guard()};
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.has_sample();
  }));

  arm.write_gripper(10);
  std::this_thread::sleep_for(std::chrono::milliseconds{300});
  EXPECT_EQ(gripper_frames(port), 0U);
  EXPECT_FALSE(arm.read_gripper().has_value());
}

TEST(SerialArmGripper, TheLinkNeverTimesOutWhileTheGripperIsServed)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), SerialArm::Timing{}, TemperatureGuard{}, true};
  wait_for_reply(arm);
  ASSERT_TRUE(arm.has_sample());

  const auto end{std::chrono::steady_clock::now() + std::chrono::seconds{1}};
  auto next_target{std::chrono::steady_clock::now()};
  std::uint8_t value{0};
  while (std::chrono::steady_clock::now() < end) {
    if (std::chrono::steady_clock::now() >= next_target) {
      value = static_cast<std::uint8_t>((value + 10) % 101);
      arm.write_gripper(value);
      next_target += std::chrono::milliseconds{100};
    }
    ASSERT_FALSE(arm.link_timed_out());
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  const auto frames{port.written()};
  EXPECT_GT(std::count_if(frames.begin(), frames.end(), is_gripper_set), 0);
  EXPECT_GT(
    std::count_if(
      frames.begin(), frames.end(),
      [](const auto & frame) {
        return request_id(frame) == GET_GRIPPER_ID;
      }),
    0);
  EXPECT_FALSE(arm.read_gripper().has_value());
}

// #103 part 2: what the plugin's unfollowed-command check reads.
TEST(SerialArm, TheLastTargetFollowsTheWireAndAnAcknowledgedStopClearsIt)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  EXPECT_FALSE(arm.last_target().has_value());  // nothing sent yet

  const JointPositions target{0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
  arm.write_angles(target);
  ASSERT_GE(wait_for_sends(port, 1).size(), 1U);
  // The thread stores the target once write() has returned, a moment after the frame is on the
  // wire: wait for it rather than read it at once (it read nullopt on a loaded machine).
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.last_target().has_value();
  }));
  EXPECT_EQ(arm.last_target(), std::optional<JointPositions>{target});

  // The stop is acknowledged: the arm is heading nowhere, there is no target to follow.
  ASSERT_TRUE(arm.halt());
  EXPECT_FALSE(arm.last_target().has_value());
}

// #170. The pose from the Atom's poll, and the turn that follows the controller's write.

namespace
{

/// ECHO_TIMING with the pose from the poll on: a 30 ms period, a 90 ms echo_timeout.
SerialArm::Timing poll_timing()
{
  SerialArm::Timing timing{ECHO_TIMING};
  timing.pose_from_poll = true;
  return timing;
}
const SerialArm::Timing POLL_TIMING{poll_timing()};

/// The replayed arm with `frames` as the Atom's poll before each sync-write, and its line quiet
/// between a send and that sync-write, like the real arm.
std::unique_ptr<ReplayTransport> polling_port(
  std::vector<std::uint8_t> frames = parse_hex(RECORDED_POLL_FRAMES))
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  transport->set_poll_frames(std::move(frames));
  transport->set_quiet_after_send(true);
  return transport;
}

/// The pose the six recorded poll frames carry, through the Atom's own conversion.
JointPositions recorded_poll_pose()
{
  positronic_protocol::JointAngles angles{};
  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    angles.at(joint) = positronic_protocol::angle_from_steps(RECORDED_POLL_STEPS.at(joint), joint);
  }
  return positronic_protocol::to_radians(angles);
}

/// The get_angles frames written between `from` and `to`.
std::size_t get_angles_between(
  ReplayTransport & port, std::chrono::steady_clock::time_point from,
  std::chrono::steady_clock::time_point to)
{
  const auto frames{port.written()};
  const auto times{port.written_at()};
  std::size_t count{0};
  for (std::size_t i{0}; i < frames.size() && i < times.size(); ++i) {
    if (request_id(frames.at(i)) == GET_ANGLES_ID && times.at(i) > from && times.at(i) < to) {
      ++count;
    }
  }
  return count;
}

/// One servo status frame, and where its 15 data bytes start (FF FF id len err).
constexpr std::size_t POLL_FRAME{21};
constexpr std::size_t POLL_DATA{5};

/// `frames` with byte `offset` of servo `joint`'s data (0 = J1) set to `value`, checksum redone:
/// `offset` 0 and 1 are the position (little-endian), 6 the voltage (0.1 V), 7 the temperature.
std::vector<std::uint8_t> with_byte(
  std::vector<std::uint8_t> frames, std::size_t joint, std::size_t offset, std::uint8_t value)
{
  const std::size_t start{joint * POLL_FRAME};
  frames.at(start + POLL_DATA + offset) = value;
  unsigned sum{0};
  for (std::size_t i{start + 2}; i < start + POLL_FRAME - 1; ++i) {
    sum += frames.at(i);
  }
  frames.at(start + POLL_FRAME - 1) = static_cast<std::uint8_t>(~sum);
  return frames;
}

/// `frames` with servo `joint` at `steps`.
std::vector<std::uint8_t> with_steps(
  std::vector<std::uint8_t> frames, std::size_t joint, std::uint16_t steps)
{
  frames = with_byte(std::move(frames), joint, 0, static_cast<std::uint8_t>(steps & 0xFFU));
  return with_byte(std::move(frames), joint, 1, static_cast<std::uint8_t>(steps >> 8U));
}

double rad_of(std::uint16_t steps, std::size_t joint)
{
  return positronic_protocol::to_radians(positronic_protocol::angle_from_steps(steps, joint));
}

/// A test's second poll moves its `marker` joint this many steps, a change no rule rejects: once
/// the pose shows it, the thread has published that poll, so whatever else the test changed in
/// it was judged. Without the marker a check that the pose stayed would pass even if the poll was
/// never read.
constexpr std::uint16_t MARKER_STEPS{10};

/// The recorded poll with `marker` moved MARKER_STEPS.
std::vector<std::uint8_t> marked_poll(std::size_t marker)
{
  const auto steps{static_cast<std::uint16_t>(RECORDED_POLL_STEPS.at(marker) + MARKER_STEPS)};
  return with_steps(parse_hex(RECORDED_POLL_FRAMES), marker, steps);
}

/// Waits until the published pose shows `marker` moved MARKER_STEPS. True when it did.
bool marker_shown(const SerialArm & arm, std::size_t marker)
{
  const double moved{
    rad_of(static_cast<std::uint16_t>(RECORDED_POLL_STEPS.at(marker) + MARKER_STEPS), marker)};
  return wait_until([&arm, marker, moved]() {
             return std::abs(arm.read_angles().at(marker) - moved) < 1e-9;
  });
}

/// The default timing (33 ms period) on the polling arm: a first send, then `delay` later a
/// second write. How long after that write its send went out, or a minute if it never did.
std::chrono::steady_clock::duration second_send_after(std::chrono::milliseconds delay)
{
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport)};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  static_cast<void>(wait_for_sends(port, 1));
  std::this_thread::sleep_for(delay);
  const auto written{std::chrono::steady_clock::now()};
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  if (wait_for_sends(port, 2).size() < 2) {
    return std::chrono::minutes{1};
  }
  const auto frames{port.written()};
  const auto times{port.written_at()};
  std::size_t second{0};
  for (std::size_t i{0}; i < frames.size() && i < times.size(); ++i) {
    if (frames.at(i).size() == 18) {
      second = i;
    }
  }
  return times.at(second) - written;
}

/// The time the first frame with request id `id` was written, if any.
std::optional<std::chrono::steady_clock::time_point> first_written(
  ReplayTransport & port, std::uint8_t id)
{
  const auto frames{port.written()};
  const auto times{port.written_at()};
  for (std::size_t i{0}; i < frames.size() && i < times.size(); ++i) {
    if (request_id(frames.at(i)) == id) {
      return times.at(i);
    }
  }
  return std::nullopt;
}

/// `frames` without servo `joint`'s frame, as when it failed its checksum.
std::vector<std::uint8_t> without_frame(std::vector<std::uint8_t> frames, std::size_t joint)
{
  const auto start{std::next(frames.begin(), static_cast<std::ptrdiff_t>(joint * POLL_FRAME))};
  frames.erase(start, std::next(start, static_cast<std::ptrdiff_t>(POLL_FRAME)));
  return frames;
}

}  // namespace

TEST(SerialArmPollPose, ThePollFramesAreThePoseAndNoGetAnglesFollowsTheSend)
{
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  const auto sends{wait_for_sends(port, 1)};
  ASSERT_EQ(sends.size(), 1u);
  const auto sent_at{port.written_at().back()};
  // The sync-write (and the frames before it) land 20 ms after the send; the pose is published
  // then, and the next turn asks nothing. Only after two periods without a write does the
  // thread poll again.
  std::this_thread::sleep_for(SYNC_WRITE_DELAY + DECODE_MARGIN);
  const JointPositions pose{arm.read_angles()};
  const JointPositions expected{recorded_poll_pose()};
  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    EXPECT_NEAR(pose.at(joint), expected.at(joint), 1e-9) << joint;
  }
  EXPECT_EQ(get_angles_between(port, sent_at, std::chrono::steady_clock::now()), 0u);
  EXPECT_EQ(arm.echo_timeouts(), 0u);
}

TEST(SerialArmPollPose, AWriteWakesTheThreadAndTheSendFollowsIt)
{
  // Past the sync-write, inside the two-period wait: a second write must be sent at once.
  EXPECT_LT(second_send_after(SYNC_WRITE_DELAY + DECODE_MARGIN), WAKE_MARGIN);
}

TEST(SerialArmPollPose, AWriteDuringTheSyncWriteWaitIsSentRightAfterIt)
{
  // Inside the wait for the sync-write (20 ms): the write lands while the thread reads the
  // line. It must go out as soon as the sync-write closes, not at the two-period deadline
  // (66 ms), which is where it went when the wait cleared the flag (1 % of the sends on the
  // arm, 20 Nov 2026).
  EXPECT_LT(second_send_after(std::chrono::milliseconds{5}), SYNC_WRITE_DELAY + DECODE_MARGIN);
}

TEST(SerialArmPollPose, WithoutWritesTheThreadPollsGetAnglesAgain)
{
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);
  const auto sent_at{port.written_at().back()};
  // Two periods (60 ms here) after the send, with no write, the fallback polling resumes. 60 ms
  // more is several 10 ms poll turns past that deadline, plus the scheduler.
  constexpr std::chrono::milliseconds FALLBACK_MARGIN{60};
  std::this_thread::sleep_for(2 * POLL_TIMING.moving_period + FALLBACK_MARGIN);
  EXPECT_GT(get_angles_between(port, sent_at, std::chrono::steady_clock::now()), 0u);
}

TEST(SerialArmPollPose, AHaltWakesTheThreadAndTheStopGoesOutAtOnce)
{
  // After a send whose poll gave the pose, the thread waits up to two periods for the next
  // write. A halt stops the writes, so it must end that wait itself: the stop goes out at once,
  // not at the deadline (up to 60 ms here, 66 ms on the arm).
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);
  // The poll's pose is published as the sync-write wait ends: the thread is in the long wait.
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.read_angles() == recorded_poll_pose();
  }));
  const auto halted_at{std::chrono::steady_clock::now()};
  arm.request_halt();
  ASSERT_TRUE(wait_until([&port]() {
      return first_written(port, STOP_ID).has_value();
  }));
  EXPECT_LT(first_written(port, STOP_ID).value_or(halted_at) - halted_at, WAKE_MARGIN);
}

TEST(SerialArmPollPose, AStopThatStaysPendingDoesNotSpinTheThread)
{
  // A stop the port refuses stays pending, and every turn tries it again. Only a new request may
  // end a turn's wait early: here the turns keep their 10 ms period (POLL_PERIOD), about 30
  // get_angles in 300 ms, where ending every wait on the pending stop asked them back to back.
  // 45 leaves half a period of margin per turn for the scheduler.
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  port.set_reject_requests(true);
  arm.request_halt();
  const auto from{std::chrono::steady_clock::now()};
  std::this_thread::sleep_for(std::chrono::milliseconds{300});
  EXPECT_LE(get_angles_between(port, from, std::chrono::steady_clock::now()), 45U);
}

/// Writes `target` every 33 ms, as the controller at 30 Hz, until `end`.
void write_until(
  SerialArm & arm, const JointPositions & target, std::chrono::steady_clock::time_point end)
{
  while (std::chrono::steady_clock::now() < end) {
    arm.write_angles(target);
    std::this_thread::sleep_for(std::chrono::milliseconds{33});
  }
}

TEST(SerialArmPollPose, ATargetWithoutItsSyncWriteIsNotSentOnEveryWrite)
{
  // An arm that never sync-writes a send (a target it refuses). With the pose from the poll an
  // unchanged target would go out again on every write, each time waiting the echo_timeout. It
  // goes out once a second (REPEAT_RETRY): once in the first 400 ms, once more by 1.5 s, ~500 ms
  // from the retry on each side. The pose comes from get_angles, the link stays up, and a
  // changed target still goes out.
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_sync_write(false);
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  const JointPositions refused{0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  write_until(arm, refused, std::chrono::steady_clock::now() + std::chrono::milliseconds{400});
  EXPECT_EQ(send_angles_frames(port.written()).size(), 1U);
  EXPECT_EQ(arm.echo_timeouts(), 1U);
  EXPECT_FALSE(arm.link_timed_out());
  write_until(arm, refused, std::chrono::steady_clock::now() + std::chrono::milliseconds{1100});
  EXPECT_EQ(send_angles_frames(port.written()).size(), 2U);
  EXPECT_EQ(arm.echo_timeouts(), 2U);
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  EXPECT_EQ(wait_for_sends(port, 3).size(), 3U);
}

TEST(SerialArmPollPose, ARepeatStoppedByOneLostSyncWriteComesBackAfterTheRetry)
{
  // One send the Atom dropped (24 Nov 2026: it went out on a late get_angles reply), then the
  // arm answers again, while the controller writes the same target at rest. Nothing goes out
  // again before REPEAT_RETRY after the unanswered send; after it the repeat is back on every
  // write. The first count is 300 ms before the retry is due, the second 700 ms after it: ~20
  // writes then.
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_sync_write(false);
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  const JointPositions held{0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  arm.write_angles(held);
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1U);
  port.set_sync_write(true);
  const auto sent{std::chrono::steady_clock::now()};

  write_until(arm, held, sent + SerialArm::REPEAT_RETRY - std::chrono::milliseconds{300});
  EXPECT_EQ(send_angles_frames(port.written()).size(), 1U);
  write_until(arm, held, sent + SerialArm::REPEAT_RETRY + std::chrono::milliseconds{700});
  EXPECT_GE(send_angles_frames(port.written()).size(), 5U);
}

TEST(SerialArmPollPose, AMissingFrameBringsTheGetAnglesBack)
{
  // Servo 3's frame left out (a bad checksum leaves it out the same way): five good frames are
  // not a pose, the next turn asks get_angles and the pose is its reply, like before #170. It
  // comes one period after the send (~30 ms): two periods leave 30 ms for the scheduler. A
  // published poll would put it at two periods plus a 10 ms idle turn (~70 ms).
  auto transport{polling_port(without_frame(parse_hex(RECORDED_POLL_FRAMES), 2))};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING};
  const JointPositions replayed{wait_for_reply(arm)};
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);
  std::this_thread::sleep_for(2 * POLL_TIMING.echo_timeout);
  const SendThenPoll turn{first_send_then_poll(port)};
  ASSERT_TRUE(turn.found);
  EXPECT_LT(turn.poll - turn.send, 2 * POLL_TIMING.moving_period);
  const JointPositions pose{arm.read_angles()};
  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    EXPECT_NEAR(pose.at(joint), replayed.at(joint), 1e-9) << joint;
  }
}

TEST(SerialArmPollPose, AtRestTheGripperTakesTheTurnOfTheRepeatedSend)
{
  // The gripper (#174) is served only in a turn that sends nothing, and with the pose from the
  // poll every write goes out, even an unchanged one. At rest the repeat gives way to the
  // gripper: its new value goes out once and it is read, while the controller keeps writing
  // the same pose every 33 ms. The replay does not answer the gripper, so each read waits its
  // 50 ms: in 600 ms the 100 ms read period leaves room for at least 2, and the sends go on.
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING, TemperatureGuard{}, true};
  static_cast<void>(wait_for_reply(arm));
  const JointPositions rest{0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  const auto write_rest{[&arm, &rest](std::chrono::milliseconds duration) {
      const auto end{std::chrono::steady_clock::now() + duration};
      while (std::chrono::steady_clock::now() < end) {
        arm.write_angles(rest);
        std::this_thread::sleep_for(std::chrono::milliseconds{33});
      }
    }};

  write_rest(std::chrono::milliseconds{300});
  arm.write_gripper(30);
  write_rest(std::chrono::milliseconds{600});

  // Between the first and the last send only: the turns before the stream are idle and serve
  // the gripper anyway.
  const auto during{between_sends(port.written())};
  EXPECT_EQ(std::count_if(during.begin(), during.end(), is_gripper_set), 1);
  EXPECT_GE(frames_with_id(during, GET_GRIPPER_ID).size(), 2U);
  EXPECT_GE(send_angles_frames(during).size(), 5U);
}

TEST(SerialArmPollPose, AWriteDuringAGetAnglesCountsForItsTurn)
{
  // At rest the controller writes the same pose every 33 ms and a due gripper takes the turn of
  // a repeat; the turn after it starts with a get_angles. Here a get_angles takes 40 ms, longer
  // than the write period, so after a gripper read every write lands during one. Counted only
  // when it ended the thread's wait, none did, and the sends stopped at the first gripper read;
  // counted for its turn whenever it lands, the repeat goes on. No clock in the check: about 2
  // sends against about 20. (On the arm a get_angles takes 7 ms, and the sends paused 48 to
  // 176 ms after each gripper read, 23 Nov 2026.)
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  port.set_angles_delay(std::chrono::milliseconds{40});
  port.set_answer_gripper(true);
  SerialArm arm{std::move(transport), POLL_TIMING, TemperatureGuard{}, true};
  static_cast<void>(wait_for_reply(arm));
  const JointPositions rest{0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  const auto end{std::chrono::steady_clock::now() + std::chrono::milliseconds{1500}};
  while (std::chrono::steady_clock::now() < end) {
    arm.write_angles(rest);
    std::this_thread::sleep_for(std::chrono::milliseconds{33});
  }

  // The sends after the first gripper read that comes after a send.
  const auto frames{port.written()};
  const auto first_send{std::find_if(frames.begin(), frames.end(), [](const auto & frame) {
        return frame.size() == 18;
  })};
  ASSERT_NE(first_send, frames.end());
  const auto first_read{std::find_if(first_send, frames.end(), [](const auto & frame) {
        return request_id(frame) == GET_GRIPPER_ID;
  })};
  ASSERT_NE(first_read, frames.end());
  const auto sends_after{std::count_if(first_read, frames.end(), [](const auto & frame) {
        return frame.size() == 18;
  })};
  EXPECT_GE(sends_after, 10);
}

TEST(SerialArmPollPose, AfterTheTemperaturesTheNextTurnSendsWithoutAGetAngles)
{
  // A poll with the J3 frame missing: no pose and no temperatures from it, so the thread asks
  // get_angles every turn and the temperatures once max_age, as with a hot servo whose frames
  // come corrupt. (With a whole poll the guard asks nothing: the frames are the reading.)
  const std::vector<std::uint8_t> missing{without_frame(parse_hex(RECORDED_POLL_FRAMES), 2)};
  auto transport{polling_port(missing)};
  ReplayTransport & port{*transport};
  // The requests are the only readings, as in AnOverdueReadingTakesTheTurn...: a stale timeout
  // over max_age.
  TemperatureGuard guard{fast_guard()};
  guard.stale_timeout = std::chrono::milliseconds{1000};
  SerialArm arm{std::move(transport), POLL_TIMING, guard};
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.read_temperatures().has_value();
  }));

  stream_targets(arm, std::chrono::milliseconds{1200});
  const auto during{between_sends(port.written())};
  // max_age 300 ms over a 1.2 s stream: a few temperature requests. The reply to each proves
  // the arm's input is free, so the turn after it sends at once: the frame after a request is
  // a send_angles, not a get_angles (200 of them in 10 min before this, 20 Nov 2026).
  ASSERT_GE(frames_with_id(during, TEMPS_ID).size(), 2U);
  for (std::size_t i{0}; i + 1 < during.size(); ++i) {
    if (request_id(during[i]) == TEMPS_ID) {
      EXPECT_EQ(during[i + 1].size(), 18U) << i;
    }
  }
  EXPECT_FALSE(arm.temperature_stale());
}

TEST(SerialArmPollPose, AJointMissingFromOnePollKeepsItsLastGoodPosition)
{
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);
  ASSERT_TRUE(wait_until([&arm]() {
      return arm.read_angles() == recorded_poll_pose();
  }));

  // J3's frame drops out of the poll. Within POLL_BRIDGE_AGE the pose stays whole from the
  // last good J3 and no get_angles goes out; after it, the get_angles comes back.
  port.set_poll_frames(without_frame(marked_poll(0), 2));  // J1 the marker
  const auto sent_at{std::chrono::steady_clock::now()};
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 2).size(), 2u);
  ASSERT_TRUE(marker_shown(arm, 0));
  EXPECT_NEAR(arm.read_angles().at(2), recorded_poll_pose().at(2), 1e-9);
  EXPECT_EQ(get_angles_between(port, sent_at, std::chrono::steady_clock::now()), 0u);

  // Past the age the poll is not published, so the next turn asks get_angles one period after
  // the send (~30 ms; the window of two periods leaves 30 ms for the scheduler). A published
  // poll would put it at two periods plus a 10 ms idle turn. Three periods: the window is over.
  std::this_thread::sleep_for(SerialArm::POLL_BRIDGE_AGE);
  arm.write_angles(JointPositions{0.3, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 3).size(), 3u);
  const auto third{port.written_at().back()};
  std::this_thread::sleep_for(3 * POLL_TIMING.moving_period);
  EXPECT_GE(get_angles_between(port, third, third + 2 * POLL_TIMING.moving_period), 1u);
}

TEST(SerialArmPollPose, TheTemperaturesComeFromThePollAndNothingIsAsked)
{
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  // A 600 ms period: without the poll as a reading the guard still asks within the 1.2 s
  // stream, and after it the first ask at rest comes ~600 ms after the last poll, so the check
  // below has ~570 ms of margin before a COOL reply replaces the frames' temperatures.
  TemperatureGuard guard{fast_guard()};
  guard.period = std::chrono::milliseconds{600};
  guard.max_age = std::chrono::milliseconds{600};
  SerialArm arm{std::move(transport), POLL_TIMING, guard};
  static_cast<void>(wait_for_reply(arm));

  stream_targets(arm, std::chrono::milliseconds{1200});
  // Every send brought a whole poll, a reading every 33 ms, so the guard never had to ask.
  EXPECT_EQ(arm.read_temperatures(), std::optional<ServoTemperatures>{RECORDED_POLL_TEMPS});
  EXPECT_EQ(frames_with_id(between_sends(port.written()), TEMPS_ID).size(), 0U);
  EXPECT_FALSE(arm.temperature_stale());
  EXPECT_FALSE(arm.overheat().has_value());
}

TEST(SerialArmPollPose, HotPollsCutTheArmOnceTheyLastHotFor)
{
  auto transport{polling_port(with_byte(parse_hex(RECORDED_POLL_FRAMES), 1, 7, 61))};  // J2 at 61
  ReplayTransport & port{*transport};
  // Every ack 60 ms late: the power off must follow the stop's ack. The Atom takes ~37 ms over
  // a stop and drops what comes before; a power off sent 30 ms after it was dropped in all three
  // recorded cuts. 60 is under ACK_TIMEOUT and far from the old 30 ms.
  port.set_ack_delay(std::chrono::milliseconds{60});
  const TemperatureGuard guard{fast_guard()};
  SerialArm arm{std::move(transport), POLL_TIMING, guard};
  static_cast<void>(wait_for_reply(arm));

  // Six hot_for of hot polls. The cut comes once they have lasted hot_for, not at the second
  // poll: a stop, a power off after its ack, and no target goes out after them.
  stream_targets(arm, std::chrono::milliseconds{600});
  ASSERT_TRUE(arm.overheat().has_value());
  const Overheat trip{arm.overheat().value_or(Overheat{})};
  EXPECT_EQ(trip.joint, 1U);
  EXPECT_EQ(trip.celsius, 61);
  const auto frames{port.written()};
  const auto times{port.written_at()};
  const auto is_send{[](const auto & frame) {
      return frame.size() == 18;
    }};
  const auto first_send{std::find_if(frames.begin(), frames.end(), is_send)};
  const auto stop_at{std::find_if(frames.begin(), frames.end(), [](const auto & frame) {
        return request_id(frame) == STOP_ID;
  })};
  ASSERT_NE(first_send, frames.end());
  ASSERT_NE(stop_at, frames.end());
  const auto off_at{std::find_if(stop_at, frames.end(), [](const auto & frame) {
        return request_id(frame) == POWER_OFF_ID;
  })};
  ASSERT_NE(off_at, frames.end());
  // The first hot reading came after the first send, the trip hot_for after it, then the stop.
  const auto send_time{times.at(static_cast<std::size_t>(first_send - frames.begin()))};
  const auto stop_time{times.at(static_cast<std::size_t>(stop_at - frames.begin()))};
  const auto off_time{times.at(static_cast<std::size_t>(off_at - frames.begin()))};
  EXPECT_GE(stop_time - send_time, guard.hot_for);
  EXPECT_GE(off_time - stop_time, std::chrono::milliseconds{60});
  EXPECT_GE(send_angles_frames({frames.begin(), stop_at}).size(), 2U);
  EXPECT_EQ(send_angles_frames({off_at, frames.end()}).size(), 0U);
}

TEST(SerialArmPollPose, ALateAnswerIsNotTakenForTheNextRequests)
{
  // 24 Nov 2026, tap at 113.39 s: a get_angles answered past its 50 ms window, the late answer
  // taken for the next request's, and what followed reached the Atom while it was still
  // answering: dropped. Here the first answer comes 52 ms after its request, 2 ms past
  // REPLY_TIMEOUT as on the arm, the others at once. Nothing may be written while it is due,
  // and the sends go on: about 15 in the 600 ms stream, 5 asked for.
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  port.set_angles_delay(std::chrono::milliseconds{52});
  SerialArm arm{std::move(transport), POLL_TIMING};
  ASSERT_TRUE(wait_until([&port]() {
      return !frames_with_id(port.written(), GET_ANGLES_ID).empty();
  }));
  port.set_angles_delay(std::chrono::milliseconds{0});
  stream_targets(arm, std::chrono::milliseconds{600});
  EXPECT_EQ(port.written_while_busy(), 0U);
  EXPECT_GE(send_angles_frames(port.written()).size(), 5U);
}

TEST(SerialArmPollPose, ALateAckIsNotTakenForTheNextRequests)
{
  // A stop acked 86 ms after it, 6 ms past ACK_TIMEOUT (seen once mid-move, 16 Nov 2026): what
  // the thread writes next, the stop again or a get_angles, must wait for the late ack. 400 ms
  // of a halt on a writing controller.
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  port.set_ack_delay(std::chrono::milliseconds{86});
  arm.request_halt();
  stream_targets(arm, std::chrono::milliseconds{400});
  EXPECT_EQ(port.written_while_busy(), 0U);
}

TEST(SerialArmPollPose, AnUnacknowledgedPowerOffGoesOutAgainNextTurn)
{
  // A cut whose power off gets no ack (dropped): it goes out again at the next turn, not a
  // period later. The period is 1 s here, so a second power off within 400 ms of the stop is
  // that resend; the repeat once a period would be the second at about 1 s.
  auto transport{polling_port(with_byte(parse_hex(RECORDED_POLL_FRAMES), 1, 7, 61))};  // J2 at 61
  ReplayTransport & port{*transport};
  port.set_ack_power_off(false);
  TemperatureGuard guard{fast_guard()};
  guard.period = std::chrono::milliseconds{1000};
  guard.stale_timeout = std::chrono::milliseconds{2000};
  SerialArm arm{std::move(transport), POLL_TIMING, guard};
  static_cast<void>(wait_for_reply(arm));
  stream_targets(arm, std::chrono::milliseconds{600});
  ASSERT_TRUE(arm.overheat().has_value());
  const auto frames{port.written()};
  const auto times{port.written_at()};
  const auto stop_at{std::find_if(frames.begin(), frames.end(), [](const auto & frame) {
        return request_id(frame) == STOP_ID;
  })};
  ASSERT_NE(stop_at, frames.end());
  const auto stop_time{times.at(static_cast<std::size_t>(stop_at - frames.begin()))};
  ASSERT_TRUE(wait_until([stop_time]() {
      return std::chrono::steady_clock::now() - stop_time > std::chrono::milliseconds{400};
  }));
  std::size_t offs{0};
  const auto all{port.written()};
  const auto all_times{port.written_at()};
  for (std::size_t i{0}; i < all.size(); ++i) {
    if (
      request_id(all[i]) == POWER_OFF_ID &&
      all_times[i] - stop_time < std::chrono::milliseconds{400})
    {
      ++offs;
    }
  }
  EXPECT_GE(offs, 2U);
}

TEST(SerialArmPollPose, PollsWithABridgedFrameLeaveTheAsksGoing)
{
  // A different joint missing in each poll: the bridge covers it, so every poll is the pose, but
  // none is a temperature reading. The guard must keep asking: were a bridged poll taken as a
  // reading, the asks would stop and the guard go blind (temperature_stale, then a halt).
  auto transport{polling_port()};  // one whole poll to bridge from
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING, fast_guard()};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.01, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::size_t sends{1};
  ASSERT_GE(wait_for_sends(port, sends).size(), sends);

  // Twice fast_guard's max_age (300 ms): an ask is due at least once in the stream.
  const auto end{std::chrono::steady_clock::now() + std::chrono::milliseconds{600}};
  while (std::chrono::steady_clock::now() < end) {
    port.set_poll_frames(without_frame(parse_hex(RECORDED_POLL_FRAMES), 2 + sends % 2));
    ++sends;
    arm.write_angles(JointPositions{0.01 * static_cast<double>(sends), 0.0, 0.0, 0.0, 0.0, 0.0});
    ASSERT_GE(wait_for_sends(port, sends).size(), sends);
  }
  EXPECT_GE(frames_with_id(between_sends(port.written()), TEMPS_ID).size(), 1U);
}

TEST(SerialArmPollPose, AnUnpublishedPollStillKeepsItsGoodFramesForTheBridge)
{
  // A poll that cannot be published (J3 missing, its last frame too old) still brings J5's good
  // frame, and the next poll, missing J5, bridges it from there. Before, the loop stopped at J3
  // without keeping J5, and the next poll fell back to get_angles too (20 Nov 2026, hot J4).
  constexpr std::size_t MARKER{4};  // J5
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_GE(wait_for_sends(port, 1).size(), 1u);
  // Past the bridge age: J3's frame of this poll can no longer stand in.
  std::this_thread::sleep_for(SerialArm::POLL_BRIDGE_AGE + SYNC_WRITE_DELAY + DECODE_MARGIN);

  port.set_poll_frames(without_frame(marked_poll(MARKER), 2));  // not published: no J3
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_GE(wait_for_sends(port, 2).size(), 2u);
  // The transport took that poll at the send: the next one can be set at once. It comes after a
  // get_angles and a send, ~50 ms after the poll above on the replay, inside the 100 ms bridge.
  port.set_poll_frames(without_frame(parse_hex(RECORDED_POLL_FRAMES), MARKER));
  arm.write_angles(JointPositions{0.3, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_GE(wait_for_sends(port, 3).size(), 3u);
  EXPECT_TRUE(marker_shown(arm, MARKER));
}

TEST(SerialArmPollPose, OffKeepsTheTurnOfBefore)
{
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm::Timing timing{POLL_TIMING};
  timing.pose_from_poll = false;
  SerialArm arm{std::move(transport), timing};
  const JointPositions replayed{wait_for_reply(arm)};
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  ASSERT_EQ(wait_for_sends(port, 1).size(), 1u);
  // The frames are on the line and ignored: 10 ms after the sync-write the pose is still the
  // replayed reply, not the poll (a poll read would stand until the turn after next), and a
  // get_angles follows the send.
  std::this_thread::sleep_until(port.written_at().back() + SYNC_WRITE_DELAY + WAKE_MARGIN);
  const JointPositions pose{arm.read_angles()};
  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    EXPECT_NEAR(pose.at(joint), replayed.at(joint), 1e-9) << joint;
  }
  std::this_thread::sleep_for(2 * POLL_TIMING.echo_timeout);
  const SendThenPoll turn{first_send_then_poll(port)};
  ASSERT_TRUE(turn.found);
  EXPECT_LT(turn.poll - turn.send, POLL_TIMING.echo_timeout);
}

// Follow-up prompt (#170, the plausibility edges):
//   Check the edges of the plausibility rules of a poll frame: voltage 4.0 and 13.0 V accepted,
//   3.9 and 13.1 V not; temperature 1 and 99 degC accepted, 0 and 100 not; a jump of 338 steps
//   from the last pose accepted and 342 steps (past 0.52 rad) not. A rejected frame is bridged
//   from the previous poll, so the pose keeps the recorded steps.

namespace
{
/// The recorded frames with `marker` moved, `joint` at `steps` and byte `offset` of its data at
/// `value`.
std::vector<std::uint8_t> frames_at(
  std::size_t joint, std::uint16_t steps, std::size_t offset, std::uint8_t value,
  std::size_t marker)
{
  return with_steps(with_byte(marked_poll(marker), joint, offset, value), joint, steps);
}

/// Sends one command with the given poll frames and returns the pose published after the poll,
/// once it shows `marker` moved.
JointPositions pose_after_poll(const std::vector<std::uint8_t> & frames, std::size_t marker)
{
  auto transport{polling_port()};
  ReplayTransport & port{*transport};
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  // A first poll publishes the recorded pose: the reference of the jump check and the bridge.
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  static_cast<void>(wait_for_sends(port, 1));
  EXPECT_TRUE(wait_until([&arm]() {
      return arm.read_angles() == recorded_poll_pose();
  }));
  port.set_poll_frames(frames);
  arm.write_angles(JointPositions{0.2, 0.0, 0.0, 0.0, 0.0, 0.0});
  static_cast<void>(wait_for_sends(port, 2));
  EXPECT_TRUE(marker_shown(arm, marker));
  return arm.read_angles();
}

}  // namespace

TEST(SerialArmPollPose, TheVoltageAndTemperatureEdgesOfAPlausibleFrame)
{
  // J3 (index 2) moved 10 steps so an accepted frame shows in the pose; offset 6 is the voltage
  // byte (0.1 V), 7 the temperature. Accepted: the pose follows. Rejected: the joint is bridged
  // from the previous poll, so the pose keeps the recorded steps. J1 is the marker.
  const std::uint16_t moved{static_cast<std::uint16_t>(RECORDED_POLL_STEPS.at(2) + 10)};
  const double followed{rad_of(moved, 2)};
  const double kept{rad_of(RECORDED_POLL_STEPS.at(2), 2)};
  EXPECT_NEAR(pose_after_poll(frames_at(2, moved, 6, 40, 0), 0).at(2), followed, 1e-9);   // 4.0 V
  EXPECT_NEAR(pose_after_poll(frames_at(2, moved, 6, 130, 0), 0).at(2), followed, 1e-9);  // 13.0 V
  EXPECT_NEAR(pose_after_poll(frames_at(2, moved, 6, 39, 0), 0).at(2), kept, 1e-9);       // 3.9 V
  EXPECT_NEAR(pose_after_poll(frames_at(2, moved, 6, 131, 0), 0).at(2), kept, 1e-9);      // 13.1 V
  EXPECT_NEAR(pose_after_poll(frames_at(2, moved, 7, 1, 0), 0).at(2), followed, 1e-9);    // 1 degC
  EXPECT_NEAR(pose_after_poll(frames_at(2, moved, 7, 99, 0), 0).at(2), followed, 1e-9);   // 99 degC
  EXPECT_NEAR(pose_after_poll(frames_at(2, moved, 7, 0, 0), 0).at(2), kept, 1e-9);        // 0 degC
  EXPECT_NEAR(pose_after_poll(frames_at(2, moved, 7, 100, 0), 0).at(2), kept, 1e-9);  // 100 degC
}

TEST(SerialArmPollPose, TheJumpEdgeOfAPlausibleFrame)
{
  // 0.52 rad is 339.1 steps: 338 steps away is a pose, 342 is not (bridged, the pose stays). J2
  // is the marker.
  const std::uint16_t near{static_cast<std::uint16_t>(RECORDED_POLL_STEPS.at(0) - 338)};
  const std::uint16_t far{static_cast<std::uint16_t>(RECORDED_POLL_STEPS.at(0) - 342)};
  EXPECT_NEAR(pose_after_poll(frames_at(0, near, 7, 0x23, 1), 1).at(0), rad_of(near, 0), 1e-9);
  EXPECT_NEAR(
    pose_after_poll(frames_at(0, far, 7, 0x23, 1), 1).at(0), rad_of(RECORDED_POLL_STEPS.at(0), 0),
    1e-9);
}

// ---------------------------------------------------------------------------
// Age of the pose (#120)
// ---------------------------------------------------------------------------

namespace
{
using positronic_hardware::PoseSample;
using SteadyClock = std::chrono::steady_clock;

/// Stops the replies and waits until the thread has published the last one: nothing of it left
/// to hand out, and a frame written since. Returns when that reply went out.
std::optional<SteadyClock::time_point> lose_replies(ScriptedArm & port)
{
  port.set_answer_angles(false);
  const auto published{[&port]() {
      return port.last_reply_published();
    }};
  if (!wait_until(published)) {
    return std::nullopt;
  }
  return port.angles_handed().back();
}

/// Reads the age now and checks it against the reply that went out at `arrived`: never negative,
/// and never more than the time since then, since the thread decodes a reply after it went out.
/// Only an upper bound: how soon after the thread decoded it depends on the machine.
PoseSample expect_age_within(const SerialArm & arm, SteadyClock::time_point arrived)
{
  const PoseSample sample{arm.read_pose()};
  const auto after{SteadyClock::now()};
  EXPECT_TRUE(sample.age.has_value());
  const SteadyClock::duration age{sample.age.value_or(SteadyClock::duration::max())};
  EXPECT_GE(age, SteadyClock::duration::zero());
  EXPECT_LE(age, after - arrived);
  return sample;
}

/// How long the replies stay lost: well past a turn (POLL_PERIOD), so the growth stands out.
constexpr std::chrono::milliseconds LOST_FOR{150};
}  // namespace

TEST(SerialArmPoseAge, IsUnknownBeforeTheFirstReply)
{
  auto transport{std::make_unique<ScriptedArm>()};
  transport->set_answer_angles(false);
  const SerialArm arm{std::move(transport)};
  std::this_thread::sleep_for(3 * SerialArm::POLL_PERIOD);
  const PoseSample sample{arm.read_pose()};
  EXPECT_FALSE(arm.has_sample());
  EXPECT_FALSE(sample.age.has_value());
  EXPECT_EQ(sample.angles, arm.read_angles());  // the placeholder, never NaN
}

TEST(SerialArmPoseAge, GrowsWhileRepliesAreLostAndDropsWithTheNextOne)
{
  auto transport{std::make_unique<ScriptedArm>()};
  ScriptedArm & port{*transport};
  const SerialArm arm{std::move(transport)};

  // Replies at known times: the age counts from the last one.
  const auto first_reply{[&port]() {
      return !port.angles_handed().empty();
    }};
  ASSERT_TRUE(wait_until(first_reply));
  const std::optional<SteadyClock::time_point> last{lose_replies(port)};
  ASSERT_TRUE(last.has_value());
  const SteadyClock::time_point last_at{last.value_or(SteadyClock::time_point{})};
  const PoseSample first{expect_age_within(arm, last_at)};
  EXPECT_EQ(first.angles, arm.read_angles());
  const JointPositions expected{expected_pose()};
  for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
    EXPECT_NEAR(first.angles.at(joint), expected.at(joint), 1e-9) << joint;
  }

  // Lost replies: the pose stays and so does its time, so the age grows by the time between the
  // two reads, at least LOST_FOR.
  std::this_thread::sleep_for(LOST_FOR);
  const PoseSample lost{expect_age_within(arm, last_at)};
  EXPECT_EQ(lost.angles, first.angles);
  const SteadyClock::duration first_age{first.age.value_or(SteadyClock::duration::zero())};
  const SteadyClock::duration lost_age{lost.age.value_or(SteadyClock::duration::zero())};
  EXPECT_GE(lost_age - first_age, LOST_FOR);

  // The next reply brings it back down: the age counts from that reply, which went out at least
  // LOST_FOR after the last one, so an age still counted from the last one would fail here.
  const std::size_t handed{port.angles_handed().size()};
  port.set_answer_angles(true);
  const auto next_reply{[&port, handed]() {
      return port.angles_handed().size() > handed;
    }};
  ASSERT_TRUE(wait_until(next_reply));
  const std::optional<SteadyClock::time_point> next{lose_replies(port)};
  ASSERT_TRUE(next.has_value());
  const SteadyClock::time_point next_at{next.value_or(SteadyClock::time_point{})};
  EXPECT_GE(next_at - last_at, LOST_FOR);
  static_cast<void>(expect_age_within(arm, next_at));
}

TEST(SerialArmPoseAge, CountsFromTheSyncWriteWhenThePoseComesFromThePoll)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  ReplayTransport & port{*transport};
  port.set_poll_frames(parse_hex(RECORDED_POLL_FRAMES));
  port.set_quiet_after_send(true);
  SerialArm arm{std::move(transport), POLL_TIMING};
  static_cast<void>(wait_for_reply(arm));
  arm.write_angles(JointPositions{0.1, 0.0, 0.0, 0.0, 0.0, 0.0});
  // Read until the poll's pose is the one published: it differs from the get_angles pose. A
  // get_angles may replace it later; the bound below holds for both, since both came after.
  const JointPositions poll_pose{recorded_poll_pose()};
  const auto is_poll_pose{[&poll_pose](const JointPositions & angles) {
      for (std::size_t joint{0}; joint < NUM_JOINTS; ++joint) {
        if (std::abs(angles.at(joint) - poll_pose.at(joint)) > 1e-9) {
          return false;
        }
      }
      return true;
    }};
  std::optional<PoseSample> seen;
  const auto deadline{SteadyClock::now() + std::chrono::seconds{2}};
  while (!seen.has_value() && SteadyClock::now() < deadline) {
    const PoseSample sample{arm.read_pose()};
    if (is_poll_pose(sample.angles)) {
      seen = sample;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
  }
  const auto after{SteadyClock::now()};
  ASSERT_TRUE(seen.has_value());
  ASSERT_FALSE(port.sync_write_times().empty());
  // Stamped when the sync-write closed, not earlier (at the start of the turn, before the send):
  // the age is never more than the time since the sync-write went out.
  const SteadyClock::duration age{
    seen.value_or(PoseSample{}).age.value_or(SteadyClock::duration::max())};
  EXPECT_GE(age, SteadyClock::duration::zero());
  EXPECT_LE(age, after - port.sync_write_times().front());
}

// Follow-up prompt (#183, the load relief on the replayed arm):
//   The poll frames carry J1 pushing (load 30) while the controller keeps writing J1 1 deg past
//   where it stands: after the hold (500 ms here) J1 goes out where it stands, the relief is
//   logged; a halt ends it, and a hold after the power on J1 is relieved again; a new command for
//   J1 goes out as is.

namespace
{
using positronic_hardware::LoadReliefCheck;
using positronic_hardware::LoadReliefLimits;
using positronic_hardware::PositionLimits;
using positronic_hardware::ReliefEvent;

constexpr double ONE_DEG{M_PI / 180.0};

/// The recorded poll, still and at load 0, with J1 reporting `load`.
std::vector<std::uint8_t> poll_with_j1_load(std::uint16_t load)
{
  std::vector<std::uint8_t> frames{
    with_byte(parse_hex(RECORDED_POLL_FRAMES), 0, 4, static_cast<std::uint8_t>(load & 0xFFU))};
  return with_byte(std::move(frames), 0, 5, static_cast<std::uint8_t>(load >> 8U));
}

/// A relief with a 500 ms hold and +-3 rad on every joint. The checks after the power on and
/// after the move must see the command on the wire before the next relief: a window of one hold
/// against writes 10 ms apart. write_until's 2 s is four holds.
LoadReliefCheck fast_relief()
{
  LoadReliefLimits limits{};
  limits.hold = std::chrono::milliseconds{500};
  PositionLimits range{};
  for (positronic_hardware::PositionLimit & limit : range) {
    limit.lower = -3.0;
    limit.upper = 3.0;
  }
  return LoadReliefCheck{limits, range};
}

/// The recorded poll's pose with J1 1 deg further: J1 stands 1 deg short of it.
JointPositions j1_short()
{
  JointPositions command{recorded_poll_pose()};
  command.at(0) += ONE_DEG;
  return command;
}

/// Writes `command` every 10 ms, as the controller does, until `done()` or 2 s. Returns done().
template<typename Done>
bool write_until(SerialArm & arm, const JointPositions & command, Done done)
{
  const auto deadline{std::chrono::steady_clock::now() + std::chrono::seconds{2}};
  while (!done() && std::chrono::steady_clock::now() < deadline) {
    arm.write_angles(command);
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  return done();
}

/// True when the last target on the wire has J1 at `rad`.
bool j1_on_the_wire(const SerialArm & arm, double rad)
{
  const std::optional<JointPositions> target{arm.last_target()};
  return target.has_value() && std::abs(target.value_or(JointPositions{}).at(0) - rad) < 1e-9;
}

/// An arm whose poll shows J1 pushing with `load`, with `relief`.
std::unique_ptr<SerialArm> arm_with(std::uint16_t load, const LoadReliefCheck & relief)
{
  auto transport{std::make_unique<ReplayTransport>(recorded_bursts())};
  transport->set_poll_frames(poll_with_j1_load(load));
  return std::make_unique<SerialArm>(
    std::move(transport), POLL_TIMING, TemperatureGuard{}, false, relief);
}
}  // namespace

TEST(SerialArmLoadRelief, AStillLoadedJointGoesOutWhereItStandsUntilMovedOrHalted)
{
  const std::unique_ptr<SerialArm> arm{arm_with(30, fast_relief())};
  static_cast<void>(wait_for_reply(*arm));
  const JointPositions command{j1_short()};
  const double stands{rad_of(RECORDED_POLL_STEPS.at(0), 0)};
  const auto relieved{[&arm, stands]() {
      return j1_on_the_wire(*arm, stands);
    }};
  ASSERT_TRUE(write_until(*arm, command, relieved));
  EXPECT_EQ(arm->last_load_relief().value_or(ReliefEvent{}).number, 1u);
  // A halt ends the relief: the first send after the power on carries the controller's
  // command, and a whole hold later J1 is relieved again.
  ASSERT_TRUE(arm->halt());
  ASSERT_TRUE(arm->power_on());
  ASSERT_TRUE(write_until(*arm, command, [&arm, &command]() {
      return j1_on_the_wire(*arm, command.at(0));
  }));
  ASSERT_TRUE(write_until(*arm, command, relieved));
  EXPECT_EQ(arm->last_load_relief().value_or(ReliefEvent{}).number, 2u);
  // The controller moves J1: the new command goes out as is.
  JointPositions moved{command};
  moved.at(0) += ONE_DEG;
  EXPECT_TRUE(write_until(*arm, moved, [&arm, &moved]() {
      return j1_on_the_wire(*arm, moved.at(0));
  }));
  // Polls without a relief leave the last one for the plugin to read.
  EXPECT_EQ(arm->last_load_relief().value_or(ReliefEvent{}).number, 2u);
}
