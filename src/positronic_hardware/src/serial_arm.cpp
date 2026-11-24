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

#include "positronic_hardware/serial_arm.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "positronic_hardware/reply_assembler.hpp"
#include "positronic_hardware/requests.hpp"
#include "positronic_hardware/servo_temperature.hpp"
#include "positronic_protocol/decode.hpp"
#include "positronic_protocol/encode.hpp"
#include "positronic_protocol/types.hpp"
#include "positronic_protocol/units.hpp"

namespace positronic_hardware
{
namespace
{
/// Maps a plain integer baud rate to its termios constant. Only the rates we use are listed.
std::optional<speed_t> baud_constant(int baud_rate)
{
  switch (baud_rate) {
    case 115200:
      return B115200;
    case 1000000:
      return B1000000;
    default:
      return std::nullopt;
  }
}
}  // namespace

PosixSerialPort::PosixSerialPort(const std::string & port, int baud_rate)
{
  const std::optional<speed_t> speed{baud_constant(baud_rate)};
  if (!speed.has_value()) {
    throw std::runtime_error{"Unsupported baud rate: " + std::to_string(baud_rate)};
  }

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::system_error{errno, std::system_category(), "Cannot open " + port};
  }

  // Exclusive access: while we hold the port, a second open() (another driver, a terminal
  // left open on it) fails with EBUSY instead of stealing half of the replies.
  if (::ioctl(fd_, TIOCEXCL) != 0) {
    const int error{errno};
    ::close(fd_);
    fd_ = -1;
    throw std::system_error{
            error, std::system_category(), "Cannot lock " + port + " for exclusive use"};
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    const int error{errno};
    ::close(fd_);
    fd_ = -1;
    throw std::system_error{error, std::system_category(), "tcgetattr failed on " + port};
  }

  ::cfmakeraw(&tty);  // 8N1, no echo, no line processing: raw bytes in and out.
  ::cfsetispeed(&tty, speed.value());
  ::cfsetospeed(&tty, speed.value());
  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);  // ignore modem lines, enable receiver.
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);        // no hardware flow control.
  tty.c_cc[VMIN] = 0;   // read() returns at once with what is there, even if that is nothing;
  tty.c_cc[VTIME] = 0;  // no wait: poll_loop's REPLY_TIMEOUT bounds a silent turn (#35).

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const int error{errno};
    ::close(fd_);
    fd_ = -1;
    throw std::system_error{error, std::system_category(), "tcsetattr failed on " + port};
  }

  // open() used O_NONBLOCK so it would not block waiting for carrier. Clear it now that the
  // port is configured, back to a normal blocking descriptor. With VMIN = VTIME = 0 a read
  // returns at once either way; poll_loop keeps reading until the reply completes or
  // REPLY_TIMEOUT passes, so a reply that arrives a few ms after the request is not missed.
  const int flags{::fcntl(fd_, F_GETFL, 0)};
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK) < 0) {
    const int error{errno};
    ::close(fd_);
    fd_ = -1;
    throw std::system_error{error, std::system_category(), "fcntl failed on " + port};
  }
}

PosixSerialPort::~PosixSerialPort()
{
  if (fd_ >= 0) {
    // The exclusive flag belongs to the device, not to this descriptor: it outlives close() if
    // anything else still holds the device open. Clear it so the port can be opened again.
    ::ioctl(fd_, TIOCNXCL);
    ::close(fd_);
  }
}

bool PosixSerialPort::write(const std::uint8_t * data, std::size_t count)
{
  std::size_t written{0};
  while (written < count) {
    const std::uint8_t * const next_byte{std::next(data, static_cast<std::ptrdiff_t>(written))};
    const ssize_t n{::write(fd_, next_byte, count - written)};
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }
      return false;
    }
    written += static_cast<std::size_t>(n);
  }
  return true;
}

std::size_t PosixSerialPort::read(std::uint8_t * dst, std::size_t max)
{
  const ssize_t n{::read(fd_, dst, max)};
  if (n <= 0) {  // 0 = nothing there (VTIME = 0); <0 with EAGAIN = same. Both mean "no bytes now".
    return 0;
  }
  return static_cast<std::size_t>(n);
}

SerialArm::SerialArm(
  const std::string & port, int baud_rate, Timing timing, TemperatureGuard guard, bool has_gripper,
  LoadReliefCheck relief)
: SerialArm{std::make_unique<PosixSerialPort>(port, baud_rate), timing, guard, has_gripper, relief}
{
}

SerialArm::SerialArm(
  std::unique_ptr<SerialTransport> transport, Timing timing, TemperatureGuard guard,
  bool has_gripper, LoadReliefCheck relief)
: transport_{std::move(transport)},
  timing_{timing},
  guard_{guard},
  temperature_watchdog_{guard.stale_timeout},
  relief_{relief},
  detector_{guard.limit_c, guard.hot_for},
  has_gripper_{has_gripper}
{
  // The first temperature request is due at once, so on_activate finds a reading, and the
  // silence of the guard counts from now (like the #35 watchdog, reset by the thread).
  last_temperatures_ = Clock::now() - guard_.max_age;
  temperature_watchdog_.reset(Clock::now());
  // read_until() keeps up to 2048 bytes plus one chunk: reserved, so a turn does not allocate.
  read_buffer_.reserve(2048 + 256);
  thread_ = std::thread{&SerialArm::poll_loop, this};
}

SerialArm::~SerialArm()
{
  {
    // Under the lock: the thread cannot miss it between its look at the flags and its wait.
    const std::lock_guard<std::mutex> lock{wake_mutex_};
    stop_.store(true);
  }
  wake_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

JointPositions SerialArm::read_angles() const
{
  return latest_.readFromRT()->value_or(START_POSE);
}

void SerialArm::write_angles(const JointPositions & angles)
{
  // Only stores the command. The poll thread picks it up on its next turn and sends it over the
  // port, so the control loop never waits for the serial link here.
  command_.writeFromNonRT(angles);
  if (timing_.pose_from_poll) {
    // Wake the thread, so the send follows this write instead of the thread's own period (#170).
    wake_thread(command_ready_);
  }
}

void SerialArm::wait_for_turn_end(Clock::time_point deadline)
{
  if (!timing_.pose_from_poll) {
    std::this_thread::sleep_until(deadline);
    return;
  }
  // The flag was cleared when this turn read its command, so a write that landed during the
  // turn, while the sync-write was being read, is still set and ends the wait at once: the
  // send follows the sync-write instead of waiting for the next write. A new stop or power on
  // ends the wait too: the controller stops writing on a halt, and the stop must not wait for
  // the two-period deadline.
  std::unique_lock<std::mutex> lock{wake_mutex_};
  std::ignore = wake_cv_.wait_until(lock, deadline, [this]() {
        return command_ready_ || request_ready_ || stop_.load();
  });
}

void SerialArm::wake_thread(bool & flag)
{
  {
    // Held for one store; the thread holds it as briefly.
    const std::lock_guard<std::mutex> lock{wake_mutex_};
    flag = true;
  }
  wake_cv_.notify_one();
}

bool SerialArm::has_sample() const
{
  // From the pose's own box, so a true here and the read_angles() after it agree.
  return latest_.readFromRT()->has_value();
}

bool SerialArm::link_timed_out() const
{
  return watchdog_.timed_out(Clock::now());
}

bool SerialArm::halt()
{
  // The hold goes first, so the turn that writes the stop already sends no target after it.
  halted_.store(true);
  return request_and_wait(Pending::HALT);
}

void SerialArm::request_halt()
{
  halted_.store(true);
  request_.store(Pending::HALT);
  wake_thread(request_ready_);
}

bool SerialArm::power_on()
{
  // After an overheat cut the servos stay off for the life of this backend: nothing to ask the
  // thread, which would refuse the frame anyway.
  if (overheated()) {
    return false;
  }
  // Drop the target stored before or during the halt: with it still there, the thread would
  // send it once the hold ends, before the plugin's first write_angles() after activate (the
  // measured pose) could replace it. Seen on the arm as a short move towards the old target.
  // Both writers here are non-RT, so the buffer's mutex is enough. The hold itself is released
  // by the thread, in the turn that puts the power on on the wire: the mirror of halt(), where
  // the hold goes on before the stop. A target written meanwhile waits for that turn.
  command_.writeFromNonRT(std::nullopt);
  gripper_command_.writeFromNonRT(std::nullopt);
  // A request_halt() that lands while the thread waits for the power_on ack puts the hold back
  // (see send_pending_request): the arm is powered but held, and must not be reported live. A
  // cut can land there too: the thread then drops the frame and clears the request, so the wait
  // ends, but the servos were not powered.
  return request_and_wait(Pending::POWER_ON) && !halted_.load() && !overheated();
}

void SerialArm::write_gripper(std::uint8_t value)
{
  gripper_command_.writeFromNonRT(value);
}

std::optional<std::uint8_t> SerialArm::read_gripper() const
{
  return *gripper_latest_.readFromRT();
}

std::optional<ServoTemperatures> SerialArm::read_temperatures() const
{
  return *temperatures_.readFromRT();
}

bool SerialArm::overheated() const
{
  return overheat_joint_.load() >= 0;
}

std::optional<Overheat> SerialArm::overheat() const
{
  const int joint{overheat_joint_.load()};
  if (joint < 0) {
    return std::nullopt;
  }
  return Overheat{static_cast<std::size_t>(joint), overheat_celsius_.load()};
}

bool SerialArm::temperature_stale() const
{
  return temperature_watchdog_.timed_out(Clock::now());
}

std::optional<JointPositions> SerialArm::last_target() const
{
  return *last_target_.readFromRT();
}

std::uint64_t SerialArm::echo_timeouts() const
{
  return echo_timeouts_.load();
}

std::optional<ReliefEvent> SerialArm::last_load_relief() const
{
  return *last_relief_.readFromRT();
}

std::optional<JointPositions> SerialArm::last_sent() const
{
  return *last_target_.readFromNonRT();
}

bool SerialArm::request_and_wait(Pending request)
{
  request_.store(request);
  wake_thread(request_ready_);
  // The poll thread writes the frame at the start of its next turn, waits for the arm's ack and
  // then puts request_ back to NONE. 1 ms steps: on the arm the ack is in within a few ms, and
  // this runs in a lifecycle transition, which may wait.
  const auto deadline{std::chrono::steady_clock::now() + REQUEST_TIMEOUT};
  while (std::chrono::steady_clock::now() < deadline) {
    if (request_.load() == Pending::NONE) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  // One last look: the thread may have cleared it right at the deadline.
  return request_.load() == Pending::NONE;
}

/// Writes the pending halt or power_on, if any, at the start of a turn (the arm's input is free
/// then) and waits for its ack. Only the ack clears the request: a frame the arm did not take, or
/// a failed write, stays pending and goes out again next turn, so nobody hears "stopped" for a
/// stop the arm never confirmed (#103). An acknowledged stop clears the last target, so the next
/// one goes out even if it equals the last. An acknowledged power on ends the hold.
void SerialArm::send_pending_request()
{
  const Pending pending{request_.load()};
  if (pending == Pending::NONE) {
    return;
  }
  if (pending == Pending::POWER_ON && overheated()) {
    // Never power a cut arm back on. The request is cleared so power_on() stops waiting; it
    // sees the cut and returns false. The hold stays.
    Pending expected{pending};
    request_.compare_exchange_strong(expected, Pending::NONE);
    return;
  }
  const Request request{pending == Pending::HALT ? stop_request() : power_on_request()};
  if (!perform(request)) {
    return;  // not taken: the request stays pending, the frame goes out again next turn
  }
  if (pending == Pending::HALT) {
    last_target_.writeFromNonRT(std::nullopt);
    gripper_last_sent_ = std::nullopt;
  } else if (pending == Pending::POWER_ON) {
    // Before the request is cleared: power_on() returns as soon as it sees NONE, and a halt()
    // right after it must find the hold already released, so its own hold cannot be undone by
    // this store arriving late. Same rule as halt(), the state changes before the signal.
    halted_.store(false);
  }
  // Not a plain store: a request made after a timeout must not be erased by the late clear of
  // the one that went out.
  Pending expected{pending};
  if (!request_.compare_exchange_strong(expected, Pending::NONE) && pending == Pending::POWER_ON) {
    // A request_halt() landed during the ack wait: it stored HALT over our POWER_ON (so the CAS
    // failed) and set halted_ before the store above cleared it. Put the hold back: the stop
    // goes out next turn, and power_on() reports false.
    halted_.store(true);
  }
}

bool SerialArm::read_until(Clock::time_point deadline, const Scanner & scan)
{
  // The frame shares the line with the servo-bus echo (~150 bytes per Atom cycle) and may come
  // behind a late pose, so the scan runs over everything read and drops what it rules out. Not
  // the ReplyAssembler: that one only knows poses.
  std::array<std::uint8_t, 256> chunk{};
  std::vector<std::uint8_t> & buffer{read_buffer_};
  buffer.clear();
  while (!stop_.load() && Clock::now() < deadline) {
    const std::size_t got{transport_->read(chunk.data(), chunk.size())};
    if (got == 0) {
      std::this_thread::sleep_for(EMPTY_READ_BACKOFF);
      continue;
    }
    buffer.insert(
      buffer.end(), chunk.begin(), std::next(chunk.begin(), static_cast<std::ptrdiff_t>(got)));
    // A scanner that keeps what it reads (the poll frames before the sync-write, #170) must not
    // grow the buffer without end: an Atom cycle is ~250 bytes, 60 ms of line about 1 KB.
    constexpr std::size_t MAX_KEPT{2048};
    if (buffer.size() > MAX_KEPT) {
      buffer.erase(
        buffer.begin(),
        std::next(buffer.begin(), static_cast<std::ptrdiff_t>(buffer.size() - MAX_KEPT)));
    }
    const ScanResult result{scan(positronic_protocol::ByteView{buffer.data(), buffer.size()})};
    if (result.found) {
      return true;
    }
    buffer.erase(
      buffer.begin(), std::next(buffer.begin(), static_cast<std::ptrdiff_t>(result.consumed)));
  }
  return false;
}

void SerialArm::wait_for_sync_write()
{
  // With the pose from the poll, every byte before the sync-write is kept (consumed 0): the
  // servo status frames in there are the pose. Without it, the bytes the decoder rules out are
  // dropped as they come.
  const bool keep{timing_.pose_from_poll};
  const auto sync_write_seen{[keep](positronic_protocol::ByteView rx) {
      const positronic_protocol::Decoded<std::uint8_t> echo{
        positronic_protocol::decode_sync_write_echo(rx)};
      const bool found{echo.status == positronic_protocol::DecodeStatus::FOUND};
      std::size_t consumed{echo.consumed};
      if (keep) {
        consumed = 0;
      }
      return ScanResult{found, consumed};
    }};
  const bool seen{read_until(Clock::now() + timing_.echo_timeout, sync_write_seen)};
  last_send_answered_ = seen;
  // A wait cut short by the destructor is not the arm's doing.
  if (!seen && !stop_.load()) {
    echo_timeouts_.fetch_add(1);
  }
  // Published now, the moment the sync-write closes, not at the start of the next turn: the
  // pose is ~13 ms fresher for the read() that comes next (#170).
  if (seen && keep && publish_poll_frames()) {
    input_free_ = true;
  }
}

bool SerialArm::frame_plausible(const positronic_protocol::ServoStatus & status)
{
  return status.voltage >= MIN_SERVO_VOLTAGE && status.voltage <= MAX_SERVO_VOLTAGE &&
         plausible_celsius(status.temperature);
}

bool SerialArm::jumped(std::uint16_t position, std::size_t joint) const
{
  if (!last_pose_.has_value()) {
    return false;
  }
  const double angle{
    positronic_protocol::to_radians(positronic_protocol::angle_from_steps(position, joint))};
  return std::abs(angle - last_pose_->at(joint)) > MAX_POLL_JUMP_RAD;
}

bool SerialArm::publish_poll_frames()
{
  namespace pp = positronic_protocol;
  const auto now{Clock::now()};
  std::array<std::optional<pp::ServoStatus>, pp::NUM_JOINTS> found{};
  pp::ByteView rest{read_buffer_.data(), read_buffer_.size()};
  while (rest.size > 0) {
    const pp::Decoded<pp::ServoStatus> status{pp::decode_servo_status(rest)};
    if (status.status != pp::DecodeStatus::FOUND) {
      break;
    }
    // The gripper (id 7) answers the poll too when plugged (19 Nov 2026): not a joint. A frame
    // that passed the checksum but cannot be true is left out like a corrupt one.
    const std::size_t joint{static_cast<std::size_t>(status.value.id) - 1};
    const bool usable{
      joint < pp::NUM_JOINTS && frame_plausible(status.value) &&
      !jumped(status.value.position, joint)};
    // Kept for the bridge even when this poll ends up unpublished: the next poll may need it.
    if (usable) {
      found.at(joint) = status.value;
      last_steps_.at(joint) = status.value.position;
      last_steps_at_.at(joint) = now;
    }
    rest = pp::ByteView{
      std::next(rest.data, static_cast<std::ptrdiff_t>(status.consumed)),
      rest.size - status.consumed};
  }
  // The load relief counts on the frames found in this poll only: a bridged one repeats an
  // older load (#183). Not after a stop that landed during this send: nothing more goes out.
  if (!halted_.load()) {
    if (const std::optional<ReliefEvent> relief{relief_.update(found, now)}) {
      last_relief_.writeFromNonRT(relief);
    }
  }
  pp::JointAngles angles{};
  ServoTemperatures temperatures{};
  bool fresh{true};
  for (std::size_t joint{0}; joint < pp::NUM_JOINTS; ++joint) {
    const bool stale{
      !last_steps_.at(joint).has_value() || now - last_steps_at_.at(joint) > POLL_BRIDGE_AGE};
    if (found.at(joint).has_value()) {
      temperatures.at(joint) = found.at(joint).value_or(pp::ServoStatus{}).temperature;
    } else if (stale) {
      return false;  // nothing recent for this joint: the next turn asks get_angles
    } else {
      fresh = false;  // the servo's last good position stands in, and no temperature
    }
    angles.at(joint) = pp::angle_from_steps(last_steps_.at(joint).value_or(0), joint);
  }
  const JointPositions pose{pp::to_radians(angles)};
  latest_.writeFromNonRT(pose);
  last_pose_ = pose;
  watchdog_.on_reply(now);
  // The frames are a temperature reading too, one per send, and the guard asks nothing while
  // they come. Not with a bridged frame: it repeats an older reading.
  if (fresh) {
    last_temperatures_ = now;
    take_temperatures(temperatures, now);
  }
  return true;
}

bool SerialArm::ask_angles(ReplyAssembler & assembler)
{
  // The reply of this request only: nothing left from an earlier one, a partial reply included,
  // may complete it.
  clear_line();
  assembler.clear();
  // A failed write is transient (busy port): no reply this turn, the pose stays, the next turn
  // asks again.
  const positronic_protocol::Frame request{positronic_protocol::encode_get_angles()};
  if (!transport_->write(request.bytes.data(), request.size)) {
    return false;
  }
  // Read until the reply completes or REPLY_TIMEOUT passes. The reply comes in many small
  // chunks behind servo-bus chatter, so a fixed number of reads is not enough, and the
  // request and its reply must be over before anything else is written: the arm drops a
  // send_angles that arrives while it is still answering a get_angles. The window counts
  // from this write, not from the turn's start, or a turn spent in a request eats its own (#103).
  std::array<std::uint8_t, 256> chunk{};
  const auto reply_deadline{Clock::now() + REPLY_TIMEOUT};
  while (!stop_.load() && Clock::now() < reply_deadline) {
    const std::size_t got{transport_->read(chunk.data(), chunk.size())};
    if (got == 0) {
      // The real port (VTIME = 0) and the test transports both return at once: back off
      // instead of spinning on the Jetson's CPU.
      std::this_thread::sleep_for(EMPTY_READ_BACKOFF);
      continue;
    }
    if (const std::optional<JointPositions> pose{assembler.push(chunk.data(), got)}) {
      latest_.writeFromNonRT(pose);
      last_pose_ = pose;
      watchdog_.on_reply(Clock::now());
      return true;
    }
  }
  late_reply_possible_ = true;
  return false;
}

void SerialArm::clear_line()
{
  auto drain_until{Clock::now()};
  if (late_reply_possible_) {
    drain_until += LATE_REPLY_DRAIN;
  }
  late_reply_possible_ = false;
  std::array<std::uint8_t, 256> chunk{};
  // Bounded: a line that never goes quiet must not hold the turn (64 chunks are ~16 KB, far
  // past what the Atom sends in LATE_REPLY_DRAIN).
  for (int reads{0}; reads < 64 && !stop_.load(); ++reads) {
    if (transport_->read(chunk.data(), chunk.size()) > 0) {
      continue;
    }
    if (Clock::now() >= drain_until) {
      return;
    }
    std::this_thread::sleep_for(EMPTY_READ_BACKOFF);
  }
}

std::chrono::milliseconds SerialArm::turn_period(bool sent) const
{
  if (!sent) {
    return POLL_PERIOD;
  }
  if (timing_.pose_from_poll && input_free_) {
    // The pose is published and the controller's next write ends the wait; the deadline only
    // catches a controller that stopped writing. At one period it raced the controller's own
    // period (33 vs 33.3 ms) and won a third of the time, each time costing a get_angles turn.
    return 2 * timing_.moving_period;
  }
  return timing_.moving_period;
}

bool SerialArm::perform(const Request & request, const Scanner & reply)
{
  clear_line();
  if (!transport_->write(request.frame.bytes.data(), request.frame.size)) {
    return false;
  }
  const Clock::time_point deadline{Clock::now() + request.timeout};
  bool found{false};
  bool ok{true};
  switch (request.expect) {
    case Expect::NONE:
      return true;
    case Expect::REPLY:
      // A reply nobody scans for is a request that cannot succeed, not a crash on the thread.
      found = static_cast<bool>(reply) && read_until(deadline, reply);
      break;
    case Expect::ACK:
      found = read_until(deadline, [&request, &ok](positronic_protocol::ByteView rx) {
            const positronic_protocol::Decoded<std::uint8_t> ack{
              positronic_protocol::decode_ack(rx, request.command)};
            if (ack.status == positronic_protocol::DecodeStatus::FOUND) {
              ok = ack.value == 1;
            }
            return ScanResult{ack.status == positronic_protocol::DecodeStatus::FOUND, ack.consumed};
      });
      break;
  }
  if (!found) {
    late_reply_possible_ = true;
  }
  return found && ok;
}

std::optional<positronic_protocol::Frame> SerialArm::encode_command(const JointPositions & target)
{
  const std::optional<positronic_protocol::JointAngles> angles{
    positronic_protocol::to_centidegrees(target)};
  if (!angles.has_value()) {
    return std::nullopt;
  }
  return positronic_protocol::encode_send_angles(angles.value(), SEND_SPEED);
}

std::optional<ServoTemperatures> SerialArm::ask_temperatures()
{
  std::optional<ServoTemperatures> temperatures{};
  const bool replied{
    perform(get_servo_temps_request(), [&temperatures](positronic_protocol::ByteView rx) {
        const positronic_protocol::Decoded<positronic_protocol::ServoTemperatures> decoded{
          positronic_protocol::decode_servo_temps_reply(rx)};
        if (decoded.status == positronic_protocol::DecodeStatus::FOUND) {
          temperatures = decoded.value;
        }
        return ScanResult{
        decoded.status == positronic_protocol::DecodeStatus::FOUND, decoded.consumed};
    })};
  return replied ? temperatures : std::nullopt;
}

bool SerialArm::cut_power(const Overheat & trip)
{
  // The hold first, like halt(): from here on no target goes out, whatever the controller does.
  halted_.store(true);
  last_target_.writeFromNonRT(std::nullopt);
  // The power off only after the stop's ack (36 to 37 ms on 24 Nov 2026): the Atom drops a frame
  // that comes while it handles the stop, and a power off sent after a fixed 30 ms was dropped
  // in all three recorded cuts, the servos torqued until its repeat 1.1 s later. Without the
  // ack it goes out at ACK_TIMEOUT plus LATE_REPLY_DRAIN.
  std::ignore = perform(stop_request());
  // Recorded once the stop is out, the value before the joint, and before the power off and its
  // ack (up to 85 ms): the plugin that sees the cut finds the arm already told to stop, and
  // latches the overheat before the silence after the power off can latch the link timeout.
  overheat_celsius_.store(trip.celsius);
  overheat_joint_.store(static_cast<int>(trip.joint));
  return perform(power_off_request());
}

/// The temperatures, once a period, when the arm's input is free (it has just answered) and no
/// stop or power on is waiting. On a turn that sends no target; a reading older than max_age is
/// taken in a moving turn too, so a long motion cannot hide a hot servo. Never between a send
/// and its sync-write: that is the end of the turn. Returns true when it asked: the turn then
/// sends nothing and the target goes out next turn, ~10 ms later. A request (8 ms, 50 ms when
/// unanswered) plus a send and its sync-write would run past the period, and would eat the
/// link watchdog's margin for two lost replies, which only the get_angles reply resets.
bool SerialArm::watch_temperatures(bool new_target)
{
  const auto now{Clock::now()};
  const bool due{now - last_temperatures_ >= guard_.period};
  const bool overdue{now - last_temperatures_ >= guard_.max_age};
  if (!due || (new_target && !overdue) || request_.load() != Pending::NONE) {
    return false;
  }
  last_temperatures_ = now;
  const std::optional<ServoTemperatures> temperatures{ask_temperatures()};
  if (!temperatures.has_value()) {
    return true;
  }
  // The reply frees the arm's input: when this turn had a target to send, the next turn sends
  // it at once (#170). Idle or halted there is nothing to send, and the get_angles of the next
  // turn is the pose; the #150 turn keeps it in every case.
  if (timing_.pose_from_poll && new_target) {
    input_free_ = true;
  }
  // The time of the ask, not of the reply: two asks a period apart are then exactly a period
  // apart for the detector, whatever the replies took.
  take_temperatures(temperatures.value(), now);
  return true;
}

void SerialArm::take_temperatures(const ServoTemperatures & temperatures, Clock::time_point at)
{
  if (plausible(temperatures)) {
    temperatures_.writeFromNonRT(temperatures);
    temperature_watchdog_.on_reply(Clock::now());
  }
  if (overheated()) {
    return;
  }
  if (const std::optional<Overheat> trip{detector_.update(temperatures, at)}) {
    // The cut is the thread's own: it does not wait for the control loop, and it works with
    // the component inactive too, where a halted arm still holds its pose.
    const bool off_written{cut_power(trip.value())};
    next_power_off_ = off_written ? Clock::now() + guard_.period : Clock::now();
  }
}

bool SerialArm::serve_after_reply(bool new_target)
{
  const bool asked_temperatures{watch_temperatures(new_target)};
  if (has_gripper_ && !new_target && !asked_temperatures && request_.load() == Pending::NONE) {
    gripper_turn();
  }
  return asked_temperatures;
}

std::optional<std::uint8_t> SerialArm::gripper_value_to_send()
{
  const std::optional<std::uint8_t> target{*gripper_command_.readFromRT()};
  if (halted_.load() || !target.has_value() || target == gripper_last_sent_) {
    return std::nullopt;
  }
  return target;
}

bool SerialArm::gripper_due()
{
  if (!has_gripper_ || request_.load() != Pending::NONE) {
    return false;
  }
  return gripper_value_to_send().has_value() ||
         Clock::now() - last_gripper_read_ >= GRIPPER_READ_PERIOD;
}

void SerialArm::gripper_turn()
{
  if (const std::optional<std::uint8_t> target{gripper_value_to_send()}) {
    const std::optional<positronic_protocol::Frame> frame{
      positronic_protocol::encode_set_gripper_value(target.value(), GRIPPER_SPEED)};
    clear_line();
    if (frame.has_value() && transport_->write(frame->bytes.data(), frame->size)) {
      gripper_last_sent_ = target;
      std::this_thread::sleep_for(timing_.send_settle);
    }
    return;
  }
  const auto now{Clock::now()};
  if (now - last_gripper_read_ < GRIPPER_READ_PERIOD) {
    return;
  }
  last_gripper_read_ = now;
  if (const std::optional<std::uint8_t> value{ask_gripper()}) {
    gripper_latest_.writeFromNonRT(value);
  }
}

std::optional<std::uint8_t> SerialArm::ask_gripper()
{
  clear_line();
  const positronic_protocol::Frame request{positronic_protocol::encode_get_gripper_value()};
  if (!transport_->write(request.bytes.data(), request.size)) {
    return std::nullopt;
  }
  std::optional<std::uint8_t> value{};
  const bool found{
    read_until(Clock::now() + GRIPPER_REPLY_TIMEOUT, [&value](positronic_protocol::ByteView rx) {
        const positronic_protocol::Decoded<std::uint8_t> reply{
          positronic_protocol::decode_ack(rx, positronic_protocol::CommandId::GET_GRIPPER_VALUE)};
        const bool got{reply.status == positronic_protocol::DecodeStatus::FOUND};
        if (got) {
          value = reply.value;
        }
        return ScanResult{got, reply.consumed};
    })};
  if (!found) {
    late_reply_possible_ = true;
  }
  return value;
}

/// After a cut, the power off goes out again every period, at the start of a turn, when the
/// arm's input is free; the first repeat comes at the next turn when the cut's own power off got
/// no ack. The arm can be powered on again without this port (the Atom's button, a power cycle),
/// and once off it answers nothing, so each repeat waits its ACK_TIMEOUT. Nothing else can use
/// the port meanwhile: it is exclusive.
void SerialArm::repeat_power_off(Clock::time_point turn_start)
{
  if (!overheated() || turn_start < next_power_off_) {
    return;
  }
  std::ignore = perform(power_off_request());
  next_power_off_ = turn_start + guard_.period;
}

std::optional<JointPositions> SerialArm::command_to_send()
{
  // A stop or a cut took every target off the wire: no relief outlives them.
  if (halted_.load()) {
    relief_.reset();
  }
  std::optional<JointPositions> target{*command_.readFromRT()};
  if (target.has_value()) {
    target = relief_.wire(*target);
  }
  return target;
}

void SerialArm::poll_loop()
{
  ReplyAssembler assembler{};
  // Count the silence from now, so an arm that never answers also times out (#35).
  watchdog_.reset(Clock::now());

  while (!stop_.load()) {
    // The deadline is set before the work, so a slow reply does not drift the period.
    const auto turn_start{std::chrono::steady_clock::now()};

    // A request from here on is one this turn has not seen: it ends the wait at the turn's end.
    {
      const std::lock_guard<std::mutex> lock{wake_mutex_};
      request_ready_ = false;
    }
    send_pending_request();

    repeat_power_off(turn_start);

    bool sent{false};

    // The pose of this turn: the Atom's poll after the last send, already published by
    // wait_for_sync_write() (#170), or a get_angles.
    bool replied{input_free_};
    input_free_ = false;
    if (!replied) {
      replied = ask_angles(assembler);
    }

    // Whether the controller wrote since the last turn read the command, whenever it landed:
    // in the wait at the end of that turn or during this turn's get_angles. Each write counts
    // once, here, where the turn decides; one from here on is left set for the wait at the end
    // of this turn (#170).
    bool wrote{false};
    {
      const std::lock_guard<std::mutex> lock{wake_mutex_};
      wrote = command_ready_;
      command_ready_ = false;
    }
    // nullopt means no command yet, or a read that lost the race with write_angles() before
    // the first one: both send nothing, the next turn reads again.
    const std::optional<JointPositions> target{command_to_send()};
    // Only a changed command goes out: a send_angles keeps the arm busy until its sync-write,
    // and while it stands still the port is better spent on fresh poses. last_sent() is the
    // thread's view of last_target_, the one memory of what went out. A turn with a write since
    // the last sends even the same command (#170): the stream follows the controller's cadence
    // and the pose keeps coming from the poll. The repeat needs the last send to have got its
    // sync-write, or REPEAT_RETRY to have passed since it, so a target the Atom refuses is not
    // sent on every write and one lost sync-write does not end the repeat; it gives way to a
    // gripper that is due, which is served only in a turn that sends nothing (#174).
    const bool changed{target.has_value() && target != last_sent()};
    const bool may_repeat{last_send_answered_ || Clock::now() - last_send_at_ >= REPEAT_RETRY};
    const bool repeat{wrote && may_repeat && !gripper_due()};
    const bool new_target{replied && !halted_.load() && target.has_value() && (changed || repeat)};

    // The guard only runs after a reply: the arm's input is free then.
    const bool asked_temperatures{replied && serve_after_reply(new_target)};

    // The arm answered, so its input is free: send the newest command now, if it is new.
    // A turn without a reply sends nothing, the command waits in command_ for the next. So
    // does a turn that asked the temperatures: the request took the time of the send.
    // halted_ again: a cut in this turn holds the target that was about to go.
    if (new_target && target.has_value() && !asked_temperatures && !halted_.load()) {
      // NaN or out of range: nothing we can put on the wire, keep the previous target.
      const std::optional<positronic_protocol::Frame> frame{encode_command(*target)};
      clear_line();
      if (frame.has_value() && transport_->write(frame->bytes.data(), frame->size)) {
        last_target_.writeFromNonRT(target);
        last_send_at_ = Clock::now();
        sent = true;
        // The arm passes the command to the servos and ignores the port meanwhile: a
        // get_angles inside this window stalls, so the next turn must not start before it.
        wait_for_sync_write();
      }
    }

    wait_for_turn_end(turn_start + turn_period(sent));
  }
}

}  // namespace positronic_hardware
