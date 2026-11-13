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
#include <cstddef>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "positronic_hardware/reply_assembler.hpp"
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

SerialArm::SerialArm(const std::string & port, int baud_rate, Timing timing, TemperatureGuard guard)
: SerialArm{std::make_unique<PosixSerialPort>(port, baud_rate), timing, guard}
{
}

SerialArm::SerialArm(
  std::unique_ptr<SerialTransport> transport, Timing timing, TemperatureGuard guard)
: transport_{std::move(transport)},
  timing_{timing},
  guard_{guard},
  temperature_watchdog_{guard.stale_timeout},
  detector_{guard.limit_c}
{
  // The first temperature request is due at once, so on_activate finds a reading, and the
  // silence of the guard counts from now (like the #35 watchdog, reset by the thread).
  last_temperature_ask_ = Clock::now() - guard_.max_age;
  temperature_watchdog_.reset(Clock::now());
  thread_ = std::thread{&SerialArm::poll_loop, this};
}

SerialArm::~SerialArm()
{
  stop_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

JointPositions SerialArm::read_angles() const
{
  // readFromRT() returns a pointer to the current buffer; never null here because the buffer is
  // seeded with START_POSE at construction.
  return *latest_.readFromRT();
}

void SerialArm::write_angles(const JointPositions & angles)
{
  // Only stores the command. The poll thread picks it up on its next turn and sends it over the
  // port, so the control loop never waits for the serial link here.
  command_.writeFromNonRT(angles);
}

bool SerialArm::has_sample() const
{
  return has_sample_.load();
}

bool SerialArm::link_timed_out() const
{
  return watchdog_.timed_out(Clock::now());
}

bool SerialArm::halt()
{
  // The hold goes first, so the turn that writes the stop already sends no target after it.
  halted_.store(true);
  return request_and_wait(Request::HALT);
}

void SerialArm::request_halt()
{
  halted_.store(true);
  request_.store(Request::HALT);
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
  // A cut can land while the power on waits: the thread then drops the frame and clears the
  // request, so the wait ends, but the servos were not powered.
  return request_and_wait(Request::POWER_ON) && !overheated();
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

bool SerialArm::request_and_wait(Request request)
{
  request_.store(request);
  // The poll thread writes the frame at the start of its next turn and puts request_ back to
  // NONE. 1 ms steps: on the arm the frame is out within a few ms, and this runs in a lifecycle
  // transition, which may wait.
  const auto deadline{std::chrono::steady_clock::now() + REQUEST_TIMEOUT};
  while (std::chrono::steady_clock::now() < deadline) {
    if (request_.load() == Request::NONE) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  // One last look: the thread may have cleared it right at the deadline.
  return request_.load() == Request::NONE;
}

std::optional<positronic_protocol::Frame> SerialArm::encode_request(Request request)
{
  switch (request) {
    case Request::HALT:
      return positronic_protocol::encode_stop();
    case Request::POWER_ON:
      return positronic_protocol::encode_power_on();
    case Request::NONE:
      return std::nullopt;
  }
  return std::nullopt;
}

/// Writes the pending halt or power_on, if any, at the start of a turn: the previous turn is
/// over, so the arm's input is free, and request_and_wait() is waiting for this. No settle after
/// it: the caller returns as soon as the frame is written, and if the arm ignores the port for a
/// while the only cost is this turn's reply. A failed write leaves the request pending, and the
/// hold as it is, for the next turn. A stop that went out clears `last_sent`: the arm is heading
/// nowhere, so the next target goes out even when it equals the last one sent. A power on that
/// went out ends the hold, with the frame already on the wire, so no target can get ahead of it.
void SerialArm::send_pending_request(std::optional<JointPositions> & last_sent)
{
  const Request pending{request_.load()};
  if (pending == Request::POWER_ON && overheated()) {
    // Never power a cut arm back on. The request is cleared so power_on() stops waiting; it
    // sees the cut and returns false. The hold stays.
    Request expected{pending};
    request_.compare_exchange_strong(expected, Request::NONE);
    return;
  }
  const std::optional<positronic_protocol::Frame> frame{encode_request(pending)};
  if (!frame.has_value() || !transport_->write(frame->bytes.data(), frame->size)) {
    return;
  }
  if (pending == Request::HALT) {
    last_sent = std::nullopt;
  } else if (pending == Request::POWER_ON) {
    // Before the request is cleared: power_on() returns as soon as it sees NONE, and a halt()
    // right after it must find the hold already released, so its own hold cannot be undone by
    // this store arriving late. Same rule as halt(), the state changes before the signal.
    halted_.store(false);
  }
  // Not a plain store: a request made after a timeout must not be erased by the late clear of
  // the one that went out.
  Request expected{pending};
  request_.compare_exchange_strong(expected, Request::NONE);
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
  const positronic_protocol::Frame request{positronic_protocol::encode_get_servo_temps()};
  if (!transport_->write(request.bytes.data(), request.size)) {
    return std::nullopt;
  }
  temperature_buffer_.clear();
  std::array<std::uint8_t, 256> chunk{};
  const auto deadline{std::chrono::steady_clock::now() + TEMPERATURE_REPLY_TIMEOUT};
  while (!stop_.load() && std::chrono::steady_clock::now() < deadline) {
    const std::size_t got{transport_->read(chunk.data(), chunk.size())};
    if (got == 0) {
      // Same as the get_angles reply in poll_loop: the port returns at once, back off.
      std::this_thread::sleep_for(EMPTY_READ_BACKOFF);
      continue;
    }
    temperature_buffer_.insert(
      temperature_buffer_.end(), chunk.begin(),
      std::next(chunk.begin(), static_cast<std::ptrdiff_t>(got)));
    const positronic_protocol::Decoded<positronic_protocol::ServoTemperatures> decoded{
      positronic_protocol::decode_servo_temps_reply(
        positronic_protocol::ByteView{temperature_buffer_.data(), temperature_buffer_.size()})};
    if (decoded.status == positronic_protocol::DecodeStatus::FOUND) {
      return decoded.value;
    }
    // Keep only what may still become the reply, so servo-bus noise cannot grow the buffer.
    temperature_buffer_.erase(
      temperature_buffer_.begin(),
      std::next(temperature_buffer_.begin(), static_cast<std::ptrdiff_t>(decoded.consumed)));
  }
  return std::nullopt;
}

bool SerialArm::cut_power(const Overheat & trip, std::optional<JointPositions> & last_sent)
{
  // The hold first, like halt(): from here on no target goes out, whatever the controller does.
  halted_.store(true);
  last_sent = std::nullopt;
  const positronic_protocol::Frame stop{positronic_protocol::encode_stop()};
  static_cast<void>(transport_->write(stop.bytes.data(), stop.size));
  // The arm ignores the port for a moment after a frame (#32): the power off waits the settle,
  // so it is not dropped.
  std::this_thread::sleep_for(timing_.send_settle);
  const positronic_protocol::Frame off{positronic_protocol::encode_power_off()};
  const bool off_written{transport_->write(off.bytes.data(), off.size)};
  // Recorded after the frames, the value before the joint: the plugin that sees the cut finds
  // the arm already told to stop and power off.
  overheat_celsius_.store(trip.celsius);
  overheat_joint_.store(static_cast<int>(trip.joint));
  return off_written;
}

/// The temperatures, once a period, when the arm's input is free (it has just answered) and no
/// stop or power on is waiting. On a turn that sends no target; a reading older than max_age is
/// taken before the send of a moving turn, so a long motion cannot hide a hot servo. Never
/// between a send and its settle: that is the end of the turn.
void SerialArm::watch_temperatures(bool new_target, std::optional<JointPositions> & last_sent)
{
  const auto now{Clock::now()};
  const bool due{now - last_temperature_ask_ >= guard_.period};
  const bool overdue{now - last_temperature_ask_ >= guard_.max_age};
  if (!due || (new_target && !overdue) || request_.load() != Request::NONE) {
    return;
  }
  last_temperature_ask_ = now;
  const std::optional<ServoTemperatures> temperatures{ask_temperatures()};
  if (!temperatures.has_value()) {
    return;
  }
  if (plausible(temperatures.value())) {
    temperatures_.writeFromNonRT(temperatures);
    temperature_watchdog_.on_reply(Clock::now());
  }
  if (overheated()) {
    return;
  }
  if (const std::optional<Overheat> trip{detector_.update(temperatures.value())}) {
    // The cut is the thread's own: it does not wait for the control loop, and it works with
    // the component inactive too, where a halted arm still holds its pose.
    const bool off_written{cut_power(trip.value(), last_sent)};
    next_power_off_ = off_written ? Clock::now() + guard_.period : Clock::now();
  }
}

/// After a cut, the power off goes out again every period, at the start of a turn, when the
/// arm's input is free. No frame is acknowledged, so a dropped power off would leave the servos
/// torqued and still heating; and the arm can be powered on again without this port (the
/// Atom's button, a power cycle). Nothing else can use the port meanwhile: it is exclusive.
void SerialArm::repeat_power_off(Clock::time_point turn_start)
{
  if (!overheated() || turn_start < next_power_off_) {
    return;
  }
  const positronic_protocol::Frame off{positronic_protocol::encode_power_off()};
  static_cast<void>(transport_->write(off.bytes.data(), off.size));
  next_power_off_ = turn_start + guard_.period;
}

void SerialArm::poll_loop()
{
  ReplyAssembler assembler{};
  std::array<std::uint8_t, 256> chunk{};

  const positronic_protocol::Frame request{positronic_protocol::encode_get_angles()};
  // The last target that went out, so an unchanged command is not sent again: a send_angles
  // keeps the arm busy for the settle, and while it stands still the port is better spent on
  // fresh poses.
  std::optional<JointPositions> last_sent{};
  // Count the silence from now, so an arm that never answers also times out (#35).
  watchdog_.reset(Clock::now());

  while (!stop_.load()) {
    // The deadline is set before the work, so a slow reply does not drift the period.
    const auto turn_start{std::chrono::steady_clock::now()};

    send_pending_request(last_sent);

    repeat_power_off(turn_start);

    bool sent{false};

    // Ask for the angles. A failed write is transient (busy port): skip this round, keep the
    // last known pose, try again next turn.
    bool replied{false};
    if (transport_->write(request.bytes.data(), request.size)) {
      // Read until the reply completes or REPLY_TIMEOUT passes. The reply comes in many small
      // chunks behind servo-bus chatter, so a fixed number of reads is not enough, and the
      // request and its reply must be over before anything else is written: the arm drops a
      // send_angles that arrives while it is still answering a get_angles.
      const auto reply_deadline{turn_start + REPLY_TIMEOUT};
      while (!replied && !stop_.load() && std::chrono::steady_clock::now() < reply_deadline) {
        const std::size_t got{transport_->read(chunk.data(), chunk.size())};
        if (got == 0) {
          // The real port (VTIME = 0) and the test transports both return at once: back off
          // instead of spinning on the Jetson's CPU.
          std::this_thread::sleep_for(EMPTY_READ_BACKOFF);
          continue;
        }
        if (const std::optional<JointPositions> pose{assembler.push(chunk.data(), got)}) {
          latest_.writeFromNonRT(pose.value());
          // Set after the write, so whoever sees true also finds the real pose in the buffer.
          has_sample_.store(true);
          watchdog_.on_reply(Clock::now());
          replied = true;
        }
      }
    }

    // nullopt means no command yet, or a read that lost the race with write_angles() before
    // the first one: both send nothing, the next turn reads again.
    const std::optional<JointPositions> target{*command_.readFromRT()};
    const bool new_target{replied && !halted_.load() && target.has_value() && target != last_sent};

    if (replied) {
      watch_temperatures(new_target, last_sent);
    }

    // The arm answered, so its input is free: send the newest command now, if it is new.
    // A turn without a reply sends nothing, the command waits in command_ for the next.
    // halted_ again: a cut in this turn holds the target that was about to go.
    if (new_target && !halted_.load()) {
      // NaN or out of range: nothing we can put on the wire, keep the previous target.
      const std::optional<positronic_protocol::Frame> frame{encode_command(target.value())};
      if (frame.has_value() && transport_->write(frame->bytes.data(), frame->size)) {
        last_sent = target;
        sent = true;
        // The arm passes the command to the servos and ignores the port meanwhile: a
        // get_angles inside this window stalls, so the next turn must not start before it.
        std::this_thread::sleep_for(timing_.send_settle);
      }
    }

    std::chrono::milliseconds period{POLL_PERIOD};
    if (sent) {
      period = timing_.moving_period;
    }
    std::this_thread::sleep_until(turn_start + period);
  }
}

}  // namespace positronic_hardware
