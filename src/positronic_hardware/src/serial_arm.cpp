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
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "positronic_hardware/reply_assembler.hpp"
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
  tty.c_cc[VMIN] = 0;   // read() returns what is there, even if that is nothing,
  tty.c_cc[VTIME] = 1;  // after at most 0.1 s, so the thread can check stop_.

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const int error{errno};
    ::close(fd_);
    fd_ = -1;
    throw std::system_error{error, std::system_category(), "tcsetattr failed on " + port};
  }

  // open() used O_NONBLOCK so it would not block waiting for carrier. Clear it now that the
  // port is configured: VMIN/VTIME only take effect in blocking mode. Left on, read() would
  // return immediately and miss the reply (which arrives a few ms after the request), picking
  // it up only on the next cycle. Cleared, read() waits up to VTIME (0.1 s) for the bytes.
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
  if (n <= 0) {  // 0 = nothing within VTIME; <0 with EAGAIN = same. Both mean "no bytes now".
    return 0;
  }
  return static_cast<std::size_t>(n);
}

SerialArm::SerialArm(const std::string & port, int baud_rate, Timing timing)
: SerialArm{std::make_unique<PosixSerialPort>(port, baud_rate), timing}
{
}

SerialArm::SerialArm(std::unique_ptr<SerialTransport> transport, Timing timing)
: transport_{std::move(transport)},
  timing_{timing}
{
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

void SerialArm::halt()
{
  halt_requested_.store(true);
}

bool SerialArm::has_sample() const
{
  return has_sample_.load();
}

bool SerialArm::send_after_reply(std::optional<JointPositions> & last_sent, bool & stop_sent)
{
  if (halt_requested_.load()) {
    if (stop_sent) {
      return false;
    }
    const positronic_protocol::Frame frame{positronic_protocol::encode_stop()};
    if (!transport_->write(frame.bytes.data(), frame.size)) {
      return false;
    }
    stop_sent = true;
    std::this_thread::sleep_for(timing_.send_settle);
    return true;
  }

  // nullopt means no command yet, or a read that lost the race with write_angles() before the
  // first one: both send nothing, the next turn reads again. An unchanged target is not sent
  // again: the arm already has it.
  const std::optional<JointPositions> target{*command_.readFromRT()};
  if (!target.has_value() || target == last_sent) {
    return false;
  }
  // NaN or out of range: nothing we can put on the wire, keep the previous target.
  const std::optional<positronic_protocol::Frame> frame{encode_command(target.value())};
  if (!frame.has_value() || !transport_->write(frame->bytes.data(), frame->size)) {
    return false;
  }
  last_sent = target;
  // The arm passes the command to the servos and ignores the port meanwhile: a get_angles
  // inside this window stalls, so the next turn must not start before it.
  std::this_thread::sleep_for(timing_.send_settle);
  return true;
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

void SerialArm::poll_loop()
{
  ReplyAssembler assembler{};
  std::array<std::uint8_t, 256> chunk{};

  const positronic_protocol::Frame request{positronic_protocol::encode_get_angles()};
  // The last target that went out, so an unchanged command is not sent again: a send_angles
  // keeps the arm busy for the settle, and while it stands still the port is better spent on
  // fresh poses.
  std::optional<JointPositions> last_sent{};
  bool stop_sent{false};

  while (!stop_.load()) {
    // The deadline is set before the work, so a slow reply does not drift the period.
    const auto turn_start{std::chrono::steady_clock::now()};
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
          // A real port blocks up to VTIME before returning 0; a test transport returns at
          // once, so yield instead of spinning.
          std::this_thread::sleep_for(std::chrono::milliseconds{1});
          continue;
        }
        if (const std::optional<JointPositions> pose{assembler.push(chunk.data(), got)}) {
          latest_.writeFromNonRT(pose.value());
          // Set after the write, so whoever sees true also finds the real pose in the buffer.
          has_sample_.store(true);
          replied = true;
        }
      }
    }

    if (replied) {
      sent = send_after_reply(last_sent, stop_sent);
    }

    std::chrono::milliseconds period{POLL_PERIOD};
    if (sent) {
      period = timing_.moving_period;
    }
    std::this_thread::sleep_until(turn_start + period);
  }
}

}  // namespace positronic_hardware
