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
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "positronic_hardware/reply_assembler.hpp"
#include "positronic_protocol/encode.hpp"
#include "positronic_protocol/types.hpp"

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
    throw std::runtime_error{"Cannot open " + port + ": " + std::strerror(errno)};
  }

  // Exclusive access: while we hold the port, a second open() (another driver, a terminal
  // left open on it) fails with EBUSY instead of stealing half of the replies.
  if (::ioctl(fd_, TIOCEXCL) != 0) {
    const std::string message{std::strerror(errno)};
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error{"Cannot lock " + port + " for exclusive use: " + message};
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    const std::string message{std::strerror(errno)};
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error{"tcgetattr failed on " + port + ": " + message};
  }

  ::cfmakeraw(&tty);  // 8N1, no echo, no line processing: raw bytes in and out.
  ::cfsetispeed(&tty, speed.value());
  ::cfsetospeed(&tty, speed.value());
  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);  // ignore modem lines, enable receiver.
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);        // no hardware flow control.
  tty.c_cc[VMIN] = 0;   // read() returns what is there, even if that is nothing,
  tty.c_cc[VTIME] = 1;  // after at most 0.1 s, so the thread can check stop_.

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const std::string message{std::strerror(errno)};
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error{"tcsetattr failed on " + port + ": " + message};
  }

  // open() used O_NONBLOCK so it would not block waiting for carrier. Clear it now that the
  // port is configured: VMIN/VTIME only take effect in blocking mode. Left on, read() would
  // return immediately and miss the reply (which arrives a few ms after the request), picking
  // it up only on the next cycle. Cleared, read() waits up to VTIME (0.1 s) for the bytes.
  const int flags{::fcntl(fd_, F_GETFL, 0)};
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK) < 0) {
    const std::string message{std::strerror(errno)};
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error{"fcntl failed on " + port + ": " + message};
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
    const ssize_t n{::write(fd_, data + written, count - written)};
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

SerialArm::SerialArm(const std::string & port, int baud_rate)
: SerialArm{std::make_unique<PosixSerialPort>(port, baud_rate)}
{
}

SerialArm::SerialArm(std::unique_ptr<SerialTransport> transport)
: transport_{std::move(transport)}
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

bool SerialArm::has_sample() const
{
  return has_sample_.load();
}

void SerialArm::poll_loop()
{
  ReplyAssembler assembler{};
  std::array<std::uint8_t, 256> chunk{};

  const positronic_protocol::Frame request{positronic_protocol::encode_get_angles()};

  while (!stop_.load()) {
    // Ask for the angles. A failed write is transient (unplugged cable, busy port): skip this
    // round, keep the last known pose, try again next tick. No request went out, so there is
    // no reply to wait for.
    if (!transport_->write(request.bytes.data(), request.size)) {
      std::this_thread::sleep_for(POLL_PERIOD);
      continue;
    }

    // Drain whatever is there and feed the assembler. One get_angles reply usually arrives over
    // a few reads, so we keep reading until the port goes quiet or a reply completes.
    for (int reads{0}; reads < 8 && !stop_.load(); ++reads) {
      const std::size_t got{transport_->read(chunk.data(), chunk.size())};
      if (got == 0) {
        break;
      }
      if (const std::optional<JointPositions> pose{assembler.push(chunk.data(), got)}) {
        latest_.writeFromNonRT(pose.value());
        // Set after the write, so whoever sees true also finds the real pose in the buffer.
        has_sample_.store(true);
        break;
      }
    }

    std::this_thread::sleep_for(POLL_PERIOD);
  }
}

}  // namespace positronic_hardware
