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

#ifndef POSITRONIC_HARDWARE__SERIAL_TRANSPORT_HPP_
#define POSITRONIC_HARDWARE__SERIAL_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace positronic_hardware
{

/// The byte pipe SerialArm talks through. One real implementation (PosixSerialPort) and, in the
/// tests, a fake that replays recorded bursts. Keeping this an interface is what lets the whole
/// serial backend be tested without the arm, the same "one interface, sim and real behind it"
/// idea the architecture uses everywhere else.
class SerialTransport
{
public:
  SerialTransport() = default;
  virtual ~SerialTransport() = default;

  SerialTransport(const SerialTransport &) = delete;
  SerialTransport & operator=(const SerialTransport &) = delete;
  SerialTransport(SerialTransport &&) = delete;
  SerialTransport & operator=(SerialTransport &&) = delete;

  /// Writes exactly `count` bytes. Returns false if the write failed or was short.
  virtual bool write(const std::uint8_t * data, std::size_t count) = 0;

  /// Reads up to `max` bytes into `dst`, returning how many arrived. Returns 0 when nothing came
  /// within the implementation's read timeout: that is normal, not an error. Must not block
  /// forever, so the reader thread can still notice a stop request.
  virtual std::size_t read(std::uint8_t * dst, std::size_t max) = 0;
};

/// A real POSIX serial port (termios). Opens /dev/ttyTHS1 at 1 000 000 baud, 8N1, raw, no flow
/// control. Implementation in serial_arm.cpp; declared here so SerialArm can own one.
class PosixSerialPort : public SerialTransport
{
public:
  PosixSerialPort(const std::string & port, int baud_rate);
  ~PosixSerialPort() override;

  bool write(const std::uint8_t * data, std::size_t count) override;
  std::size_t read(std::uint8_t * dst, std::size_t max) override;

private:
  int fd_{-1};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__SERIAL_TRANSPORT_HPP_
