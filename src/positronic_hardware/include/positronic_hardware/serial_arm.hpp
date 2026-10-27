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

#ifndef POSITRONIC_HARDWARE__SERIAL_ARM_HPP_
#define POSITRONIC_HARDWARE__SERIAL_ARM_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/serial_transport.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace positronic_hardware
{

/// The real arm behind the ArmBackend interface (#22).
///
/// A dedicated thread owns the serial port: it asks the arm for its angles (get_angles), decodes
/// the reply and keeps the newest pose in a lock-free buffer. read_angles(), called from the
/// control loop, only copies that buffer out, so the loop never blocks on the port. This is the
/// "serial I/O in a dedicated thread owning the port, latest sample through a RealtimeBuffer, no
/// blocking call in the control loop" that #22 asks for, with the actual device work of #23.
class SerialArm : public ArmBackend
{
public:
  /// How often the thread polls the arm for angles. The link is 1 Mbaud and a reply is ~17 bytes
  /// plus servo-bus noise; tune against the measured get_angles rate (#20). 100 Hz is a safe
  /// starting point that keeps the state fresh without flooding the bus.
  static constexpr std::chrono::milliseconds POLL_PERIOD{10};

  /// Opens `port` at `baud_rate` with a real POSIX serial port and starts polling.
  /// Throws std::runtime_error if the port cannot be opened (the plugin's on_configure catches it).
  SerialArm(const std::string & port, int baud_rate);

  /// Takes an already-built transport. Used by the tests with a fake that replays recorded bytes.
  explicit SerialArm(std::unique_ptr<SerialTransport> transport);

  ~SerialArm() override;

  SerialArm(const SerialArm &) = delete;
  SerialArm & operator=(const SerialArm &) = delete;
  SerialArm(SerialArm &&) = delete;
  SerialArm & operator=(SerialArm &&) = delete;

  /// The newest pose the thread has decoded, in radians. Non-blocking; safe from the control
  /// loop. Before the first reply arrives it returns START_POSE, never NaN.
  [[nodiscard]] JointPositions read_angles() const override;

private:
  static constexpr JointPositions START_POSE{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  void poll_loop();

  std::unique_ptr<SerialTransport> transport_;
  // readFromRT() is non-const but read_angles() is logically const (it observes state), so the
  // buffer is mutable. writeFromNonRT() runs on the poll thread, readFromRT() on the control loop.
  mutable realtime_tools::RealtimeBuffer<JointPositions> latest_{START_POSE};
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__SERIAL_ARM_HPP_
