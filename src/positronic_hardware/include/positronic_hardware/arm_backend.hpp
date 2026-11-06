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

#ifndef POSITRONIC_HARDWARE__ARM_BACKEND_HPP_
#define POSITRONIC_HARDWARE__ARM_BACKEND_HPP_

#include <array>
#include <cstddef>

namespace positronic_hardware
{

/// Number of joints of the myCobot 280.
inline constexpr std::size_t NUM_JOINTS{6};

/// Six joint angles in radians, J1 (base) to J6 (flange).
using JointPositions = std::array<double, NUM_JOINTS>;

/// Where the driver reads the joint angles from and sends the targets to.
///
/// The plugin never knows whether it talks to the real serial port or to a stand-in: read()
/// only calls read_angles() and has_sample(), write() only calls write_angles(). This is the
/// seam that lets the fake arm and the real arm be swapped by a parameter, and the control
/// loop stay the same.
///
/// Contract for read_angles():
/// - It is called from the ros2_control read() loop, so it must NOT block and must NOT touch
///   the serial port directly. An implementation that talks to hardware keeps the latest
///   sample in a lock-free buffer filled by its own thread and just copies it out here.
/// - It always returns six finite angles. Before the first real sample is available an
///   implementation returns its start pose, never NaN. That pose is a placeholder, not a
///   measurement: callers check has_sample() before trusting it.
class ArmBackend
{
public:
  ArmBackend() = default;
  virtual ~ArmBackend() = default;

  ArmBackend(const ArmBackend &) = delete;
  ArmBackend & operator=(const ArmBackend &) = delete;
  ArmBackend(ArmBackend &&) = delete;
  ArmBackend & operator=(ArmBackend &&) = delete;

  /// The latest known joint angles, in radians. Non-blocking, safe from the control loop.
  [[nodiscard]] virtual JointPositions read_angles() const = 0;

  /// The target joint angles, in radians. Safe from the control loop: the implementation only
  /// stores the newest command and never waits for the serial port here. A newer call
  /// replaces the previous command before it was sent. The plugin calls this only while
  /// active, so an implementation does not need to check that itself.
  virtual void write_angles(const JointPositions & angles) = 0;

  /// Stops the arm where it is and holds every target until power_on(). True when the stop
  /// went out on the port within the deadline, false when the deadline passed with the stop
  /// still pending. Called from a lifecycle transition, never from the control loop: it may
  /// wait up to 300 ms.
  [[nodiscard]] virtual bool halt() = 0;

  /// The same stop and hold as halt(), without the wait: it only leaves the request, the
  /// implementation sends the stop on its own thread. For on_error, which may run on the
  /// control loop (a write() that returned ERROR) and must not wait there. Nothing reports
  /// whether the stop went out: the plugin drops the backend on the next configure anyway.
  virtual void request_halt() = 0;

  /// Powers the servos on, so the arm holds its pose, and lets targets go out again after
  /// halt(). A target stored before or during the halt is dropped: the next write_angles() is
  /// the first to reach the arm. True when the power on went out on the port within the
  /// deadline, false when the deadline passed with it still pending. Called from a lifecycle
  /// transition, never from the control loop: it may wait up to 300 ms.
  [[nodiscard]] virtual bool power_on() = 0;

  /// True once read_angles() returns a real measurement. Non-blocking, safe from the control
  /// loop. Until then the plugin reports "unknown" (NaN) instead of the placeholder pose.
  [[nodiscard]] virtual bool has_sample() const = 0;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__ARM_BACKEND_HPP_
