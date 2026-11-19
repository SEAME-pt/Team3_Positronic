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
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace positronic_hardware
{

/// Number of joints of the myCobot 280.
inline constexpr std::size_t NUM_JOINTS{6};

/// Six joint angles in radians, J1 (base) to J6 (flange).
using JointPositions = std::array<double, NUM_JOINTS>;

/// The six servo temperatures in degC, J1 first, as the arm reports them (one byte each).
using ServoTemperatures = std::array<std::uint8_t, NUM_JOINTS>;

/// The servo that tripped the over-temperature cut, and the reading that did it.
struct Overheat
{
  /// 0 for J1 ... 5 for J6.
  std::size_t joint{0};
  /// degC.
  int celsius{0};
};

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

  /// How long halt() and power_on() may wait for the arm to acknowledge their request. Part of
  /// the contract, so the plugin can log the deadline without knowing which backend it holds.
  static constexpr std::chrono::milliseconds REQUEST_TIMEOUT{300};

  /// Stops the arm where it is and holds every target until power_on(). True when the arm
  /// acknowledged the stop within the deadline, false otherwise: the stop may have been lost,
  /// and the backend keeps resending it (#103). Called from a lifecycle transition, never from
  /// the control loop: it may wait up to REQUEST_TIMEOUT.
  [[nodiscard]] virtual bool halt() = 0;

  /// The same stop and hold as halt(), without the wait: it only leaves the request, the
  /// implementation sends the stop on its own thread. For on_error, which may run on the
  /// control loop (a write() that returned ERROR) and must not wait there. Nothing reports
  /// whether the stop went out: the plugin drops the backend on the next configure anyway.
  virtual void request_halt() = 0;

  /// Powers the servos on, so the arm holds its pose, and lets targets go out again after
  /// halt(). A target stored before or during the halt is dropped: the next write_angles() is
  /// the first to reach the arm. True when the arm acknowledged the power on within the
  /// deadline, false otherwise, or when a halt was requested meanwhile and the hold is back on
  /// (#103). Called from a lifecycle transition, never from the control loop: it may wait up to
  /// REQUEST_TIMEOUT.
  [[nodiscard]] virtual bool power_on() = 0;

  /// True once read_angles() returns a real measurement. Non-blocking, safe from the control
  /// loop. Until then the plugin reports "unknown" (NaN) instead of the placeholder pose.
  [[nodiscard]] virtual bool has_sample() const = 0;

  /// True when the link to the arm has gone silent for longer than the backend's timeout (#35),
  /// so the pose from read_angles() is stale and no target can be trusted to arrive. It does
  /// not latch: it turns false again once replies resume. Non-blocking, safe from the control
  /// loop.
  [[nodiscard]] virtual bool link_timed_out() const = 0;

  /// The newest plausible temperatures of the six servos, or nullopt before the first one.
  /// Non-blocking, safe from the control loop.
  [[nodiscard]] virtual std::optional<ServoTemperatures> read_temperatures() const = 0;

  /// Set once a servo has reached the backend's temperature limit. By then the backend has
  /// already stopped the arm and powered its servos off, on its own, without waiting for the
  /// plugin: the arm goes limp. From then on it holds every target and refuses power_on(), for
  /// as long as it lives; a new configure creates a new backend. Non-blocking, safe from the
  /// control loop.
  [[nodiscard]] virtual std::optional<Overheat> overheat() const = 0;

  /// True when no plausible temperature has arrived for longer than the backend's timeout, so
  /// an overheat could go unseen. Non-blocking, safe from the control loop.
  [[nodiscard]] virtual bool temperature_stale() const = 0;

  /// The last send_angles that reached the serial line, or nullopt before the first one and
  /// after an acknowledged stop or an overheat cut: then the arm is heading nowhere and there is
  /// nothing the pose should follow. What the plugin's unfollowed-command check (#103) compares
  /// the pose with every cycle. Safe from the control loop.
  [[nodiscard]] virtual std::optional<JointPositions> last_target() const = 0;

  /// How many send_angles got no sync-write from the arm within the backend's echo timeout,
  /// since the backend started (#150). Each made its turn wait the whole timeout: the arm took
  /// no target from it, as with a target outside the arm's own joint limits (19 Nov 2026), or it
  /// answers in a way the driver does not know. Never decreases. Safe from the control loop.
  [[nodiscard]] virtual std::uint64_t echo_timeouts() const = 0;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__ARM_BACKEND_HPP_
