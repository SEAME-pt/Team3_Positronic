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

/// Where the driver reads the joint angles from.
///
/// One method: "where are the joints now, in radians?". The plugin (read()) never knows
/// whether the answer came from the real serial port or from a stand-in; it only calls
/// read_angles(). This is the seam that lets the fake arm and the real arm be swapped by a
/// parameter, and the control loop stay the same.
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

  /// True once read_angles() returns a real measurement. Non-blocking, safe from the control
  /// loop. Until then the plugin reports "unknown" (NaN) instead of the placeholder pose.
  [[nodiscard]] virtual bool has_sample() const = 0;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__ARM_BACKEND_HPP_
