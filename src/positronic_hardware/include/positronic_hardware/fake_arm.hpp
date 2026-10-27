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

#ifndef POSITRONIC_HARDWARE__FAKE_ARM_HPP_
#define POSITRONIC_HARDWARE__FAKE_ARM_HPP_

#include <array>
#include <cstddef>

namespace positronic_hardware
{

/// Number of joints of the myCobot 280.
inline constexpr std::size_t NUM_JOINTS{6};

/// Six joint angles in radians, J1 (base) to J6 (flange).
using JointPositions = std::array<double, NUM_JOINTS>;

/// A pretend arm, so the driver runs without the robot. It never touches the serial port.
///
/// It stands still in a fixed pose. The real arm (#23) will answer the same question
/// from the serial port instead.
class FakeArm
{
public:
  /// The pose the fake arm reports, in radians.
  static constexpr JointPositions START_POSE{0.0, 0.5, 0.3, 0.0, 0.0, 0.0};

  /// Where the joints are now.
  JointPositions read_angles() const
  {
    return angles_;
  }

private:
  JointPositions angles_{START_POSE};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__FAKE_ARM_HPP_
