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

#include "positronic_hardware/arm_backend.hpp"

namespace positronic_hardware
{

/// A pretend arm, so the driver runs without the robot. It never touches the serial port.
///
/// It starts in START_POSE and is at every target as soon as write_angles() gets it, with no
/// motion in between. SerialArm does the same over the serial port.
class FakeArm : public ArmBackend
{
public:
  /// The pose the fake arm reports, in radians.
  static constexpr JointPositions START_POSE{0.0, 0.5, 0.3, 0.0, 0.0, 0.0};

  /// Where the joints are now.
  [[nodiscard]] JointPositions read_angles() const override
  {
    return angles_;
  }

  /// Where the joints will be.
  void write_angles(const JointPositions & angles) override
  {
    angles_ = angles;
  }

  /// Nothing to interrupt: the fake arm is at every target the moment it gets it.
  [[nodiscard]] bool halt() override
  {
    return true;
  }

  /// Nothing to interrupt here either.
  void request_halt() override {}

  /// The fake arm has no servos to power.
  [[nodiscard]] bool power_on() override
  {
    return true;
  }

  /// The fake pose is known from the start.
  [[nodiscard]] bool has_sample() const override
  {
    return true;
  }

  /// There is no serial line to go silent, so the fake link never times out.
  [[nodiscard]] bool link_timed_out() const override
  {
    return false;
  }

private:
  JointPositions angles_{START_POSE};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__FAKE_ARM_HPP_
