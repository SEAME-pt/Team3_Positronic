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

#ifndef POSITRONIC_HARDWARE__VELOCITY_ESTIMATOR_HPP_
#define POSITRONIC_HARDWARE__VELOCITY_ESTIMATOR_HPP_

#include <cstddef>

#include "positronic_hardware/arm_backend.hpp"

namespace positronic_hardware
{

/// Estimates joint velocity from a stream of positions, because the protocol gives position
/// only (get_angles returns angles, never speed).
///
/// Raw finite differences (dq / dt) are noisy: the angle comes in 0.01 deg steps, so a tiny
/// quantisation jump over a short period reads as a large, jittery velocity. We low-pass the
/// raw estimate with a first-order filter:
///
///     v_filtered = alpha * v_raw + (1 - alpha) * v_previous
///
/// alpha in (0, 1]: closer to 1 follows motion faster but keeps more noise, closer to 0 is
/// smoother but lags. The default is a starting point; tune it against the real arm (#23) and
/// the measured get_angles rate (#20).
///
/// Two guards protect the finite difference:
/// - A period at or below MIN_DT (two read()s in the same tick) holds the last estimate, so we
///   never divide by (almost) zero.
/// - A period above `max_gap` (the serial link went silent and then came back) does NOT get
///   differentiated: the joint appears to have jumped across the hole, which would read as a
///   huge fake velocity. We restart at zero and re-anchor the baseline instead.
///
/// Pure and ROS-free on purpose: the plugin owns one instance and feeds it the positions it
/// already reads, and the unit tests drive it directly without the arm.
class VelocityEstimator
{
public:
  /// Smoothing factor of the low-pass filter, in (0, 1]. Tune against real data.
  static constexpr double DEFAULT_ALPHA{0.3};

  /// Longest period (seconds) still differentiated. Above it, the link is treated as having been
  /// silent and the estimate restarts at zero. A starting point; tune against the get_angles rate.
  static constexpr double DEFAULT_MAX_GAP{0.5};

  /// Periods shorter than this (seconds) are treated as "no time passed": the estimate is
  /// held, so a double read() in the same tick cannot divide by (almost) zero.
  static constexpr double MIN_DT{1e-6};

  explicit VelocityEstimator(double alpha = DEFAULT_ALPHA, double max_gap = DEFAULT_MAX_GAP)
  : alpha_{alpha}, max_gap_{max_gap}
  {
  }

  /// Feeds a new position sample taken `dt` seconds after the previous one, and returns the
  /// filtered velocity of every joint, in rad/s.
  ///
  /// The first call has no previous sample to difference, so it returns zeros and just stores
  /// the baseline. `dt` at or below MIN_DT holds the last estimate. `dt` above `max_gap_`
  /// restarts at zero (the link was silent) and re-anchors the baseline at this sample.
  JointPositions update(const JointPositions & position, double dt)
  {
    if (!has_previous_) {
      previous_position_ = position;
      has_previous_ = true;
      return velocity_;
    }

    if (dt > max_gap_) {
      velocity_ = JointPositions{};
      previous_position_ = position;
    } else if (dt > MIN_DT) {
      for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
        const double raw{(position.at(i) - previous_position_.at(i)) / dt};
        velocity_.at(i) = alpha_ * raw + (1.0 - alpha_) * velocity_.at(i);
      }
      previous_position_ = position;
    }
    return velocity_;
  }

  /// Forgets the history, so the next update() starts a fresh baseline at zero velocity.
  /// Called on_configure(), when the backend is (re)connected.
  void reset()
  {
    has_previous_ = false;
    previous_position_ = JointPositions{};
    velocity_ = JointPositions{};
  }

private:
  double alpha_{DEFAULT_ALPHA};
  double max_gap_{DEFAULT_MAX_GAP};
  bool has_previous_{false};
  JointPositions previous_position_{};
  JointPositions velocity_{};
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__VELOCITY_ESTIMATOR_HPP_