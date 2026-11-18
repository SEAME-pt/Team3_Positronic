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

#include "positronic_protocol/units.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include "positronic_protocol/types.hpp"

namespace positronic_protocol
{

namespace
{

// std::numbers::pi is C++20.
constexpr double PI{3.14159265358979323846};

/// 180 deg = pi rad, and 1 deg = 100 centidegrees.
constexpr double CENTIDEGREES_PER_RADIAN{18000.0 / PI};

}  // namespace

std::optional<CentiDegrees> to_centidegrees(double radians) noexcept
{
  if (!std::isfinite(radians)) {
    return std::nullopt;
  }
  const double centidegrees{radians * CENTIDEGREES_PER_RADIAN};
  if (std::abs(centidegrees) >= static_cast<double>(MAX_CENTIDEGREES) + 0.5) {
    return std::nullopt;
  }
  return static_cast<CentiDegrees>(std::lround(centidegrees));
}

double to_radians(CentiDegrees value) noexcept
{
  return static_cast<double>(value) / CENTIDEGREES_PER_RADIAN;
}

std::optional<JointAngles> to_centidegrees(const JointRadians & radians) noexcept
{
  JointAngles angles{};
  bool ok{true};
  std::transform(radians.begin(), radians.end(), angles.begin(), [&ok](double joint) {
      const std::optional<CentiDegrees> converted{to_centidegrees(joint)};
      ok = ok && converted.has_value();
      return converted.value_or(0);
  });
  if (!ok) {
    return std::nullopt;
  }
  return angles;
}

JointRadians to_radians(const JointAngles & angles) noexcept
{
  JointRadians radians{};
  std::transform(angles.begin(), angles.end(), radians.begin(), [](CentiDegrees joint) {
      return to_radians(joint);
  });
  return radians;
}

}  // namespace positronic_protocol
