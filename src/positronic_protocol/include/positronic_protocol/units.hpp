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

#ifndef POSITRONIC_PROTOCOL__UNITS_HPP_
#define POSITRONIC_PROTOCOL__UNITS_HPP_

#include <optional>

#include "positronic_protocol/types.hpp"

namespace positronic_protocol
{

/// Largest angle the wire can carry, in 0.01 deg (327.67 deg).
inline constexpr CentiDegrees MAX_CENTIDEGREES{32767};

/// Converts radians to the wire unit, rounding to the nearest 0.01 deg.
///
/// pymycobot truncates (`int(angle * 100)`), so 0.29 deg becomes 28 there and 29 here.
/// @return std::nullopt for NaN, infinity, or a value that rounds beyond +-327.67 deg.
[[nodiscard]] std::optional<CentiDegrees> to_centidegrees(double radians) noexcept;

/// Converts the wire unit to radians.
[[nodiscard]] double to_radians(CentiDegrees value) noexcept;

/// Converts six joint angles from radians to the wire unit.
/// @return std::nullopt when any of the six fails (see the single-angle version).
[[nodiscard]] std::optional<JointAngles> to_centidegrees(const JointRadians & radians) noexcept;

/// Converts six joint angles from the wire unit to radians.
[[nodiscard]] JointRadians to_radians(const JointAngles & angles) noexcept;

}  // namespace positronic_protocol

#endif  // POSITRONIC_PROTOCOL__UNITS_HPP_
