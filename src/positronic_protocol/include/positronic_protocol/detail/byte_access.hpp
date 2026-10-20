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

#ifndef POSITRONIC_PROTOCOL__DETAIL__BYTE_ACCESS_HPP_
#define POSITRONIC_PROTOCOL__DETAIL__BYTE_ACCESS_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "positronic_protocol/types.hpp"

/// Internal helpers of positronic_protocol. Not part of the public API.
///
/// This is the only place in the library that reads a buffer with a runtime index.
/// The decoder goes through read_byte(), so the bounds check and the NOLINT live in one
/// spot that is easy to review. The encoder needs no runtime index: its frame sizes are
/// known at compile time (see build_frame() in encode.cpp).
///
/// The suppression uses the BEGIN/END form on purpose: ament_cpplint rejects the
/// single-line form when it names a clang-tidy check, and it ignores the BEGIN/END form.
namespace positronic_protocol::detail
{

/// Byte at `index` of `view`, or 0 when `index` is out of range or `view` is empty.
///
/// Callers check `view.size` before reading. The guard only keeps a logic error from
/// becoming undefined behaviour (an out-of-bounds read).
[[nodiscard]] constexpr std::uint8_t read_byte(ByteView view, std::size_t index) noexcept
{
  if (view.data == nullptr || index >= view.size) {
    return 0;
  }
  // Bounds checked above. std::span is C++20 and there is no .at() on a raw pointer.
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  return view.data[index];
  // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

/// Read-only view over the valid bytes of `frame`.
///
/// `frame.size` is clamped to the capacity, so a corrupted size cannot widen the view.
[[nodiscard]] constexpr ByteView view_of(const Frame & frame) noexcept
{
  return ByteView{frame.bytes.data(), std::min(frame.size, frame.bytes.size())};
}

}  // namespace positronic_protocol::detail

#endif  // POSITRONIC_PROTOCOL__DETAIL__BYTE_ACCESS_HPP_
