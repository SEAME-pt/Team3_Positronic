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

// AI-generated (Claude, Anthropic).
// Prompt:
//   Write test-only helpers shared by the suites: parse_hex and to_hex, bytes_of(Frame),
//   view_of and prefix_of over a byte vector, and concat of byte sequences.

#ifndef TEST_HELPERS_HPP_
#define TEST_HELPERS_HPP_

// Test-only helpers. Tests may allocate (std::vector, std::string): they are not on
// the real-time path.

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "positronic_protocol/detail/byte_access.hpp"
#include "positronic_protocol/types.hpp"

namespace test_helpers
{

using Bytes = std::vector<std::uint8_t>;

/// "fe fe 02 10 fa" -> {0xFE, 0xFE, 0x02, 0x10, 0xFA}.
inline Bytes parse_hex(const std::string & text)
{
  Bytes bytes{};
  std::istringstream stream{text};
  std::string token{};
  while (stream >> token) {
    bytes.push_back(static_cast<std::uint8_t>(std::stoul(token, nullptr, 16)));
  }
  return bytes;
}

/// {0xFE, 0x10} -> "fe 10", for readable failure messages.
inline std::string to_hex(const Bytes & bytes)
{
  std::ostringstream stream{};
  for (std::size_t i{0}; i < bytes.size(); ++i) {
    stream << (i == 0 ? "" : " ") << std::hex << std::setw(2) << std::setfill('0')
           << int{bytes.at(i)};
  }
  return stream.str();
}

/// The valid bytes of a frame.
inline Bytes bytes_of(const positronic_protocol::Frame & frame)
{
  const positronic_protocol::ByteView view{positronic_protocol::detail::view_of(frame)};
  Bytes bytes{};
  for (std::size_t i{0}; i < view.size; ++i) {
    bytes.push_back(positronic_protocol::detail::read_byte(view, i));
  }
  return bytes;
}

/// A view over a whole vector.
inline positronic_protocol::ByteView view_of(const Bytes & bytes)
{
  return positronic_protocol::ByteView{bytes.data(), bytes.size()};
}

/// A view over the first `size` bytes of a vector (a partial read of the UART).
inline positronic_protocol::ByteView prefix_of(const Bytes & bytes, std::size_t size)
{
  return positronic_protocol::ByteView{bytes.data(), size};
}

/// Glues byte sequences together: concat({noise, frame}).
inline Bytes concat(std::initializer_list<Bytes> parts)
{
  Bytes all{};
  for (const auto & part : parts) {
    all.insert(all.end(), part.begin(), part.end());
  }
  return all;
}

}  // namespace test_helpers

#endif  // TEST_HELPERS_HPP_
