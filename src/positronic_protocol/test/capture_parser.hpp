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
//   Write a test-only parser for the capture files (## section headers with TX / RX /
//   RESULT / RXT lines): return the sections in order, split a TX line into frames by LEN,
//   give the first command id of a section, and parse a RESULT list into centidegrees.

#ifndef CAPTURE_PARSER_HPP_
#define CAPTURE_PARSER_HPP_

// Reads the capture files written by scripts/capture_frames/capture_frames.py.
// Test-only: std::string, std::vector and std::ifstream are fine here.
//
// Format, one section per pymycobot call:
//   ## <name>
//   TX <hex bytes>             everything we sent (may hold several frames)
//   RX <hex bytes>             everything received
//   RESULT <text>              what pymycobot returned
//   TXT/RXT +<ms> <hex bytes>  the same bytes, split in the bursts the UART delivered
// Lines starting with '#' and unknown keywords are ignored.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "test_helpers.hpp"

namespace capture_parser
{

using test_helpers::Bytes;

/// One pymycobot call in the capture.
struct Section
{
  std::string name;
  Bytes tx;
  Bytes rx;
  std::string result;
  std::vector<Bytes> rx_bursts;
};

/// Text after the first space of `line`, or "" when there is none.
inline std::string after_keyword(const std::string & line)
{
  const std::size_t space{line.find(' ')};
  return space == std::string::npos ? std::string{} : line.substr(space + 1);
}

/// All sections of a capture file, in file order. Empty when the file cannot be read.
inline std::vector<Section> parse_capture(const std::string & path)
{
  std::vector<Section> sections{};
  std::ifstream file{path};
  std::string line{};
  while (std::getline(file, line)) {
    if (line.rfind("## ", 0) == 0) {
      sections.push_back(Section{});
      sections.back().name = line.substr(3);
      continue;
    }
    if (line.empty() || line.front() == '#' || sections.empty()) {
      continue;
    }
    Section & section{sections.back()};
    const std::string keyword{line.substr(0, line.find(' '))};
    const std::string rest{after_keyword(line)};
    if (keyword == "TX") {
      section.tx = test_helpers::parse_hex(rest);
    } else if (keyword == "RX") {
      section.rx = test_helpers::parse_hex(rest);
    } else if (keyword == "RESULT") {
      section.result = rest;
    } else if (keyword == "RXT") {
      section.rx_bursts.push_back(test_helpers::parse_hex(after_keyword(rest)));
    }
  }
  return sections;
}

/// Splits a TX line into frames using LEN. stop_during_motion sends two frames.
inline std::vector<Bytes> split_frames(const Bytes & tx)
{
  std::vector<Bytes> frames{};
  std::size_t start{0};
  while (start + 2 < tx.size()) {
    const std::size_t size{std::size_t{tx.at(start + 2)} + 3};
    if (start + size > tx.size()) {
      break;
    }
    frames.emplace_back(
      tx.begin() + static_cast<std::ptrdiff_t>(start),
      tx.begin() + static_cast<std::ptrdiff_t>(start + size));
    start += size;
  }
  return frames;
}

/// Command id of the first frame sent in `section`, if any.
inline std::optional<std::uint8_t> first_command(const Section & section)
{
  if (section.tx.size() < 4) {
    return std::nullopt;
  }
  return section.tx.at(3);
}

/// "[0.87, 0, -0.61]" -> {87, 0, -61} (degrees x 100, rounded). nullopt if not a list.
inline std::optional<std::vector<int>> centidegrees_from_result(const std::string & result)
{
  if (result.empty() || result.front() != '[' || result.back() != ']') {
    return std::nullopt;
  }
  std::string inside{result.substr(1, result.size() - 2)};
  for (auto & character : inside) {
    character = (character == ',') ? ' ' : character;
  }
  std::istringstream stream{inside};
  std::vector<int> values{};
  double degrees{0.0};
  while (stream >> degrees) {
    values.push_back(static_cast<int>(std::lround(degrees * 100.0)));
  }
  return values;
}

}  // namespace capture_parser

#endif  // CAPTURE_PARSER_HPP_
