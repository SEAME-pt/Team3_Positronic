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
//   Write a GoogleTest suite for fault_code.hpp (#121), the one table of the driver's fault
//   codes: each known code's line starts with its own name and has a reason after it; a value
//   outside the enum is "UNKNOWN: unknown fault code", never dropped.

#include <gtest/gtest.h>

#include <array>
#include <string_view>
#include <utility>

#include "positronic_hardware/fault_code.hpp"

using positronic_hardware::describe_fault;
using positronic_hardware::FaultCode;

TEST(FaultCode, EachCodeHasItsLineAndAnUnknownValueIsUnknown)
{
  const std::array<std::pair<FaultCode, std::string_view>, 4> names{{
    {FaultCode::NONE, "NONE: "},
    {FaultCode::LINK_TIMEOUT, "LINK_TIMEOUT: "},
    {FaultCode::TRACKING_ERROR, "TRACKING_ERROR: "},
    {FaultCode::OVERHEAT, "OVERHEAT: "},
  }};
  for (const auto & [code, name] : names) {
    const std::string_view line{describe_fault(code)};
    EXPECT_EQ(line.substr(0, name.size()), name);
    EXPECT_GT(line.size(), name.size()) << name;  // a reason after the name
  }
  for (const int value : {4, 255}) {
    EXPECT_EQ(describe_fault(static_cast<FaultCode>(value)), "UNKNOWN: unknown fault code")
      << value;
  }
}
