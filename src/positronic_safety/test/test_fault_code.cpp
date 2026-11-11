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
//   Write a GoogleTest suite for fault_code.hpp (issue #45). find_fault_code() looks for the
//   driver's fault code in a /dynamic_joint_states message and returns nothing when the message
//   has no usable one. Build the messages by hand, like the one joint_state_broadcaster
//   publishes for our arm (six joints with position and velocity, then the driver gpio). Check
//   that: the code is found among the joints, 0 and non-zero; other interfaces of the driver
//   gpio do not get in the way; a joint that has an interface with the same name is not taken
//   for the driver; a missing gpio, a missing interface, an empty message, NaN and infinity
//   all give nothing; lists of different lengths are never read past the shorter one.

#include <gtest/gtest.h>

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "positronic_safety/fault_code.hpp"

namespace
{
using control_msgs::msg::DynamicJointState;
using control_msgs::msg::InterfaceValue;
using positronic_safety::find_fault_code;

/// Adds one entry to `message`: a joint or a gpio, with its interfaces and their values.
void add_entry(
  DynamicJointState & message, const std::string & name,
  const std::vector<std::string> & interfaces, const std::vector<double> & values)
{
  InterfaceValue entry;
  entry.interface_names = interfaces;
  entry.values = values;
  message.joint_names.push_back(name);
  message.interface_values.push_back(entry);
}

/// What joint_state_broadcaster publishes for our arm, with `fault_code` in the driver gpio.
DynamicJointState arm_message(double fault_code)
{
  DynamicJointState message;
  for (const char * joint : {"joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
      "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"})
  {
    add_entry(message, joint, {"position", "velocity"}, {0.5, 0.0});
  }
  add_entry(message, "driver", {"fault_code"}, {fault_code});
  return message;
}
}  // namespace

TEST(FindFaultCode, FindsNoFaultAmongTheJoints)
{
  const std::optional<double> code = find_fault_code(arm_message(0.0), "driver", "fault_code");
  EXPECT_TRUE(code == 0.0);
}

TEST(FindFaultCode, FindsAFault)
{
  // 1 is LINK_TIMEOUT in positronic_hardware; the supervisor halts on any value but 0.
  EXPECT_TRUE(find_fault_code(arm_message(1.0), "driver", "fault_code") == 1.0);
  EXPECT_TRUE(find_fault_code(arm_message(2.0), "driver", "fault_code") == 2.0);
}

TEST(FindFaultCode, OtherDriverInterfacesDoNotGetInTheWay)
{
  // S3 adds more state interfaces to the driver gpio (#115).
  DynamicJointState message;
  add_entry(message, "driver", {"temperature", "fault_code", "voltage"}, {41.5, 1.0, 12.1});
  EXPECT_TRUE(find_fault_code(message, "driver", "fault_code") == 1.0);
}

TEST(FindFaultCode, AJointWithTheSameInterfaceNameIsNotTheDriver)
{
  DynamicJointState message;
  add_entry(message, "joint2_to_joint1", {"fault_code"}, {1.0});
  EXPECT_FALSE(find_fault_code(message, "driver", "fault_code").has_value());
}

TEST(FindFaultCode, NothingWhenTheGpioIsMissing)
{
  DynamicJointState message = arm_message(0.0);
  message.joint_names.pop_back();
  message.interface_values.pop_back();
  EXPECT_FALSE(find_fault_code(message, "driver", "fault_code").has_value());
}

TEST(FindFaultCode, NothingWhenTheInterfaceIsMissing)
{
  DynamicJointState message;
  add_entry(message, "driver", {"temperature"}, {41.5});
  EXPECT_FALSE(find_fault_code(message, "driver", "fault_code").has_value());
}

TEST(FindFaultCode, NothingInAnEmptyMessage)
{
  EXPECT_FALSE(find_fault_code(DynamicJointState{}, "driver", "fault_code").has_value());
}

TEST(FindFaultCode, NothingWhenTheValueIsNotANumber)
{
  // The driver fills the interface with a whole number. NaN is what a state interface holds
  // before its first read: "unknown", which must not pass for "no fault".
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(find_fault_code(arm_message(nan), "driver", "fault_code").has_value());
  EXPECT_FALSE(find_fault_code(arm_message(infinity), "driver", "fault_code").has_value());
  EXPECT_FALSE(find_fault_code(arm_message(-infinity), "driver", "fault_code").has_value());
}

TEST(FindFaultCode, NeverReadsPastTheShorterList)
{
  // A name with no entry for it.
  DynamicJointState no_entry = arm_message(0.0);
  no_entry.interface_values.pop_back();
  EXPECT_FALSE(find_fault_code(no_entry, "driver", "fault_code").has_value());

  // An interface name with no value for it.
  DynamicJointState no_value;
  add_entry(no_value, "driver", {"temperature", "fault_code"}, {41.5});
  EXPECT_FALSE(find_fault_code(no_value, "driver", "fault_code").has_value());
}
