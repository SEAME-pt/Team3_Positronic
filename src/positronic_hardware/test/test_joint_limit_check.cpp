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
//   Write a GoogleTest suite for joint_limit_check.hpp (issue #34). check_command() takes a
//   position command and the six joint limits and either accepts it or reports the first bad
//   joint, its value and why. Check that: a command inside the limits is accepted; values
//   exactly at a limit and within LIMIT_TOLERANCE of it are accepted; the issue's hardware case
//   (limit + 5 deg) is refused as ABOVE_UPPER / BELOW_LOWER with the right joint and value;
//   every joint J1..J6 is checked, on both sides; NaN and +-inf are refused as NOT_FINITE;
//   a wrong joint count (too few, too many, zero, null pointer) is refused as WRONG_COUNT;
//   when several joints are bad, the first one in J1..J6 order is reported; a NaN is never
//   reported as a limit fault; limits_valid() rejects swapped, equal and non-finite limits;
//   to_string() gives a distinct text for every fault.
// Follow-up prompt:
//   Drop the test that casts 255 to CommandFault: clang-tidy (EnumCastOutOfRange) fails the
//   strict CI on it, and the repo keeps a single documented NOLINT.
//   Close the anonymous namespace before the TEST blocks, as the other test files do: with
//   gtest unparsed, ament_cppcheck misreads a TEST macro inside an anonymous namespace.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/joint_limit_check.hpp"

namespace
{
using positronic_hardware::check_command;
using positronic_hardware::CommandCheck;
using positronic_hardware::CommandFault;
using positronic_hardware::CommandView;
using positronic_hardware::JointPositions;
using positronic_hardware::LIMIT_TOLERANCE;
using positronic_hardware::limits_valid;
using positronic_hardware::NUM_JOINTS;
using positronic_hardware::PositionLimits;
using positronic_hardware::to_string;

constexpr double DEG{3.14159265358979323846 / 180.0};

// The limits of the myCobot 280 URDF (mycobot_280_jn.urdf), in radians. Test data only: the
// plugin reads its limits from the URDF, never from this table.
constexpr PositionLimits URDF_LIMITS{{
  {-2.9321, 2.9321},    // J1
  {-2.4434, 2.4434},    // J2
  {-2.6179, 2.6179},    // J3
  {-2.6179, 2.6179},    // J4
  {-2.7052, 2.7925},    // J5, not symmetric
  {-3.14159, 3.14159},  // J6
}};

constexpr JointPositions HOME{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void expect_fault(const CommandCheck & result, CommandFault fault, std::size_t joint, double value)
{
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.fault, fault) << to_string(result.fault);
  EXPECT_EQ(result.joint, joint);
  EXPECT_DOUBLE_EQ(result.value, value);
}

}  // namespace

TEST(JointLimitCheck, HomeIsAccepted)
{
  const CommandCheck result{check_command(HOME, URDF_LIMITS)};
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.fault, CommandFault::NONE);
}

TEST(JointLimitCheck, ExactlyAtEveryLimitIsAccepted)
{
  JointPositions lower{};
  JointPositions upper{};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    lower.at(i) = URDF_LIMITS.at(i).lower;
    upper.at(i) = URDF_LIMITS.at(i).upper;
  }
  EXPECT_TRUE(check_command(lower, URDF_LIMITS).ok());
  EXPECT_TRUE(check_command(upper, URDF_LIMITS).ok());
}

TEST(JointLimitCheck, InsideToleranceIsAcceptedOutsideIsRefused)
{
  JointPositions command{HOME};
  command[1] = URDF_LIMITS[1].upper + 0.5 * LIMIT_TOLERANCE;
  EXPECT_TRUE(check_command(command, URDF_LIMITS).ok());

  command[1] = URDF_LIMITS[1].upper + 2.0 * LIMIT_TOLERANCE;
  expect_fault(check_command(command, URDF_LIMITS), CommandFault::ABOVE_UPPER, 1, command[1]);
}

TEST(JointLimitCheck, ZeroToleranceIsStrict)
{
  JointPositions command{HOME};
  command[0] = URDF_LIMITS[0].upper + 1e-9;
  EXPECT_FALSE(check_command(command, URDF_LIMITS, 0.0).ok());
}

// The acceptance criterion on hardware: a goal at the limit + 5 degrees.
TEST(JointLimitCheck, LimitPlusFiveDegreesIsRefusedOnEveryJoint)
{
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    JointPositions above{HOME};
    above.at(i) = URDF_LIMITS.at(i).upper + 5.0 * DEG;
    expect_fault(check_command(above, URDF_LIMITS), CommandFault::ABOVE_UPPER, i, above.at(i));

    JointPositions below{HOME};
    below.at(i) = URDF_LIMITS.at(i).lower - 5.0 * DEG;
    expect_fault(check_command(below, URDF_LIMITS), CommandFault::BELOW_LOWER, i, below.at(i));
  }
}

TEST(JointLimitCheck, OneProtocolStepOverTheLimitIsRefused)
{
  // 0.01 deg is the smallest step the protocol can send: even that is outside.
  JointPositions command{HOME};
  command[4] = URDF_LIMITS[4].upper + 0.01 * DEG;
  expect_fault(check_command(command, URDF_LIMITS), CommandFault::ABOVE_UPPER, 4, command[4]);
}

TEST(JointLimitCheck, AsymmetricLimitUsesItsOwnEnds)
{
  // J5 goes to +2.7925 but only to -2.7052: -2.75 is fine as +2.75, refused as -2.75.
  JointPositions command{HOME};
  command[4] = 2.75;
  EXPECT_TRUE(check_command(command, URDF_LIMITS).ok());
  command[4] = -2.75;
  expect_fault(check_command(command, URDF_LIMITS), CommandFault::BELOW_LOWER, 4, -2.75);
}

TEST(JointLimitCheck, NanIsRefusedOnEveryJoint)
{
  const double nan{std::numeric_limits<double>::quiet_NaN()};
  for (std::size_t i{0}; i < NUM_JOINTS; ++i) {
    JointPositions command{HOME};
    command.at(i) = nan;
    const CommandCheck result{check_command(command, URDF_LIMITS)};
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.fault, CommandFault::NOT_FINITE);
    EXPECT_EQ(result.joint, i);
    EXPECT_TRUE(std::isnan(result.value));
  }
}

TEST(JointLimitCheck, InfinityIsRefusedAsNotFinite)
{
  JointPositions command{HOME};
  command[2] = std::numeric_limits<double>::infinity();
  expect_fault(check_command(command, URDF_LIMITS), CommandFault::NOT_FINITE, 2, command[2]);
  command[2] = -std::numeric_limits<double>::infinity();
  expect_fault(check_command(command, URDF_LIMITS), CommandFault::NOT_FINITE, 2, command[2]);
}

TEST(JointLimitCheck, WrongCountIsRefused)
{
  const std::vector<double> five(NUM_JOINTS - 1, 0.0);
  expect_fault(
    check_command(CommandView{five.data(), five.size()}, URDF_LIMITS), CommandFault::WRONG_COUNT, 0,
    static_cast<double>(NUM_JOINTS - 1));

  const std::vector<double> seven(NUM_JOINTS + 1, 0.0);
  expect_fault(
    check_command(CommandView{seven.data(), seven.size()}, URDF_LIMITS), CommandFault::WRONG_COUNT,
    0, static_cast<double>(NUM_JOINTS + 1));

  expect_fault(check_command(CommandView{}, URDF_LIMITS), CommandFault::WRONG_COUNT, 0, 0.0);
}

TEST(JointLimitCheck, NullPointerWithSixIsRefused)
{
  expect_fault(
    check_command(CommandView{nullptr, NUM_JOINTS}, URDF_LIMITS), CommandFault::WRONG_COUNT, 0,
    static_cast<double>(NUM_JOINTS));
}

TEST(JointLimitCheck, CountIsCheckedBeforeValues)
{
  // Seven values with a NaN in them: the count is the fault, the NaN is never read.
  std::vector<double> seven(NUM_JOINTS + 1, 0.0);
  seven[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    check_command(CommandView{seven.data(), seven.size()}, URDF_LIMITS).fault,
    CommandFault::WRONG_COUNT);
}

TEST(JointLimitCheck, ViewOfSixIsTheSameAsTheArray)
{
  const std::vector<double> six{0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  EXPECT_TRUE(check_command(CommandView{six.data(), six.size()}, URDF_LIMITS).ok());
}

TEST(JointLimitCheck, FirstBadJointIsReported)
{
  JointPositions command{HOME};
  command[3] = URDF_LIMITS[3].upper + 1.0;  // J4 bad
  command[5] = URDF_LIMITS[5].lower - 1.0;  // J6 bad too
  expect_fault(check_command(command, URDF_LIMITS), CommandFault::ABOVE_UPPER, 3, command[3]);
}

TEST(JointLimitCheck, NanIsNeverReportedAsALimitFault)
{
  // Every comparison with NaN is false, so a naive "value < lower || value > upper" check
  // would let a NaN through. The finite check must come first.
  JointPositions command{HOME};
  command[0] = std::numeric_limits<double>::quiet_NaN();
  const CommandCheck result{check_command(command, URDF_LIMITS)};
  EXPECT_NE(result.fault, CommandFault::BELOW_LOWER);
  EXPECT_NE(result.fault, CommandFault::ABOVE_UPPER);
  EXPECT_EQ(result.fault, CommandFault::NOT_FINITE);
}

TEST(JointLimitCheck, UrdfLimitsAreValid)
{
  EXPECT_TRUE(limits_valid(URDF_LIMITS));
}

TEST(JointLimitCheck, BadLimitsAreInvalid)
{
  PositionLimits swapped{URDF_LIMITS};
  swapped[2] = {1.0, -1.0};
  EXPECT_FALSE(limits_valid(swapped));

  PositionLimits equal{URDF_LIMITS};
  equal[2] = {0.5, 0.5};
  EXPECT_FALSE(limits_valid(equal));

  PositionLimits nan_end{URDF_LIMITS};
  nan_end[5].upper = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(limits_valid(nan_end));

  // What ros2_control leaves when a joint has no position limit: +-numeric_limits::max()
  // is finite, so only the missing-limit flag can catch it (see the integration sketch).
  PositionLimits infinite{URDF_LIMITS};
  infinite[0] = {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  EXPECT_FALSE(limits_valid(infinite));

  const PositionLimits all_zero{};
  EXPECT_FALSE(limits_valid(all_zero));
}

TEST(JointLimitCheck, EveryFaultHasItsOwnText)
{
  const std::array<CommandFault, 5> faults{
    CommandFault::NONE, CommandFault::WRONG_COUNT, CommandFault::NOT_FINITE,
    CommandFault::BELOW_LOWER, CommandFault::ABOVE_UPPER};
  std::set<std::string> texts{};
  for (const CommandFault fault : faults) {
    texts.insert(to_string(fault));
  }
  EXPECT_EQ(texts.size(), faults.size());
}
