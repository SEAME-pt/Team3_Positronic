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
//   Unit tests for requests.hpp (#149): every factory carries the frame its command encodes and
//   the command the arm echoes, what each one waits for, the command byte at index 3 of every
//   frame, and ACK_TIMEOUT between the arm's 60 ms and the 200 ms watchdog.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <vector>

#include "positronic_hardware/requests.hpp"
#include "positronic_protocol/encode.hpp"
#include "positronic_protocol/types.hpp"

namespace
{

using positronic_hardware::ACK_TIMEOUT;
using positronic_hardware::Expect;
using positronic_hardware::Request;
using positronic_hardware::REQUEST_REPLY_TIMEOUT;
using positronic_hardware::get_servo_temps_request;
using positronic_hardware::power_off_request;
using positronic_hardware::power_on_request;
using positronic_hardware::stop_request;
using positronic_protocol::CommandId;

std::vector<std::uint8_t> bytes_of(const positronic_protocol::Frame & frame)
{
  return {frame.bytes.begin(), frame.bytes.begin() + static_cast<std::ptrdiff_t>(frame.size)};
}

CommandId command_byte(const Request & request)
{
  return static_cast<CommandId>(request.frame.bytes.at(3));
}

}  // namespace

// Each factory carries the frame the protocol encodes for its command, and the command the arm
// will echo: a wrong pair would make perform() wait for an ack that never matches.
TEST(Requests, StopIsTheStopFrameAndWaitsForItsAck)
{
  const Request request{stop_request()};
  EXPECT_EQ(request.command, CommandId::STOP);
  EXPECT_EQ(bytes_of(request.frame), bytes_of(positronic_protocol::encode_stop()));
  EXPECT_EQ(request.expect, Expect::ACK);
  EXPECT_EQ(request.timeout, ACK_TIMEOUT);
}

TEST(Requests, PowerOnIsThePowerOnFrameAndWaitsForItsAck)
{
  const Request request{power_on_request()};
  EXPECT_EQ(request.command, CommandId::POWER_ON);
  EXPECT_EQ(bytes_of(request.frame), bytes_of(positronic_protocol::encode_power_on()));
  EXPECT_EQ(request.expect, Expect::ACK);
  EXPECT_EQ(request.timeout, ACK_TIMEOUT);
}

TEST(Requests, PowerOffWaitsForNothing)
{
  const Request request{power_off_request()};
  EXPECT_EQ(request.command, CommandId::POWER_OFF);
  EXPECT_EQ(bytes_of(request.frame), bytes_of(positronic_protocol::encode_power_off()));
  EXPECT_EQ(request.expect, Expect::NONE);
  EXPECT_EQ(request.timeout, std::chrono::milliseconds{0});
}

TEST(Requests, GetServoTempsWaitsForAReply)
{
  const Request request{get_servo_temps_request()};
  EXPECT_EQ(request.command, CommandId::GET_SERVO_TEMPS);
  EXPECT_EQ(bytes_of(request.frame), bytes_of(positronic_protocol::encode_get_servo_temps()));
  EXPECT_EQ(request.expect, Expect::REPLY);
  EXPECT_EQ(request.timeout, REQUEST_REPLY_TIMEOUT);
}

// The command byte sits at index 3 of every request frame (`FE FE 02 <cmd> FA`): the ack scanner
// in perform() relies on the factory's command being that byte.
TEST(Requests, TheCommandIsTheFourthByteOfEveryFrame)
{
  EXPECT_EQ(command_byte(stop_request()), CommandId::STOP);
  EXPECT_EQ(command_byte(power_on_request()), CommandId::POWER_ON);
  EXPECT_EQ(command_byte(power_off_request()), CommandId::POWER_OFF);
  EXPECT_EQ(command_byte(get_servo_temps_request()), CommandId::GET_SERVO_TEMPS);
}

// 80 ms sits above the 60 ms power_on ack measured on the arm and under the 200 ms link watchdog.
TEST(Requests, TheAckTimeoutSitsBetweenTheArmAndTheWatchdog)
{
  EXPECT_GT(ACK_TIMEOUT, std::chrono::milliseconds{60});
  EXPECT_LT(ACK_TIMEOUT, std::chrono::milliseconds{200});
}
