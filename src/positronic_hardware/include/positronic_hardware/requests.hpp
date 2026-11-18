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

#ifndef POSITRONIC_HARDWARE__REQUESTS_HPP_
#define POSITRONIC_HARDWARE__REQUESTS_HPP_

#include <chrono>
#include <cstdint>

#include "positronic_protocol/encode.hpp"
#include "positronic_protocol/types.hpp"

namespace positronic_hardware
{

/// What the Atom sends back for a request, and so what SerialArm::perform() waits for.
enum class Expect : std::uint8_t
{
  /// Nothing: the frame goes out and the request is done.
  NONE,
  /// The ack `FE FE 03 <cmd> 01 FA` of the request's command.
  ACK,
  /// A reply frame, recognised by the scanner the caller gives perform().
  REPLY
};

/// How long perform() waits for the ack of a stop or power_on. On the bench (16 Nov 2026, 259
/// stops and 69 power_on) a stop is acknowledged in 37 to 41 ms when the arm is not moving, up to
/// 45 ms right after a send_angles, once 86 ms mid-move. A power_on takes 58 to 60 ms. 80 ms
/// covers all of that but the one 86 ms, plus the 2 ms read backoff, and keeps a request turn
/// under the 200 ms link watchdog (~130 ms of silence at worst). No ack by then: the frame goes
/// out again next turn (#103). That is what the 86 ms ack costs, a second stop one turn later.
inline constexpr std::chrono::milliseconds ACK_TIMEOUT{80};

/// How long perform() waits for the reply of a request that answers with data. The same 50 ms
/// as SerialArm::REPLY_TIMEOUT for the get_angles of the turn, under its own name.
inline constexpr std::chrono::milliseconds REQUEST_REPLY_TIMEOUT{50};

/// One request to the Atom, as data: the serial thread performs any of them the same way (#149).
/// A new request is a factory below and a test in test_requests, not a function in the loop.
struct Request
{
  /// The command the arm echoes in its ack or its reply.
  positronic_protocol::CommandId command;
  /// The bytes to write.
  positronic_protocol::Frame frame;
  /// What to wait for after the write.
  Expect expect;
  /// How long to wait for it.
  std::chrono::milliseconds timeout;
};

namespace requests
{

/// Stops the current motion. Acknowledged; the thread resends it every turn until it is (#103).
inline Request stop()
{
  return {
    positronic_protocol::CommandId::STOP, positronic_protocol::encode_stop(), Expect::ACK,
    ACK_TIMEOUT};
}

/// Powers the servos on. Acknowledged (#103).
inline Request power_on()
{
  return {
    positronic_protocol::CommandId::POWER_ON, positronic_protocol::encode_power_on(), Expect::ACK,
    ACK_TIMEOUT};
}

/// Powers the servos off. The arm answers nothing: the overheat cut repeats it once a period.
inline Request power_off()
{
  return {
    positronic_protocol::CommandId::POWER_OFF, positronic_protocol::encode_power_off(),
    Expect::NONE, std::chrono::milliseconds{0}};
}

/// Asks the six servo temperatures. Answered with a reply frame.
inline Request get_servo_temps()
{
  return {
    positronic_protocol::CommandId::GET_SERVO_TEMPS, positronic_protocol::encode_get_servo_temps(),
    Expect::REPLY, REQUEST_REPLY_TIMEOUT};
}

}  // namespace requests

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__REQUESTS_HPP_
