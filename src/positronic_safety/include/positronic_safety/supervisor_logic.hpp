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

#ifndef POSITRONIC_SAFETY__SUPERVISOR_LOGIC_HPP_
#define POSITRONIC_SAFETY__SUPERVISOR_LOGIC_HPP_

#include <cstdint>

namespace positronic_safety
{

/// What the supervisor is doing. Published in its status, so the values are part of the
/// interface: add new ones at the end, never reuse or renumber one.
enum class Mode : std::uint8_t
{
  /// The arm is stopped, and stays stopped until a person activates the supervisor.
  STOPPED = 0,
  /// A person activated the supervisor: the resume steps are running.
  RESUMING = 1,
  /// The arm may move and every trigger is watched.
  MONITORING = 2,
  /// A trigger fired: the halt steps are running.
  HALTING = 3,
};

/// Why the arm was stopped. Published in the status: same rule as Mode for new values.
enum class Reason : std::uint8_t
{
  /// Not stopped by the supervisor (yet).
  NONE = 0,
  /// Someone called the stop service.
  STOP_SERVICE = 1,
  /// No mission heartbeat for longer than the timeout.
  HEARTBEAT_LOST = 2,
  /// The driver reported a fault: driver/fault_code is not 0.
  DRIVER_FAULT = 3,
  /// No valid driver/fault_code for longer than the timeout: the supervisor cannot see the
  /// driver any more, so it must assume the worst.
  FAULT_DATA_LOST = 4,
  /// A person deactivated the supervisor.
  DEACTIVATED = 5,
  /// A resume step failed or was not answered in time.
  RESUME_FAILED = 6,
};

/// One request the supervisor sends. The enumerator order is the order they are sent in.
enum class Step : std::uint8_t
{
  /// Nothing to send.
  NONE = 0,
  /// Halt 1: set the hardware component inactive. Its on_deactivate sends the stop, so this
  /// goes first: the arm stops as early as possible.
  HARDWARE_INACTIVE = 1,
  /// Halt 2: cancel every goal of the trajectory controller, so nothing resumes an old move.
  CANCEL_GOALS = 2,
  /// Halt 3: deactivate the controllers that command the arm.
  CONTROLLERS_OFF = 3,
  /// Resume 1: set the hardware component active. Before the controllers: every command then
  /// starts at the measured pose.
  HARDWARE_ACTIVE = 4,
  /// Resume 2: activate the controllers.
  CONTROLLERS_ON = 5,
};

/// What the node must send next. The ticket names this request: the node hands it back with
/// the answer, and an answer with an old ticket is ignored.
struct Order
{
  Step step{Step::NONE};
  std::uint64_t ticket{0};
};

/// The decisions of the safety supervisor (#45), with no ROS in them: which request goes out
/// next, and what state the supervisor is in. The node does the talking (topics, services,
/// timers) and asks this class what to do, so every rule below is a unit test.
///
/// - One halt path for every trigger: HARDWARE_INACTIVE, CANCEL_GOALS, CONTROLLERS_OFF.
/// - One resume path: HARDWARE_ACTIVE, CONTROLLERS_ON.
/// - One request in flight at a time. The next one leaves when the node reports the answer,
///   or the lack of one, with step_done().
/// - A halt never stops half way: a step that failed does not hold back the next one.
/// - A resume step that failed turns the resume into a halt.
/// - Only resume() leaves STOPPED: the arm stays stopped until a person asks.
///
/// Not thread-safe on purpose: the supervisor runs every callback on one thread.
class SupervisorLogic
{
public:
  /// A person activated the supervisor. Starts the resume and returns its first step. Refused
  /// (an order with Step::NONE) unless the mode is STOPPED.
  [[nodiscard]] Order resume()
  {
    if (mode_ != Mode::STOPPED) {
      return {};
    }
    mode_ = Mode::RESUMING;
    reason_ = Reason::NONE;
    hardware_stopped_ = false;
    return start(Step::HARDWARE_ACTIVE);
  }

  /// A trigger fired. Starts the halt and returns its first step, from any mode: a resume in
  /// progress is dropped, and a stop asked while STOPPED runs the halt again. Ignored (an order
  /// with Step::NONE) while a halt is already running, so the first reason is the one kept.
  [[nodiscard]] Order halt(Reason reason)
  {
    if (mode_ == Mode::HALTING) {
      return {};
    }
    mode_ = Mode::HALTING;
    reason_ = reason;
    hardware_stopped_ = false;
    return start(Step::HARDWARE_INACTIVE);
  }

  /// The request `ticket` was answered: `ok` is false when it failed or timed out. Returns the
  /// next step, or Step::NONE when the sequence is over (see mode()) or when the ticket is not
  /// the one in flight (an answer that came after its timeout, or after a halt replaced it).
  [[nodiscard]] Order step_done(std::uint64_t ticket, bool ok)
  {
    if (!waiting_for(ticket)) {
      return {};
    }
    if (mode_ == Mode::RESUMING && !ok) {
      return halt(Reason::RESUME_FAILED);
    }
    switch (step_) {
      case Step::HARDWARE_INACTIVE:
        hardware_stopped_ = ok;
        return start(Step::CANCEL_GOALS);
      case Step::CANCEL_GOALS:
        return start(Step::CONTROLLERS_OFF);
      case Step::CONTROLLERS_OFF:
        return finish(Mode::STOPPED);
      case Step::HARDWARE_ACTIVE:
        return start(Step::CONTROLLERS_ON);
      case Step::CONTROLLERS_ON:
        return finish(Mode::MONITORING);
      case Step::NONE:
        break;
    }
    return {};
  }

  /// True when `ticket` names the request in flight. The node asks before it acts on an
  /// answer, so a late one changes nothing and logs nothing.
  [[nodiscard]] bool waiting_for(std::uint64_t ticket) const
  {
    return step_ != Step::NONE && ticket == ticket_;
  }

  [[nodiscard]] Mode mode() const
  {
    return mode_;
  }

  /// Why the last halt started. NONE before the first halt and after a resume().
  [[nodiscard]] Reason reason() const
  {
    return reason_;
  }

  /// The request in flight, Step::NONE between sequences.
  [[nodiscard]] Step step() const
  {
    return step_;
  }

  /// True when the last halt got the hardware component out of ACTIVE (the node decides what
  /// counts: INACTIVE, or UNCONFIGURED after a link fault). False means the supervisor asked
  /// but has no proof that the driver stopped the arm.
  [[nodiscard]] bool hardware_stopped() const
  {
    return hardware_stopped_;
  }

private:
  Order start(Step step)
  {
    step_ = step;
    ++ticket_;
    return {step_, ticket_};
  }

  Order finish(Mode mode)
  {
    mode_ = mode;
    step_ = Step::NONE;
    return {};
  }

  Mode mode_{Mode::STOPPED};
  Reason reason_{Reason::NONE};
  Step step_{Step::NONE};
  /// Counts every request ever started; 0 is "no request", so the first ticket is 1.
  std::uint64_t ticket_{0};
  bool hardware_stopped_{false};
};

/// Names for the log. A value outside the enum gives "UNKNOWN".
[[nodiscard]] constexpr const char * to_string(Mode mode) noexcept
{
  switch (mode) {
    case Mode::STOPPED:
      return "STOPPED";
    case Mode::RESUMING:
      return "RESUMING";
    case Mode::MONITORING:
      return "MONITORING";
    case Mode::HALTING:
      return "HALTING";
  }
  return "UNKNOWN";
}

[[nodiscard]] constexpr const char * to_string(Reason reason) noexcept
{
  switch (reason) {
    case Reason::NONE:
      return "NONE";
    case Reason::STOP_SERVICE:
      return "STOP_SERVICE";
    case Reason::HEARTBEAT_LOST:
      return "HEARTBEAT_LOST";
    case Reason::DRIVER_FAULT:
      return "DRIVER_FAULT";
    case Reason::FAULT_DATA_LOST:
      return "FAULT_DATA_LOST";
    case Reason::DEACTIVATED:
      return "DEACTIVATED";
    case Reason::RESUME_FAILED:
      return "RESUME_FAILED";
  }
  return "UNKNOWN";
}

[[nodiscard]] constexpr const char * to_string(Step step) noexcept
{
  switch (step) {
    case Step::NONE:
      return "NONE";
    case Step::HARDWARE_INACTIVE:
      return "HARDWARE_INACTIVE";
    case Step::CANCEL_GOALS:
      return "CANCEL_GOALS";
    case Step::CONTROLLERS_OFF:
      return "CONTROLLERS_OFF";
    case Step::HARDWARE_ACTIVE:
      return "HARDWARE_ACTIVE";
    case Step::CONTROLLERS_ON:
      return "CONTROLLERS_ON";
  }
  return "UNKNOWN";
}

}  // namespace positronic_safety

#endif  // POSITRONIC_SAFETY__SUPERVISOR_LOGIC_HPP_
