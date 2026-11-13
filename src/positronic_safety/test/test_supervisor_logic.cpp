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
//   Write a GoogleTest suite for supervisor_logic.hpp (issue #45). SupervisorLogic decides
//   which request the safety supervisor sends next and has no ROS in it. Check the decisions
//   the team made: the halt is always hardware inactive, then cancel the goals, then
//   controllers off, whatever the trigger; the resume is hardware active, then controllers on;
//   a halt goes on when a step fails, and hardware_stopped() tells whether the first step
//   succeeded; a second trigger during a halt is ignored and the first reason is kept; the
//   supervisor stays STOPPED until resume(); resume() is refused in any other mode; a resume
//   step that fails turns into a halt with RESUME_FAILED; a halt during a resume drops the
//   resume; an answer with an old ticket, or with nothing in flight, is ignored; tickets never
//   repeat; a stop while STOPPED runs the halt again; every enum value has its own name.
// Follow-up prompt:
//   waiting_for() is true only for the ticket of the request in flight.
//   No cast of an out-of-range value to an enum: clang-tidy (EnumCastOutOfRange) fails the
//   strict CI on it. Close the anonymous namespace before the TEST blocks.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string_view>
#include <vector>

#include "positronic_safety/supervisor_logic.hpp"

namespace
{
using positronic_safety::Mode;
using positronic_safety::Order;
using positronic_safety::Reason;
using positronic_safety::Step;
using positronic_safety::SupervisorLogic;
using positronic_safety::to_string;

const std::vector<Step> HALT_STEPS{
  Step::HARDWARE_INACTIVE, Step::CANCEL_GOALS, Step::CONTROLLERS_OFF};
const std::vector<Step> RESUME_STEPS{Step::HARDWARE_ACTIVE, Step::CONTROLLERS_ON};

/// Answers `order` and every order after it with `ok`, until the logic has nothing more to
/// send. Returns the steps in the order they were sent.
std::vector<Step> run_to_end(SupervisorLogic & logic, Order order, bool ok = true)
{
  std::vector<Step> steps;
  while (order.step != Step::NONE) {
    steps.push_back(order.step);
    order = logic.step_done(order.ticket, ok);
  }
  return steps;
}

/// A logic in MONITORING, as after a person activated the supervisor and all went well.
SupervisorLogic monitoring_logic()
{
  SupervisorLogic logic;
  run_to_end(logic, logic.resume());
  return logic;
}
}  // namespace

TEST(SupervisorLogic, StartsStoppedWithNothingInFlight)
{
  const SupervisorLogic logic;
  EXPECT_EQ(logic.mode(), Mode::STOPPED);
  EXPECT_EQ(logic.reason(), Reason::NONE);
  EXPECT_EQ(logic.step(), Step::NONE);
  EXPECT_FALSE(logic.hardware_stopped());
}

TEST(SupervisorLogic, ResumeActivatesTheHardwareBeforeTheControllers)
{
  SupervisorLogic logic;
  const Order first = logic.resume();
  EXPECT_EQ(first.step, Step::HARDWARE_ACTIVE);
  EXPECT_EQ(logic.mode(), Mode::RESUMING);

  EXPECT_EQ(run_to_end(logic, first), RESUME_STEPS);
  EXPECT_EQ(logic.mode(), Mode::MONITORING);
  EXPECT_EQ(logic.step(), Step::NONE);
}

TEST(SupervisorLogic, HaltIsHardwareThenGoalsThenControllers)
{
  SupervisorLogic logic = monitoring_logic();
  const Order first = logic.halt(Reason::HEARTBEAT_LOST);
  EXPECT_EQ(first.step, Step::HARDWARE_INACTIVE);
  EXPECT_EQ(logic.mode(), Mode::HALTING);

  EXPECT_EQ(run_to_end(logic, first), HALT_STEPS);
  EXPECT_EQ(logic.mode(), Mode::STOPPED);
  EXPECT_EQ(logic.reason(), Reason::HEARTBEAT_LOST);
  EXPECT_EQ(logic.step(), Step::NONE);
  EXPECT_TRUE(logic.hardware_stopped());
}

TEST(SupervisorLogic, EveryTriggerTakesTheSameHaltPath)
{
  const std::array<Reason, 6> triggers{
    Reason::STOP_SERVICE, Reason::HEARTBEAT_LOST, Reason::DRIVER_FAULT,
    Reason::FAULT_DATA_LOST, Reason::DEACTIVATED, Reason::RESUME_FAILED};
  for (const Reason trigger : triggers) {
    SupervisorLogic logic = monitoring_logic();
    EXPECT_EQ(run_to_end(logic, logic.halt(trigger)), HALT_STEPS) << to_string(trigger);
    EXPECT_EQ(logic.mode(), Mode::STOPPED) << to_string(trigger);
    EXPECT_EQ(logic.reason(), trigger);
  }
}

TEST(SupervisorLogic, HaltGoesOnWhenEveryStepFails)
{
  // A controller manager that does not answer must not keep the later steps from being tried.
  SupervisorLogic logic = monitoring_logic();
  EXPECT_EQ(run_to_end(logic, logic.halt(Reason::DRIVER_FAULT), false), HALT_STEPS);
  EXPECT_EQ(logic.mode(), Mode::STOPPED);
  EXPECT_FALSE(logic.hardware_stopped());
}

TEST(SupervisorLogic, HardwareStoppedFollowsTheFirstHaltStepOnly)
{
  SupervisorLogic stopped = monitoring_logic();
  Order order = stopped.halt(Reason::STOP_SERVICE);
  order = stopped.step_done(order.ticket, true);    // hardware inactive: done
  order = stopped.step_done(order.ticket, false);   // cancel goals: failed
  order = stopped.step_done(order.ticket, false);   // controllers off: failed
  EXPECT_EQ(order.step, Step::NONE);
  EXPECT_TRUE(stopped.hardware_stopped());

  SupervisorLogic unknown = monitoring_logic();
  order = unknown.halt(Reason::STOP_SERVICE);
  order = unknown.step_done(order.ticket, false);   // hardware inactive: failed
  order = unknown.step_done(order.ticket, true);
  order = unknown.step_done(order.ticket, true);
  EXPECT_EQ(order.step, Step::NONE);
  EXPECT_FALSE(unknown.hardware_stopped());
}

TEST(SupervisorLogic, SecondTriggerDuringAHaltIsIgnored)
{
  SupervisorLogic logic = monitoring_logic();
  const Order first = logic.halt(Reason::DRIVER_FAULT);
  // The heartbeat is lost too while the halt runs: nothing new to send, first reason kept.
  EXPECT_EQ(logic.halt(Reason::HEARTBEAT_LOST).step, Step::NONE);
  EXPECT_EQ(logic.reason(), Reason::DRIVER_FAULT);
  // The halt in flight is untouched: its first answer is still accepted.
  EXPECT_EQ(run_to_end(logic, first), HALT_STEPS);
  EXPECT_EQ(logic.reason(), Reason::DRIVER_FAULT);
}

TEST(SupervisorLogic, StaysStoppedUntilAPersonResumes)
{
  SupervisorLogic logic = monitoring_logic();
  const Order first = logic.halt(Reason::HEARTBEAT_LOST);
  run_to_end(logic, first);
  ASSERT_EQ(logic.mode(), Mode::STOPPED);

  // No answer, old or new, brings it back.
  EXPECT_EQ(logic.step_done(first.ticket, true).step, Step::NONE);
  EXPECT_EQ(logic.step_done(first.ticket + 10, true).step, Step::NONE);
  EXPECT_EQ(logic.mode(), Mode::STOPPED);

  // Only resume() does, and it clears what the halt left.
  EXPECT_EQ(logic.resume().step, Step::HARDWARE_ACTIVE);
  EXPECT_EQ(logic.mode(), Mode::RESUMING);
  EXPECT_EQ(logic.reason(), Reason::NONE);
  EXPECT_FALSE(logic.hardware_stopped());
}

TEST(SupervisorLogic, ResumeIsRefusedUnlessStopped)
{
  SupervisorLogic resuming;
  const Order in_flight = resuming.resume();
  EXPECT_EQ(resuming.resume().step, Step::NONE);
  EXPECT_EQ(resuming.step(), in_flight.step);  // the resume in flight is untouched

  SupervisorLogic monitoring = monitoring_logic();
  EXPECT_EQ(monitoring.resume().step, Step::NONE);
  EXPECT_EQ(monitoring.mode(), Mode::MONITORING);

  SupervisorLogic halting = monitoring_logic();
  const Order halt = halting.halt(Reason::STOP_SERVICE);
  EXPECT_EQ(halting.resume().step, Step::NONE);
  EXPECT_EQ(halting.mode(), Mode::HALTING);
  EXPECT_EQ(halting.step(), halt.step);
}

TEST(SupervisorLogic, FailedHardwareActivationBecomesAHalt)
{
  // The driver refused to activate (a silent link, an arm outside its limits): the
  // controllers are never activated, and the full halt runs.
  SupervisorLogic logic;
  const std::vector<Step> expected{
    Step::HARDWARE_ACTIVE, Step::HARDWARE_INACTIVE, Step::CANCEL_GOALS, Step::CONTROLLERS_OFF};
  EXPECT_EQ(run_to_end(logic, logic.resume(), false), expected);
  EXPECT_EQ(logic.mode(), Mode::STOPPED);
  EXPECT_EQ(logic.reason(), Reason::RESUME_FAILED);
}

TEST(SupervisorLogic, FailedControllerActivationBecomesAHalt)
{
  SupervisorLogic logic;
  Order order = logic.resume();
  order = logic.step_done(order.ticket, true);    // hardware active: done
  ASSERT_EQ(order.step, Step::CONTROLLERS_ON);
  order = logic.step_done(order.ticket, false);   // controllers on: failed
  EXPECT_EQ(order.step, Step::HARDWARE_INACTIVE);
  EXPECT_EQ(logic.mode(), Mode::HALTING);
  EXPECT_EQ(logic.reason(), Reason::RESUME_FAILED);

  EXPECT_EQ(run_to_end(logic, order), HALT_STEPS);
  EXPECT_EQ(logic.mode(), Mode::STOPPED);
  EXPECT_TRUE(logic.hardware_stopped());
}

TEST(SupervisorLogic, HaltDuringAResumeDropsTheResume)
{
  SupervisorLogic logic;
  const Order resume = logic.resume();
  const Order halt = logic.halt(Reason::STOP_SERVICE);
  EXPECT_EQ(halt.step, Step::HARDWARE_INACTIVE);
  EXPECT_NE(halt.ticket, resume.ticket);

  // The answer to the dropped resume step arrives late: ignored, the controllers stay off.
  EXPECT_EQ(logic.step_done(resume.ticket, true).step, Step::NONE);
  EXPECT_EQ(logic.mode(), Mode::HALTING);
  EXPECT_EQ(logic.step(), Step::HARDWARE_INACTIVE);

  EXPECT_EQ(run_to_end(logic, halt), HALT_STEPS);
  EXPECT_EQ(logic.mode(), Mode::STOPPED);
  EXPECT_EQ(logic.reason(), Reason::STOP_SERVICE);
}

TEST(SupervisorLogic, AnswerAfterItsTimeoutIsIgnored)
{
  SupervisorLogic logic = monitoring_logic();
  const Order first = logic.halt(Reason::STOP_SERVICE);
  // The node gives up on the first step (timeout) and the halt moves on.
  const Order second = logic.step_done(first.ticket, false);
  ASSERT_EQ(second.step, Step::CANCEL_GOALS);

  // Then the late answer comes: it must neither skip a step nor rewrite the result.
  EXPECT_EQ(logic.step_done(first.ticket, true).step, Step::NONE);
  EXPECT_EQ(logic.step(), Step::CANCEL_GOALS);
  EXPECT_FALSE(logic.hardware_stopped());
}

TEST(SupervisorLogic, AnswerWithNothingInFlightIsIgnored)
{
  SupervisorLogic logic;
  EXPECT_EQ(logic.step_done(0, true).step, Step::NONE);
  EXPECT_EQ(logic.step_done(1, true).step, Step::NONE);
  EXPECT_EQ(logic.mode(), Mode::STOPPED);
  EXPECT_EQ(logic.step(), Step::NONE);
}

TEST(SupervisorLogic, WaitsOnlyForTheRequestInFlight)
{
  SupervisorLogic logic;
  EXPECT_FALSE(logic.waiting_for(0));  // nothing in flight yet
  EXPECT_FALSE(logic.waiting_for(1));

  const Order first = logic.resume();
  EXPECT_TRUE(logic.waiting_for(first.ticket));
  EXPECT_FALSE(logic.waiting_for(first.ticket + 1));

  const Order second = logic.step_done(first.ticket, true);
  EXPECT_FALSE(logic.waiting_for(first.ticket));  // answered: now an old ticket
  EXPECT_TRUE(logic.waiting_for(second.ticket));

  EXPECT_EQ(logic.step_done(second.ticket, true).step, Step::NONE);
  EXPECT_FALSE(logic.waiting_for(second.ticket));  // the sequence is over
}

TEST(SupervisorLogic, TicketsNeverRepeat)
{
  SupervisorLogic logic;
  std::vector<std::uint64_t> tickets;
  const auto run = [&logic, &tickets](Order order) {
      while (order.step != Step::NONE) {
        tickets.push_back(order.ticket);
        order = logic.step_done(order.ticket, true);
      }
    };
  run(logic.resume());
  run(logic.halt(Reason::STOP_SERVICE));
  run(logic.resume());

  ASSERT_EQ(tickets.size(), 7U);  // 2 resume + 3 halt + 2 resume
  EXPECT_EQ(tickets.front(), 1U);  // 0 means "no request"
  for (std::size_t i = 1; i < tickets.size(); ++i) {
    EXPECT_GT(tickets[i], tickets[i - 1]);
  }
}

TEST(SupervisorLogic, StopWhileStoppedRunsTheHaltAgain)
{
  // Someone may have activated the hardware by hand behind the supervisor: a stop request is
  // never answered with "already stopped".
  SupervisorLogic logic;
  ASSERT_EQ(logic.mode(), Mode::STOPPED);
  EXPECT_EQ(run_to_end(logic, logic.halt(Reason::STOP_SERVICE)), HALT_STEPS);
  EXPECT_EQ(logic.mode(), Mode::STOPPED);
  EXPECT_EQ(logic.reason(), Reason::STOP_SERVICE);
  EXPECT_TRUE(logic.hardware_stopped());
}

TEST(SupervisorLogic, EveryValueHasItsOwnName)
{
  const std::set<std::string_view> modes{
    to_string(Mode::STOPPED), to_string(Mode::RESUMING), to_string(Mode::MONITORING),
    to_string(Mode::HALTING)};
  EXPECT_EQ(modes.size(), 4U);
  EXPECT_EQ(modes.count("UNKNOWN"), 0U);

  const std::set<std::string_view> reasons{
    to_string(Reason::NONE), to_string(Reason::STOP_SERVICE), to_string(Reason::HEARTBEAT_LOST),
    to_string(Reason::DRIVER_FAULT), to_string(Reason::FAULT_DATA_LOST),
    to_string(Reason::DEACTIVATED), to_string(Reason::RESUME_FAILED)};
  EXPECT_EQ(reasons.size(), 7U);
  EXPECT_EQ(reasons.count("UNKNOWN"), 0U);

  const std::set<std::string_view> steps{
    to_string(Step::NONE), to_string(Step::HARDWARE_INACTIVE), to_string(Step::CANCEL_GOALS),
    to_string(Step::CONTROLLERS_OFF), to_string(Step::HARDWARE_ACTIVE),
    to_string(Step::CONTROLLERS_ON)};
  EXPECT_EQ(steps.size(), 6U);
  EXPECT_EQ(steps.count("UNKNOWN"), 0U);
}
