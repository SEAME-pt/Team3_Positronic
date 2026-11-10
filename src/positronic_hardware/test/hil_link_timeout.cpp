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
//   #35 hardware check. On the Jetson, drive SerialArm over the real port through a transport
//   that can drop every reply. Per trial: move J1 by 5 degrees, drop the replies mid-motion,
//   and log the time from the last bytes delivered to link_timed_out(); then let replies
//   through again and log where J1 was at the cut and where it ended, against the target. Run
//   by hand, never in CI.
//
// MOVES THE REAL ARM. Two people, bench clear, a hand on the power switch. Stop the driver
// first: the port is exclusive. Run in the arm container: make nano-shell, then
//   ./build/positronic_hardware/hil_link_timeout
// Safe state at the end: J1 back at its start angle, stop sent, servos powered and holding.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

#include "positronic_hardware/serial_arm.hpp"
#include "positronic_hardware/serial_transport.hpp"

namespace
{
using positronic_hardware::JointPositions;
using positronic_hardware::PosixSerialPort;
using positronic_hardware::SerialArm;
using positronic_hardware::SerialTransport;
using Clock = SerialArm::Clock;

constexpr const char * PORT{"/dev/ttyTHS1"};
constexpr int BAUD{1000000};
constexpr int TRIALS{6};
constexpr double STEP_RAD{5.0 * 3.14159265358979323846 / 180.0};
constexpr double RAD_TO_DEG{180.0 / 3.14159265358979323846};
/// The arm is still moving this long after the 5 degree target goes out (30 % speed).
constexpr std::chrono::milliseconds MID_MOTION{150};
constexpr std::chrono::milliseconds GIVE_UP{1000};

/// Passes everything through, except that while dropping it swallows every received byte: the
/// arm still answers, the driver just never sees it, like a line that loses the replies.
class DroppingTransport : public SerialTransport
{
public:
  explicit DroppingTransport(std::unique_ptr<SerialTransport> inner)
  : inner_{std::move(inner)}
  {
  }

  bool write(const std::uint8_t * data, std::size_t count) override
  {
    return inner_->write(data, count);
  }

  std::size_t read(std::uint8_t * dst, std::size_t max) override
  {
    const std::size_t got{inner_->read(dst, max)};
    if (dropping_.load()) {
      return 0;
    }
    if (got > 0) {
      last_delivered_.store(Clock::now().time_since_epoch().count());
    }
    return got;
  }

  void set_dropping(bool dropping)
  {
    dropping_.store(dropping);
  }

  /// When bytes last reached the driver: the end of the last reply, within a read.
  Clock::time_point last_delivered() const
  {
    return Clock::time_point{Clock::duration{last_delivered_.load()}};
  }

private:
  std::unique_ptr<SerialTransport> inner_;
  std::atomic<bool> dropping_{false};
  std::atomic<Clock::rep> last_delivered_{0};
};

double ms_between(Clock::time_point from, Clock::time_point to)
{
  return std::chrono::duration<double, std::milli>(to - from).count();
}

/// One trial: move, drop the replies mid-motion, time the fault, let replies through, report.
bool run_trial(SerialArm & arm, DroppingTransport & port, int trial)
{
  const JointPositions start{arm.read_angles()};
  JointPositions target{start};
  target.at(0) += (trial % 2 == 0) ? STEP_RAD : -STEP_RAD;
  arm.write_angles(target);
  std::this_thread::sleep_for(MID_MOTION);

  port.set_dropping(true);
  // The newest pose the driver decoded before the cut: tells "finished" from "held" below.
  const double at_cut{arm.read_angles().at(0)};
  const auto deadline{Clock::now() + GIVE_UP};
  while (!arm.link_timed_out() && Clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }
  const auto fault_at{Clock::now()};
  const bool faulted{arm.link_timed_out()};
  // Read before the replies come back: from then on last_delivered() moves again.
  const auto last_reply{port.last_delivered()};
  port.set_dropping(false);

  // Long enough for the arm to finish the 5 degrees and for a fresh reply.
  std::this_thread::sleep_for(std::chrono::milliseconds{1000});
  const JointPositions end{arm.read_angles()};
  std::cout << trial << ',' << (faulted ? ms_between(last_reply, fault_at) : -1.0) << ','
            << target.at(0) * RAD_TO_DEG << ',' << at_cut * RAD_TO_DEG << ','
            << end.at(0) * RAD_TO_DEG << '\n';

  arm.write_angles(start);  // back where it began
  std::this_thread::sleep_for(std::chrono::milliseconds{1000});
  return faulted;
}

int run()
{
  auto owned{std::make_unique<DroppingTransport>(std::make_unique<PosixSerialPort>(PORT, BAUD))};
  DroppingTransport & port{*owned};
  SerialArm arm{std::move(owned)};

  const auto deadline{Clock::now() + GIVE_UP};
  while (!arm.has_sample() && Clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  if (!arm.has_sample() || !arm.power_on()) {
    std::cerr << "The arm did not answer on " << PORT << ".\n";
    return 1;
  }

  std::cout << "trial,ms_last_reply_to_fault,j1_target_deg,j1_at_cut_deg,j1_end_deg\n";
  bool all_faulted{true};
  for (int trial{0}; trial < TRIALS; ++trial) {
    all_faulted = run_trial(arm, port, trial) && all_faulted;
  }
  static_cast<void>(arm.halt());
  return all_faulted ? 0 : 2;
}
}  // namespace

int main()
{
  try {
    return run();
  } catch (const std::exception & error) {
    std::cerr << "hil_link_timeout: " << error.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "hil_link_timeout: unknown error\n";
    return 1;
  }
}
