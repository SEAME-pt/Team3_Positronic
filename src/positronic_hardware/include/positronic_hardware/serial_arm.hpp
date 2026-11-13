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

#ifndef POSITRONIC_HARDWARE__SERIAL_ARM_HPP_
#define POSITRONIC_HARDWARE__SERIAL_ARM_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/link_watchdog.hpp"
#include "positronic_hardware/serial_transport.hpp"
#include "positronic_hardware/servo_temperature.hpp"
#include "positronic_protocol/types.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace positronic_hardware
{

/// The two timings of a serial-thread turn that sends a command. The defaults are the measured
/// values; the plugin lets the URDF override them (moving_period_ms, send_settle_ms) so a sweep
/// on the arm needs no rebuild. A namespace-level struct so it can be a default argument.
struct SerialTiming
{
  /// One turn while the arm moves: a get_angles and a send_angles. The arm answers in 8 ms
  /// (20 ms worst seen), a send_angles costs under 1 ms and the settle 30 ms, so 50 ms leaves
  /// 11 ms of slack and the next get_angles comes 42 ms after the send. 50 held five 2-minute
  /// runs with zero stalls; 40 had one stall in five (the arm still busy 31 ms after a send).
  /// A worst-case reply overruns the turn by 1 ms and the next turn starts at once.
  std::chrono::milliseconds moving_period{50};

  /// How long the thread waits after a send_angles before the next get_angles. Measured on
  /// the arm with turns just above the settle: 30 ms gave 0 stalls in 4464 requests, 25 ms
  /// gave 43, 20 ms gave 110 and once froze the arm for 20 s.
  std::chrono::milliseconds send_settle{30};
};

/// How the serial thread watches the servo temperatures. The plugin sets the limit from the URDF
/// (servo_temp_limit_c); the times are fixed, and only the tests shorten them.
struct TemperatureGuard
{
  /// A servo at or above this, READINGS_TO_TRIP readings in a row, cuts the arm. degC.
  int limit_c{SERVO_TEMP_LIMIT_C};

  /// How often the thread asks for the temperatures (get_servo_temps), on a turn that sends no
  /// target. A temperature climbs over minutes; once a second costs ~1 % of the line.
  std::chrono::milliseconds period{1000};

  /// A reading older than this is taken even on a turn that sends a target, before the send,
  /// so a long motion cannot hide a hot servo. 47 of the 50 ms of a moving turn.
  std::chrono::milliseconds max_age{3000};

  /// No plausible reading for this long and temperature_stale() turns true: the guard is blind.
  std::chrono::milliseconds stale_timeout{10000};
};

/// The real arm behind the ArmBackend interface (#22).
///
/// A dedicated thread owns the serial port: it asks the arm for its angles (get_angles), decodes
/// the reply and keeps the newest pose in a lock-free buffer. read_angles(), called from the
/// control loop, only copies that buffer out, so the loop never blocks on the port. This is the
/// "serial I/O in a dedicated thread owning the port, latest sample through a RealtimeBuffer, no
/// blocking call in the control loop" that #22 asks for, with the actual device work of #23.
class SerialArm : public ArmBackend
{
public:
  using Clock = LinkWatchdog::Clock;
  using Timing = SerialTiming;

  /// One turn of the thread while the arm stands still: a get_angles and nothing else. The link
  /// is 1 Mbaud and a reply is ~17 bytes plus servo-bus noise; 100 Hz keeps the pose fresh
  /// without flooding the bus (#20: 10 minutes of reads at this rate, no stall).
  static constexpr std::chrono::milliseconds POLL_PERIOD{10};

  /// Speed byte of every send_angles, percent of the firmware maximum (#32: 30 % on hardware).
  static constexpr std::uint8_t SEND_SPEED{30};

  /// How long a turn waits for the get_angles reply. A reply takes ~8 ms and arrives in a dozen
  /// small chunks behind servo-bus chatter, so the turn reads until the frame completes, not a
  /// fixed number of chunks. Nothing else goes on the wire before that: a send_angles that
  /// follows a get_angles request before its reply is dropped by the arm.
  static constexpr std::chrono::milliseconds REPLY_TIMEOUT{50};

  /// How long the thread sleeps after a read that returned no bytes, before it reads again.
  /// The port returns at once (VTIME = 0), so without it the thread would spin while it waits
  /// for a reply. 2 ms adds at most 2 ms to a reply that takes ~8 ms, at a few hundred cheap
  /// reads per second on the Jetson.
  static constexpr std::chrono::milliseconds EMPTY_READ_BACKOFF{2};

  // REQUEST_TIMEOUT, from the contract, covers the worst turn a request can land in, with a
  // margin: a silent arm holds a turn for REPLY_TIMEOUT (~50 ms, the reads return at once). An
  // arm that answers get_angles (up to 20 ms) but not the temperature request holds it for that
  // plus TEMPERATURE_REPLY_TIMEOUT, ~70 ms. The request may land just after a turn has started,
  // so the frame is out within ~120 ms, and no temperature request is written while one is
  // pending.

  /// How long a temperature request waits for its reply, like REPLY_TIMEOUT for get_angles.
  static constexpr std::chrono::milliseconds TEMPERATURE_REPLY_TIMEOUT{50};

  /// Opens `port` at `baud_rate` with a real POSIX serial port and starts polling.
  /// Throws std::runtime_error if the port cannot be opened (the plugin's on_configure catches it).
  SerialArm(
    const std::string & port, int baud_rate, Timing timing = {}, TemperatureGuard guard = {});

  /// Takes an already-built transport. Used by the tests with a fake that replays recorded bytes.
  explicit SerialArm(
    std::unique_ptr<SerialTransport> transport, Timing timing = {}, TemperatureGuard guard = {});

  ~SerialArm() override;

  SerialArm(const SerialArm &) = delete;
  SerialArm & operator=(const SerialArm &) = delete;
  SerialArm(SerialArm &&) = delete;
  SerialArm & operator=(SerialArm &&) = delete;

  /// The newest pose the thread has decoded, in radians. Non-blocking; safe from the control
  /// loop. Before the first reply arrives it returns START_POSE, never NaN.
  [[nodiscard]] JointPositions read_angles() const override;

  /// The target pose the thread sends next, in radians. Safe from the control loop: it only
  /// stores the newest command, the poll thread sends it over the port on its next turn. It can
  /// wait 0.5 ms, once in a long while, when it meets the poll thread swapping the buffer.
  void write_angles(const JointPositions & angles) override;

  /// True once the thread has decoded a first reply from the arm.
  [[nodiscard]] bool has_sample() const override;

  /// Holds every target, then asks the thread to send a stop at the start of its next turn and
  /// waits for it, up to REQUEST_TIMEOUT.
  [[nodiscard]] bool halt() override;

  /// halt() without the wait: holds every target and leaves the stop for the start of the
  /// thread's next turn. Two atomic stores, safe from the control loop.
  void request_halt() override;

  /// True when the poll thread has decoded no valid reply for longer than
  /// LinkWatchdog::DEFAULT_TIMEOUT. One atomic load, safe from the control loop.
  [[nodiscard]] bool link_timed_out() const override;

  /// Drops the target held since halt(), then asks the thread to send a power_on at the start
  /// of its next turn and waits for it, up to REQUEST_TIMEOUT. The thread releases the hold in
  /// the turn that writes the frame, so targets go out again only once the power on is on the
  /// wire. If the frame never made it, the hold stays. After an overheat cut it returns false
  /// at once and sends nothing: the servos stay off.
  [[nodiscard]] bool power_on() override;

  /// The newest plausible reading of get_servo_temps, or nullopt before the first one.
  [[nodiscard]] std::optional<ServoTemperatures> read_temperatures() const override;

  /// Set by the thread when it cut the arm: stop, then power off, sent by the thread itself in
  /// the turn that completed the trip. The power off goes out again every guard period after
  /// that, in case the arm dropped it or was powered on again without this port.
  [[nodiscard]] std::optional<Overheat> overheat() const override;

  /// True when no plausible temperature has arrived for TemperatureGuard::stale_timeout.
  [[nodiscard]] bool temperature_stale() const override;

private:
  static constexpr JointPositions START_POSE{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  enum class Request : std::uint8_t
  {
    NONE,
    HALT,
    POWER_ON
  };
  /// The stop or power_on frame for `request`, or nullopt for NONE.
  static std::optional<positronic_protocol::Frame> encode_request(Request request);
  /// Sends the pending request, if any, at the start of a turn, and applies what it means for
  /// the thread: a stop clears `last_sent`, a power on ends the hold.
  void send_pending_request(std::optional<JointPositions> & last_sent);

  void poll_loop();

  /// The send_angles frame for `target`, or nullopt when an angle is NaN or out of range.
  static std::optional<positronic_protocol::Frame> encode_command(const JointPositions & target);

  bool request_and_wait(Request request);

  /// True once the thread has cut the arm for an overheat.
  [[nodiscard]] bool overheated() const;

  /// Writes a get_servo_temps request and reads until its reply completes or
  /// TEMPERATURE_REPLY_TIMEOUT passes. Only when the arm's input is free: after the get_angles
  /// reply of a turn, with nothing else on the wire.
  std::optional<ServoTemperatures> ask_temperatures();

  /// The guard's part of a turn whose get_angles was answered: asks for the temperatures when
  /// they are due and allowed (see poll_loop), publishes a plausible reading, and cuts the arm
  /// when the detector trips. `new_target` says whether this turn is about to send one.
  void watch_temperatures(bool new_target, std::optional<JointPositions> & last_sent);

  /// After a cut, sends the power off again once a period, at the start of a turn.
  void repeat_power_off(Clock::time_point turn_start);

  /// The cut: holds every target, sends a stop and, after the settle, a power off. Then records
  /// `trip`, so the plugin sees it with the arm already cut. True when the power off was
  /// written.
  bool cut_power(const Overheat & trip, std::optional<JointPositions> & last_sent);

  std::unique_ptr<SerialTransport> transport_;
  const Timing timing_;
  const TemperatureGuard guard_;
  // Time since the last valid reply (#35). The poll thread feeds it; the plugin asks it.
  LinkWatchdog watchdog_;
  // readFromRT() is non-const but read_angles() is logically const (it observes state), so the
  // buffer is mutable. writeFromNonRT() runs on the poll thread, readFromRT() on the control loop.
  mutable realtime_tools::RealtimeBuffer<JointPositions> latest_{START_POSE};
  // The newest command from write_angles(). The control loop writes it, the poll thread reads it.
  // nullopt until the first command: a readFromRT() that meets a write in progress keeps its old
  // box, and before the first swap that box must hold "no target", never a pose.
  realtime_tools::RealtimeBuffer<std::optional<JointPositions>> command_{std::nullopt};
  std::atomic<bool> stop_{false};
  std::atomic<bool> has_sample_{false};
  // The frame the thread sends at the start of its next turn, before the get_angles. halt() and
  // power_on() set it from a lifecycle transition; the thread puts it back to NONE once the
  // frame is on the wire, and that is what the callers wait for.
  std::atomic<Request> request_{Request::NONE};
  // True from halt() until the thread has written the power on: it sends no target meanwhile,
  // so a command stored before the stop, or one that arrives while inactive, cannot go out after
  // it. halt() sets it before request_, so the turn that sends the stop already sees the hold;
  // the thread clears it in the turn that sends the power on, so no target gets ahead of that.
  std::atomic<bool> halted_{false};
  // The newest plausible temperatures. Written by the poll thread once a period, read by the
  // plugin (on_activate). nullopt until the first plausible reading.
  mutable realtime_tools::RealtimeBuffer<std::optional<ServoTemperatures>> temperatures_{
    std::nullopt};
  // Time since the last plausible temperature, the way watchdog_ counts the last pose.
  LinkWatchdog temperature_watchdog_;
  // The overheat cut: -1 until the thread cuts the arm, then the joint (0 = J1). The reading is
  // stored first, so whoever sees the joint also finds its value.
  std::atomic<int> overheat_joint_{-1};
  std::atomic<int> overheat_celsius_{0};
  // The guard's bookkeeping, used by the poll thread only.
  OverheatDetector detector_;
  Clock::time_point last_temperature_ask_;
  Clock::time_point next_power_off_;
  std::vector<std::uint8_t> temperature_buffer_;
  std::thread thread_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__SERIAL_ARM_HPP_
