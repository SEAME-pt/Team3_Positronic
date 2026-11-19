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
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/link_watchdog.hpp"
#include "positronic_hardware/requests.hpp"
#include "positronic_hardware/serial_transport.hpp"
#include "positronic_hardware/servo_temperature.hpp"
#include "positronic_protocol/types.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace positronic_hardware
{

/// The timings of a serial-thread turn that sends a command. The defaults are the measured
/// values; the plugin lets the URDF override two of them (moving_period_ms, send_settle_ms) so a
/// sweep on the arm needs no rebuild. A namespace-level struct so it can be a default argument.
struct SerialTiming
{
  /// One turn while the arm moves: a get_angles, a send_angles and the wait for the Atom's
  /// sync-write (#150). In the 10 minutes at 33 ms of 19 Nov 2026 the reply came 7.8 ms after
  /// the request (p99 9.4) and the sync-write ended 19.2 ms after the send (p99 28.0), so a
  /// turn takes about 27 ms and 33 ms is one control period at 30 Hz: 17216 sends, 0 stalls.
  /// A slow turn overruns and the next one starts at once. A shorter period gains nothing at
  /// 30 Hz: the controller has no new target for every turn.
  std::chrono::milliseconds moving_period{33};

  /// The fixed wait after a frame that leaves no sign of when the arm is free again: between the
  /// stop and the power off of the overheat cut. Measured on the arm as the old wait after a
  /// send_angles: 30 ms gave 0 stalls in 4464 requests, 25 ms gave 43, 20 ms gave 110 and once
  /// froze the arm for 20 s.
  std::chrono::milliseconds send_settle{30};

  /// The longest the thread waits after a send_angles for the Atom's sync-write to the servos,
  /// which ends the wait (#150): 20 ms after the send at p50, 27 ms at p99 in the tap of 13 Nov
  /// 2026, 19.2 and 28.0 ms in the 10 minutes of 19 Nov. It may be longer than moving_period,
  /// as it only counts when the Atom is late: on 19 Nov two sync-writes came 32.7 and 54 ms
  /// after their send, past the old 30 ms, and the get_angles sent at 30 ms was lost both
  /// times. The 54 ms is an outlier, the only sync-write above 50 ms in normal operation that
  /// day, but the bound must cover it. With 60, 10 minutes lost no reply.
  std::chrono::milliseconds echo_timeout{60};
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

  // REQUEST_TIMEOUT (300 ms, from the contract) covers the worst case: a turn already running
  // with a late arm (~50 ms for get_angles, ~70 ms when its temperature request goes unanswered
  // too), its send and the wait for the sync-write (~20 ms, up to echo_timeout, 60 ms, when the
  // Atom is late, #150), then the ack (up to 80 ms), so an accepted request is acknowledged
  // within ~170 ms, ~210 ms with a late sync-write. One lost frame still fits a retry while the
  // sync-write comes in time; with a late one as well, the retry can pass the 300 ms. No
  // temperature request is written while a stop or power_on is pending. ACK_TIMEOUT itself
  // lives in requests.hpp.

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

  /// Holds every target, then has the thread send a stop at its next turn and waits for the
  /// arm's ack, up to REQUEST_TIMEOUT. The thread resends the stop every turn until the ack
  /// comes: false means no ack within the deadline, and the stop still pending (#103).
  [[nodiscard]] bool halt() override;

  /// halt() without the wait: holds every target and leaves the stop for the start of the
  /// thread's next turn. Two atomic stores, safe from the control loop.
  void request_halt() override;

  /// True when the poll thread has decoded no valid reply for longer than
  /// LinkWatchdog::DEFAULT_TIMEOUT. One atomic load, safe from the control loop.
  [[nodiscard]] bool link_timed_out() const override;

  /// Drops the target held since halt(), then has the thread send a power_on at its next turn
  /// and waits for the arm's ack, up to REQUEST_TIMEOUT. The hold ends in the turn the ack
  /// comes, so no target goes out before the power on is confirmed. Without the ack the hold
  /// stays. False also when a request_halt() landed during the wait: the hold is back on, the
  /// power_on is dropped, and the arm must not be treated as live (#103). After an overheat
  /// cut it returns false at once and sends nothing: the servos stay off.
  [[nodiscard]] bool power_on() override;

  /// The newest plausible reading of get_servo_temps, or nullopt before the first one.
  [[nodiscard]] std::optional<ServoTemperatures> read_temperatures() const override;

  /// Set by the thread when it cut the arm: stop, then power off, sent by the thread itself in
  /// the turn that completed the trip. The power off goes out again every guard period after
  /// that, in case the arm dropped it or was powered on again without this port.
  [[nodiscard]] std::optional<Overheat> overheat() const override;

  /// True when no plausible temperature has arrived for TemperatureGuard::stale_timeout.
  [[nodiscard]] bool temperature_stale() const override;
  [[nodiscard]] std::optional<JointPositions> last_target() const override;

  /// The sends whose sync-write did not come within SerialTiming::echo_timeout (#150).
  [[nodiscard]] std::uint64_t echo_timeouts() const override;

private:
  static constexpr JointPositions START_POSE{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  /// What halt() and power_on() leave for the thread to send at the start of its next turn.
  enum class Pending : std::uint8_t
  {
    NONE,
    HALT,
    POWER_ON
  };
  /// Sends the pending request, if any, at the start of a turn, waits for its ack, and applies
  /// what it means for the thread: a stop clears the last target, a power on ends the hold. No ack
  /// leaves the request pending for the next turn.
  void send_pending_request();

  /// What a Scanner found in the bytes read so far: `found` ends the read, `consumed` is how
  /// many bytes from the front are noise the scanner has ruled out and the loop may drop.
  struct ScanResult
  {
    bool found{false};
    std::size_t consumed{0};
  };
  using Scanner = std::function<ScanResult(positronic_protocol::ByteView)>;

  /// Reads the port into a local buffer until `scan` reports found or `deadline` passes, with
  /// EMPTY_READ_BACKOFF between empty reads. Serial thread only. Serves the ack today, and any
  /// other frame to wait for later (the Atom's sync-write after a send_angles). True when found.
  bool read_until(Clock::time_point deadline, const Scanner & scan);

  /// After a send_angles: reads until the Atom's sync-write to the servos has passed, the last
  /// frame it sends while it ignores the port (#150), or until echo_timeout when it never comes.
  /// Serial thread only.
  void wait_for_sync_write();

  /// Writes `request` and waits for what it expects (requests.hpp): nothing, the ack of its
  /// command with status 1, or a reply that `reply` recognises, with the request's timeout from
  /// now. Bytes that are not it (servo-bus echo, a late pose) are dropped. Serial thread only.
  /// False when the port refused the frame or the deadline passed: the caller decides whether
  /// the request goes out again.
  bool perform(const Request & request, const Scanner & reply = Scanner{});

  void poll_loop();

  /// The send_angles frame for `target`, or nullopt when an angle is NaN or out of range.
  static std::optional<positronic_protocol::Frame> encode_command(const JointPositions & target);

  bool request_and_wait(Pending request);

  /// True once the thread has cut the arm for an overheat.
  [[nodiscard]] bool overheated() const;

  /// Performs a get_servo_temps request, REQUEST_REPLY_TIMEOUT for the reply. Only when the arm's
  /// input is free: after the get_angles reply of a turn, with nothing else on the wire.
  std::optional<ServoTemperatures> ask_temperatures();

  /// The guard's part of a turn whose get_angles was answered: asks for the temperatures when
  /// they are due and allowed (see poll_loop), publishes a plausible reading, and cuts the arm
  /// when the detector trips. `new_target` says whether this turn is about to send one.
  void watch_temperatures(bool new_target);

  /// After a cut, sends the power off again once a period, at the start of a turn.
  void repeat_power_off(Clock::time_point turn_start);

  /// The cut: holds every target, sends a stop and, after the settle, a power off. Then records
  /// `trip`, so the plugin sees it with the arm already cut. True when the power off was
  /// written.
  bool cut_power(const Overheat & trip);

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
  // The last target the thread put on the wire, for the plugin's unfollowed-command check
  // (#103). The thread writes it where it sends and clears it with an acknowledged stop or a
  // cut; the control loop reads it. Same rule as latest_.
  mutable realtime_tools::RealtimeBuffer<std::optional<JointPositions>> last_target_{std::nullopt};
  /// The thread's own view of last_target_: the one memory of what went out, so an unchanged
  /// command is not sent again and the plugin's check compares with the same value.
  std::optional<JointPositions> last_sent() const;
  std::atomic<bool> stop_{false};
  std::atomic<bool> has_sample_{false};
  // The frame the thread sends at the start of its next turn, before the get_angles. halt() and
  // power_on() set it from a lifecycle transition; the thread puts it back to NONE once the
  // frame is on the wire, and that is what the callers wait for.
  std::atomic<Pending> request_{Pending::NONE};
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
  // Sends whose sync-write did not come within echo_timeout (#150). Written by the poll thread.
  std::atomic<std::uint64_t> echo_timeouts_{0};
  // The guard's bookkeeping, used by the poll thread only.
  OverheatDetector detector_;
  Clock::time_point last_temperature_ask_;
  Clock::time_point next_power_off_;
  // What read_until() has read and not yet ruled out. Reserved once, so the thread does not
  // allocate in a turn.
  std::vector<std::uint8_t> read_buffer_;
  std::thread thread_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__SERIAL_ARM_HPP_
