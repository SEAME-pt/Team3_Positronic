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

#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/link_watchdog.hpp"
#include "positronic_hardware/serial_transport.hpp"
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

  /// Wait for the arm's ack (`FE FE 03 <cmd> 01 FA`) of a stop or power_on. On the bench the
  /// ack comes in 1.8 ms, 3.6 ms at p99, once 58 ms. 80 ms covers that plus the 2 ms read
  /// backoff, and keeps a request turn under the 200 ms link watchdog (~130 ms of silence at
  /// worst). No ack by then: the frame goes out again next turn (#103).
  static constexpr std::chrono::milliseconds ACK_TIMEOUT{80};

  // REQUEST_TIMEOUT (300 ms, from the contract) covers the worst case: a turn already running
  // with a late arm (~50 ms), its send and settle (30 ms), then the ack (up to 80 ms), so an
  // accepted request is acknowledged within ~170 ms, and one lost frame still fits a retry.

  /// Opens `port` at `baud_rate` with a real POSIX serial port and starts polling.
  /// Throws std::runtime_error if the port cannot be opened (the plugin's on_configure catches it).
  SerialArm(const std::string & port, int baud_rate, Timing timing = {});

  /// Takes an already-built transport. Used by the tests with a fake that replays recorded bytes.
  explicit SerialArm(std::unique_ptr<SerialTransport> transport, Timing timing = {});

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
  /// power_on is dropped, and the arm must not be treated as live (#103).
  [[nodiscard]] bool power_on() override;

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
  /// Sends the pending request, if any, at the start of a turn, waits for its ack, and applies
  /// what it means for the thread: a stop clears `last_sent`, a power on ends the hold. No ack
  /// leaves the request pending for the next turn.
  void send_pending_request(std::optional<JointPositions> & last_sent);

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

  /// read_until() for the arm's ack of `command`, with ACK_TIMEOUT from now. Bytes that are not
  /// the ack (servo-bus echo, a late pose) are dropped. True when the ack came with status 1.
  bool await_ack(positronic_protocol::CommandId command);

  void poll_loop();

  /// The send_angles frame for `target`, or nullopt when an angle is NaN or out of range.
  static std::optional<positronic_protocol::Frame> encode_command(const JointPositions & target);

  bool request_and_wait(Request request);

  std::unique_ptr<SerialTransport> transport_;
  const Timing timing_;
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
  std::thread thread_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__SERIAL_ARM_HPP_
