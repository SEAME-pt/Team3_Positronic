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

#include "positronic_hardware/arm_backend.hpp"
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

  /// How long halt() and power_on() wait for the thread to put their frame on the wire. Two
  /// turns of the worst case: a silent arm holds a turn for REPLY_TIMEOUT plus one VTIME read
  /// (0.1 s), ~150 ms, and the request may land just after a turn has started.
  static constexpr std::chrono::milliseconds REQUEST_TIMEOUT{300};

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

  /// Holds every target, then asks the thread to send a stop at the start of its next turn and
  /// waits for it, up to REQUEST_TIMEOUT.
  [[nodiscard]] bool halt() override;

  /// Drops the target held since halt(), lets targets go out again, then asks the thread to send
  /// a power_on at the start of its next turn and waits for it, up to REQUEST_TIMEOUT. The hold
  /// is released whether or not the frame made it.
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
  /// Sends the pending request, if any, at the start of a turn. True when it was a stop.
  bool send_pending_request();

  void poll_loop();

  /// The send_angles frame for `target`, or nullopt when an angle is NaN or out of range.
  static std::optional<positronic_protocol::Frame> encode_command(const JointPositions & target);

  bool request_and_wait(Request request);

  std::unique_ptr<SerialTransport> transport_;
  const Timing timing_;
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
  // True from halt() until power_on(): the thread sends no target meanwhile, so a command stored
  // before the stop, or one that arrives while inactive, cannot go out after it. Set before
  // request_, so the turn that sends the stop already sees the hold.
  std::atomic<bool> halted_{false};
  std::thread thread_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__SERIAL_ARM_HPP_
