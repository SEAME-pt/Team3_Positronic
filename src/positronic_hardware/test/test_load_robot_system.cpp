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
//   Write the M2 load test for the positronic_hardware plugin (#22). Load RobotSystem the
//   way the controller manager does: a ResourceManager (ROS 2 Jazzy, ros2_control 4.48,
//   ResourceManagerParams API) given the real URDF from positronic_description plus a
//   <ros2_control> block, so the test fails if pluginlib registration or the joint names
//   drift. Check that on_init succeeds and exactly the six <joint>/position state
//   interfaces are exported, and that on_init rejects five joints and a joint without a
//   position state interface.
// Follow-up prompt:
//   Add tests for the fake arm: positions are NaN after configure and before any read,
//   read() reports FakeArm::START_POSE, and the component activates and still reads.
// Follow-up prompt:
//   Apply the review of #22: each joint has position and velocity state interfaces and no
//   command interface; test twelve exported states, a joint without velocity, a joint with
//   a command interface, and check loaded_ in the rejection tests.
// Follow-up prompt:
//   on_init reads the <hardware> parameters backend, port, baud and velocity_filter_alpha.
//   Every test URDF selects backend=fake; test that valid values load and that an unknown
//   parameter or a bad value is rejected.
// Follow-up prompt:
//   on_configure creates SerialArm or FakeArm from the backend parameter and on_cleanup
//   destroys it. Test that configure fails cleanly with backend=serial on a port that does not
//   exist, and that the component can be cleaned up and configured again.
// Follow-up prompt:
//   The states stay NaN until the arm's first reply, and on_activate fails if no reply comes.
//   Test it on a pseudo-terminal that opens like a serial port but never answers.
// Follow-up prompt (#36):
//   on_deactivate halts the arm and on_error releases it, on_activate powers the servos on, and
//   write() returns early when the component is not active. With the fake arm, check that a
//   command written after deactivate never reaches the arm, that activate resumes commands from
//   the measured pose, and that cleanup still works afterwards. On the silent pseudo-terminal,
//   check that a failed activate leaves the component UNCONFIGURED and configurable again.
// Follow-up prompt (#36):
//   on_deactivate returning ERROR is the safety path of the change and no test reaches it,
//   because the fake arm's halt() always succeeds. Use a pseudo-terminal that answers get_angles
//   so activate succeeds, then close its controller side so every write fails: the stop cannot
//   leave, deactivate must return ERROR and the component must end in UNCONFIGURED.
// Follow-up prompt (#34):
//   #34: the plugin reads the joint limits from the URDF and write() refuses a command past a
//   limit, NaN, and halts. The test joints carry <limits enable="false"/> like the real xacro.
//   Test that: a command exactly at a limit is written; limit + 5 deg (upper side, and the lower
//   side of the asymmetric J5) and NaN make write() return ERROR and take the component out of
//   ACTIVE; after a refusal the component configures and activates again; on_init refuses a
//   joint without a revolute <limit> and swapped limits; on_activate refuses an arm that stands
//   outside its limits.
// Follow-up prompt (#35):
//   The plugin exports driver/fault_code from a <gpio name="driver"> and latches a link timeout
//   there. Test that: the interface is exported; on_init refuses a missing driver gpio, a missing
//   fault_code state and a fault_code command, and accepts extra driver states and other gpios;
//   the code is 0 while the link is alive; on a pseudo-terminal, an arm that stops answering
//   gives fault 1 with the component still ACTIVE, the fault stays latched and no send_angles
//   leaves when replies resume, deactivate and activate clear it and targets go out again; and
//   on_activate refuses a link that went silent.
// Follow-up prompt (servo over-temperature):
//   The plugin reads servo_temp_limit_c (whole degC, 30 to 60, default 60), refuses to activate
//   without a first servo temperature or while a servo is within 5 degC of the limit, and
//   read() returns ERROR once the serial thread has cut an overheated arm (stop, then power
//   off). The pseudo-terminal arm now also answers get_servo_temps (0xE5) with temperatures
//   the test sets, and records what it receives. Test: the parameter is accepted and its bad
//   values rejected; activation is refused on a hot arm and on an arm that never answers 0xE5;
//   and an arm that heats while active is cut (stop then power off on the wire), read()
//   returns ERROR and the component ends in UNCONFIGURED.
// Follow-up prompt (overheat fault):
//   An overheat now also latches driver/fault_code = 3 (FaultCode::OVERHEAT), written before
//   read() returns ERROR. Test, in the same end-to-end run: after the cut and the ERROR, a
//   configure shows fault 3 (it survives on_error), and once the arm reports a cool temperature
//   again, activate clears it to 0. Then the arm the real way: it goes silent after the power
//   off, so by the first read() after the cut the link has timed out too. The fault must still
//   be 3, the cause, not 1, its consequence: read() checks the temperatures before the link.
// Follow-up prompt (#103, part 2):
//   The unfollowed command: a joint further than tracking_error_deg from the last target on the
//   wire for longer than tracking_error_ms latches fault 2 while active. On the pseudo-terminal
//   arm, which answers the same pose whatever it is told: 2 deg away is never a fault, 9 deg for
//   300 ms is, 9 deg for 100 ms and then back is not. A bad parameter value is rejected.
// Follow-up prompt (#150):
//   The settle is only the wait of the overheat cut now: a settle longer than the period loads.
// Follow-up prompt (#150):
//   echo_timeout_ms replaces send_settle_ms as the parameter: the old name is refused, a value
//   out of 20 to 80 is refused, and an echo timeout longer than the period loads.
// Follow-up prompt (#170):
//   pose_from_poll 'false' loads; a value that is not 'true' or 'false' is refused.
// Follow-up prompt (#122 part 1):
//   fake_mode=measured reaches the fake arm and its time is the sum of the periods read() gets:
//   a J1 step reported unmoved by a read of period 0 and 1 - 1/e of the way after a read of one
//   lag time constant, without a sleep. A fake_mode that is neither instant nor measured is
//   refused.
// Follow-up prompt (#122 part 2):
//   One test feeds the plugin, at the 30 Hz control rate, the points of a 3-point trajectory
//   (hold, +10 deg, +20 deg) on J1 and the wrist J5 with fake_mode=measured, and checks the
//   largest error between command and state: under the controller's 0.087 rad trajectory
//   tolerance at 10 deg/s, over it at 30 deg/s.
// Follow-up prompt (#120):
//   The driver gpio may also declare driver/pose_age_ms, the age in ms of the pose read()
//   publishes, NaN until the first pose. FAULT_GPIO now declares it, as the xacro does. Test:
//   it is exported; it is NaN after configure and 0 after a read on the fake arm; a URDF
//   without it still loads and runs, with no such interface; on the pseudo-terminal arm it is
//   finite and small while the arm answers, and at least the silence once the arm goes quiet.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/resource_manager_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "positronic_hardware/arm_backend.hpp"
#include "positronic_hardware/fake_arm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace
{

/// Name of the <hardware> component in the test URDFs.
constexpr std::string_view HARDWARE_NAME{"PositronicArm"};

/// Revolute joints of mycobot_280_jn.urdf, J1 (base) to J6 (flange).
const std::vector<std::string> JOINTS{"joint2_to_joint1", "joint3_to_joint2",
  "joint4_to_joint3", "joint5_to_joint4",
  "joint6_to_joint5", "joint6output_to_joint6"};

/// The arm's URDF as installed by positronic_description, without a <ros2_control> block.
std::string read_description()
{
  const std::string path{
    ament_index_cpp::get_package_share_directory("positronic_description") +
    "/urdf/mycobot_280_jn_sim/mycobot_280_jn.urdf"};
  const std::ifstream file{path};
  std::stringstream content;
  content << file.rdbuf();
  return content.str();
}

constexpr std::string_view POSITION{"<state_interface name=\"position\"/>"};
constexpr std::string_view VELOCITY{"<state_interface name=\"velocity\"/>"};
constexpr std::string_view COMMAND{"<command_interface name=\"position\"/>"};
/// ros2_control's clamping limiter off, as in positronic_real.urdf.xacro (#34).
constexpr std::string_view LIMITS_OFF{"<limits enable=\"false\"/>"};
/// The driver <gpio>, exactly as in positronic_real.urdf.xacro: the fault the supervisor reads
/// (#35) and the age of the pose (#120).
constexpr std::string_view FAULT_GPIO{R"(<gpio name="driver"><state_interface name="fault_code"/>)"
  R"(<state_interface name="pose_age_ms"/></gpio>)"};
/// The driver <gpio> as before #120: the fault alone. The pose age is optional.
constexpr std::string_view FAULT_ONLY_GPIO{
  R"(<gpio name="driver"><state_interface name="fault_code"/></gpio>)"};
/// The values of FaultCode the supervisor relies on: written out here, not taken from the
/// header, so the test fails if someone renumbers them.
constexpr double NO_FAULT{0.0};
constexpr double LINK_TIMEOUT_FAULT{1.0};
constexpr double OVERHEAT_FAULT{3.0};
constexpr double TRACKING_ERROR_FAULT{2.0};

/// One <joint> entry of the <ros2_control> block, with the given interface tags.
std::string joint_block(const std::string & name, std::string_view interfaces)
{
  return "<joint name=\"" + name + "\">" + std::string{interfaces} + "</joint>";
}

/// A valid joint: position and velocity state interfaces, position command interface, and the
/// ros2_control limiter off, exactly like the real xacro.
std::string good_joint(const std::string & name)
{
  return joint_block(
    name,
    std::string{LIMITS_OFF} + std::string{COMMAND} + std::string{POSITION} + std::string{VELOCITY});
}

/// `text` with its one occurrence of `from` replaced by `to`. Used to edit the URDF limits.
std::string replaced(std::string text, std::string_view from, std::string_view to)
{
  const std::size_t at{text.find(from)};
  if (at != std::string::npos) {
    text.replace(at, from.size(), to);
  }
  return text;
}

/// The <limit> tags of J1, J2 and J5 in mycobot_280_jn.urdf, verbatim. Each is unique.
constexpr std::string_view J1_LIMIT{
  R"(<limit effort="10.0" lower="-2.9321" upper="2.9321" velocity="2.618"/>)"};
constexpr std::string_view J2_LIMIT{
  R"(<limit effort="10.0" lower="-2.4434" upper="2.4434" velocity="2.618"/>)"};

/// The URDF limits the refusal tests aim at, in radians.
constexpr double J2_UPPER{2.4434};
constexpr double J5_LOWER{-2.7052};
constexpr double DEG{3.14159265358979323846 / 180.0};

/// All six joints valid, except joint `index`, which gets `interfaces` instead.
std::string joints_with_one_bad(std::size_t index, const std::string & interfaces)
{
  std::string blocks;
  for (std::size_t i{0}; i < JOINTS.size(); ++i) {
    blocks += i == index ? joint_block(JOINTS.at(i), interfaces) : good_joint(JOINTS.at(i));
  }
  return blocks;
}

/// One <param> of the <hardware> tag.
std::string param(std::string_view name, std::string_view value)
{
  return "<param name=\"" + std::string{name} + "\">" + std::string{value} + "</param>";
}

/// Tests never open a serial port.
constexpr std::string_view FAKE_BACKEND{"<param name=\"backend\">fake</param>"};

/// `description` (the arm's URDF by default) with a <ros2_control> block that uses RobotSystem
/// for `joint_blocks`, with `parameters` in its <hardware> tag and `gpio` after the joints.
std::string urdf_with(
  const std::string & joint_blocks, std::string_view parameters = FAKE_BACKEND,
  const std::string & description = read_description(), std::string_view gpio = FAULT_GPIO)
{
  std::string urdf{description};
  const std::string ros2_control{
    std::string{"<ros2_control name=\""} + std::string{HARDWARE_NAME} +
    "\" type=\"system\">"
    "<hardware><plugin>positronic_hardware/RobotSystem</plugin>" +
    std::string{parameters} + "</hardware>" + joint_blocks + std::string{gpio} + "</ros2_control>"};
  urdf.insert(urdf.rfind("</robot>"), ros2_control);
  return urdf;
}

std::string all_good_joints()
{
  std::string blocks;
  for (const auto & joint : JOINTS) {
    blocks += good_joint(joint);
  }
  return blocks;
}

/// Opens a pseudo-terminal, which opens and configures like a serial port but never answers,
/// and writes its device path into `port`. Returns the controller descriptor, which the caller
/// closes, or -1.
int open_pseudo_terminal(std::array<char, 128> & port)
{
  const int controller{::posix_openpt(O_RDWR | O_NOCTTY)};
  if (controller < 0) {
    return -1;
  }
  if (
    ::grantpt(controller) != 0 || ::unlockpt(controller) != 0 ||
    ::ptsname_r(controller, port.data(), port.size()) != 0)
  {
    ::close(controller);
    return -1;
  }
  return controller;
}

/// One get_angles reply as the arm sends it, recorded on the real arm (the pose the SerialArm
/// tests decode). Written back for every request, so a power_on also gets a pose: the arm
/// answers that one with an ack, but the thread only looks for poses.
constexpr std::array<std::uint8_t, 17> RECORDED_REPLY{0xFE, 0xFE, 0x0E, 0x20, 0xFF, 0xF8,
  0xFF, 0xEF, 0xFF, 0xC3, 0xFF, 0x85,
  0x00, 0x2B, 0x00, 0x3D, 0xFA};

/// What the Atom puts on the line when it polls the six servos, recorded on the arm (the bytes
/// the SerialArm tests replay): the ping, the sync read and the six echo frames. It precedes
/// every reply to the driver and every sync-write.
constexpr std::string_view RECORDED_POLL{
  "ff ff 07 02 01 f5 ff ff fe 0a 82 38 0f 01 02 03 04 05 06 19 ff ff 01 11 00 01 08 00 00 00 00 "
  "79 26 00 00 00 01 08 00 00 3c ff ff 02 11 00 02 08 00 00 00 00 79 23 00 00 00 02 08 00 00 3c "
  "ff ff 03 11 00 f9 07 00 00 00 00 7b 21 00 00 00 f9 07 00 00 4f ff ff 04 11 00 0e 08 00 00 00 "
  "00 4a 21 00 00 00 0e 08 00 00 53 ff ff 05 11 00 fb 07 00 00 00 00 49 20 00 00 00 fb 07 00 00 "
  "7c ff ff 06 11 00 f9 07 00 00 00 00 49 23 00 00 00 f9 07 00 00 7c"};

/// The Atom's sync-write to the servos after a send_angles, recorded on the arm on 13 Nov 2026.
constexpr std::string_view RECORDED_SYNC_WRITE{
  "ff ff fe 34 83 29 07 01 32 e8 07 00 00 f1 01 02 32 01 08 00 00 32 01 03 32 4a 08 00 00 2c 01 "
  "04 32 3f 08 00 00 2c 01 05 32 13 08 00 00 9e 01 06 32 de 08 00 00 fe 01 2a"};

std::vector<std::uint8_t> parse_hex(std::string_view hex)
{
  std::vector<std::uint8_t> bytes{};
  for (std::size_t i{0}; i + 1 < hex.size(); i += 3) {
    bytes.push_back(
      static_cast<std::uint8_t>(std::stoul(std::string{hex.substr(i, 2)}, nullptr, 16)));
  }
  return bytes;
}

/// Answers the driver on the controller side of a pseudo-terminal, frame by frame, until stopped: a
/// stand-in for the arm with the real traffic. A get_angles gets the recorded poll of the servos
/// and then RECORDED_REPLY, a stop or power_on gets the arm's ack (FE FE 03 <id> 01 FA) after
/// `ack_delay` (1.8 ms on the bench, 58 ms seen once for a power_on), a power_off its ack at once,
/// a send_angles gets its ack, the poll and the recorded sync-write ~20 ms later, as the arm does
/// it, a get_servo_temps gets the temperatures set with set_temperature() (all six servos alike)
/// unless set_answer_temperatures(false), anything else nothing. With ack_requests off the stop and
/// power_on frames are taken and never answered. With set_silent_after_power_off(true) it acks the
/// first power_off and answers nothing after it, like the real arm with its servos off. Frames are
/// cut like the arm does it: FE FE <len> ..., len + 3 bytes, resynchronised on FE FE, with a carry
/// across reads. It keeps every byte it receives, so a test can check what the driver put on the
/// wire.
class ReplyingArm
{
public:
  explicit ReplyingArm(int controller)
  : thread_{[this, controller]() {
        std::array<std::uint8_t, 64> chunk{};
        std::vector<std::uint8_t> carry{};
        while (!stop_.load()) {
          pollfd waiting{controller, POLLIN, 0};
          if (::poll(&waiting, 1, 10) <= 0) {
            continue;
          }
          const ssize_t got{::read(controller, chunk.data(), chunk.size())};
          if (got <= 0) {
            continue;
          }
          {
            const std::lock_guard<std::mutex> lock{mutex_};
            received_.insert(received_.end(), chunk.begin(), std::next(chunk.begin(), got));
          }
          carry.insert(carry.end(), chunk.begin(), std::next(chunk.begin(), got));
          for (std::optional<std::vector<std::uint8_t>> frame{next_frame(carry)}; frame;
            frame = next_frame(carry))
          {
            answer(controller, frame.value());
          }
        }
      }}
  {
  }

  /// An arm that never acknowledges a stop or power on (default: it does).
  void set_ack_requests(bool ack)
  {
    ack_requests_.store(ack);
  }

  /// How long the arm takes to acknowledge a request (default 0).
  void set_ack_delay(std::chrono::milliseconds delay)
  {
    ack_delay_.store(delay);
  }

  /// True if a send_angles frame (FE FE 0F 22 ...) arrived since this arm started.
  bool received_send_angles()
  {
    return received_sends() > 0;
  }

  /// How many send_angles frames arrived since this arm started.
  std::size_t received_sends()
  {
    return count_frames(18, 0x22);
  }

  /// How many 5-byte request frames with command `id` arrived (stop 0x29, power_on 0x10).
  std::size_t received_requests(std::uint8_t id)
  {
    return count_frames(5, id);
  }

  /// Index in the received bytes of the last request frame with command `id`, or nullopt.
  std::optional<std::size_t> last_request(std::uint8_t id)
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    std::optional<std::size_t> last{};
    for (std::size_t i{0}; i + 5 <= received_.size(); ++i) {
      if (
        received_[i] == 0xFE && received_[i + 1] == 0xFE && received_[i + 2] == 0x02 &&
        received_[i + 3] == id && received_[i + 4] == 0xFA)
      {
        last = i;
      }
    }
    return last;
  }

  /// True once the driver has sent anything at all.
  bool received_anything()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return !received_.empty();
  }

  ~ReplyingArm()
  {
    stop_.store(true);
    thread_.join();
  }

  ReplyingArm(const ReplyingArm &) = delete;
  ReplyingArm & operator=(const ReplyingArm &) = delete;
  ReplyingArm(ReplyingArm &&) = delete;
  ReplyingArm & operator=(ReplyingArm &&) = delete;

  /// The temperature every servo reports from now on, degC.
  void set_temperature(std::uint8_t celsius)
  {
    celsius_.store(celsius);
  }

  /// False: a firmware without get_servo_temps, the request gets no reply at all.
  void set_answer_temperatures(bool answer)
  {
    answer_temperatures_.store(answer);
  }

  /// True: from the first power_off on, no request gets a reply (the real arm, servos off).
  void set_silent_after_power_off(bool silent)
  {
    silent_after_power_off_.store(silent);
  }

  /// Every byte the driver wrote to the arm so far.
  std::vector<std::uint8_t> received()
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    return received_;
  }

private:
  /// Cuts the first complete frame off the front of `carry`: FE FE <len> + len bytes. Bytes
  /// before a FE FE are dropped, as the arm drops them.
  static std::optional<std::vector<std::uint8_t>> next_frame(std::vector<std::uint8_t> & carry)
  {
    std::size_t start{0};
    while (start + 1 < carry.size() && (carry[start] != 0xFE || carry[start + 1] != 0xFE)) {
      ++start;
    }
    if (start + 3 > carry.size()) {
      carry.erase(carry.begin(), std::next(carry.begin(), static_cast<std::ptrdiff_t>(start)));
      return std::nullopt;
    }
    const std::size_t size{static_cast<std::size_t>(carry[start + 2]) + 3};
    if (start + size > carry.size()) {
      carry.erase(carry.begin(), std::next(carry.begin(), static_cast<std::ptrdiff_t>(start)));
      return std::nullopt;
    }
    std::vector<std::uint8_t> frame(size);
    std::copy_n(std::next(carry.begin(), static_cast<std::ptrdiff_t>(start)), size, frame.begin());
    carry.erase(carry.begin(), std::next(carry.begin(), static_cast<std::ptrdiff_t>(start + size)));
    return frame;
  }

  static void put(int controller, const std::vector<std::uint8_t> & bytes)
  {
    static_cast<void>(::write(controller, bytes.data(), bytes.size()));
  }

  void answer(int controller, const std::vector<std::uint8_t> & frame)
  {
    static const std::vector<std::uint8_t> poll{parse_hex(RECORDED_POLL)};
    static const std::vector<std::uint8_t> sync_write{parse_hex(RECORDED_SYNC_WRITE)};
    const std::uint8_t id{frame[3]};
    if (powered_off_.load()) {
      return;
    }
    if (id == 0x11) {
      // The arm acks a power off (62 ms after it on 24 Nov 2026), then answers nothing more.
      put(controller, {0xFE, 0xFE, 0x03, 0x11, 0x01, 0xFA});
      powered_off_.store(silent_after_power_off_.load());
      return;
    }
    if (id == 0xE5) {
      if (answer_temperatures_.load()) {
        const std::uint8_t t{celsius_.load()};
        put(controller, {0xFE, 0xFE, 0x08, 0xE5, t, t, t, t, t, t, 0xFA});
      }
      return;
    }
    if (id == 0x20) {
      put(controller, poll);
      put(controller, {RECORDED_REPLY.begin(), RECORDED_REPLY.end()});
      return;
    }
    const bool request{id == 0x29 || id == 0x10};
    if (request && !ack_requests_.load()) {
      return;
    }
    if (request || id == 0x22) {
      if (request) {
        std::this_thread::sleep_for(ack_delay_.load());
      }
      put(controller, {0xFE, 0xFE, 0x03, id, 0x01, 0xFA});
    }
    if (id == 0x22) {
      // The arm polls the servos, then writes them the target: the sync-write is on the line
      // ~20 ms after the send_angles. The sleeps also keep this arm busy, as the real one is.
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
      put(controller, poll);
      std::this_thread::sleep_for(std::chrono::milliseconds{10});
      put(controller, sync_write);
    }
  }

  std::size_t count_frames(std::size_t size, std::uint8_t id)
  {
    const std::lock_guard<std::mutex> lock{mutex_};
    std::size_t count{0};
    for (std::size_t i{0}; i + size <= received_.size(); ++i) {
      if (
        received_[i] == 0xFE && received_[i + 1] == 0xFE &&
        received_[i + 2] == static_cast<std::uint8_t>(size - 3) && received_[i + 3] == id)
      {
        ++count;
      }
    }
    return count;
  }

  std::atomic<bool> answer_temperatures_{true};
  std::atomic<bool> silent_after_power_off_{false};
  std::atomic<bool> powered_off_{false};
  std::atomic<std::uint8_t> celsius_{30};
  std::atomic<bool> stop_{false};
  std::atomic<bool> ack_requests_{true};
  std::atomic<std::chrono::milliseconds> ack_delay_{std::chrono::milliseconds{0}};
  std::mutex mutex_;
  std::vector<std::uint8_t> received_;
  std::thread thread_;
};

/// Index of the first `FE FE 02 id FA` in `bytes` at or after `from`, or bytes.size().
std::size_t find_request(
  const std::vector<std::uint8_t> & bytes, std::uint8_t id, std::size_t from = 0)
{
  const std::array<std::uint8_t, 5> frame{0xFE, 0xFE, 0x02, id, 0xFA};
  const std::size_t first{std::min(from, bytes.size())};
  const auto start{std::next(bytes.begin(), static_cast<std::ptrdiff_t>(first))};
  return static_cast<std::size_t>(
    std::distance(bytes.begin(), std::search(start, bytes.end(), frame.begin(), frame.end())));
}

class LoadRobotSystem : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  /// Loads `urdf` into a fresh ResourceManager, as the controller manager does at start-up.
  hardware_interface::ResourceManager & load(const std::string & urdf)
  {
    hardware_interface::ResourceManagerParams params;
    params.robot_description = urdf;
    params.clock = std::make_shared<rclcpp::Clock>();
    params.logger = rclcpp::get_logger("test_load_robot_system");
    resource_manager_ = std::make_unique<hardware_interface::ResourceManager>(params, false);
    loaded_ = resource_manager_->load_and_initialize_components(params);
    return *resource_manager_;
  }

  /// Lifecycle state id of the component, or PRIMARY_STATE_UNKNOWN if it is not listed.
  std::uint8_t component_state()
  {
    const auto & status{resource_manager_->get_components_status()};
    const auto it{status.find(std::string{HARDWARE_NAME})};
    if (it == status.end()) {
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    return it->second.state.id();
  }

  /// Moves the component to another lifecycle state, as the controller manager does.
  hardware_interface::return_type go_to(std::uint8_t id, const char * label)
  {
    rclcpp_lifecycle::State target{id, label};
    return resource_manager_->set_component_state(std::string{HARDWARE_NAME}, target);
  }

  /// configure, as the controller manager does before an activate.
  hardware_interface::return_type configure()
  {
    return go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  /// Configures (if needed) and activates the component, as the controller manager does.
  hardware_interface::return_type activate()
  {
    return go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
  }

  /// Opens a pseudo-terminal and loads the plugin on its serial backend, port not opened yet
  /// (that is configure). The controller side is in controller_, for a ReplyingArm to answer
  /// on, or for a test to close. False when the terminal could not be opened or the plugin did
  /// not load.
  [[nodiscard]] bool load_on_pseudo_terminal()
  {
    std::array<char, 128> port{};
    controller_ = open_pseudo_terminal(port);
    if (controller_ < 0) {
      return false;
    }
    load(urdf_with(all_good_joints(), param("backend", "serial") + param("port", port.data())));
    return loaded_;
  }

  /// One controller-manager cycle of read(), 10 ms long.
  hardware_interface::return_type read_once()
  {
    return resource_manager_->read(clock_.now(), rclcpp::Duration::from_seconds(0.01)).result;
  }

  /// The value a controller would see for `joint`/`interface`.
  std::optional<double> state(const std::string & joint, const std::string & interface)
  {
    const auto loaned{resource_manager_->claim_state_interface(joint + "/" + interface)};
    return loaned.get_optional();
  }

  /// One controller-manager cycle of write(), 10 ms long.
  hardware_interface::return_type write_once()
  {
    return resource_manager_->write(clock_.now(), rclcpp::Duration::from_seconds(0.01)).result;
  }

  /// `n` control cycles, a write then a read, 30 ms apart like the Jetson's loop. False at
  /// the first write or read that is not OK.
  [[nodiscard]] bool cycles(int n)
  {
    for (int cycle{0}; cycle < n; ++cycle) {
      if (
        write_once() != hardware_interface::return_type::OK ||
        read_once() != hardware_interface::return_type::OK)
      {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{30});
    }
    return true;
  }

  /// Writes `value` to `joint`/position the way a controller does.
  bool command(const std::string & joint, double value)
  {
    auto loaned{resource_manager_->claim_command_interface(joint + "/position")};
    return loaned.set_value(value);
  }

  rclcpp::Clock clock_;
  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;
  bool loaded_{false};
  /// The controller side of the pseudo-terminal of load_on_pseudo_terminal(), or -1.
  int controller_{-1};
};

}  // namespace

TEST_F(LoadRobotSystem, DescriptionHasTheSixJoints)
{
  // Guards the other tests: if this fails, the URDF moved or a joint was renamed.
  const std::string urdf{read_description()};
  ASSERT_FALSE(urdf.empty());
  for (const auto & joint : JOINTS) {
    EXPECT_NE(urdf.find("<joint name=\"" + joint + "\" type=\"revolute\""), std::string::npos)
      << joint;
  }
}

TEST_F(LoadRobotSystem, DescriptionHasTheLimitsTheTestsEdit)
{
  // Guards the #34 tests that edit a <limit>: if the URDF text changes, they must not pass
  // for the wrong reason.
  const std::string urdf{read_description()};
  EXPECT_NE(urdf.find(J1_LIMIT), std::string::npos);
  EXPECT_NE(urdf.find(J2_LIMIT), std::string::npos);
  EXPECT_NE(urdf.find("lower=\"-2.7052\""), std::string::npos);
}

TEST_F(LoadRobotSystem, InitSucceedsWithSixPositionJoints)
{
  load(urdf_with(all_good_joints()));

  EXPECT_TRUE(loaded_);
  // on_init returned SUCCESS: the component sits in UNCONFIGURED.
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, ExportsTheTwelveJointStatesTheFaultCodeAndThePoseAge)
{
  auto & resource_manager{load(urdf_with(all_good_joints()))};
  ASSERT_TRUE(loaded_);

  std::vector<std::string> expected;
  expected.reserve(2 * JOINTS.size() + 2);
  for (const auto & joint : JOINTS) {
    expected.push_back(joint + "/position");
    expected.push_back(joint + "/velocity");
  }
  expected.emplace_back("driver/fault_code");
  expected.emplace_back("driver/pose_age_ms");
  auto keys{resource_manager.state_interface_keys()};
  std::sort(keys.begin(), keys.end());
  std::sort(expected.begin(), expected.end());
  EXPECT_EQ(keys, expected);
}

TEST_F(LoadRobotSystem, InitRejectsFiveJoints)
{
  std::string blocks;
  for (std::size_t i{0}; i + 1 < JOINTS.size(); ++i) {
    blocks += good_joint(JOINTS.at(i));
  }
  load(urdf_with(blocks));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsJointWithoutPositionState)
{
  load(urdf_with(joints_with_one_bad(2, std::string{VELOCITY})));  // J3: velocity only

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsJointWithoutVelocityState)
{
  load(urdf_with(joints_with_one_bad(2, std::string{POSITION})));  // J3: position only

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsJointWithoutCommandInterface)
{
  // J1 cannot be commanded: a controller could never move it, so the driver refuses to load.
  load(urdf_with(joints_with_one_bad(0, std::string{POSITION} + std::string{VELOCITY})));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsVelocityCommandInterface)
{
  // J1 asks for a velocity command: the arm only takes angles, so nothing could honour it.
  constexpr std::string_view VELOCITY_COMMAND{"<command_interface name=\"velocity\"/>"};
  load(urdf_with(joints_with_one_bad(
    0, std::string{VELOCITY_COMMAND} + std::string{POSITION} + std::string{VELOCITY})));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitAcceptsEveryParameter)
{
  load(urdf_with(
    all_good_joints(), std::string{FAKE_BACKEND} + param("port", "/dev/ttyUSB0") +
                         param("baud", "115200") + param("velocity_filter_alpha", "0.5") +
                         param("moving_period_ms", "40") + param("echo_timeout_ms", "45") +
                         param("pose_from_poll", "false") + param("servo_temp_limit_c", "50")));

  EXPECT_TRUE(loaded_);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsBadParameters)
{
  // Each case is the fake backend plus one wrong <param>: a typo must stop the driver.
  const std::vector<std::pair<std::string, std::string>> bad{
    {"baudrate", "115200"},             // unknown name
    {"backend", "simulated"},           // only serial or fake
    {"port", ""},                       // empty
    {"baud", "9600"},                   // a rate PosixSerialPort cannot set
    {"baud", "115200x"},                // not only a number
    {"velocity_filter_alpha", "0"},     // the filter would never move
    {"velocity_filter_alpha", "1.5"},   // above 1
    {"velocity_filter_alpha", "fast"},  // not a number
    {"velocity_filter_alpha", "nan"},   // from_chars reads it, the range check must not
    {"moving_period_ms", "10"},         // below the arm's own busy time
    {"moving_period_ms", "150"},        // too close to the 200 ms link timeout (#35)
    {"moving_period_ms", "66.5"},       // whole ms only
    {"echo_timeout_ms", "100"},         // two lost replies after it would pass the #35 timeout
    {"echo_timeout_ms", "slow"},        // not a number
    {"send_settle_ms", "30"},           // no parameter since #150: the settle is a constant
    {"pose_from_poll", "maybe"},        // true or false only (#170)
    {"servo_temp_limit_c", "61"},       // the limit can only be lowered
    {"servo_temp_limit_c", "29"},       // a cool arm would be cut at once
    {"servo_temp_limit_c", "55.5"},     // whole degrees only
    {"servo_temp_limit_c", "hot"},      // not a number
    {"fake_mode", "slow"},              // only instant or measured (#122)
  };
  for (const auto & [name, value] : bad) {
    SCOPED_TRACE(::testing::Message{} << name << "=" << value);
    // "backend" replaces the fake one rather than appearing twice.
    const std::string parameters{
      name == "backend" ? param(name, value) : std::string{FAKE_BACKEND} + param(name, value)};
    load(urdf_with(all_good_joints(), parameters));

    EXPECT_FALSE(loaded_);
    EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_F(LoadRobotSystem, InitAcceptsAnEchoTimeoutLongerThanThePeriod)
{
  // #150: the wait for the sync-write only counts when the Atom is late, so it does not have to
  // fit in the turn.
  load(urdf_with(
    all_good_joints(),
    std::string{FAKE_BACKEND} + param("moving_period_ms", "25") + param("echo_timeout_ms", "60")));

  EXPECT_TRUE(loaded_);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, PositionsAreUnknownBeforeTheFirstRead)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  for (const auto & joint : JOINTS) {
    for (const std::string interface : {"position", "velocity"}) {
      const auto value{state(joint, interface)};
      ASSERT_TRUE(value.has_value()) << joint << "/" << interface;
      EXPECT_TRUE(std::isnan(value.value_or(0.0))) << joint << "/" << interface;
    }
  }
}

TEST_F(LoadRobotSystem, ReadReportsTheFakeArmPose)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  for (std::size_t i{0}; i < JOINTS.size(); ++i) {
    const auto position{state(JOINTS.at(i), "position")};
    ASSERT_TRUE(position.has_value()) << JOINTS.at(i);
    EXPECT_DOUBLE_EQ(
      position.value_or(std::nan("")), positronic_hardware::FakeArm::START_POSE.at(i))
      << JOINTS.at(i);
    // The fake arm stands still.
    const auto velocity{state(JOINTS.at(i), "velocity")};
    ASSERT_TRUE(velocity.has_value()) << JOINTS.at(i);
    EXPECT_DOUBLE_EQ(velocity.value_or(std::nan("")), 0.0) << JOINTS.at(i);
  }
}

TEST_F(LoadRobotSystem, ActivatesAndReadStillWorks)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);

  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(read_once(), hardware_interface::return_type::OK);
}

TEST_F(LoadRobotSystem, WriteSendsTheCommandToTheFakeArm)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);

  // A controller asks J1 for 0.25 rad. The fake arm gets there at once, so the next read()
  // reports it: the whole path from command interface to backend and back.
  ASSERT_TRUE(command(JOINTS.at(0), 0.25));
  ASSERT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(state(JOINTS.at(0), "position").value_or(std::nan("")), 0.25);
  // The other joints were commanded "stay" by on_activate, so they still report START_POSE.
  EXPECT_DOUBLE_EQ(
    state(JOINTS.at(1), "position").value_or(std::nan("")),
    positronic_hardware::FakeArm::START_POSE.at(1));
}

// #122: the measured fake arm lags, and its clock is the sum of the periods read() gets, so
// the test moves it without sleeping.
TEST_F(LoadRobotSystem, MeasuredFakeModeLagsOnThePeriodsOfRead)
{
  using positronic_hardware::FakeArm;
  load(urdf_with(all_good_joints(), std::string{FAKE_BACKEND} + param("fake_mode", "measured")));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);

  const double step{0.1};
  const double rest_error{FakeArm::REST_ERROR_DEG.at(0) * 3.14159265358979323846 / 180.0};
  const auto read_after{[this](double seconds) {
      return resource_manager_->read(clock_.now(), rclcpp::Duration::from_seconds(seconds)).result;
    }};
  ASSERT_TRUE(command(JOINTS.at(0), FakeArm::START_POSE.at(0) + step));
  ASSERT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_after(0.0), hardware_interface::return_type::OK);
  EXPECT_NEAR(
    state(JOINTS.at(0), "position").value_or(std::nan("")), FakeArm::START_POSE.at(0) - rest_error,
    1e-12);

  ASSERT_EQ(read_after(FakeArm::LAG_TIME_CONSTANT_S), hardware_interface::return_type::OK);
  EXPECT_NEAR(
    state(JOINTS.at(0), "position").value_or(std::nan("")),
    FakeArm::START_POSE.at(0) + step * (1.0 - std::exp(-1.0)) - rest_error, 1e-9);
}

// #122: what the measured fake arm is for. The controller's trajectory tolerance
// (jetson_controllers.yaml, 0.087 rad) holds at 10 deg/s on the arm and fails at 30 (#33); the
// instant fake arm would pass both. J1 and the wrist J5 move together through three points,
// fed at the 30 Hz control rate; the time is the periods passed to read(), so no sleep.
TEST_F(LoadRobotSystem, MeasuredFakeArmKeepsTheToleranceAt10DegPerSecondNotAt30)
{
  using positronic_hardware::FakeArm;
  constexpr double DEG{3.14159265358979323846 / 180.0};
  constexpr double TOLERANCE_RAD{0.087};
  constexpr double PERIOD_S{1.0 / 30.0};
  static constexpr std::array<std::size_t, 2> MOVING{0, 4};

  const auto largest_error_rad{[this](double deg_per_s) {
      load(urdf_with(all_good_joints(),
        std::string{FAKE_BACKEND} + param("fake_mode", "measured")));
      EXPECT_EQ(
      go_to(
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        hardware_interface::lifecycle_state_names::ACTIVE),
      hardware_interface::return_type::OK);
      const auto cycle{[this]() {
          EXPECT_EQ(write_once(), hardware_interface::return_type::OK);
          EXPECT_EQ(
        resource_manager_->read(clock_.now(), rclcpp::Duration::from_seconds(PERIOD_S)).result,
        hardware_interface::return_type::OK);
        }};
    // on_activate commands the pose it read, START_POSE less the rest error. Two seconds
    // holding it, so the arm has settled one rest error below before the trajectory starts.
      for (int i{0}; i < 60; ++i) {
        cycle();
      }
    // Points 0, 1 and 2 at hold, hold + 10 deg and hold + 20 deg, at `deg_per_s` between them,
    // then one second holding point 2.
      const double span{20.0 * DEG};
      const double duration{span / (deg_per_s * DEG)};
      double largest{0.0};
      for (int i{1}; i * PERIOD_S <= duration + 1.0; ++i) {
        const double travelled{std::min(deg_per_s * DEG * i * PERIOD_S, span)};
        for (const std::size_t joint : MOVING) {
          const double hold{FakeArm::START_POSE.at(joint) - FakeArm::REST_ERROR_DEG.at(joint) *
            DEG};
          EXPECT_TRUE(command(JOINTS.at(joint), hold + travelled));
        }
        cycle();
        for (const std::size_t joint : MOVING) {
          const double hold{FakeArm::START_POSE.at(joint) - FakeArm::REST_ERROR_DEG.at(joint) *
            DEG};
          const double error{hold + travelled - state(JOINTS.at(joint), "position").value_or(0.0)};
          largest = std::max(largest, std::abs(error));
        }
      }
      return largest;
    }};

  const double slow{largest_error_rad(10.0)};
  EXPECT_LT(slow, TOLERANCE_RAD);
  const double fast{largest_error_rad(30.0)};
  EXPECT_GT(fast, TOLERANCE_RAD);
  RecordProperty("largest_error_10_deg_per_s_rad", std::to_string(slow));
  RecordProperty("largest_error_30_deg_per_s_rad", std::to_string(fast));
}

TEST_F(LoadRobotSystem, WriteSendsACommandExactlyAtTheLimit)
{
  // The URDF says "up to here": the limit itself is allowed.
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  ASSERT_TRUE(command(JOINTS.at(1), J2_UPPER));
  EXPECT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state(JOINTS.at(1), "position").value_or(std::nan("")), J2_UPPER);
}

TEST_F(LoadRobotSystem, WriteRefusesACommandPastTheLimit)
{
  // Acceptance criterion of #34: a goal at the limit + 5 deg is refused and the arm halts.
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  ASSERT_TRUE(command(JOINTS.at(1), J2_UPPER + 5.0 * DEG));
  EXPECT_EQ(write_once(), hardware_interface::return_type::ERROR);
  // on_error ran: the component left ACTIVE, so no controller can write to it any more.
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LoadRobotSystem, WriteRefusesACommandBelowAnAsymmetricLimit)
{
  // J5 goes to +2.7925 but only to -2.7052: the lower side has its own end.
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  ASSERT_TRUE(command(JOINTS.at(4), J5_LOWER - 5.0 * DEG));
  EXPECT_EQ(write_once(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LoadRobotSystem, WriteRefusesNaN)
{
  // Bad data is refused the same way. Every comparison with NaN is false, so a plain range
  // check would let it through.
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);

  ASSERT_TRUE(command(JOINTS.at(3), std::nan("")));
  EXPECT_EQ(write_once(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LoadRobotSystem, ComesBackAfterARefusal)
{
  // The refusal is not forever: configure and activate start again from "stay where you are".
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_TRUE(command(JOINTS.at(1), J2_UPPER + 5.0 * DEG));
  ASSERT_EQ(write_once(), hardware_interface::return_type::ERROR);

  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(
    state(JOINTS.at(1), "position").value_or(std::nan("")),
    positronic_hardware::FakeArm::START_POSE.at(1));
}

TEST_F(LoadRobotSystem, InitRejectsAJointWithoutALimit)
{
  // J1 made continuous, without its <limit>: ros2_control accepts it, the driver must not.
  const std::string description{replaced(
    replaced(
      read_description(), R"(<joint name="joint2_to_joint1" type="revolute">)",
      R"(<joint name="joint2_to_joint1" type="continuous">)"),
    J1_LIMIT, "")};
  load(urdf_with(all_good_joints(), FAKE_BACKEND, description));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, InitRejectsSwappedLimits)
{
  // J2 with lower above upper: the check would refuse every command, so the driver stops here.
  const std::string description{replaced(
    read_description(), J2_LIMIT,
    R"(<limit effort="10.0" lower="2.4434" upper="-2.4434" velocity="2.618"/>)")};
  load(urdf_with(all_good_joints(), FAKE_BACKEND, description));

  EXPECT_FALSE(loaded_);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, ActivateRefusesAnArmOutsideItsLimits)
{
  // J2 narrowed to [-0.2, 0.2] rad: the fake arm stands at 0.5 rad, outside. The first
  // command would be refused at once, so activation fails with the reason instead.
  const std::string description{replaced(
    read_description(), J2_LIMIT,
    R"(<limit effort="10.0" lower="-0.2" upper="0.2" velocity="2.618"/>)")};
  load(urdf_with(all_good_joints(), FAKE_BACKEND, description));
  ASSERT_TRUE(loaded_);

  EXPECT_EQ(activate(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LoadRobotSystem, ConfigureFailsWhenTheSerialPortIsMissing)
{
  // The serial path without an arm: opening the port throws, configure must report an error.
  load(urdf_with(
    all_good_joints(), param("backend", "serial") + param("port", "/dev/positronic_no_such_port")));
  ASSERT_TRUE(loaded_);

  EXPECT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(LoadRobotSystem, CleanupThenConfigureAgain)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED),
    hardware_interface::return_type::OK);
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  const auto position{state(JOINTS.at(1), "position")};
  ASSERT_TRUE(position.has_value());
  EXPECT_DOUBLE_EQ(position.value_or(std::nan("")), positronic_hardware::FakeArm::START_POSE.at(1));
}

TEST_F(LoadRobotSystem, ActivateFailsWhenTheArmIsSilent)
{
  // A pseudo-terminal stands in for the serial port: it opens and configures like one, but
  // nothing ever answers get_angles. No arm is needed.
  ASSERT_TRUE(load_on_pseudo_terminal());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  // No reply yet: read() works but reports "unknown", not the placeholder pose of zeros.
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  for (const auto & joint : JOINTS) {
    for (const std::string interface : {"position", "velocity"}) {
      const auto value{state(joint, interface)};
      ASSERT_TRUE(value.has_value()) << joint << "/" << interface;
      EXPECT_TRUE(std::isnan(value.value_or(0.0))) << joint << "/" << interface;
    }
  }

  // And the component refuses to become active on an arm that never answered.
  EXPECT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  resource_manager_.reset();  // closes the port before its pseudo-terminal goes away
  ::close(controller_);
}

TEST_F(LoadRobotSystem, DeactivateHoldsCommandsAndActivateResumesThem)
{
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);
  ASSERT_TRUE(command(JOINTS.at(0), 0.25));
  ASSERT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  ASSERT_DOUBLE_EQ(state(JOINTS.at(0), "position").value_or(std::nan("")), 0.25);

  // Deactivate: on_deactivate halts the arm (nothing to interrupt on the fake) and succeeds.
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // A controller still writing its command interface reaches nothing: the resource manager
  // skips an inactive component's write(), and write() itself returns early when not active.
  ASSERT_TRUE(command(JOINTS.at(0), 0.5));
  EXPECT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state(JOINTS.at(0), "position").value_or(std::nan("")), 0.25);

  // Activate again: power_on, commands start from the measured pose, and then follow.
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(command(JOINTS.at(0), 0.5));
  ASSERT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state(JOINTS.at(0), "position").value_or(std::nan("")), 0.5);

  // The rest of the lifecycle still works after a deactivate.
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED),
    hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, AFailedActivateLeavesTheComponentConfigurable)
{
  // Same silent pseudo-terminal: activate fails, and on_error must take the component back to
  // UNCONFIGURED, never FINALIZED (which needs a plugin reload), so a supervisor can configure
  // it again.
  ASSERT_TRUE(load_on_pseudo_terminal());
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // on_error kept the backend, and with it the port: configure drops it before opening again.
  EXPECT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  resource_manager_.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, DeactivateFailsWhenTheStopCannotLeaveAndTheComponentIsConfigurable)
{
  // A pseudo-terminal with an arm behind it that answers every request, so the component gets
  // its first reply and becomes ACTIVE like on the bench.
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE),
    hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_FALSE(std::isnan(state(JOINTS.at(0), "position").value_or(std::nan(""))));

  // The arm goes away: with the controller side closed every write on the port fails, so the
  // stop can never leave. halt() gives up after REQUEST_TIMEOUT and on_deactivate must report
  // it: a component that could not stop the arm must not stay ACTIVE, and on_error takes it to
  // UNCONFIGURED, where a supervisor can configure it again. (The thread does not spin
  // meanwhile: a failed get_angles write skips the read and sleeps out the turn.)
  arm.reset();
  ::close(controller_);
  const auto start{std::chrono::steady_clock::now()};
  EXPECT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::ERROR);
  const auto elapsed{std::chrono::steady_clock::now() - start};
  EXPECT_GE(elapsed, positronic_hardware::ArmBackend::REQUEST_TIMEOUT);
  EXPECT_LT(elapsed, 2 * positronic_hardware::ArmBackend::REQUEST_TIMEOUT);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  resource_manager_.reset();
}

TEST_F(LoadRobotSystem, ActivateRefusesAHotArm)
{
  // Every servo at 57 degC: under the 60 degC cut, but within the 5 degC cool-down margin. The
  // servos are not powered on.
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  arm->set_temperature(57);
  EXPECT_EQ(activate(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(find_request(arm->received(), 0x10), arm->received().size());  // no power_on

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, ActivateRefusesAnArmThatDoesNotReportItsTemperature)
{
  // Firmware without get_servo_temps: the pose answers, the temperature never does. The
  // driver does not run without its over-temperature guard.
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  arm->set_answer_temperatures(false);
  EXPECT_EQ(activate(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, AnOverheatCutsTheArmAndTakesTheComponentOutOfService)
{
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  const std::size_t before_heat{arm->received().size()};

  // The servos reach 61 degC. Two readings, one a second, then the serial thread cuts the arm
  // on its own, and the next read() reports it.
  arm->set_temperature(61);
  hardware_interface::return_type result{hardware_interface::return_type::OK};
  const auto deadline{std::chrono::steady_clock::now() + std::chrono::seconds{5}};
  while (result == hardware_interface::return_type::OK &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
    result = read_once();
  }
  EXPECT_EQ(result, hardware_interface::return_type::ERROR);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // On the wire, after the heat: a stop, then a power off.
  const std::vector<std::uint8_t> sent{arm->received()};
  const std::size_t stop_at{find_request(sent, 0x29, before_heat)};
  ASSERT_LT(stop_at, sent.size());
  EXPECT_LT(find_request(sent, 0x11, stop_at), sent.size());

  // #35: the overheat is latched as fault 3. It survives the on_error: a configure publishes it
  // again for the supervisor. Only an activate clears it, and only a cool arm activates.
  arm->set_temperature(30);
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), OVERHEAT_FAULT);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), NO_FAULT);

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, AnOverheatIsStillTheFaultWhenTheArmGoesSilentAfterTheCut)
{
  // The real arm stops answering once its servos are off, so after the cut the link times out
  // too. Only the first fault is kept: read() must latch OVERHEAT, the cause, not LINK_TIMEOUT.
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  arm->set_silent_after_power_off(true);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  // No read() while the arm heats: wait for the power off on the wire, then let the silence run
  // past the 200 ms link timeout. 400 ms is twice that, a margin for a loaded test machine; the
  // cut is recorded once the stop is acknowledged, before the power off, well inside it.
  // One copy of the bytes per look: two calls to received() can see different lengths.
  const auto powered_off{[&arm]() {
      const std::vector<std::uint8_t> sent{arm->received()};
      return find_request(sent, 0x11) < sent.size();
    }};
  arm->set_temperature(61);
  const auto deadline{std::chrono::steady_clock::now() + std::chrono::seconds{5}};
  while (!powered_off() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
  }
  ASSERT_TRUE(powered_off());
  std::this_thread::sleep_for(std::chrono::milliseconds{400});

  // Both are true now: the arm was cut and the link timed out. The first read() reports the cut.
  EXPECT_EQ(read_once(), hardware_interface::return_type::ERROR);
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), OVERHEAT_FAULT);

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

// AI-generated (Claude, Anthropic).
// Prompt:
//   #46 test review H1: on a clean exit the controller manager takes the component from ACTIVE
//   straight to FINALIZED; check that a stop reaches the arm on the way.

TEST_F(LoadRobotSystem, ShutdownFromActiveSendsAStop)
{
  // A clean exit (Ctrl+C, launch stopped): the controller manager's pre-shutdown callback takes
  // every component to FINALIZED. From ACTIVE, ros2_control runs on_deactivate (which sends the
  // stop and waits for its ack) and then on_shutdown (which closes the port). Our safety on exit
  // depends on that order in code we do not own: this test locks it in.
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  const std::size_t stops_before{arm->received_requests(0x29)};

  EXPECT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
      hardware_interface::lifecycle_state_names::FINALIZED),
    hardware_interface::return_type::OK);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  // halt() returns only once the arm acknowledged the stop, so it has already arrived.
  EXPECT_GT(arm->received_requests(0x29), stops_before) << "No stop reached the arm on shutdown";

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, InitRejectsAMissingOrWrongFaultInterface)
{
  // #35: without its fault interface the supervisor would read nothing, so the driver refuses
  // to start, the same way it refuses a joint without its interfaces.
  const std::vector<std::string> bad{
    "",                                                                 // no <gpio>
    R"(<gpio name="arm"><state_interface name="fault_code"/></gpio>)",  // wrong name
    R"(<gpio name="driver"><state_interface name="fault"/></gpio>)",    // wrong interface
    R"(<gpio name="driver"><state_interface name="fault_code"/>)"
    R"(<command_interface name="fault_code"/></gpio>)",  // a command too
  };
  for (const auto & gpio : bad) {
    SCOPED_TRACE(gpio);
    load(urdf_with(all_good_joints(), FAKE_BACKEND, read_description(), gpio));

    EXPECT_FALSE(loaded_);
    EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
}

TEST_F(LoadRobotSystem, InitAcceptsMoreDriverStatesAndOtherGpios)
{
  // Only the fault is required: S3 adds driver states such as a temperature (#115), and other
  // gpios may come, without the driver refusing to start.
  const std::string gpios{R"(<gpio name="driver"><state_interface name="fault_code"/>)"
    R"(<state_interface name="temperature_max"/></gpio>)"
    R"(<gpio name="gripper"><state_interface name="position"/></gpio>)"};
  load(urdf_with(all_good_joints(), FAKE_BACKEND, read_description(), gpios));

  EXPECT_TRUE(loaded_);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LoadRobotSystem, FaultCodeIsZeroWhileTheLinkIsAlive)
{
  // The fake arm never times out: once active, every read() reports "no fault".
  load(urdf_with(all_good_joints()));
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), NO_FAULT);
}

// #120: the fake pose is computed at the read() that publishes it, so its age is 0, and NaN
// ("unknown") before any read, like the joints.
TEST_F(LoadRobotSystem, PoseAgeIsUnknownAfterConfigureAndZeroOnTheFakeArm)
{
  load(urdf_with(all_good_joints()));
  ASSERT_TRUE(loaded_);
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_TRUE(std::isnan(state("driver", "pose_age_ms").value_or(0.0)));

  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "pose_age_ms").value_or(std::nan("")), 0.0);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "pose_age_ms").value_or(std::nan("")), 0.0);
}

// #120: the pose age is optional. A URDF without it, as before, loads and runs, and the plugin
// exports no such interface.
TEST_F(LoadRobotSystem, RunsWithoutThePoseAge)
{
  auto & resource_manager{
    load(urdf_with(all_good_joints(), FAKE_BACKEND, read_description(), FAULT_ONLY_GPIO))};
  ASSERT_TRUE(loaded_);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), NO_FAULT);
  const auto keys{resource_manager.state_interface_keys()};
  EXPECT_EQ(std::count(keys.begin(), keys.end(), "driver/pose_age_ms"), 0);
}

// #120, end to end on the pseudo-terminal arm. While it answers, a reply comes every turn (10 ms
// at rest), so the pose read() publishes is young: 100 ms leaves room for a turn that asked the
// temperatures and for a loaded test machine. Once the arm is quiet, the pose stays the last
// one and its age is at least the silence. The age is not reset by the link fault.
TEST_F(LoadRobotSystem, PoseAgeIsSmallWhileTheArmAnswersAndGrowsWhenItGoesQuiet)
{
  constexpr std::chrono::milliseconds QUIET_FOR{300};
  constexpr double YOUNG_MS{100.0};
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  const double alive{state("driver", "pose_age_ms").value_or(std::nan(""))};
  EXPECT_TRUE(std::isfinite(alive));
  EXPECT_GE(alive, 0.0);
  EXPECT_LT(alive, YOUNG_MS);

  arm.reset();
  std::this_thread::sleep_for(QUIET_FOR);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_GE(
    state("driver", "pose_age_ms").value_or(std::nan("")), static_cast<double>(QUIET_FOR.count()));

  resource_manager_.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, DeactivateFailsWhenTheArmDoesNotAcknowledgeTheStop)
{
  // #103. The arm answers poses but never acknowledges the stop: the line is alive and the
  // request is lost. on_deactivate must report ERROR after REQUEST_TIMEOUT (a stop nobody
  // confirmed is not a stop), on_error takes the component to UNCONFIGURED, the thread keeps
  // resending the stop meanwhile, and the component can be configured again.
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  arm->set_ack_requests(false);
  const auto start{std::chrono::steady_clock::now()};
  EXPECT_EQ(
    go_to(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE),
    hardware_interface::return_type::ERROR);
  const auto elapsed{std::chrono::steady_clock::now() - start};
  EXPECT_GE(elapsed, positronic_hardware::ArmBackend::REQUEST_TIMEOUT);
  EXPECT_LT(elapsed, 2 * positronic_hardware::ArmBackend::REQUEST_TIMEOUT);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  // One stop per turn while the request stays pending: at least two by now.
  std::this_thread::sleep_for(std::chrono::milliseconds{100});
  EXPECT_GE(arm->received_requests(0x29), 2u);

  // The arm acknowledges again: configure and activate work as on a fresh start.
  arm->set_ack_requests(true);
  EXPECT_EQ(configure(), hardware_interface::return_type::OK);
  EXPECT_EQ(activate(), hardware_interface::return_type::OK);

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, ActivateFailsWhenTheArmDoesNotAcknowledgeThePowerOn)
{
  // #103. The power_on is taken and never acknowledged: activate must fail with ERROR, the
  // component goes to UNCONFIGURED through on_error, which asks for a stop, so a stop follows
  // the last power_on on the wire and no target ever leaves. Once the arm acknowledges again,
  // activate succeeds.
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  arm->set_ack_requests(false);
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  const auto start{std::chrono::steady_clock::now()};
  EXPECT_EQ(activate(), hardware_interface::return_type::ERROR);
  const auto elapsed{std::chrono::steady_clock::now() - start};
  EXPECT_GE(elapsed, positronic_hardware::ArmBackend::REQUEST_TIMEOUT);
  EXPECT_LT(elapsed, 2 * positronic_hardware::ArmBackend::REQUEST_TIMEOUT);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  std::this_thread::sleep_for(std::chrono::milliseconds{100});
  EXPECT_GE(arm->received_requests(0x10), 2u);
  EXPECT_FALSE(arm->received_send_angles());
  // on_error asked for a halt: a stop went out after the last power_on.
  const std::optional<std::size_t> last_power_on{arm->last_request(0x10)};
  const std::optional<std::size_t> last_stop{arm->last_request(0x29)};
  ASSERT_TRUE(last_power_on.has_value());
  ASSERT_TRUE(last_stop.has_value());
  EXPECT_GT(last_stop.value_or(0), last_power_on.value_or(0));

  arm->set_ack_requests(true);
  EXPECT_EQ(activate(), hardware_interface::return_type::OK);

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, LinkTimeoutLatchesTheFaultHoldsTargetsAndActivateClearsIt)
{
  // #35, end to end through the resource manager, on a pseudo-terminal like the real UART.
  // The test sleeps because SerialArm reads the real clock; the exact 200 ms boundary is
  // tested without sleeps in test_link_watchdog.cpp.
  constexpr std::chrono::milliseconds PAST_THE_TIMEOUT{300};
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  // The slowest ack seen on the bench (a power_on, 58 ms): activate must still succeed.
  arm->set_ack_delay(std::chrono::milliseconds{58});
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), NO_FAULT);

  // The arm goes silent mid-run. The port stays open, so the driver's writes still succeed:
  // only the replies are missing, like a line that drops the arm's answers.
  arm.reset();
  std::this_thread::sleep_for(PAST_THE_TIMEOUT);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), LINK_TIMEOUT_FAULT);
  // Reported, not an ERROR: the component stays ACTIVE for the supervisor to act on.
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // The arm answers again and a controller asks for a new J1. The fault stays latched and no
  // target reaches the wire, even though the link would carry it now.
  arm = std::make_unique<ReplyingArm>(controller_);
  const double j1{state(JOINTS.at(0), "position").value_or(0.0)};
  ASSERT_TRUE(command(JOINTS.at(0), j1 + 0.1));
  ASSERT_TRUE(cycles(10));
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), LINK_TIMEOUT_FAULT);
  EXPECT_TRUE(arm->received_anything());  // the driver kept asking for angles
  EXPECT_FALSE(arm->received_send_angles());

  // Only a deactivate and an activate clear it, and then targets go out again.
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), NO_FAULT);
  ASSERT_TRUE(command(JOINTS.at(0), j1 + 0.1));
  bool sent{false};
  for (int cycle{0}; cycle < 20 && !sent; ++cycle) {
    ASSERT_EQ(write_once(), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds{30});
    sent = arm->received_send_angles();
  }
  EXPECT_TRUE(sent);

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, ActivateRefusesASilentLink)
{
  // has_sample() stays true after the first reply, so a link that died later must be caught
  // by on_activate itself, before power_on.
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);

  arm.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds{300});
  EXPECT_EQ(activate(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  resource_manager_.reset();
  ::close(controller_);
}

// #103 part 2. The pseudo-terminal arm answers the same recorded pose whatever it is told, so a
// target N deg away stays N deg away for as long as it is commanded: the arm "does not follow".

TEST_F(LoadRobotSystem, ATargetInsideTheTrackingThresholdIsNeverAFault)
{
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  // 2 deg from the pose: inside the 8 deg default, for twice the 300 ms.
  const double j1{state(JOINTS.at(0), "position").value_or(0.0)};
  ASSERT_TRUE(command(JOINTS.at(0), j1 + 2.0 * DEG));
  ASSERT_TRUE(cycles(20));
  EXPECT_TRUE(arm->received_send_angles());
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), NO_FAULT);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, ATargetTheArmDoesNotFollowLatchesTheTrackingFault)
{
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  // 9 deg from the pose, past the 8 deg default, for 600 ms: fault 2 after the 300 ms.
  const double j1{state(JOINTS.at(0), "position").value_or(0.0)};
  ASSERT_TRUE(command(JOINTS.at(0), j1 + 9.0 * DEG));
  ASSERT_TRUE(cycles(20));
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), TRACKING_ERROR_FAULT);
  // Reported like the link timeout: the component stays ACTIVE for the supervisor, the backend
  // was asked to stop, and the stop went out after the last send.
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_GE(arm->received_requests(0x29), 1U);
  const std::size_t sends_at_fault{arm->received_sends()};

  // Latched: a new target does not clear it and does not go out.
  ASSERT_TRUE(command(JOINTS.at(0), j1));
  ASSERT_TRUE(cycles(10));
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), TRACKING_ERROR_FAULT);
  EXPECT_EQ(arm->received_sends(), sends_at_fault);

  // Only a deactivate and an activate clear it.
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), NO_FAULT);

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, ATargetThatComesBackInsideBeforeTheTimeIsNoFault)
{
  ASSERT_TRUE(load_on_pseudo_terminal());
  auto arm{std::make_unique<ReplyingArm>(controller_)};
  ASSERT_EQ(configure(), hardware_interface::return_type::OK);
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);

  // 9 deg away for about 100 ms, then the target is the pose again: the clock never reaches
  // 300 ms, and the next target is followed at once (the arm is already there).
  const double j1{state(JOINTS.at(0), "position").value_or(0.0)};
  ASSERT_TRUE(command(JOINTS.at(0), j1 + 9.0 * DEG));
  ASSERT_TRUE(cycles(3));
  ASSERT_TRUE(command(JOINTS.at(0), j1));
  ASSERT_TRUE(cycles(20));
  EXPECT_DOUBLE_EQ(state("driver", "fault_code").value_or(std::nan("")), NO_FAULT);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  resource_manager_.reset();
  arm.reset();
  ::close(controller_);
}

TEST_F(LoadRobotSystem, InitRejectsABadTrackingParameter)
{
  // Below the 1 deg floor would call every move a fault; the others are the other edges of the
  // two ranges, a NaN and a fraction where an integer is expected.
  for (const auto & [name, value] :
    {std::pair{"tracking_error_deg", "0.5"}, std::pair{"tracking_error_deg", "45.5"},
      std::pair{"tracking_error_deg", "nan"}, std::pair{"tracking_error_ms", "49"},
      std::pair{"tracking_error_ms", "5001"}, std::pair{"tracking_error_ms", "300.5"}})
  {
    load(urdf_with(all_good_joints(), param("backend", "fake") + param(name, value)));
    EXPECT_FALSE(loaded_) << name << "=" << value;
    EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) << name;
  }
}

namespace
{

constexpr char GRIPPER[]{"gripper_controller"};
constexpr double GRIPPER_CLOSED{-0.74};
constexpr double GRIPPER_OPEN{0.15};
constexpr std::string_view GRIPPER_LIMIT{
  R"(<limit effort="10.0" lower="-0.74" upper="0.15" velocity="1.0"/>)"};

std::string read_gripper_description()
{
  const std::string path{
    ament_index_cpp::get_package_share_directory("positronic_description") +
    "/urdf/mycobot_280_jn_sim/mycobot_280_jn_gripper_camera.urdf"};
  const std::ifstream file{path};
  std::stringstream content;
  content << file.rdbuf();
  return content.str();
}

std::string urdf_with_gripper(const std::string & description = read_gripper_description())
{
  return urdf_with(all_good_joints() + good_joint(GRIPPER), FAKE_BACKEND, description);
}

}  // namespace

TEST_F(LoadRobotSystem, GripperDescriptionHasTheGripperLimit)
{
  EXPECT_NE(read_gripper_description().find(GRIPPER_LIMIT), std::string::npos);
}

TEST_F(LoadRobotSystem, InitAcceptsTheGripperAsASeventhJoint)
{
  auto & resource_manager{load(urdf_with_gripper())};
  ASSERT_TRUE(loaded_);
  EXPECT_EQ(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  const std::string gripper{GRIPPER};
  const auto states{resource_manager.state_interface_keys()};
  EXPECT_NE(std::find(states.begin(), states.end(), gripper + "/position"), states.end());
  EXPECT_NE(std::find(states.begin(), states.end(), gripper + "/velocity"), states.end());
  const auto commands{resource_manager.command_interface_keys()};
  EXPECT_NE(std::find(commands.begin(), commands.end(), gripper + "/position"), commands.end());
}

TEST_F(LoadRobotSystem, InitRejectsAGripperWithSwappedLimits)
{
  load(urdf_with_gripper(replaced(
    read_gripper_description(), GRIPPER_LIMIT,
    R"(<limit effort="10.0" lower="0.15" upper="-0.74" velocity="1.0"/>)")));
  EXPECT_FALSE(loaded_);
}

TEST_F(LoadRobotSystem, GripperStartsAtItsMeasuredValue)
{
  load(urdf_with_gripper());
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_EQ(state(GRIPPER, "position").value_or(std::nan("")), GRIPPER_OPEN);
  EXPECT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_EQ(state(GRIPPER, "position").value_or(std::nan("")), GRIPPER_OPEN);
  EXPECT_EQ(state(GRIPPER, "velocity").value_or(std::nan("")), 0.0);
}

TEST_F(LoadRobotSystem, GripperTargetsReachTheFakeArm)
{
  load(urdf_with_gripper());
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  for (const double target : {GRIPPER_CLOSED, (GRIPPER_CLOSED + GRIPPER_OPEN) / 2.0,
      GRIPPER_OPEN})
  {
    ASSERT_TRUE(command(GRIPPER, target));
    ASSERT_EQ(write_once(), hardware_interface::return_type::OK) << target;
    ASSERT_EQ(read_once(), hardware_interface::return_type::OK) << target;
    EXPECT_NEAR(state(GRIPPER, "position").value_or(std::nan("")), target, 1e-9) << target;
  }
}

TEST_F(LoadRobotSystem, GripperVelocityFollowsThePosition)
{
  load(urdf_with_gripper());
  ASSERT_EQ(activate(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  ASSERT_TRUE(command(GRIPPER, GRIPPER_CLOSED));
  ASSERT_EQ(write_once(), hardware_interface::return_type::OK);
  ASSERT_EQ(read_once(), hardware_interface::return_type::OK);
  EXPECT_NEAR(
    state(GRIPPER, "velocity").value_or(std::nan("")), (GRIPPER_CLOSED - GRIPPER_OPEN) / 0.01,
    1e-6);
}

TEST_F(LoadRobotSystem, WriteRefusesAGripperTargetPastItsLimits)
{
  for (const double target :
    {GRIPPER_OPEN + 5.0 * DEG, GRIPPER_CLOSED - 5.0 * DEG, std::nan(""),
      std::numeric_limits<double>::infinity()})
  {
    load(urdf_with_gripper());
    ASSERT_EQ(activate(), hardware_interface::return_type::OK);
    ASSERT_TRUE(command(GRIPPER, target));
    EXPECT_EQ(write_once(), hardware_interface::return_type::ERROR) << target;
    EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) << target;
  }
}

TEST_F(LoadRobotSystem, WriteSendsAGripperTargetExactlyAtItsLimits)
{
  for (const double target : {GRIPPER_CLOSED, GRIPPER_OPEN}) {
    load(urdf_with_gripper());
    ASSERT_EQ(activate(), hardware_interface::return_type::OK);
    ASSERT_TRUE(command(GRIPPER, target));
    EXPECT_EQ(write_once(), hardware_interface::return_type::OK) << target;
  }
}

TEST_F(LoadRobotSystem, ActivateFailsWhenTheGripperNeverAnswers)
{
  std::array<char, 128> port{};
  const int controller{open_pseudo_terminal(port)};
  ASSERT_GE(controller, 0);
  auto arm{std::make_unique<ReplyingArm>(controller)};

  load(urdf_with(
    all_good_joints() + good_joint(GRIPPER),
    param("backend", "serial") + param("port", port.data()), read_gripper_description()));
  ASSERT_TRUE(loaded_);
  EXPECT_EQ(activate(), hardware_interface::return_type::ERROR);
  EXPECT_NE(component_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_LT(find_request(arm->received(), 0x65), arm->received().size());
  EXPECT_EQ(find_request(arm->received(), 0x10), arm->received().size());

  resource_manager_.reset();
  arm.reset();
  ::close(controller);
}
