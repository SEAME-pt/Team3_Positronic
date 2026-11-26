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

#ifndef POSITRONIC_HARDWARE__POSITRONIC_HARDWARE_HPP_
#define POSITRONIC_HARDWARE__POSITRONIC_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "positronic_hardware/echo_timeout_report.hpp"
#include "positronic_hardware/fake_arm.hpp"
#include "positronic_hardware/fault_code.hpp"
#include "positronic_hardware/joint_limit_check.hpp"
#include "positronic_hardware/link_watchdog.hpp"
#include "positronic_hardware/parameters.hpp"
#include "positronic_hardware/serial_arm.hpp"
#include "positronic_hardware/servo_temperature.hpp"
#include "positronic_hardware/tracking_check.hpp"
#include "positronic_hardware/velocity_estimator.hpp"
#include "positronic_interfaces/msg/heartbeat.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace positronic_hardware
{

/// The <gpio> of the <ros2_control> block that carries the fault, and its one state interface.
/// The supervisor reads "driver/fault_code".
inline constexpr std::string_view FAULT_GPIO{"driver"};
inline constexpr std::string_view FAULT_STATE{"fault_code"};

/// #120: the age of the pose read() publishes, in seconds (SI), on the same gpio: the time from the
/// arrival of the reply that carried the pose to the read() that publishes it. NaN until the
/// first pose. Optional: exported only when the <gpio name="driver"> declares it, as
/// positronic_real.urdf.xacro does; nothing reads it yet.
inline constexpr std::string_view POSE_AGE_STATE{"pose_age_s"};

/// Where the safety supervisor publishes its heartbeat (#136), every check of its timer.
inline constexpr std::string_view SUPERVISOR_HEARTBEAT_TOPIC{"/safety_supervisor/heartbeat"};

class RobotSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Fills config_ from the <hardware> parameters through parameters.hpp, and logs the result:
  /// the reason on a refusal, one INFO line with every value otherwise. A typo in the URDF must
  /// stop the driver, not be ignored.
  bool read_parameters(const std::unordered_map<std::string, std::string> & parameters);

  /// Fills limits_ from the <limit> tags of the URDF (#34). It parses the URDF itself
  /// (info.original_xml): with <limits enable="false"/> on the joints, ros2_control leaves no
  /// position limit in info.limits. False, with an error logged, when a joint has no usable
  /// position limit: the driver must not run without its last guard.
  bool load_limits(const hardware_interface::HardwareInfo & info);

  /// Latches `code` (#35): stores it, asks the backend to stop the arm, and logs a link timeout
  /// once (an overheat is logged by check_temperatures(), which knows the joint and the value,
  /// a supervisor loss by read()).
  /// Only the first fault is kept; on_activate clears it. No allocation: safe from read().
  void raise_fault(FaultCode code);

  /// The driver's status on /diagnostics (#121): OK for no fault, ERROR with the name and the
  /// reason of the latched one. Runs on the executor that spins the component's node, beside a
  /// transition that may be replacing arm_: it reads fault_ only.
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status);
  /// on_init's part of #121: the Updater on the component's node, with diagnose() and the timer
  /// that publishes a new fault at once. Nothing without a node (most tests).
  void start_diagnostics(const std::string & hardware_id);
  /// on_activate's wait for a link that died after the first reply (#35, #121): true once the
  /// arm answers again before `deadline`, false with one log line if it stays silent.
  bool link_alive_by(std::chrono::steady_clock::time_point deadline);

  /// Refuses a bad command (#34): asks the backend to stop the arm and logs the joint name and
  /// value. write() then returns ERROR, which takes the component out of ACTIVE.
  void refuse(const CommandCheck & check);

  /// on_activate's report after power_on (#133): the fresh mode frames sent and acknowledged.
  void report_power_on() const;

  /// on_activate's temperature check, before the servos get torque: waits for the first reading
  /// and refuses an arm that was cut or is within ACTIVATE_COOLDOWN_C of the limit. False, with
  /// the reason logged, when activation must not go on.
  bool temperatures_allow_activation();

  /// read()'s temperature check. ERROR, with one log line, when the serial thread has cut the
  /// arm for an overheat (after latching FaultCode::OVERHEAT), or when the component is active
  /// and the temperatures went stale.
  hardware_interface::return_type check_temperatures();
  /// read()'s unfollowed-command check (#103), while active and without a fault: compares the
  /// pose with the backend's last target and latches FaultCode::TRACKING_ERROR, with one log
  /// line (joint, error, limits), once a joint has been off for too long.
  void check_tracking();
  /// read()'s report of the sends that got no sync-write (#150), while active: one warning line
  /// with how many since the last one, at most every EchoTimeoutReport::DEFAULT_PERIOD, so turns
  /// that got slow do not go unseen.
  void report_echo_timeouts();
  /// on_init's part of the dead-man check (#136): one log line with the mode, and when it is on,
  /// the subscription to the supervisor's heartbeat. False, with the reason logged, when the
  /// check is on but the component has no ROS node to subscribe with.
  bool start_supervisor_check();
  /// on_activate's part: waits up to supervisor_timeout_ms for a heartbeat. True at once when
  /// the check is off.
  bool supervisor_allows_activation();
  /// True when the check is on and no heartbeat came for longer than supervisor_timeout_ms.
  /// One atomic load and one clock read: safe from read().
  [[nodiscard]] bool supervisor_silent() const;
  /// True while the component is ACTIVE: the only state in which targets go out and faults count.
  [[nodiscard]] bool is_active() const;

  /// The interfaces of one joint, looked up once so read() and write() build no string: two
  /// states the plugin fills, one command the controller fills and the plugin sends.
  struct JointHandles
  {
    hardware_interface::CommandInterface::SharedPtr command;
    hardware_interface::StateInterface::SharedPtr position;
    hardware_interface::StateInterface::SharedPtr velocity;
  };

  /// Asks the arm where the joints are, estimates the velocity from the change since the last
  /// call `dt` seconds ago, and stores both in the state interfaces, with the age of that pose
  /// (#120). No allocation, no blocking.
  void update_states(double dt);

  /// Writes `age` to driver/pose_age_s in seconds, NaN for nullopt, when the URDF declares it.
  void set_pose_age(std::optional<std::chrono::steady_clock::duration> age);

  HardwareConfig config_{};
  std::unique_ptr<ArmBackend> arm_;
  VelocityEstimator velocity_estimator_;
  TrackingCheck tracking_;
  std::array<JointHandles, NUM_JOINTS> handles_{};
  /// "driver/fault_code", looked up once like the joint handles.
  hardware_interface::StateInterface::SharedPtr fault_handle_;
  /// "driver/pose_age_s" (#120), or null when the URDF does not declare it.
  hardware_interface::StateInterface::SharedPtr pose_age_handle_;
  /// The latched fault. read() runs on the control loop and the lifecycle callbacks may run on
  /// another thread: an atomic keeps both sides safe without a lock.
  std::atomic<FaultCode> fault_{FaultCode::NONE};
  /// The sum of the periods read() got, in ns: the clock of FakeArm (#122).
  std::atomic<std::int64_t> control_time_ns_{0};

  /// #150: when read() warns about sends without a sync-write. Reset with every new backend,
  /// whose count starts at zero.
  EchoTimeoutReport echo_report_;

  /// Position limits of J1..J6 from the URDF, read once in on_init. write() checks every
  /// command against them before it reaches the arm.
  PositionLimits limits_{};

  bool has_gripper_{false};
  double gripper_closed_{0.0};
  double gripper_open_{0.0};
  JointHandles gripper_handles_{};
  double gripper_previous_{0.0};
  bool gripper_previous_valid_{false};

  /// The time of the supervisor's last heartbeat (#136), the same lock-free watchdog as the
  /// link's: the subscription callback writes it on an executor thread, read() reads it on the
  /// control loop. Empty while the check is off.
  std::optional<LinkWatchdog> supervisor_watchdog_;
  /// Created once in on_init and kept until the plugin is destroyed: a subscription made again
  /// at each configure would have to be discovered again before on_activate. Declared after the
  /// watchdog, so it goes first and its callback never sees the watchdog gone.
  rclcpp::Subscription<positronic_interfaces::msg::Heartbeat>::SharedPtr heartbeat_subscription_;

  // #121, last so they go first: their callbacks read the members above. Both run on the
  // node's default callback group, never at the same time, so diagnosed_ needs no atomic.
  /// The fault diagnose() last published.
  FaultCode diagnosed_{FaultCode::NONE};
  /// Null without a node: a resource manager loaded without an executor (most tests).
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  /// Publishes a new fault at once instead of at the next once-a-second status.
  rclcpp::TimerBase::SharedPtr fault_watch_;
};

}  // namespace positronic_hardware

#endif  // POSITRONIC_HARDWARE__POSITRONIC_HARDWARE_HPP_
