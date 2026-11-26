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

#include "positronic_hardware/parameters.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <vector>

#include "positronic_hardware/link_watchdog.hpp"

namespace positronic_hardware
{

namespace
{

using Parameters = std::unordered_map<std::string, std::string>;

/// #35: one lost reply costs up to moving_period + REPLY_TIMEOUT + a late reply of silence. A
/// period at most half the link timeout keeps room for at least one lost reply, so a single
/// bad turn never trips the fault.
constexpr int MAX_MOVING_PERIOD_MS{static_cast<int>(LinkWatchdog::DEFAULT_TIMEOUT.count() / 2)};

/// #150: the wait for the sync-write is the first part of the silence after a send. With two
/// lost replies (2 x 50 ms, each followed by the 8 ms LATE_REPLY_DRAIN) and a late reply (20 ms)
/// after it, the default 60 comes to ~196 ms, under the 200 ms link timeout; at 80 two lost
/// replies trip it. Below 20 ms almost every sync-write would be
/// late (it ends 19 ms after the send at p50).
constexpr int MIN_ECHO_TIMEOUT_MS{20};
constexpr int MAX_ECHO_TIMEOUT_MS{80};

/// One numeric <param>: its name, the range it accepts, whether it must be a whole number, and
/// the field of the config it fills. A new numeric parameter is one row of NUMERIC_PARAMETERS.
struct NumericParameter
{
  std::string_view name;
  double min{0.0};
  double max{0.0};
  bool whole{true};
  using Store = void (*)(HardwareConfig &, double);
  Store store{nullptr};
};

// The two timings of the serial thread that the URDF sets (#32, #150): a period above the arm's
// own busy time and at most half the #35 link timeout, and the longest wait for the Atom's
// sync-write after a send. The echo timeout may be longer than the period: it only counts when
// the Atom is late. The settle is a constant since #150, when the moving turn stopped using it
// (SerialTiming::send_settle), and only the gripper's set waits it. servo_temp_limit_c only goes
// down from 60: a lower limit cuts a warm arm on purpose, which is how the hardware test
// exercises the cut without heating anything (#115). The tracking ranges are wide for the bench
// to try values. The one rule no range can check is that tracking_error_deg stays wider than the
// controller's trajectory tolerance (#103). supervisor_timeout_ms starts at 0, which turns the
// dead-man check off (#136); 5 s is already far longer than any halt should wait.
constexpr std::array NUMERIC_PARAMETERS{
  NumericParameter{
    "moving_period_ms", 20, MAX_MOVING_PERIOD_MS, true,
    [](HardwareConfig & config, double value) {
      config.timing.moving_period = std::chrono::milliseconds{static_cast<int>(value)};
    }},
  NumericParameter{
    "echo_timeout_ms", MIN_ECHO_TIMEOUT_MS, MAX_ECHO_TIMEOUT_MS, true,
    [](HardwareConfig & config, double value) {
      config.timing.echo_timeout = std::chrono::milliseconds{static_cast<int>(value)};
    }},
  NumericParameter{
    "servo_temp_limit_c", MIN_SERVO_TEMP_LIMIT_C, SERVO_TEMP_LIMIT_C, true,
    [](HardwareConfig & config, double value) {
      config.servo_temp_limit_c = static_cast<int>(value);
    }},
  NumericParameter{
    "tracking_error_deg", MIN_TRACKING_ERROR_DEG, MAX_TRACKING_ERROR_DEG, false,
    [](HardwareConfig & config, double value) {
      config.tracking_error_deg = value;
    }},
  NumericParameter{
    "tracking_error_ms", MIN_TRACKING_ERROR_MS, MAX_TRACKING_ERROR_MS, true,
    [](HardwareConfig & config, double value) {
      config.tracking_error_ms = static_cast<int>(value);
    }},
  NumericParameter{
    "supervisor_timeout_ms", 0, MAX_SUPERVISOR_TIMEOUT_MS, true,
    [](HardwareConfig & config, double value) {
      config.supervisor_timeout_ms = static_cast<int>(value);
    }},
};

/// The <param> names that are not a number in a range, each with its own rule in read_special().
constexpr std::array SPECIAL_NAMES{
  std::string_view{"backend"},
  std::string_view{"fake_mode"},
  std::string_view{"port"},
  std::string_view{"baud"},
  std::string_view{"velocity_filter_alpha"},
  std::string_view{"pose_from_poll"},
  std::string_view{"load_relief"}};

/// The value of `key`, or nullptr if the URDF does not set it.
const std::string * find_parameter(const Parameters & parameters, std::string_view key)
{
  const auto it{parameters.find(std::string{key})};
  if (it == parameters.end()) {
    return nullptr;
  }
  return &it->second;
}

/// `text` as a number, or nullopt unless the whole text is one. from_chars ignores the locale
/// and throws nothing, unlike std::stoi/std::stod.
template<typename Number>
std::optional<Number> parse_number(const std::string & text)
{
  Number value{};
  const char * const end{std::next(text.data(), static_cast<std::ptrdiff_t>(text.size()))};
  const auto [stop, error]{std::from_chars(text.data(), end, value)};
  if (error != std::errc{} || stop != end) {
    return std::nullopt;
  }
  return value;
}

/// Reads one row of NUMERIC_PARAMETERS into `config` when the URDF sets it. Empty when the
/// parameter is absent or valid, the reason otherwise. A NaN fails the range: from_chars accepts
/// "nan", and every comparison with it is false.
std::string read_numeric(
  const Parameters & parameters, const NumericParameter & parameter, HardwareConfig & config)
{
  const std::string * text{find_parameter(parameters, parameter.name)};
  if (text == nullptr) {
    return {};
  }
  std::optional<double> value{};
  std::string kind{"a number"};
  if (parameter.whole) {
    value = parse_number<int>(*text);
    kind = "a whole number";
  } else {
    value = parse_number<double>(*text);
  }
  if (value.has_value() && value.value() >= parameter.min && value.value() <= parameter.max) {
    parameter.store(config, value.value());
    return {};
  }
  return "Parameter '" + std::string{parameter.name} + "' is '" + *text + "'; use " + kind +
         " in [" + std::to_string(std::lround(parameter.min)) + ", " +
         std::to_string(std::lround(parameter.max)) + "].";
}

/// The SPECIAL_NAMES into `config` when the URDF sets them. Empty, or the reason.
std::string read_special(const Parameters & parameters, HardwareConfig & config)
{
  if (const std::string * backend{find_parameter(parameters, "backend")}) {
    if (*backend != "serial" && *backend != "fake") {
      return "Parameter 'backend' is '" + *backend + "'; use 'serial' or 'fake'.";
    }
    config.backend = Backend::FAKE;
    if (*backend == "serial") {
      config.backend = Backend::SERIAL;
    }
  }
  if (const std::string * mode{find_parameter(parameters, "fake_mode")}) {
    if (*mode != "instant" && *mode != "measured") {
      return "Parameter 'fake_mode' is '" + *mode + "'; use 'instant' or 'measured'.";
    }
    config.fake_mode = FakeMode::INSTANT;
    if (*mode == "measured") {
      config.fake_mode = FakeMode::MEASURED;
    }
  }
  if (const std::string * port{find_parameter(parameters, "port")}) {
    if (port->empty()) {
      return "Parameter 'port' is empty.";
    }
    config.port = *port;
  }
  if (const std::string * baud{find_parameter(parameters, "baud")}) {
    // Only the rates PosixSerialPort can set.
    const std::optional<int> value{parse_number<int>(*baud)};
    if (value != 115200 && value != 1000000) {
      return "Parameter 'baud' is '" + *baud + "'; use 115200 or 1000000.";
    }
    config.baud = value.value();
  }
  if (const std::string * alpha{find_parameter(parameters, "velocity_filter_alpha")}) {
    // (0, 1]: an alpha of 0 would freeze the estimate.
    const double value{parse_number<double>(*alpha).value_or(0.0)};
    if (std::isnan(value) || value <= 0.0 || value > 1.0) {
      return "Parameter 'velocity_filter_alpha' is '" + *alpha + "'; use a number in (0, 1].";
    }
    config.velocity_filter_alpha = value;
  }
  if (const std::string * poll{find_parameter(parameters, "pose_from_poll")}) {
    // #170: the pose from the Atom's poll and the write-triggered turn, or the #150 turn.
    if (*poll != "true" && *poll != "false") {
      return "Parameter 'pose_from_poll' is '" + *poll + "'; use 'true' or 'false'.";
    }
    config.timing.pose_from_poll = (*poll == "true");
  }
  if (const std::string * relief{find_parameter(parameters, "load_relief")}) {
    // #183: the relief of a still joint pushing short of its target, or none.
    if (*relief != "true" && *relief != "false") {
      return "Parameter 'load_relief' is '" + *relief + "'; use 'true' or 'false'.";
    }
    config.relief.enabled = (*relief == "true");
  }
  return {};
}

/// `items` separated by ", ".
std::string join(const std::vector<std::string> & items)
{
  std::string joined;
  for (const std::string & item : items) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += item;
  }
  return joined;
}

/// Empty when every <param> is one the driver knows. Otherwise the reason, with every unknown
/// name in alphabetical order, so one run shows every typo, and every known name in table order.
std::string unknown_parameters(const Parameters & parameters)
{
  const auto known{[](const std::string & name) {
      const auto same{[&name](const NumericParameter & row) {
          return row.name == name;
        }};
      return std::find(SPECIAL_NAMES.begin(), SPECIAL_NAMES.end(), name) != SPECIAL_NAMES.end() ||
             std::any_of(NUMERIC_PARAMETERS.begin(), NUMERIC_PARAMETERS.end(), same);
    }};
  std::vector<std::string> unknown;
  for (const auto & entry : parameters) {
    if (!known(entry.first)) {
      unknown.push_back("'" + entry.first + "'");
    }
  }
  if (unknown.empty()) {
    return {};
  }
  std::sort(unknown.begin(), unknown.end());
  std::string noun{"parameter"};
  if (unknown.size() > 1) {
    noun = "parameters";
  }
  std::vector<std::string> names{SPECIAL_NAMES.begin(), SPECIAL_NAMES.end()};
  for (const NumericParameter & row : NUMERIC_PARAMETERS) {
    names.emplace_back(row.name);
  }
  return "Unknown hardware " + noun + " " + join(unknown) + ". Known: " + join(names) + ".";
}

}  // namespace

ParsedParameters parse_hardware_parameters(const Parameters & parameters)
{
  HardwareConfig config{};
  std::string error{unknown_parameters(parameters)};
  if (error.empty()) {
    error = read_special(parameters, config);
  }
  for (const NumericParameter & parameter : NUMERIC_PARAMETERS) {
    if (error.empty()) {
      error = read_numeric(parameters, parameter, config);
    }
  }
  if (!error.empty()) {
    return {std::nullopt, error};
  }
  return {config, {}};
}

std::string describe(const HardwareConfig & config)
{
  std::string backend{"fake"};
  if (config.backend == Backend::SERIAL) {
    backend = "serial";
  } else if (config.fake_mode == FakeMode::MEASURED) {
    backend = "fake' in mode 'measured";
  }
  std::string poll{"off"};
  if (config.timing.pose_from_poll) {
    poll = "on";
  }
  std::string relief{"off"};
  if (config.relief.enabled) {
    relief = "on";
  }
  std::ostringstream line;
  line << std::fixed << "Backend '" << backend << "', port " << config.port << " at " << config.baud
       << " baud, velocity filter alpha " << std::setprecision(3) << config.velocity_filter_alpha
       << ", moving period " << config.timing.moving_period.count() << " ms, echo timeout "
       << config.timing.echo_timeout.count() << " ms, pose from poll " << poll
       << ", servo temperature limit " << config.servo_temp_limit_c << " degC, tracking error "
       << std::setprecision(1) << config.tracking_error_deg << " deg for "
       << config.tracking_error_ms << " ms, supervisor timeout " << config.supervisor_timeout_ms
       << " ms, load relief " << relief << ".";
  return line.str();
}

}  // namespace positronic_hardware
