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

// The two timings of the serial thread (#32): a period above the arm's own busy time and at most
// half the #35 link timeout, a settle that fits below the longest period. servo_temp_limit_c only
// goes down from 60: a lower limit cuts a warm arm on purpose, which is how the hardware test
// exercises the cut without heating anything (#115). The tracking ranges are wide for the bench
// to try values. The one rule no range can check is that tracking_error_deg stays wider than the
// controller's trajectory tolerance (#103).
constexpr std::array NUMERIC_PARAMETERS{
  NumericParameter{
    "moving_period_ms", 20, MAX_MOVING_PERIOD_MS, true,
    [](HardwareConfig & config, double value) {
      config.timing.moving_period = std::chrono::milliseconds{static_cast<int>(value)};
    }},
  NumericParameter{
    "send_settle_ms", 0, MAX_MOVING_PERIOD_MS - 1, true,
    [](HardwareConfig & config, double value) {
      config.timing.send_settle = std::chrono::milliseconds{static_cast<int>(value)};
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
};

/// The <param> names that are not a number in a range, each with its own rule in read_special().
constexpr std::array SPECIAL_NAMES{
  std::string_view{"backend"}, std::string_view{"port"}, std::string_view{"baud"},
  std::string_view{"velocity_filter_alpha"}};

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

/// The four SPECIAL_NAMES into `config` when the URDF sets them. Empty, or the reason.
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
  // The settle is part of a turn, with the reply before it. A settle that fills the turn would
  // only stretch every turn by itself and hide the value under test in a sweep.
  if (error.empty() && config.timing.send_settle >= config.timing.moving_period) {
    error = "send_settle_ms (" + std::to_string(config.timing.send_settle.count()) +
      ") must be below moving_period_ms (" +
      std::to_string(config.timing.moving_period.count()) + ").";
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
  }
  std::ostringstream line;
  line << std::fixed << "Backend '" << backend << "', port " << config.port << " at " << config.baud
       << " baud, velocity filter alpha " << std::setprecision(3) << config.velocity_filter_alpha
       << ", moving period " << config.timing.moving_period.count() << " ms, send settle "
       << config.timing.send_settle.count() << " ms, servo temperature limit "
       << config.servo_temp_limit_c << " degC, tracking error " << std::setprecision(1)
       << config.tracking_error_deg << " deg for " << config.tracking_error_ms << " ms.";
  return line.str();
}

}  // namespace positronic_hardware
