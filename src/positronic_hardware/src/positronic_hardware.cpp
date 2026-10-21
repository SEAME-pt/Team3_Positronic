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

#include "positronic_hardware/positronic_hardware.hpp"
#include <string>
#include <vector>

namespace positronic_hardware
{
CallbackReturn RobotSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn RobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }

  return CallbackReturn::SUCCESS;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  (void)period;
  /*
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
        const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
        set_state(name_vel, get_command(name_vel));
        set_state(name_pos, get_state(name_pos) + get_state(name_vel) * period.seconds());
      }
      */
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

}  // namespace positronic_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(positronic_hardware::RobotSystem, hardware_interface::SystemInterface)
