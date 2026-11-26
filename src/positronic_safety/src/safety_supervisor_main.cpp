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

#include <memory>

#include "positronic_safety/safety_supervisor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto supervisor = std::make_shared<positronic_safety::SafetySupervisor>();
  // #136: on Ctrl+C rclcpp runs no lifecycle transition, so on_shutdown would never halt the
  // arm. rclcpp calls this on its signal thread before it shuts down, while spin() below still
  // runs the supervisor's callbacks. A weak pointer: the context outlives main().
  const std::weak_ptr<positronic_safety::SafetySupervisor> weak{supervisor};
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback([weak]() {
      if (const auto alive = weak.lock()) {
        alive->halt_before_exit();
      }
  });
  // One thread for every callback: the supervisor has no lock, on purpose.
  rclcpp::spin(supervisor->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
