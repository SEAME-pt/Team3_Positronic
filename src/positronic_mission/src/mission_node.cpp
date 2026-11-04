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

#include "positronic_mission/mission_node.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Heartbeat = positronic_interfaces::msg::Heartbeat;

class MissionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MissionNode()
  : LifecycleNode("mission_node"),
    _publisher(nullptr),
    _timer(nullptr),
    _state(7)
  {
    RCLCPP_INFO(get_logger(), "IN constructor");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "IN on_configure");
    // Create everything here
    _publisher = create_publisher<Heartbeat>("/mission/heartbeat", 10);

    _timer = create_wall_timer(100ms, std::bind(&MissionNode::heartBeat, this));
    _timer->cancel();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "IN on_cleanup");
    _publisher = nullptr;
    _timer = nullptr;

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "IN on_activate");
    LifecycleNode::on_activate(state);

    _timer->reset();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(get_logger(), "IN on_deactivate");

    _timer->cancel();
    LifecycleNode::on_deactivate(state);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "IN on_shutdown");
    // _timer.reset() also works!
    _publisher = nullptr;
    _timer = nullptr;

    return CallbackReturn::SUCCESS;
  }

  void heartBeat()
  {
    auto msg = Heartbeat();
    msg.stamp = now();

    // Will need a new variable for it later. And to update it
    msg.state = Heartbeat::STATE_IDLE;
    msg.node_name = get_name();

    _publisher->publish(msg);
  }

private:
  /*
        Need:
            action server
            action client
            publisher 10Hz
            subscriber /joint_states
            timer to track 10Hz
            state
    */
  rclcpp_lifecycle::LifecyclePublisher<Heartbeat>::SharedPtr _publisher;

  rclcpp::TimerBase::SharedPtr _timer;
  uint8_t _state;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto mission_node = std::make_shared<MissionNode>();
  rclcpp::spin(mission_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

/*
    Three places that use state:

    handle_goal reads it to decide whether to accept (idle) or reject
    the execution code and the cancel/stop code write it as the arm moves between states
    heartBeat() reads it to fill msg.state
*/
