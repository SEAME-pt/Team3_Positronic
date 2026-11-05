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

positronic_mission::MissionNode::MissionNode()
    : LifecycleNode("mission_node"),
    publisher_(nullptr),
    timer_(nullptr)
{
    RCLCPP_INFO(get_logger(), "IN constructor");
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_configure(const rclcpp_lifecycle::State & /* state */)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    RCLCPP_INFO(get_logger(), "IN on_configure");
    // Create everything here
    publisher_ = create_publisher<Heartbeat>("/mission/heartbeat", qos);

    timer_ = create_wall_timer(100ms, [this]{heart_beat();});
    timer_->cancel();

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_cleanup(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "IN on_cleanup");
    publisher_ = nullptr;
    timer_ = nullptr;

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "IN on_activate");
    LifecycleNode::on_activate(state);

    timer_->reset();

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "IN on_deactivate");

    timer_->cancel();
    LifecycleNode::on_deactivate(state);

    return CallbackReturn::SUCCESS;
}

positronic_mission::CallbackReturn positronic_mission::MissionNode::on_shutdown(const rclcpp_lifecycle::State & /* state */)
{
    RCLCPP_INFO(get_logger(), "IN on_shutdown");
    // timer_.reset() also works!
    publisher_ = nullptr;
    timer_ = nullptr;

    return CallbackReturn::SUCCESS;
}

void positronic_mission::MissionNode::heart_beat()
{
    auto msg = Heartbeat();
    msg.stamp = now();

    // Will need a new variable for it later. And to update it
    msg.state = Heartbeat::STATE_IDLE;
    msg.node_name = get_name();

    publisher_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto mission_node = std::make_shared<positronic_mission::MissionNode>();
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
