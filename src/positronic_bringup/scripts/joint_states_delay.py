#!/usr/bin/env python3

# Copyright 2026 Positronic
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""
Measure the delay of /joint_states on this machine.

For every message: delay = now on this machine - header.stamp. Prints the
average and the maximum every second. Only valid if this machine and the
Jetson have their clocks synchronised (NTP). Stop with Ctrl+C.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatesDelay(Node):

    def __init__(self):
        super().__init__('joint_states_delay')
        self.delays_ms = []
        self.create_subscription(JointState, '/joint_states', self.on_message, 10)
        self.create_timer(1.0, self.report)

    def on_message(self, msg):
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        self.delays_ms.append((now - stamp) / 10**6)

    def report(self):
        if not self.delays_ms:
            self.get_logger().info('no messages')
            return
        n = len(self.delays_ms)
        avg = sum(self.delays_ms) / n
        worst = max(self.delays_ms)
        self.get_logger().info(f'{n} msgs  avg {avg:.1f} ms  max {worst:.1f} ms')
        self.delays_ms = []


def main():
    rclpy.init()
    node = JointStatesDelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
