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
Show the real arm in RViz on a laptop.

Starts robot_state_publisher with the arm URDF and RViz with a saved config.
/joint_states is not published here: it comes from the Jetson driver over
ROS_DOMAIN_ID 17. Do not run it together with a joint_state_publisher_gui.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # The URDF is read here, when Python runs, so the path is built with os.path, not
    # with launch substitutions, which only get a value later.
    urdf = os.path.join(
        get_package_share_directory('positronic_description'),
        'urdf', 'mycobot_280_jn_sim', 'mycobot_280_jn.urdf')
    with open(urdf) as f:
        robot_description = f.read()

    rviz_config = os.path.join(
        get_package_share_directory('positronic_bringup'), 'rviz', 'mirror.rviz')

    # Reads /joint_states and publishes the TF of every link.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='both',
        arguments=['-d', rviz_config])

    return LaunchDescription([robot_state_publisher, rviz])
