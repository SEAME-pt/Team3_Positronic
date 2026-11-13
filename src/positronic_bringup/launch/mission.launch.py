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
Start the mission node with the MoveIt parameters it needs.

The MoveIt configuration is built the same way as in move_group.launch.py, so
both launches load the same robot. Only robot_description, the SRDF and
kinematics.yaml are passed to the node: its MoveIt backend reads them on an
internal node to build MoveGroupInterface. The node starts unconfigured; bring
it up with `ros2 lifecycle set /mission_node configure` and `activate`.
Activation fails if move_group isn't reachable. Set use_sim_time:=true when
Gazebo provides the clock.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    """Build the same MoveIt configuration as move_group.launch.py."""
    return (
        MoveItConfigsBuilder('positronic', package_name='positronic_moveit_config')
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('positronic_description'),
                'urdf',
                'positronic_gazebo.urdf.xacro',
            )
        )
        .robot_description_semantic(file_path='config/positronic.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )


def launch_setup(context, *args, **kwargs):
    use_sim_time = (
        LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    )
    config = build_moveit_config()

    mission_node = Node(
        package='positronic_mission',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[
            config.robot_description,
            config.robot_description_semantic,
            config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )
    return [mission_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='true when the clock comes from Gazebo',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
