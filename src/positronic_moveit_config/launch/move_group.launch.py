# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    """Return the MoveIt configuration."""
    return (
        MoveItConfigsBuilder('positronic', package_name='positronic_moveit_config')
        .robot_description(
            file_path=os.path.join(get_package_share_directory('positronic_description'),
                                   'urdf', 'positronic_gazebo.urdf.xacro')
        )
        .robot_description_semantic(file_path='config/positronic.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    config = build_moveit_config()

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[config.to_dict(), {'use_sim_time': use_sim_time}],
    )

    rviz_file = os.path.join(
        get_package_share_directory('positronic_moveit_config'), 'config', 'moveit.rviz'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_file],
        parameters=[
            config.robot_description,
            config.robot_description_semantic,
            config.robot_description_kinematics,
            config.planning_pipelines,
            config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    planning_scene = Node(
        package='positronic_moveit_config',
        executable='planning_scene.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('planning_scene')),
    )
    return [move_group, planning_scene, rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false',
                                  description='true when the clock comes from Gazebo'),
            DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
            DeclareLaunchArgument('planning_scene', default_value='true',
                                  description='Load the workcell obstacles (#42) into move_group'),
            OpaqueFunction(function=launch_setup),
        ]
    )
