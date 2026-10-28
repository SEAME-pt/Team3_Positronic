# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('positronic_bringup'), 'launch',
                         'gazebo.launch.py')
        )
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('positronic_moveit_config'), 'launch',
                         'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': 'true',
        }.items(),
    )
    return LaunchDescription(
        [
            gazebo,
            move_group,
        ]
    )
