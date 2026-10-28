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
Start Gazebo with the arm and the two ros2_control controllers.

Gazebo, robot_state_publisher, the /clock bridge and the spawn start together:
the spawn waits for Gazebo and for robot_description by itself. The two
controller spawners are chained after the spawn exits: joint_state_broadcaster
first, then arm_controller. The /clock bridge makes ROS 2 follow the
simulation time.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Gazebo finds package://positronic_description/... under this folder.
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([FindPackageShare('positronic_description'), '..']))

    # Workcell world from this package, by full path. -r runs it from the start.
    # Closing Gazebo ends the launch.
    world = PathJoinSubstitution(
        [FindPackageShare('positronic_bringup'), 'worlds', 'workcell.sdf'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items())

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('positronic_bringup'), 'config', 'ros2_controllers.yaml'])

    # The xacro output goes to the robot_description parameter, as plain text.
    # The yaml path is passed to the xacro, so the model does not know where it lives.
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('positronic_description'), 'urdf', 'positronic_gazebo.urdf.xacro',
            ]),
            ' controllers_file:=', controllers_yaml,
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # Simulation time from Gazebo to ROS 2.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='both',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    # Reads robot_description and creates the arm in Gazebo, on top of the
    # 0.75 m bench of workcell.sdf.
    spawn_arm = Node(
        package='ros_gz_sim',
        executable='create',
        output='both',
        arguments=['-topic', 'robot_description', '-name', 'positronic', '-z', '0.75'])

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=['joint_state_broadcaster'])

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=['gripper_action_controller', '--param-file', controllers_yaml])

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=['arm_controller', '--param-file', controllers_yaml])

    return LaunchDescription([
        resource_path,
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_arm,
        # The controller_manager only exists after the spawn, so chain the spawners.
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_arm, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster, on_exit=[arm_controller])),
        RegisterEventHandler(
            OnProcessExit(target_action=arm_controller, on_exit=[gripper_controller])),
    ])
