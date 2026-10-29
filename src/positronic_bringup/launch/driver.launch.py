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
Run the real arm driver on the Jetson and publish /joint_states (#23).

Three nodes:
- robot_state_publisher, which publishes the URDF on /robot_description (latched). The
  controller_manager reads the description from that topic.
- ros2_control_node (the controller_manager), which loads the hardware plugin and, on the
  serial backend, opens the port and reads the arm.
- the spawner, which activates joint_state_broadcaster, the controller that turns the plugin's
  state interfaces into /joint_states.

Read-only: no controller can move the arm (#32).

The mirror launch (mirror.launch.py, #24) runs its own robot_state_publisher. Running the
driver and the mirror at the same time on one ROS_DOMAIN_ID gives two /tf publishers for the
same robot, which RViz warns about but tolerates. The 10 minute run only records
/joint_states and does not need the mirror.

Launch arguments:
  backend:=serial|fake        fake stands still, to try the launch without the arm
  port:=/dev/ttyTHS1          the arm's UART
  baud:=1000000               the arm's firmware rate (115200 or 1000000)
  update_rate:=50             Hz, of the control loop and of /joint_states
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = DeclareLaunchArgument(
        'backend', default_value='serial',
        description='serial: the real arm. fake: a standing arm, no serial port.')
    port = DeclareLaunchArgument(
        'port', default_value='/dev/ttyTHS1',
        description="The arm's serial port.")
    baud = DeclareLaunchArgument(
        'baud', default_value='1000000',
        description='Baud rate of the arm serial link: 115200 or 1000000.')
    # Same default as jetson_controllers.yaml. Here so the 10 minute run can try another rate
    # without editing the yaml.
    update_rate = DeclareLaunchArgument(
        'update_rate', default_value='50',
        description='Hz of the control loop, and so of /joint_states.')

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('positronic_bringup'), 'config', 'jetson_controllers.yaml'])

    # The xacro output goes to the robot_description parameter, as plain text.
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('positronic_description'), 'urdf', 'positronic_real.urdf.xacro',
            ]),
            ' backend:=', LaunchConfiguration('backend'),
            ' port:=', LaunchConfiguration('port'),
            ' baud:=', LaunchConfiguration('baud'),
        ]),
        value_type=str)

    # Publishes the URDF on /robot_description (latched), which the controller_manager reads.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}])

    # The controller_manager. It reads /robot_description, loads the hardware plugin, and on the
    # serial backend opens the port. The update_rate argument comes after the yaml, so it wins.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[
            controllers_yaml,
            {'update_rate': ParameterValue(LaunchConfiguration('update_rate'), value_type=int)},
        ])

    # Publishes /joint_states from the plugin's state interfaces, once per update.
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=['joint_state_broadcaster'])

    return LaunchDescription([
        backend,
        port,
        baud,
        update_rate,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster,
    ])
