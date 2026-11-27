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

- robot_state_publisher, which publishes the URDF on /robot_description (latched). The
  controller_manager reads the description from that topic.
- ros2_control_node (the controller_manager), which loads the hardware plugin and, on the
  serial backend, opens the port and reads the arm.
- the spawner, which activates joint_state_broadcaster, the controller that turns the plugin's
  state interfaces into /joint_states.

Forward_position_controller is in the
yaml for the first motion and for jogging; spawn it by hand with the bench clear (#32).

The mirror launch (mirror.launch.py, #24) runs its own robot_state_publisher. Running the
driver and the mirror at the same time on one ROS_DOMAIN_ID gives two /tf publishers for the
same robot, which RViz warns about but tolerates. The 10 minute run only records
/joint_states and does not need the mirror.

Launch arguments. The driver's (backend to load_relief) have no default here: unset,
the driver keeps its own (HardwareConfig and SerialTiming in positronic_hardware, the table in
its README), and the INFO line of on_init prints the values in use. A default here would win
over the driver's and hide a change made there.
  backend:=serial|fake        fake stands still, to try the launch without the arm
  fake_mode:=instant|measured how the fake arm moves: at once, or like the real arm (#122)
  port:=                      the arm's UART
  baud:=                      the arm's firmware rate (115200 or 1000000)
  moving_period_ms:=          ms of a serial-thread turn that sends a command
  echo_timeout_ms:=           ms the thread waits at most for the arm's sync-write after a send
  pose_from_poll:=true|false  the pose from the arm's own poll of its servos, the send on the
                              controller's write (#170); false gives a get_angles every turn
  servo_temp_limit_c:=        degC that cuts the arm (stop, then power off); 30 to 60
  tracking_error_deg:=        deg a joint may lag its target before the time counts (#103)
  tracking_error_ms:=         ms over that lag that make fault 2, the arm did not follow
  supervisor_timeout_ms:=0    ms without the safety supervisor's heartbeat that halt the arm
                              (#136). 0 here, the check off: this launch alone has no
                              supervisor. supervised.launch.py sets 500.
  load_relief:=true|false     a still joint pushing short of its target is sent where it
                              stands (#183)
  update_rate:=               Hz, of the control loop and of /joint_states; by default the
                              update_rate of jetson_controllers.yaml
  start_inactive:=false       true: the hardware component and arm_controller come up
                              inactive, and only the safety supervisor makes them active
                              (#45). supervised.launch.py sets it.
  camera:=false               true: also start the wrist camera (camera.launch.py). Off
                              unless asked: the camera costs CPU on the Jetson and the
                              container needs /dev/video0 (make nano-shell maps it when it is
                              there). Its arguments (device:=, width:=, height:=, fps:=, ...)
                              pass through, as this launch's reach it from supervised.launch.py.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml

# jetson_controllers.yaml next to this file, in the source tree and in the install space alike.
CONTROLLERS_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'jetson_controllers.yaml')


def yaml_update_rate():
    """Return the update_rate of jetson_controllers.yaml, the one place it is set."""
    with open(CONTROLLERS_YAML, encoding='utf-8') as config:
        return yaml.safe_load(config)['controller_manager']['ros__parameters']['update_rate']


def generate_launch_description():
    """Return the driver launch: the arguments, the nodes and the xacro call."""
    # The driver's parameters: empty means unset, and the xacro then writes no <param>.
    backend = DeclareLaunchArgument(
        'backend', default_value='',
        description='serial: the real arm. fake: a standing arm, no serial port.')
    fake_mode = DeclareLaunchArgument(
        'fake_mode', default_value='',
        description='instant: the fake arm is at every target at once. measured: it lags like '
                    'the real arm (#122).')
    port = DeclareLaunchArgument(
        'port', default_value='',
        description="The arm's serial port.")
    baud = DeclareLaunchArgument(
        'baud', default_value='',
        description='Baud rate of the arm serial link: 115200 or 1000000.')
    # Timings of the serial thread (#32). Arguments so a sweep on the arm needs no rebuild.
    moving_period_ms = DeclareLaunchArgument(
        'moving_period_ms', default_value='',
        description='ms of a serial-thread turn that sends a command.')
    echo_timeout_ms = DeclareLaunchArgument(
        'echo_timeout_ms', default_value='',
        description="ms the thread waits at most for the arm's sync-write after a send.")
    pose_from_poll = DeclareLaunchArgument(
        'pose_from_poll', default_value='',
        description="true: the pose from the arm's own poll of its servos and the send on the "
                    'controller write (#170); false: a get_angles every turn.')
    # The over-temperature cut. Only lower it, for the hardware test of the cut.
    servo_temp_limit_c = DeclareLaunchArgument(
        'servo_temp_limit_c', default_value='',
        description='degC at which a servo stops the arm and powers it off (30 to 60).')
    # The unfollowed-command fault (#103). Arguments so the bench can try other values.
    tracking_error_deg = DeclareLaunchArgument(
        'tracking_error_deg', default_value='',
        description='deg a joint may lag its target before the time counts (1 to 45).')
    tracking_error_ms = DeclareLaunchArgument(
        'tracking_error_ms', default_value='',
        description='ms a joint may stay over tracking_error_deg before fault 2 (50 to 5000).')
    # The dead-man check (#136). The one driver argument with a default here, on purpose: it is a
    # mode, not a tuned value. This launch alone runs no supervisor (bench, sweep.sh, HIL), and
    # with the driver's own 500 ms the arm would never activate. supervised.launch.py sets 500.
    # The driver keeps 500 as its default, so a URDF that forgets the parameter is safe.
    supervisor_timeout_ms = DeclareLaunchArgument(
        'supervisor_timeout_ms', default_value='0',
        description='ms without a supervisor heartbeat that halt the arm (0: check off).')
    load_relief = DeclareLaunchArgument(
        'load_relief', default_value='',
        description='true or false: relieve a still joint pushing short of its target.')
    # The yaml's value by default, read from the yaml. Here so the 10 minute run can try another
    # rate without editing the yaml.
    update_rate = DeclareLaunchArgument(
        'update_rate', default_value=str(yaml_update_rate()),
        description='Hz of the control loop, and so of /joint_states.')
    # false keeps the launch as it was: everything active, the arm can move at once. true is
    # for the safety supervisor (#45): nothing can move the arm until it activates the hardware
    # component and the controller.
    start_inactive = DeclareLaunchArgument(
        'start_inactive', default_value='false',
        description='true: the hardware component and arm_controller come up inactive.')
    inactive = LaunchConfiguration('start_inactive')
    # The wrist camera is off by default: started only with camera:=true.
    camera = DeclareLaunchArgument(
        'camera', default_value='false',
        description='true: also start the wrist camera (camera.launch.py).')

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
            ' fake_mode:=', LaunchConfiguration('fake_mode'),
            ' port:=', LaunchConfiguration('port'),
            ' baud:=', LaunchConfiguration('baud'),
            ' moving_period_ms:=', LaunchConfiguration('moving_period_ms'),
            ' echo_timeout_ms:=', LaunchConfiguration('echo_timeout_ms'),
            ' pose_from_poll:=', LaunchConfiguration('pose_from_poll'),
            ' servo_temp_limit_c:=', LaunchConfiguration('servo_temp_limit_c'),
            ' tracking_error_deg:=', LaunchConfiguration('tracking_error_deg'),
            ' tracking_error_ms:=', LaunchConfiguration('tracking_error_ms'),
            ' supervisor_timeout_ms:=', LaunchConfiguration('supervisor_timeout_ms'),
            ' load_relief:=', LaunchConfiguration('load_relief'),
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
    manager_parameters = [
        controllers_yaml,
        {'update_rate': ParameterValue(LaunchConfiguration('update_rate'), value_type=int)},
    ]
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=manager_parameters,
        condition=UnlessCondition(inactive))
    # The same node, told to configure the hardware component and stop there: the port is
    # open and the pose is published, but write() sends nothing until someone activates it.
    control_node_inactive = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=manager_parameters + [
            {'hardware_components_initial_state.inactive': ['positronic_arm']},
        ],
        condition=IfCondition(inactive))

    # Publishes /joint_states from the plugin's state interfaces, once per update.
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=['joint_state_broadcaster'])

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=['arm_controller', 'gripper_action_controller', '--param-file',
                   controllers_yaml],
        condition=UnlessCondition(inactive))
    # Loaded and configured, not activated: it takes no trajectory goal yet.
    arm_controller_inactive = Node(
        package='controller_manager',
        executable='spawner',
        output='both',
        arguments=['arm_controller', 'gripper_action_controller', '--param-file',
                   controllers_yaml, '--inactive'],
        condition=IfCondition(inactive))

    # The camera launch. Its arguments given to this launch reach it by themselves.
    wrist_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('positronic_bringup'), 'launch', 'camera.launch.py'])),
        condition=IfCondition(LaunchConfiguration('camera')))

    return LaunchDescription([
        backend,
        fake_mode,
        port,
        baud,
        moving_period_ms,
        echo_timeout_ms,
        pose_from_poll,
        servo_temp_limit_c,
        tracking_error_deg,
        tracking_error_ms,
        supervisor_timeout_ms,
        load_relief,
        update_rate,
        start_inactive,
        camera,
        robot_state_publisher,
        control_node,
        control_node_inactive,
        joint_state_broadcaster,
        arm_controller,
        arm_controller_inactive,
        wrist_camera,
    ])
