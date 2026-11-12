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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""
Run the real arm under the safety supervisor (#45).

- driver.launch.py with start_inactive:=true: the driver reads the arm and publishes
  /joint_states, but the hardware component and arm_controller are inactive, so nothing can
  move the arm.
- the safety supervisor (positronic_safety), configured by this launch and left INACTIVE.

The arm can move only after a person activates the supervisor, which needs the mission node
running and active (its heartbeat):

    ros2 lifecycle set /safety_supervisor activate

After any halt the supervisor is INACTIVE again and the same command resumes. The state and
the reason of the last halt are on /safety_supervisor/status.

Both run on the Jetson, next to each other (ADR 0005). The parameters of the supervisor are
in config/safety_supervisor.yaml.

Launch arguments: the ones of driver.launch.py, which this launch passes on (backend, port,
baud, update_rate, moving_period_ms, send_settle_ms).
"""

from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Return the supervised launch: the driver, inactive, and the supervisor."""
    bringup = FindPackageShare('positronic_bringup')

    # The arguments given to this launch (backend:=fake, ...) reach driver.launch.py by
    # themselves; only start_inactive is forced.
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, 'launch', 'driver.launch.py'])),
        launch_arguments={'start_inactive': 'true'}.items())

    supervisor = LifecycleNode(
        package='positronic_safety',
        executable='safety_supervisor',
        name='safety_supervisor',
        namespace='',
        output='both',
        parameters=[PathJoinSubstitution([bringup, 'config', 'safety_supervisor.yaml'])])

    # unconfigured -> inactive: the supervisor starts following the heartbeat and the fault
    # code, and its stop service exists. Activating it is left to a person, on purpose.
    configure_supervisor = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(supervisor),
        transition_id=Transition.TRANSITION_CONFIGURE))

    return LaunchDescription([
        driver,
        supervisor,
        configure_supervisor,
    ])
