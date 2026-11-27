#!/usr/bin/env python3
# Copyright 2026 Team Positronic
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Park the arm when a servo gets hot, before the driver's cut.

The driver publishes each servo's temperature on /diagnostics, once a second, as
"temperature_c/<joint>" in its "positronic_arm" status. When one servo stays at or above
warn_c (63 degC) for hot_for_s (1 s), this node sends the arm to the rest pose with one
trajectory on /arm_controller/joint_trajectory, slowly. J1 and J6 stay where they are:
turning the base or the wrist does not change whether the folded arm stands on its own, and
it keeps the move short.

driver.launch.py starts it next to the driver (thermal_park:=false to leave it out), so every
way of bringing the arm up, supervised.launch.py included, has it, on the same machine as the
driver: no network between the temperatures and the park.

No collision check: the trajectory goes joint by joint in a straight line, so keep the path
to the rest pose clear. It skips MoveIt on purpose, for now: the workcell model of the
planning scene refuses poses that are free on the real bench (START_STATE_IN_COLLISION).
Once that model is fixed, this can plan through move_group again.

Once the arm is at the rest pose (or after a timeout), the node calls the safety supervisor's
stop (/safety_supervisor/stop): the driver is deactivated, the arm holds its pose with
torque, and no goal moves it until someone activates it again, which the driver allows only
once every servo is 5 degC under the cut. The driver's cut (65 degC) works while deactivated
too: a servo that still gets there powers the arm off where it stands. Without a supervisor
(driver.launch.py alone) there is no stop: the arm stays parked, active.

One park per hot spell: the node parks again only after every servo went under
rearm_below_c.
"""

import math
import sys

from builtin_interfaces.msg import Duration
from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TEMPERATURE_KEY_PREFIX = 'temperature_c/'

# The joints of arm_controller, in its order (positronic_bringup/config/*_controllers.yaml).
ARM_JOINTS = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]

# The folded rest pose, rad, measured on the arm on 26 Nov 2026 with pymycobot get_angles()
# (degrees: 147.83, 0.87, -126.65, -16.08, -14.58, 107.88). Same values as the "rest"
# group_state of positronic.srdf.
REST_POSITIONS = [2.5801, 0.0152, -2.2105, -0.2806, -0.2545, 1.8829]

# Kept where they are at the warning, not moved to REST_POSITIONS.
KEEP_JOINTS = ['joint2_to_joint1', 'joint6output_to_joint6']

# Shortest park, s: a trajectory point the controller reaches at once would be a jump.
MIN_PARK_S = 2.0

# The working speed limit of arm_controller, rad/s (15 deg/s, positronic_bringup/config).
MAX_SPEED_RAD_S = 0.26

# The driver's default over-temperature cut, degC (SERVO_TEMP_LIMIT_C): the park must come
# before it.
DRIVER_CUT_C = 65


def temperatures_from(message, hardware_id):
    """Return {joint: degC} from the driver's status in a DiagnosticArray, or None if absent."""
    for status in message.status:
        if status.hardware_id != hardware_id:
            continue
        temperatures = {}
        for value in status.values:
            if not value.key.startswith(TEMPERATURE_KEY_PREFIX):
                continue
            try:
                temperatures[value.key[len(TEMPERATURE_KEY_PREFIX):]] = int(value.value)
            except ValueError:
                continue
        return temperatures
    return None


class HotWatch:
    """
    Say when to park: one servo at or above warn_c for hot_for_s, once per hot spell.

    Each servo has its own clock, as in the driver: J1 gives false hot readings, each one
    alone, and they must not park the arm. A reading under warn_c, or a missing one, stops
    that servo's clock.
    """

    def __init__(self, warn_c, hot_for_s, rearm_below_c):
        if rearm_below_c > warn_c:
            raise ValueError('rearm_below_c must not be above warn_c')
        self.warn_c = warn_c
        self.hot_for_s = hot_for_s
        self.rearm_below_c = rearm_below_c
        self._hot_since = {}
        self._parked = False

    def update(self, temperatures, now_s):
        """Return the (joint, degC) to park for, once per hot spell, else None."""
        for joint in list(self._hot_since):
            if temperatures.get(joint, 0) < self.warn_c:
                del self._hot_since[joint]
        if self._parked:
            if temperatures and all(c < self.rearm_below_c for c in temperatures.values()):
                self._parked = False
            return None
        for joint, celsius in temperatures.items():
            if celsius < self.warn_c:
                continue
            since = self._hot_since.setdefault(joint, now_s)
            if now_s - since >= self.hot_for_s:
                self._parked = True
                self._hot_since.clear()
                return joint, celsius
        return None


def park_targets(current, rest=None, keep=None):
    """
    Return the 6 target positions: the rest pose, with the kept joints at `current`.

    `current` is {joint: rad}. None if any arm joint is missing or not finite: the driver
    publishes NaN until its first pose, and a NaN would pass into the target (J1, J6) or hide
    in the move time (max() skips it), giving a long move in MIN_PARK_S.
    """
    rest = REST_POSITIONS if rest is None else rest
    keep = KEEP_JOINTS if keep is None else keep
    if any(joint not in current or not math.isfinite(current[joint]) for joint in ARM_JOINTS):
        return None
    return [float(current[joint]) if joint in keep else float(position)
            for joint, position in zip(ARM_JOINTS, rest)]


def park_duration_s(current, targets, speed_rad_s):
    """Return the time for the joint that moves furthest to get there at speed_rad_s."""
    furthest = max(abs(target - float(current[joint]))
                   for joint, target in zip(ARM_JOINTS, targets))
    return max(MIN_PARK_S, furthest / speed_rad_s)


def build_trajectory(targets, duration_s):
    """
    Return a one-point JointTrajectory to `targets`, reached after duration_s.

    The controller interpolates from where the arm is to the point; the empty stamp means
    "start now".
    """
    point = JointTrajectoryPoint()
    point.positions = [float(target) for target in targets]
    point.velocities = [0.0] * len(ARM_JOINTS)
    whole = int(duration_s)
    point.time_from_start = Duration(sec=whole, nanosec=int((duration_s - whole) * 1e9))
    trajectory = JointTrajectory()
    trajectory.joint_names = list(ARM_JOINTS)
    trajectory.points = [point]
    return trajectory


def arrived(current, targets, tolerance_rad):
    """Return True when every arm joint in `current` is within tolerance_rad of its target."""
    return all(joint in current and abs(float(current[joint]) - target) <= tolerance_rad
               for joint, target in zip(ARM_JOINTS, targets))


class ThermalPark(Node):

    def __init__(self):
        super().__init__('thermal_park')
        self.declare_parameter('warn_c', 63)
        self.declare_parameter('hot_for_s', 1.0)
        self.declare_parameter('rearm_below_c', 60)
        # 0.3 of the 0.26 rad/s (15 deg/s) working limit of arm_controller: new motion starts
        # slow. Raise it once the park is tried on the arm.
        self.declare_parameter('speed_rad_s', 0.078)
        # The goal tolerance of arm_controller (4 deg).
        self.declare_parameter('arrived_tolerance_rad', 0.070)
        # Added to the move time before the stop comes anyway.
        self.declare_parameter('arrive_margin_s', 5.0)
        self.declare_parameter('hardware_id', 'positronic_arm')
        self.declare_parameter('trajectory_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('stop_service', '/safety_supervisor/stop')
        self.declare_parameter('stop_after_park', True)
        param = self.get_parameter
        self._watch = HotWatch(
            param('warn_c').value, param('hot_for_s').value, param('rearm_below_c').value)
        self._hardware_id = param('hardware_id').value
        self._speed_rad_s = param('speed_rad_s').value
        if not 0.0 < self._speed_rad_s <= MAX_SPEED_RAD_S:
            raise ValueError(
                f'speed_rad_s must be in (0, {MAX_SPEED_RAD_S}] rad/s, the arm_controller limit')
        if not self._watch.warn_c < DRIVER_CUT_C:
            raise ValueError(f'warn_c must be under the driver cut ({DRIVER_CUT_C} degC)')
        self._tolerance_rad = param('arrived_tolerance_rad').value
        self._margin_s = param('arrive_margin_s').value
        self._stop_after_park = param('stop_after_park').value
        self._joints = {}
        self._targets = None
        self._deadline_s = None
        self._publisher = self.create_publisher(
            JointTrajectory, param('trajectory_topic').value, 10)
        self._stop_client = self.create_client(Trigger, param('stop_service').value)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)
        self.create_timer(0.2, self._check_arrival)
        self.get_logger().info(
            f'Parks the arm at {self._watch.warn_c} degC (for {self._watch.hot_for_s} s), '
            f'at {self._speed_rad_s} rad/s, no collision check')

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joint_states(self, message):
        for joint, position in zip(message.name, message.position):
            self._joints[joint] = position

    def _on_diagnostics(self, message):
        temperatures = temperatures_from(message, self._hardware_id)
        if temperatures is None:
            return
        hot = self._watch.update(temperatures, self._now_s())
        if hot is not None:
            self._park(*hot)

    def _park(self, joint, celsius):
        logger = self.get_logger()
        logger.warn(f'Servo {joint} at {celsius} degC: parking the arm at its rest pose')
        targets = park_targets(self._joints)
        if targets is None:
            logger.error('No /joint_states yet: not parking. The driver cut still applies.')
            return
        duration_s = park_duration_s(self._joints, targets, self._speed_rad_s)
        self._publisher.publish(build_trajectory(targets, duration_s))
        self._targets = targets
        self._deadline_s = self._now_s() + duration_s + self._margin_s
        logger.info(f'Park trajectory sent, {duration_s:.1f} s')

    def _check_arrival(self):
        if self._targets is None:
            return
        if arrived(self._joints, self._targets, self._tolerance_rad):
            self.get_logger().info('Arm parked at its rest pose')
        elif self._now_s() < self._deadline_s:
            return
        else:
            self.get_logger().error(
                'The arm did not reach its rest pose in time. The driver cut still applies.')
        self._targets = None
        self._stop()

    def _stop(self):
        """Ask the supervisor to halt: the arm holds with torque, no goal until activated."""
        if not self._stop_after_park:
            return
        # No wait: this runs in a callback, and a blocking wait would hold /diagnostics.
        if not self._stop_client.service_is_ready():
            self.get_logger().warn(
                'No safety supervisor stop service: the arm stays active. Wait for it to cool '
                'before sending goals.')
            return
        self._stop_client.call_async(Trigger.Request()).add_done_callback(self._on_stopped)

    def _on_stopped(self, future):
        self.get_logger().info(
            f'Supervisor stop: {future.result().message} Activate again once every servo is '
            'under the cut minus 5 degC.')


def main(argv=None):
    rclpy.init(args=sys.argv)
    node = ThermalPark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
