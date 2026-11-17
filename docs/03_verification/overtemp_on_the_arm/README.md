# Evidence: servo over-temperature cut on the real arm (#115)

## Verdict

| Criterion (#115) | Result | Measured |
|---|---|---|
| A servo at `servo_temp_limit_c` on two readings in a row stops the arm and powers its servos off | PASS | with the limit lowered under the hottest servo, the stressed servo reached it and the arm cut: it went limp and dropped (held by hand) |
| `read()` returns ERROR, `driver/fault_code` latches 3, the component leaves ACTIVE | `<PASS / ...>` | log: `<paste the "Servo overheated: joint '...' at .. degC (limit .. degC)..." line>`; `ros2 control list_hardware_components` showed the component `unconfigured` |
| Recovery: cool the arm, power on, configure and activate again | `<PASS / ...>` | `<power_on while holding the arm, then re-launch; note whether activate succeeded>` |
| Activation is refused while a servo is within 5 degC of the limit | covered by unit tests | not forced on the arm; see `test_load_robot_system` (activation refused on a hot arm) |

## Setup

| | |
|---|---|
| Date | `<date of the test, e.g. 2026-11-13>` |
| Arm | myCobot 280 on the Jetson, `/dev/ttyTHS1`, 1000000 baud |
| Driver | `driver.launch.py` of branch `115-spike-servo-temperature-as-a-driver-fault`, serial backend, in the `positronic-runtime:nano` container, `ROS_DOMAIN_ID=17` |
| Limit | `servo_temp_limit_c:=<final limit used>` (lowered on purpose so a healthy servo could reach it) |
| Commanding | second terminal, `/arm_controller/joint_trajectory`, JointTrajectoryController |
| People | two: one holding the arm, bench clear |

Safety: the cut powers the servos off, so J2 and J3 fall. The whole test was done with one
person holding the arm. No new motion code: the heating pose was sent through the JTC under the
working limit (<= 15 deg/s).

## What was done

1. Driver stopped. Read the servo temperatures with pymycobot (the port is exclusive, so the
   driver must be down first):

   `python3 -c "from pymycobot import MyCobot280; print(MyCobot280('/dev/ttyTHS1', 1000000).get_servo_temps())"`

   Start temperatures, J1..J6: **38, 34, 33, 33, 32, 34 degC** (J1 the hottest; it carries no
   load, the base rotation).

2. Launched the driver with the limit a few degrees above the hottest servo and confirmed
   `Successful 'activate' of hardware`.

3. Commanded a loaded diagonal pose, arm straight so J2 bears the most, changing only J2:

   `ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6], points: [{positions: [0.227, 1.05, -0.0085, -0.1878, 0.006, -0.0085], time_from_start: {sec: 4}}]}"`

   (0.79 rad = 45 deg, 1.05 rad = 60 deg; about 7.5 deg/s.) Held the pose so the servo kept
   working against gravity.

4. At `servo_temp_limit_c:=46` the servo did not gain enough in the time given (the arm does not
   heat easily under a simple load). The limit was then **lowered** and the hotter, already-stressed
   servo reached it. On two readings at or above the limit, the serial thread sent stop and then
   power off, `read()` returned ERROR with `driver/fault_code = 3`, and the component went to
   UNCONFIGURED. The arm went limp and dropped; it was held.

## Result

| | |
|---|---|
| Servo that overheated | `<joint, e.g. joint2_to_joint1>` |
| Temperature at the cut | `<T> degC` (limit `<final limit>`) |
| Time from the command to the cut | `<minutes>` |
| Log line | `<Servo overheated: joint '...' at <T> degC (limit <final limit> degC)...>` |
| Component after the cut | `<unconfigured, from ros2 control list_hardware_components>` |
| Arm behaviour | went limp and dropped (held by hand), as expected for a power off |
| Recovery | `<power_on while holding, then configure + activate: OK / ...>` |

## Known limitation

Nothing publishes the live temperature while the driver runs, so the climb was not logged: the
temperature only appears in the log at the moment of the cut. Watching it live (a topic for
`driver/fault_code` and the temperatures) is part of #45, not this issue.
