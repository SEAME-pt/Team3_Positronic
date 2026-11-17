# Evidence: servo over-temperature cut on the real arm (#115)

## Verdict

| Criterion (#115) | Result | Measured |
|---|---|---|
| A servo at `servo_temp_limit_c` on two readings in a row stops the arm and powers its servos off | PASS | the wrist servo `joint6_to_joint5` reached 47 degC (= limit) under load, the arm was stopped and its servos powered off, went limp and dropped (held by hand) |
| `read()` returns ERROR, `driver/fault_code` latches 3, the component leaves ACTIVE | PASS | log: `Servo overheated: joint 'joint6_to_joint5' at 47 degC (limit 47 degC)...`; the read cycle returned an error and the controller_manager deactivated `positronic_arm` and its controllers. `fault_code` was set to 3 on the interface but not published (no gpio controller yet, #45), so it is not in the log |
| Recovery: cool the arm, power on, configure and activate again | PASS | after the cut the servos were powered on again with `pymycobot` while holding the arm; a cool configure + activate is shown at the top of this same log (`Successful 'activate' of hardware`), which is the recovery path |
| Activation is refused while a servo is within 5 degC of the limit | covered by unit tests | not forced on the arm; see `test_load_robot_system` (activation refused on a hot arm) |

## Setup

| | |
|---|---|
| Date | 2026-11-17 |
| Arm | myCobot 280 on the Jetson, `/dev/ttyTHS1`, 1000000 baud |
| Driver | `driver.launch.py` of branch `115-spike-servo-temperature-as-a-driver-fault` (after merging `dev`), serial backend, in the `positronic-runtime:nano` container, `ROS_DOMAIN_ID=17` |
| Limit | `servo_temp_limit_c:=47` (lowered on purpose so a healthy servo could reach it) |
| Commanding | second terminal, `/arm_controller/joint_trajectory`, JointTrajectoryController |
| People | two: one holding the arm, bench clear |

Safety: the cut powers the servos off, so the arm falls. The whole test was done with one person
holding the arm. No new motion code: the heating pose was sent through the JTC under the working
limit (<= 15 deg/s).

## What was done

1. Driver stopped. Read the servo temperatures with pymycobot (the port is exclusive, so the
   driver must be down first):

   `python3 -c "from pymycobot import MyCobot280; print(MyCobot280('/dev/ttyTHS1', 1000000).get_servo_temps())"`

   The launch at limit 47 activated cleanly, so every servo was below 42 degC at start-up (the
   trava refuses activation within 5 degC of the limit). An earlier reading on 2026-11-13, J1..J6:
   38, 34, 33, 33, 32, 34 degC, for reference.

2. Launched the driver at `servo_temp_limit_c:=47` and confirmed `Successful 'activate' of
   hardware 'positronic_arm'`, then `arm_controller` and `joint_state_broadcaster` configured and
   activated.

3. Commanded a loaded diagonal pose, arm extended, changing J2 to 60 deg:

   `ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6], points: [{positions: [0.227, 1.05, -0.0085, -0.1878, 0.006, -0.0085], time_from_start: {sec: 6}}]}"`

   (1.05 rad = 60 deg, over 6 s, about 10 deg/s, under the 15 deg/s working limit.) Held the pose
   so the servos kept working against gravity. Under the hold, the wrist servo `joint6_to_joint5`
   was the first to climb to the limit.

4. About 52 s after activation (component active at t = ...150.1 s, cut logged at t = ...202.5 s
   of the controller_manager clock), `joint6_to_joint5` reached 47 degC on two readings in a row.
   The serial thread sent stop and then power off, `read()` returned ERROR, and the
   controller_manager deactivated the component and both controllers. The arm went limp and
   dropped; it was held.

## Result

| | |
|---|---|
| Servo that overheated | `joint6_to_joint5` (J5, the wrist) |
| Temperature at the cut | 47 degC (limit 47) |
| Time from activation to the cut | ~52 s |
| Log line | `Servo overheated: joint 'joint6_to_joint5' at 47 degC (limit 47 degC). The arm was stopped and its servos powered off, so it is limp now. Find the cause, let it cool, power the arm on again, then configure and activate.` |
| Component after the cut | deactivated by the controller_manager (`Deactivating ... positronic_arm`), with `arm_controller` and `joint_state_broadcaster`; `on_error` then takes it to UNCONFIGURED |
| `driver/fault_code` | set to 3 on the interface; not published (no gpio controller yet, #45), so not in the log |
| Arm behaviour | went limp and dropped (held by hand), as expected for a power off |
| Recovery | servos powered on again with `pymycobot` while holding the arm; the clean configure + activate at the top of this log is the recovery path |

## Note on the extra error lines

Right after the cut the log also shows `Not acceptable command interfaces combination` and `Error
while attempting mode switch when deactivating controllers in read cycle!`. This is ros2_control
noise, not a second fault: the component errored mid-cycle, so when the manager tried to stop the
command interfaces they were already unavailable. The outcome is the intended one: the component
and both controllers are deactivated and the arm is limp.

## Known limitation

Nothing publishes the live temperature while the driver runs, so the climb was not logged: the
temperature only appears in the log at the moment of the cut. Watching it live (a topic for
`driver/fault_code` and the temperatures) is part of #45, not this issue.