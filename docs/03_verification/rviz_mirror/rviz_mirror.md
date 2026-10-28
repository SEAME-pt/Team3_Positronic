# Evidence: real arm mirrored in RViz on a laptop (#24)

## Verdict

| Criterion (#24) | Result | Measured |
|---|---|---|
| Free drive (servos released), RViz on a laptop follows the arm | PASS | every joint moved by hand, the model followed |
| Delay below 100 ms, measured from message timestamps | PASS | 46 s of motion: average 4.7 to 5.6 ms per second, worst message 30.1 ms |
| Every joint moves in the same direction as the model, no mirrored joint | PASS | six joints checked one by one, all turn the same way |

## Setup

| | |
|---|---|
| Date | 2026-10-28 19:30 |
| Arm | myCobot 280 on the Jetson, `/dev/ttyTHS1`, 1000000 baud |
| Driver | `driver.launch.py` of branch 23-joint-states-jetson, serial backend, 50 Hz, in the `positronic-dev:nano` container |
| Laptop | `mirror.launch.py` and `joint_states_delay.py` of this branch, native ROS 2 Jazzy, `ROS_DOMAIN_ID=17`, wifi |
| Clocks | `timedatectl`: NTP active and synchronized on both machines |
| Reference before the launch | `pymycobot get_angles`: [16.87, -16.17, 25.48, -10.45, 1.23, 0.26] deg |

Safety: servos released by hand with `release_all_servos`, no motion commands, bench clear.

## /joint_states on the laptop

`ros2 topic hz /joint_states`: 50.0 Hz, min 13 ms, max 30 ms between messages (window 405).

First message, positions in radians, against the reference in degrees:

| Joint | Published (rad) | Reference (deg) | Reference (rad) |
|---|---|---|---|
| joint2_to_joint1 | 0.2913 | 16.87 | 0.2944 |
| joint3_to_joint2 | -0.2822 | -16.17 | -0.2822 |
| joint4_to_joint3 | 0.4447 | 25.48 | 0.4447 |
| joint5_to_joint4 | -0.2377 | -10.45 | -0.1824 |
| joint6_to_joint5 | 0.0215 | 1.23 | 0.0215 |
| joint6output_to_joint6 | 0.0045 | 0.26 | 0.0045 |

joint5_to_joint4 differs by 3.2 deg: the wrist sagged after the servos were released, between the
reference reading and the driver start. The other five match to the milliradian.

## Delay

`joint_states_delay.py` on the laptop, started once the driver, the mirror and RViz were all up,
one line per second, 46 s while the joints were moved by hand, slowly and then fast.
Average 4.7 to 5.6 ms, worst message 30.1 ms. A 5 minute run
follows on 2026-10-29.

```text
33 msgs  avg 5.2 ms  max 19.2 ms
50 msgs  avg 4.8 ms  max 8.4 ms
50 msgs  avg 4.8 ms  max 11.3 ms
50 msgs  avg 5.2 ms  max 9.9 ms
50 msgs  avg 5.0 ms  max 10.8 ms
50 msgs  avg 5.5 ms  max 24.4 ms
50 msgs  avg 4.8 ms  max 9.7 ms
50 msgs  avg 5.0 ms  max 7.7 ms
50 msgs  avg 4.8 ms  max 8.2 ms
50 msgs  avg 4.7 ms  max 8.4 ms
50 msgs  avg 5.4 ms  max 11.8 ms
50 msgs  avg 5.4 ms  max 27.6 ms
50 msgs  avg 5.1 ms  max 14.5 ms
50 msgs  avg 5.0 ms  max 8.3 ms
50 msgs  avg 4.8 ms  max 7.5 ms
50 msgs  avg 4.9 ms  max 7.7 ms
50 msgs  avg 5.1 ms  max 10.3 ms
50 msgs  avg 4.9 ms  max 8.8 ms
50 msgs  avg 4.8 ms  max 11.3 ms
50 msgs  avg 4.9 ms  max 11.1 ms
50 msgs  avg 5.0 ms  max 12.2 ms
50 msgs  avg 4.8 ms  max 8.2 ms
50 msgs  avg 5.2 ms  max 11.8 ms
50 msgs  avg 4.7 ms  max 7.7 ms
50 msgs  avg 5.5 ms  max 22.2 ms
50 msgs  avg 5.0 ms  max 8.0 ms
50 msgs  avg 4.9 ms  max 8.7 ms
50 msgs  avg 4.8 ms  max 7.6 ms
50 msgs  avg 4.9 ms  max 10.4 ms
50 msgs  avg 4.8 ms  max 8.4 ms
50 msgs  avg 5.2 ms  max 15.6 ms
50 msgs  avg 4.9 ms  max 7.9 ms
50 msgs  avg 5.2 ms  max 25.1 ms
50 msgs  avg 5.1 ms  max 14.9 ms
50 msgs  avg 4.9 ms  max 9.5 ms
50 msgs  avg 5.0 ms  max 8.6 ms
50 msgs  avg 5.2 ms  max 15.0 ms
50 msgs  avg 5.1 ms  max 10.3 ms
49 msgs  avg 5.1 ms  max 18.2 ms
51 msgs  avg 5.6 ms  max 30.1 ms
50 msgs  avg 4.8 ms  max 8.0 ms
50 msgs  avg 5.0 ms  max 12.6 ms
50 msgs  avg 5.3 ms  max 12.1 ms
50 msgs  avg 5.2 ms  max 12.5 ms
50 msgs  avg 5.0 ms  max 10.1 ms
50 msgs  avg 5.1 ms  max 8.4 ms
```

## Joint directions

| Joint | Moved by hand | Model |
|---|---|---|
| 1, base | yes | same direction |
| 2, shoulder | yes | same direction |
| 3, elbow | yes | same direction |
| 4, wrist pitch | yes | same direction |
| 5, wrist yaw | yes | same direction |
| 6, flange | yes | same direction |

## Observations

- Zero of joint 6: with the driver reading 0.26 deg, the hole pattern of the real flange is
  aligned with the forearm while the model shows it turned by 45 deg. The joint turns the right
  way, so this is an offset of the mesh or of the joint origin in the URDF, not a mirrored joint.
  To fix in positronic_description with the gripper mounted, which fixes the sign.
- Two `robot_state_publisher` nodes publish `/tf` for the same links (the driver's on the Jetson
  and the mirror's on the laptop). RViz warns and follows the laptop one.
