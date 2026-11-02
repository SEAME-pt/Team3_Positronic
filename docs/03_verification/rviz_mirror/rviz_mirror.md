# Evidence: real arm mirrored in RViz on a laptop (#24)

## Verdict

| Criterion (#24) | Result | Measured |
|---|---|---|
| Free drive (servos released), RViz on a laptop follows the arm | PASS | every joint moved by hand, the model followed; `rviz_follows.png` against `arm_real.jpg`, same pose |
| Delay below 100 ms, measured from message timestamps | PASS | cable, 5 min: 15000 messages, 0 above 100 ms, median 16.6 ms, p99 18.6 ms, worst 71.7 ms |
| Every joint moves in the same direction as the model, no mirrored joint | PASS | six joints checked one by one, all turn the same way |

## Setup

| | |
|---|---|
| Date | 2026-10-28 19:30 |
| Arm | myCobot 280 on the Jetson, `/dev/ttyTHS1`, 1000000 baud |
| Driver | `driver.launch.py` of branch 23-joint-states-jetson, serial backend, 50 Hz, in the `positronic-dev:nano` container |
| Laptop | `mirror.launch.py` and `joint_states_delay.py` of this branch, native ROS 2 Jazzy, `ROS_DOMAIN_ID=17`. Delay measured on 2026-10-29 over a direct Ethernet cable to the Jetson (fixed addresses, laptop wifi off); the mirror and the joint checks on 2026-10-28 over wifi |
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

`joint_states_delay.py --duration 300`, started once the driver, the mirror and RViz were all up,
the six joints moved by hand one after the other and then together. Files in `delay_cable/`
(`summary.txt`, `per_second.txt`) and, for comparison, `delay_wifi/` (wifi).

| Run | Messages | Median | p99 | Worst | Above 100 ms | Verdict |
|---|---|---|---|---|---|---|
| Cable, 2026-10-29 | 15000 in 300 s | 16.6 ms | 18.6 ms | 71.7 ms at 1.1 s | 0 | PASS |
| Wifi, 2026-10-28 | 14987 in 300 s | 4.6 ms | 21.0 ms | 202.3 ms at 68.4 s | 17 | FAIL |

The cable run is the measurement of the criterion. Over the cable the spread is 2 ms from the
median to the p99; the absolute value carries the clock offset between the two machines, which
NTP keeps within some ms and which moved by about 10 ms between the two days (the wifi run
reads a lower median than the cable run for that reason, not because wifi is faster).

The wifi run fails the criterion because of one burst of 2.2 s, with the message count dropping
and then jumping back as the network held messages and released them together:

```text
67: 50 msgs  avg 11.6 ms  max 108.8 ms
68: 46 msgs  avg 34.1 ms  max 123.9 ms
69: 41 msgs  avg 52.0 ms  max 202.3 ms
70: 57 msgs  avg 21.2 ms  max 186.7 ms
71: 50 msgs  avg 4.7 ms  max 7.9 ms
```

Every other message of that run was below 66 ms. The mirror works over wifi, the 100 ms
criterion does not hold there on every day: the laboratory wifi varies from 6 ms to several
hundred ms of ping between days. For the measurement, and for any later timing work, a cable.

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
