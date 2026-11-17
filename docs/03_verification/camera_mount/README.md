# Evidence: wrist camera mounted on the real arm, model matched to it (#97)

## Verdict

<PASS / FAIL>. The camera module sits in line with the finger line, lens outside one finger,
on the real arm and in the four camera URDFs. Real camera and model camera point the same way at
all-zero and at joint 6 = +90 deg, the cable reaches +90 deg without pulling, and the Gazebo pick
still sees and lifts the cube.

| Criterion (#97) | Result | Evidence |
|---|---|---|
| Mounting decided and documented: holes, side, cable route | <PASS / FAIL> | [Mounting](#mounting) |
| Cable reaches joint 6 = +90 deg without pulling | <PASS / FAIL> | [Real arm and model](#real-arm-and-model) |
| `joint6output_to_camera_flange` matches the mounting, same in the camera variants | PASS | `test_camera_mount.py`, 9 tests |
| Real and model camera point the same way at all-zero and at joint 6 = +90 deg | <PASS / FAIL> | photos next to Gazebo |
| Gazebo pick still works, wrist camera sees the cube at pre-grasp | <PASS / FAIL> | [Gazebo pick](#gazebo-pick) |

## Decision

The mount is chosen on the real arm and the model follows it:

- The camera module is in line with the finger line, with the lens past the end of one finger,
  so the gripper does not cover the view. It is the position where the latches of the camera
  bracket fit with the gripper mounted as in #94.
- The model of #91 had the camera 90 deg from the fingers. It moves to the real mount in the
  four camera URDFs, the Gazebo one included.

Note on the earlier records: the comment of #91 in the URDF said the 90 deg mount was "seen on
the real arm on 2026-10-28"; #94 found the camera in line with the gripper. This document
replaces both: the mount below is the one checked on <date>.

## Mounting

| | |
|---|---|
| Gripper | turned 180 deg on the flange, as in #94 (unchanged) |
| Camera module | in line with the finger line, lens past the end of one finger |
| Side | `-x` of `joint6_flange`. At all-zero the arm stands up, the tool points along world `+x`, and the flange sits 45 deg turned (#94): the camera module points up on the diagonal, away from the arm |
| Lens to flange axis | <mm, measured>; the model has 41.5 mm (`camera_link`) |
| Holes / latches | <which latches of the bracket, which holes of the flange> |
| Cable route | <where the cable leaves the module, how it runs along link 6 and link 5, where it is tied, slack left at the wrist> |
| Mounted by | <names>, <date> |

Photos of the mount: <`figures/mount_side.jpg`>, <`figures/mount_front.jpg`>.

## Image orientation

The raw image of the real camera lies on its side: it is upright once turned 90 deg
counterclockwise (`ffplay -f v4l2 -i /dev/video0 -vf transpose=2`). Upright here means the
gripper at the bottom of the image, which is what the model gives (`camera_link` roll, #91).

The camera node, still to be written, turns every frame before publishing it
(`cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)`), and the camera is calibrated on the
turned images, so `camera_info` matches the published image and the frame `camera_link`.

Check: in the turned image of the real camera, the finger near the lens is at the bottom.
<yes / no>, <`figures/real_image_turned.png`> next to <`figures/gazebo_image.png`>.

## Model

| File | `joint6output_to_camera_flange` rpy | `camera_flange_to_camera_link` rpy | gripper rpy |
|---|---|---|---|
| `mycobot_280_jn_gripper_camera.urdf` (Gazebo) | `1.5708 0 -1.5708` → `1.5708 0 3.1416` | `1.5708 0 1.5708` (unchanged) | `1.5708 0 3.1416` (unchanged) |
| `mycobot_280_jn_gripper_camera_no_base.urdf` | `1.5708 0 0` → `1.5708 0 3.1416` | `0 0 1.5708` → `1.5708 0 1.5708` | `1.5708 0 0` → `1.5708 0 3.1416` |
| `mycobot_280_jn_camera.urdf` | `1.5708 0 0` → `1.5708 0 3.1416` | `0 0 1.5708` → `1.5708 0 1.5708` | no gripper |
| `mycobot_280_jn_camera_no_base.urdf` | `1.5708 0 0` → `1.5708 0 3.1416` | `0 0 1.5708` → `1.5708 0 1.5708` | no gripper |

`test_camera_mount.py` keeps the four files equal and at these values, and checks the geometry
behind them: the camera module on the finger line, on `-x` of the flange, looking along the tool
axis, with the gripper at the bottom of the image. It also checks that the 90 deg mount of #91
and the camera on the other finger are rejected.

Where the model puts the camera, from the URDF (offset of `camera_link` from the flange origin,
world frame, mm):

| Pose | Offset x, y, z | Camera module |
|---|---|---|
| all-zero | 18.3, +29.3, +29.3 | above the flange, world `+y` side, on the diagonal |
| joint 6 = +90 deg | 18.3, -29.3, +29.3 | above the flange, world `-y` side, on the other diagonal |

## Real arm and model

Safety: joint 6 moves once to +90 deg and back, at speed 30. Bench clear, one person with a hand
on the plug, a second at the laptop. Stop the driver first: only one process can hold the port.

On the Jetson:

```python
from pymycobot import MyCobot280

mc = MyCobot280('/dev/ttyTHS1', 1000000)
mc.send_angles([0, 0, 0, 0, 0, 0], 30)   # photo 1, then check the cable
mc.send_angles([0, 0, 0, 0, 0, 90], 30)  # watch the cable on the way, photo 2
print(mc.get_angles())
mc.send_angles([0, 0, 0, 0, 0, 0], 30)
```

In Gazebo, same two poses (`gazebo.launch.py`, then):

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3,
    joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6],
    points: [{positions: [0, 0, 0, 0, 0, 1.5708], time_from_start: {sec: 4}}]}}"
```

Take the Gazebo screenshot from the same side as the photo.

| Pose | Real arm | Gazebo | Same way? |
|---|---|---|---|
| all-zero | <`figures/real_zero.jpg`> | <`figures/gazebo_zero.png`> | <yes / no> |
| joint 6 = +90 deg | <`figures/real_j6_90.jpg`> | <`figures/gazebo_j6_90.png`> | <yes / no> |

`get_angles()` at +90 deg: <reading>. Cable at +90 deg: <slack left, nothing pulled / what pulled>.

## Gazebo pick

The pick of #91 and #94, same world and script, on <date>, with the camera moved: pre-grasp and
grasp <planned / not>, cube lifted <m> (0.05 m required, 0.115 m in #94). Wrist camera at
pre-grasp: <`figures/pick_pre_grasp_camera.png`>, the cube <in view, finger at the bottom edge /
not in view>.

## Checks run

| Check | Where | Result |
|---|---|---|
| `test_camera_mount.py` (9 tests), `test_joint6_zero.py`, `test_gazebo_description.py` | `make test` | <PASS> |
| flake8, pep257 | `make test` | <PASS> |
| Real arm, two poses | the arm | <PASS / FAIL> |
| Gazebo pick | laptop, `ROS_DOMAIN_ID=<n>` | <PASS / FAIL> |
