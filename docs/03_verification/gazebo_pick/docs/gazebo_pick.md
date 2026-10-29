# Gazebo pick verification

The arm picks a 2.5 cm cube off a pedestal in Gazebo, with the gripper and the wrist camera of
the URDF. It checks, in one run, what the simulation stack must do before any pick and place
work: the gripper fingers close together, MoveIt plans and executes, the wrist camera image
reaches ROS 2, and a straight descent and lift do not push the object away.

Same layout as `serial_measurement`: this document in `docs/`, the scripts in `tools/`.

| File | What it is |
|---|---|
| `tools/run.sh` | Starts Gazebo with `workcell_pick.sdf`, the controllers, `move_group` with the planning scene, a window per camera, runs `pick.py` and prints PASS or FAIL. |
| `tools/pick.py` | Opens the gripper, moves above the cube, goes straight down, closes, goes straight up. Saves one wrist camera frame per step. Exit code 0 when the cube rose at least 5 cm. |

## Run

From the repository root, after `make build`:

```bash
make verify V=gazebo_pick
```

About three minutes without a GPU. Two windows open, the wrist camera and a fixed camera in
front of the bench, plus Gazebo. Logs and the frames `1_open.png` to `5_lifted.png` go to
`log/gazebo_pick/<date_time>/`. Frame 3 shows the cube between the two finger pads before
closing, frame 5 shows it held with the pedestal gone from view.

## What it checks

| Check | Where |
|---|---|
| The five finger joints follow `gripper_controller` | the gripper closes on the cube and blocks part way, `pick.log` says "blocked on the object" |
| `/wrist_camera/image_raw` arrives in ROS 2 | `pick.py` fails if no frame came |
| MoveIt plans and executes against the planning scene | every move prints `error_code 1` |
| The cube was lifted | `cube: z 0.8425 -> ... rise ... m` in `pick.log`, read from the Gazebo pose topic |

## Run of 2026-10-29

`make verify V=gazebo_pick` on a laptop without a GPU, software rendering, real-time factor about
0.2. `figures/pick.log`:

```text
gripper -> +0.10: at +0.090
pre-grasp (0.17, 0.0, 0.2525): error_code 1
grasp (0.17400000000000002, 0.0, 0.1575): straight path 100%
grasp: executed, error_code 1
gripper -> -0.60: blocked on the object
lift (0.17400000000000002, 0.0, 0.2925): straight path 88%
lift: executed, error_code 1
cube: z 0.842 -> 0.957 m, rise 0.114 m, needed 0.05
PASS
```

Frame 3, at the cube before closing, and frame 5, lifted:

![at the cube](figures/3_at_cube.png)
![lifted](figures/5_lifted.png)

## Numbers behind the heights

Measured on the collision meshes of the gripper (2026-10-28): the palm is 3 cm below the
`gripper_base` origin and the finger pads reach 7 cm. The grasp target is `gripper_base` 6.5 cm
above the cube centre: the pads surround the upper half of the cube and the palm stays 1.2 cm
above it. At 4 cm the palm lands on the cube, at 5.5 cm the pads land on the pedestal and the
fingers cannot close. The descent must be a straight line (`compute_cartesian_path`): a free
joint-space plan between the same two poses swings sideways and pushes the cube off.

## Known limits

- Without a GPU the simulation runs at 0.2 to 0.3 of real time because of the two cameras.
- The lift is accepted from 50 % of the straight path: near the top the arm runs into a joint
  limit. The cube still rises well above the 5 cm bar.
- A gripper blocked on the cube never finishes its action goal: `pick.py` waits 6 s and goes on.
