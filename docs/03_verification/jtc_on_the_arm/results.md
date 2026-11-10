# Per-run results of the joint_trajectory_controller on the arm (#33)

The tables behind [README.md](README.md). For run A, one row per waypoint of every run (the 3 deg
criterion is per waypoint), then the last feedback (`last`) and the largest error at any cycle of
the run (`max abs`). For the other sets only `last` and `max abs`, which is what the README's
claims rest on. Error per joint in degrees. Positive means the arm is ahead of the command, negative
behind. Generated with `tools/waypoint_error.py` over the recordings of `ros2 action send_goal
--feedback`, saved with `tee`.

## Run A, default tolerances

Gripper and camera mounted, tolerances at their defaults (0, no check).

| file | status | waypoint | j1 | j2 | j3 | j4 | j5 | j6 | max |
|---|---|---|---|---|---|---|---|---|---|
| T1 run 1 | SUCCEEDED | 2 s | -0.00 | -0.00 | +0.00 | +1.23 | -0.00 | +0.00 | 1.23 |
| T1 run 1 | SUCCEEDED | 5 s | +1.42 | -0.00 | +0.00 | +1.23 | -0.00 | +0.00 | 1.42 |
| T1 run 1 | SUCCEEDED | 8 s | -1.21 | -0.00 | +0.00 | +1.23 | -0.00 | +0.00 | 1.23 |
| T1 run 1 | SUCCEEDED | last | -1.21 | -0.00 | +0.00 | +1.23 | -0.00 | +0.00 | 1.23 |
| T1 run 1 | SUCCEEDED | max abs | +1.79 | +0.00 | +0.00 | +1.23 | +0.00 | +0.00 | 1.79 |
| T1 run 2 | SUCCEEDED | 2 s | -1.32 | -0.00 | +0.00 | +2.38 | -0.00 | +0.00 | 2.38 |
| T1 run 2 | SUCCEEDED | 5 s | +1.29 | -0.00 | +0.00 | +2.29 | -0.00 | +0.00 | 2.29 |
| T1 run 2 | SUCCEEDED | 8 s | -1.29 | -0.00 | +0.00 | +2.20 | -0.00 | +0.00 | 2.20 |
| T1 run 2 | SUCCEEDED | last | -1.29 | -0.00 | +0.00 | +2.20 | -0.00 | +0.00 | 2.20 |
| T1 run 2 | SUCCEEDED | max abs | +1.79 | +0.00 | +0.00 | +2.38 | +0.00 | +0.00 | 2.38 |
| T1 run 3 | SUCCEEDED | 2 s | -1.05 | -0.17 | +0.00 | +2.55 | -0.00 | +0.00 | 2.55 |
| T1 run 3 | SUCCEEDED | 5 s | +1.29 | -0.17 | +0.00 | +2.38 | -0.00 | +0.00 | 2.38 |
| T1 run 3 | SUCCEEDED | 8 s | -1.58 | -0.17 | +0.00 | +2.38 | -0.00 | +0.00 | 2.38 |
| T1 run 3 | SUCCEEDED | last | -1.58 | -0.17 | +0.00 | +2.38 | -0.00 | +0.00 | 2.38 |
| T1 run 3 | SUCCEEDED | max abs | +1.79 | +0.17 | +0.00 | +2.55 | +0.00 | +0.00 | 2.55 |
| T2 run 1 | SUCCEEDED | 2 s | -1.32 | -0.00 | +0.00 | +2.73 | -0.00 | +0.00 | 2.73 |
| T2 run 1 | SUCCEEDED | 6 s | +1.63 | +1.02 | -0.18 | +1.85 | -0.00 | +0.00 | 1.85 |
| T2 run 1 | SUCCEEDED | 10 s | -1.78 | -1.08 | +0.00 | +1.67 | -0.00 | +0.00 | 1.78 |
| T2 run 1 | SUCCEEDED | last | -1.78 | -1.08 | +0.00 | +1.67 | -0.00 | +0.00 | 1.78 |
| T2 run 1 | SUCCEEDED | max abs | +2.14 | +1.85 | +0.18 | +2.73 | +0.00 | +0.00 | 2.73 |
| T2 run 2 | SUCCEEDED | 2 s | -1.32 | -0.17 | +0.00 | +2.55 | -0.00 | +0.00 | 2.55 |
| T2 run 2 | SUCCEEDED | 6 s | +1.54 | +0.84 | -0.09 | +1.67 | -0.00 | +0.00 | 1.67 |
| T2 run 2 | SUCCEEDED | 10 s | -1.76 | -1.23 | +0.00 | +1.59 | -0.00 | +0.00 | 1.76 |
| T2 run 2 | SUCCEEDED | last | -1.76 | -1.23 | +0.00 | +1.59 | -0.00 | +0.00 | 1.76 |
| T2 run 2 | SUCCEEDED | max abs | +2.00 | +1.94 | +0.26 | +2.67 | +0.00 | +0.00 | 2.67 |
| T2 run 3 | SUCCEEDED | 2 s | -1.30 | -0.34 | +0.00 | +2.61 | -0.00 | +0.00 | 2.61 |
| T2 run 3 | SUCCEEDED | 6 s | +1.90 | +1.02 | -0.18 | +1.67 | -0.00 | +0.00 | 1.90 |
| T2 run 3 | SUCCEEDED | 10 s | -1.69 | -0.90 | +0.00 | +1.59 | -0.00 | +0.00 | 1.69 |
| T2 run 3 | SUCCEEDED | last | -1.69 | -0.90 | +0.00 | +1.59 | -0.00 | +0.00 | 1.69 |
| T2 run 3 | SUCCEEDED | max abs | +2.14 | +1.94 | +0.26 | +2.64 | +0.00 | +0.00 | 2.64 |
| T3 run 1 | SUCCEEDED | 2 s | -1.03 | -0.34 | +0.00 | +2.70 | -0.00 | +0.00 | 2.70 |
| T3 run 1 | SUCCEEDED | 4 s | +2.08 | +1.21 | +0.00 | +1.94 | -0.00 | +0.00 | 2.08 |
| T3 run 1 | SUCCEEDED | 6 s | -2.30 | -1.42 | +0.00 | +1.50 | -0.00 | +0.00 | 2.30 |
| T3 run 1 | SUCCEEDED | last | -2.30 | -1.42 | +0.00 | +1.50 | -0.00 | +0.00 | 2.30 |
| T3 run 1 | SUCCEEDED | max abs | +2.71 | +2.32 | +0.26 | +2.70 | +0.00 | +0.00 | 2.71 |
| T3 run 2 | SUCCEEDED | 2 s | -0.72 | +0.16 | +0.00 | +2.47 | -0.00 | +0.00 | 2.47 |
| T3 run 2 | SUCCEEDED | 4 s | +2.08 | +1.21 | -0.09 | +1.94 | -0.00 | +0.00 | 2.08 |
| T3 run 2 | SUCCEEDED | 6 s | -2.83 | -1.95 | +0.00 | +1.59 | -0.00 | +0.00 | 2.83 |
| T3 run 2 | SUCCEEDED | last | -2.83 | -1.95 | +0.00 | +1.59 | -0.00 | +0.00 | 2.83 |
| T3 run 2 | SUCCEEDED | max abs | +2.93 | +2.32 | +0.26 | +2.53 | +0.00 | +0.00 | 2.93 |
| T3 run 3 | SUCCEEDED | 2 s | -1.03 | -0.34 | +0.00 | +2.61 | -0.00 | +0.00 | 2.61 |
| T3 run 3 | SUCCEEDED | 4 s | +2.17 | +1.21 | +0.00 | +1.94 | -0.00 | +0.00 | 2.17 |
| T3 run 3 | SUCCEEDED | 6 s | -2.33 | -1.44 | +0.00 | +1.50 | -0.00 | +0.00 | 2.33 |
| T3 run 3 | SUCCEEDED | last | -2.33 | -1.44 | +0.00 | +1.50 | -0.00 | +0.00 | 2.33 |
| T3 run 3 | SUCCEEDED | max abs | +2.85 | +2.32 | +0.26 | +2.61 | +0.00 | +0.00 | 2.85 |

## Run B, with the tolerances

Gripper and camera mounted, tolerances of `jetson_controllers.yaml`.

| file | status | waypoint | j1 | j2 | j3 | j4 | j5 | j6 | max |
|---|---|---|---|---|---|---|---|---|---|
| T1 run 1 | SUCCEEDED | last | -1.47 | +0.70 | +0.53 | +2.29 | -0.80 | +0.53 | 2.29 |
| T1 run 1 | SUCCEEDED | max abs | +1.73 | +0.70 | +0.53 | +2.64 | +0.80 | +0.53 | 2.64 |
| T1 run 2 | SUCCEEDED | last | -1.59 | +0.70 | +0.53 | +2.38 | -0.80 | +0.53 | 2.38 |
| T1 run 2 | SUCCEEDED | max abs | +1.79 | +0.70 | +0.53 | +2.67 | +0.80 | +0.53 | 2.67 |
| T1 run 3 | SUCCEEDED | last | -1.38 | +0.70 | +0.53 | +2.38 | -0.80 | +0.53 | 2.38 |
| T1 run 3 | SUCCEEDED | max abs | +1.79 | +0.70 | +0.53 | +2.66 | +0.80 | +0.53 | 2.66 |
| T2 run 1 | SUCCEEDED | last | -1.69 | -0.90 | +0.44 | +1.59 | -0.80 | +0.53 | 1.69 |
| T2 run 1 | SUCCEEDED | max abs | +2.14 | +1.85 | +0.53 | +2.73 | +0.80 | +0.53 | 2.73 |
| T2 run 2 | SUCCEEDED | last | -1.61 | -0.90 | +0.44 | +1.67 | -0.80 | +0.44 | 1.67 |
| T2 run 2 | SUCCEEDED | max abs | +2.06 | +1.85 | +0.53 | +2.64 | +0.80 | +0.53 | 2.64 |
| T2 run 3 | SUCCEEDED | last | -1.69 | -0.90 | +0.44 | +1.59 | -0.80 | +0.53 | 1.69 |
| T2 run 3 | SUCCEEDED | max abs | +1.99 | +1.85 | +0.53 | +2.65 | +0.80 | +0.53 | 2.65 |
| T3 run 1 | SUCCEEDED | last | -2.66 | -1.69 | +0.44 | +1.50 | -0.80 | +0.44 | 2.66 |
| T3 run 1 | SUCCEEDED | max abs | +2.70 | +2.48 | +0.53 | +2.61 | +0.80 | +0.53 | 2.70 |
| T3 run 2 | SUCCEEDED | last | -2.33 | -1.36 | +0.44 | +1.50 | -0.80 | +0.53 | 2.33 |
| T3 run 2 | SUCCEEDED | max abs | +2.68 | +2.48 | +0.53 | +2.53 | +0.80 | +0.53 | 2.68 |
| T3 run 3 | SUCCEEDED | last | -2.24 | -1.35 | +0.44 | +1.59 | -0.80 | +0.53 | 2.24 |
| T3 run 3 | SUCCEEDED | max abs | +3.12 | +2.65 | +0.53 | +2.53 | +0.80 | +0.53 | 3.12 |
| T6 run 1 | SUCCEEDED | last | -3.59 | -2.54 | +0.62 | +1.50 | -0.88 | +0.53 | 3.59 |
| T6 run 1 | SUCCEEDED | max abs | +3.73 | +3.28 | +0.88 | +2.81 | +0.88 | +0.53 | 3.73 |
| T6 run 2 | SUCCEEDED | last | -3.31 | -2.17 | +0.62 | +1.67 | -0.88 | +0.53 | 3.31 |
| T6 run 2 | SUCCEEDED | max abs | +4.00 | +3.43 | +0.70 | +2.55 | +0.88 | +0.53 | 4.00 |
| T6 run 3 | SUCCEEDED | last | -3.49 | -2.43 | +0.62 | +1.50 | -0.88 | +0.53 | 3.49 |
| T6 run 3 | SUCCEEDED | max abs | +3.78 | +3.37 | +0.70 | +2.65 | +0.88 | +0.53 | 3.78 |
| T4 run 1 | SUCCEEDED | last | -4.20 | -3.15 | +0.44 | +1.50 | -0.80 | +0.53 | 4.20 |
| T4 run 1 | SUCCEEDED | max abs | +4.47 | +3.65 | +0.53 | +2.61 | +0.80 | +0.53 | 4.47 |
| T4 run 2 | SUCCEEDED | last | -4.51 | -3.37 | +0.62 | +1.59 | -0.88 | +0.53 | 4.51 |
| T4 run 2 | SUCCEEDED | max abs | +4.85 | +4.50 | +0.88 | +2.66 | +0.88 | +0.53 | 4.85 |
| T4 run 3 | SUCCEEDED | last | -4.22 | -3.07 | +0.62 | +1.50 | -0.88 | +0.53 | 4.22 |
| T4 run 3 | SUCCEEDED | max abs | +4.83 | +4.06 | +0.70 | +2.64 | +0.88 | +0.53 | 4.83 |
| T4 run 4 | SUCCEEDED | last | -4.60 | -3.28 | +0.62 | +1.50 | -0.88 | +0.44 | 4.60 |
| T4 run 4 | SUCCEEDED | max abs | +4.74 | +4.29 | +0.70 | +2.57 | +0.88 | +0.53 | 4.74 |
| T8 run 1 | ABORTED | last | +4.33 | +3.63 | +0.70 | +2.38 | -0.88 | +0.44 | 4.33 |
| T8 run 1 | ABORTED | max abs | +4.33 | +4.06 | +0.70 | +2.55 | +0.88 | +0.53 | 4.33 |
| T8 run 2 | ABORTED | last | +4.25 | +3.11 | +0.79 | +1.94 | -0.88 | +0.44 | 4.25 |
| T8 run 2 | ABORTED | max abs | +4.66 | +3.94 | +0.88 | +2.73 | +0.88 | +0.53 | 4.66 |
| T8 run 3 | ABORTED | last | +4.32 | +3.17 | +0.79 | +1.94 | -0.88 | +0.53 | 4.32 |
| T8 run 3 | ABORTED | max abs | +4.71 | +3.94 | +0.79 | +2.73 | +0.88 | +0.53 | 4.71 |
| T7 run 1 | ABORTED | last | +4.56 | +3.86 | +0.70 | +2.55 | -0.88 | +0.53 | 4.56 |
| T7 run 1 | ABORTED | max abs | +4.69 | +4.43 | +0.70 | +2.73 | +0.88 | +0.53 | 4.69 |
| T5 run 1 | ABORTED | last | +2.95 | +3.83 | +0.53 | +2.64 | -0.80 | +0.53 | 3.83 |
| T5 run 1 | ABORTED | max abs | +2.95 | +3.83 | +0.53 | +2.64 | +0.80 | +0.53 | 3.83 |

## Without the gripper and the camera

Same goals, gripper and camera taken off, tolerances on.

| file | status | waypoint | j1 | j2 | j3 | j4 | j5 | j6 | max |
|---|---|---|---|---|---|---|---|---|---|
| T1 run 1 | SUCCEEDED | last | -1.18 | +0.61 | -0.61 | +0.80 | +1.39 | +0.53 | 1.39 |
| T1 run 1 | SUCCEEDED | max abs | +1.56 | +0.61 | +0.61 | +0.88 | +1.39 | +0.53 | 1.56 |
| T1 run 2 | SUCCEEDED | last | -1.27 | -0.26 | -0.44 | +0.80 | +1.39 | +0.53 | 1.39 |
| T1 run 2 | SUCCEEDED | max abs | +1.56 | +0.35 | +0.44 | +0.88 | +1.39 | +0.53 | 1.56 |
| T1 run 3 | SUCCEEDED | last | -1.27 | -0.26 | -0.44 | +0.80 | +1.39 | +0.53 | 1.39 |
| T1 run 3 | SUCCEEDED | max abs | +1.56 | +0.35 | +0.44 | +0.90 | +1.39 | +0.53 | 1.56 |
| T2 run 1 | SUCCEEDED | last | -1.51 | -1.07 | -0.44 | +0.53 | +1.39 | +0.53 | 1.51 |
| T2 run 1 | SUCCEEDED | max abs | +1.91 | +1.94 | +0.44 | +0.90 | +1.39 | +0.53 | 1.94 |
| T2 run 2 | SUCCEEDED | last | -1.51 | -1.07 | -0.44 | +0.62 | +1.39 | +0.53 | 1.51 |
| T2 run 2 | SUCCEEDED | max abs | +1.91 | +1.85 | +0.44 | +0.88 | +1.39 | +0.53 | 1.91 |
| T2 run 3 | SUCCEEDED | last | -1.61 | -1.17 | -0.44 | +0.62 | +1.39 | +0.53 | 1.61 |
| T2 run 3 | SUCCEEDED | max abs | +1.91 | +1.94 | +0.44 | +0.88 | +1.39 | +0.53 | 1.94 |
| T3 run 1 | SUCCEEDED | last | -2.57 | -1.95 | -0.61 | +0.53 | +1.39 | +0.53 | 2.57 |
| T3 run 1 | SUCCEEDED | max abs | +2.72 | +2.54 | +0.61 | +0.91 | +1.39 | +0.53 | 2.72 |
| T3 run 2 | SUCCEEDED | last | -2.24 | -1.61 | -0.44 | +0.53 | +1.39 | +0.53 | 2.24 |
| T3 run 2 | SUCCEEDED | max abs | +2.63 | +2.48 | +0.44 | +0.88 | +1.39 | +0.53 | 2.63 |
| T3 run 3 | SUCCEEDED | last | -2.40 | -1.79 | -0.44 | +0.53 | +1.39 | +0.53 | 2.40 |
| T3 run 3 | SUCCEEDED | max abs | +2.62 | +2.50 | +0.44 | +0.88 | +1.39 | +0.53 | 2.62 |
| T6 run 1 | SUCCEEDED | last | -3.08 | -2.37 | +0.35 | +0.53 | -0.00 | +0.53 | 3.08 |
| T6 run 1 | SUCCEEDED | max abs | +3.50 | +3.10 | +0.70 | +1.02 | +0.00 | +0.65 | 3.50 |
| T4 run 1 | SUCCEEDED | last | -4.60 | -3.55 | -0.44 | +0.53 | +1.39 | +0.53 | 4.60 |
| T4 run 1 | SUCCEEDED | max abs | +4.68 | +4.41 | +0.44 | +0.88 | +1.39 | +0.53 | 4.68 |
| T8 run 1 | ABORTED | last | -4.26 | -3.91 | +0.35 | -0.26 | -0.00 | +0.53 | 4.26 |
| T8 run 1 | ABORTED | max abs | +4.96 | +3.99 | +0.62 | +0.90 | +0.00 | +0.53 | 4.96 |
| T8 run 2 | ABORTED | last | -4.41 | -3.53 | +0.35 | +0.44 | -0.00 | +0.53 | 4.41 |
| T8 run 2 | ABORTED | max abs | +4.95 | +4.66 | +0.53 | +0.91 | +0.00 | +0.53 | 4.95 |
| T8 run 3 | ABORTED | last | -4.32 | -3.44 | +0.35 | +0.44 | -0.00 | +0.53 | 4.32 |
| T8 run 3 | ABORTED | max abs | +4.71 | +4.32 | +0.53 | +0.90 | +0.00 | +0.53 | 4.71 |
| T5 run 1 | ABORTED | last | +1.88 | +2.41 | -0.35 | +0.88 | +1.48 | +0.53 | 2.41 |
| T5 run 1 | ABORTED | max abs | +1.88 | +2.41 | +0.35 | +0.88 | +1.48 | +0.53 | 2.41 |
| T9 run 1 | SUCCEEDED | last | -3.82 | -2.94 | +0.35 | +0.53 | -0.00 | +0.53 | 3.82 |
| T9 run 1 | SUCCEEDED | max abs | +3.89 | +3.45 | +0.53 | +0.82 | +0.00 | +0.53 | 3.89 |
| T9 run 2 | SUCCEEDED | last | -4.12 | -3.24 | +0.35 | +0.53 | -0.00 | +0.53 | 4.12 |
| T9 run 2 | SUCCEEDED | max abs | +4.17 | +3.74 | +0.62 | +0.90 | +0.00 | +0.53 | 4.17 |
| T9 run 3 | SUCCEEDED | last | -4.12 | -3.33 | +0.35 | +0.53 | -0.00 | +0.53 | 4.12 |
| T9 run 3 | SUCCEEDED | max abs | +4.17 | +3.74 | +0.62 | +0.90 | +0.00 | +0.53 | 4.17 |

## The wrist joints

Joints 3 to 6 moving, 1 and 2 still, gripper and camera mounted, tolerances on.

| file | status | waypoint | j1 | j2 | j3 | j4 | j5 | j6 | max |
|---|---|---|---|---|---|---|---|---|---|
| T11 run 1 | SUCCEEDED | last | -0.52 | +0.52 | -1.58 | -0.18 | -2.30 | -2.02 | 2.30 |
| T11 run 1 | SUCCEEDED | max abs | +0.79 | +0.52 | +2.31 | +3.67 | +4.24 | +2.02 | 4.24 |
| T11 run 2 | SUCCEEDED | last | -0.52 | +0.52 | -1.58 | -0.18 | -2.30 | -1.67 | 2.30 |
| T11 run 2 | SUCCEEDED | max abs | +0.61 | +0.52 | +2.41 | +3.77 | +3.51 | +2.02 | 3.77 |
| T11 run 3 | SUCCEEDED | last | -0.52 | +0.52 | -1.45 | -0.05 | -1.90 | -1.54 | 1.90 |
| T11 run 3 | SUCCEEDED | max abs | +0.61 | +0.52 | +2.20 | +3.59 | +3.85 | +2.01 | 3.85 |
| T12 run 1 | SUCCEEDED | last | -0.52 | +0.52 | -1.99 | -1.73 | -2.71 | -2.08 | 2.71 |
| T12 run 1 | SUCCEEDED | max abs | +0.79 | +0.52 | +3.03 | +4.33 | +4.12 | +2.77 | 4.33 |
| T12 run 2 | SUCCEEDED | last | -0.52 | +0.52 | -1.99 | -1.73 | -2.62 | -2.16 | 2.62 |
| T12 run 2 | SUCCEEDED | max abs | +0.79 | +0.52 | +3.10 | +4.33 | +3.63 | +2.74 | 4.33 |
| T12 run 3 | SUCCEEDED | last | -0.52 | +0.52 | -1.93 | -1.49 | -2.56 | -2.10 | 2.56 |
| T12 run 3 | SUCCEEDED | max abs | +0.61 | +0.52 | +2.82 | +4.03 | +3.85 | +2.39 | 4.03 |
| T13 run 1 | SUCCEEDED | last | -0.52 | +0.52 | -2.41 | -2.33 | -2.78 | -2.32 | 2.78 |
| T13 run 1 | SUCCEEDED | max abs | +0.61 | +0.52 | +3.13 | +4.44 | +4.68 | +2.86 | 4.68 |
| T13 run 2 | SUCCEEDED | last | -0.52 | +0.52 | -2.53 | -2.27 | -3.60 | -2.80 | 3.60 |
| T13 run 2 | SUCCEEDED | max abs | +0.61 | +0.52 | +3.51 | +4.74 | +4.29 | +3.04 | 4.74 |
| T13 run 3 | SUCCEEDED | last | -0.52 | +0.43 | -2.31 | -2.24 | -2.69 | -2.31 | 2.69 |
| T13 run 3 | SUCCEEDED | max abs | +0.79 | +0.52 | +3.22 | +4.45 | +4.16 | +2.77 | 4.45 |
| T10 run 1 | ABORTED | last | -0.61 | -0.00 | +3.25 | +4.40 | +4.38 | +2.56 | 4.40 |
| T10 run 1 | ABORTED | max abs | +0.61 | +0.00 | +3.29 | +4.61 | +4.38 | +2.56 | 4.61 |
| T10 run 2 | ABORTED | last | -0.52 | +0.18 | +3.82 | +4.97 | +4.60 | +3.22 | 4.97 |
| T10 run 2 | ABORTED | max abs | +0.52 | +0.18 | +3.82 | +4.97 | +4.60 | +3.22 | 4.97 |
