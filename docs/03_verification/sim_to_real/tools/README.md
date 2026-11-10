# Sim-to-real tools (#37)

Sends one trajectory to `arm_controller` in Gazebo and on the real arm, records both, and
compares them per joint. The report is [`../sim_to_real_v1.md`](../sim_to_real_v1.md).

| File | What it is |
|---|---|
| `trajectory.yaml` | The one trajectory both worlds run: home, 5 points, back home, 20 s. Radians. |
| `run_trajectory.py` | Checks the file (URDF names, 10 deg inside the limits, peak speed at most 15 deg/s), moves to home, sends the trajectory as one goal, prints the result. Ctrl-C cancels the goal. `--check-only` sends nothing. |
| `record_run.sh` | One run, one folder: `record_run.sh <sim\|real> <n>` waits for data, records `/joint_states` and `/arm_controller/controller_state` while `run_trajectory.py` runs, writes `run_info.txt`. A failed run is renamed `_FAILED`. |
| `compare_runs.py` | Lines up two or more runs, prints per joint max, mean, signed mean, RMS and final gap in degrees, and each run's tracking error. `--plot` writes a PNG. |
| `test_run_trajectory.py`, `test_compare_runs.py` | Unit tests, no ROS 2 needed: `python3 -m pytest docs/03_verification/sim_to_real/tools` |

Run everything from the repo root (`/ws` in the container). The world is picked by
`ROS_DOMAIN_ID`: **18 = Gazebo, 17 = the real arm**. `record_run.sh` refuses a run on the wrong
domain.

## Gazebo runs

Two shells, both opened with `ROS_DOMAIN_ID=18 make shell` and `source install/setup.bash`:

```bash
# shell 1
ros2 launch positronic_bringup gazebo.launch.py

# shell 2
ros2 control list_controllers      # arm_controller and joint_state_broadcaster: active
bash docs/03_verification/sim_to_real/tools/record_run.sh sim 1
```

Repeat with `sim 2`, `sim 3`, ... with the arm standing still between runs. Each run takes
about 30 s and ends with `error_code 0 Goal successfully reached!`.

## Real arm runs

**This moves the arm.** Follow the safety protocol of
[`../../jtc_on_the_arm/README.md`](../../jtc_on_the_arm/README.md) before the launch: two
people, one with a hand on the power plug for the whole session, bench clear, a 5 deg test move
first. `run_trajectory.py` moves to home before the trajectory: check that the arm rests near
home, since that move comes first.

Record on the Jetson, inside the container, so Wi-Fi cannot drop messages:

```bash
# shell 1 (laptop)
NANO=<user@jetson> make nano-build
NANO=<user@jetson> make nano-shell
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
ros2 launch positronic_bringup driver.launch.py

# shell 2, second SSH session into the same container
ssh -t <user@jetson> docker exec -it positronic-arm bash
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
export ROS_DOMAIN_ID=17
RUNS_DIR=/ws/runs bash docs/03_verification/sim_to_real/tools/record_run.sh real 1
```

**Copy the runs to your laptop right away.** `make nano-sync`, `nano-build` and `nano-shell`
mirror the laptop onto the Jetson with `rsync --delete`, which deletes them:

```bash
scp -r <user@jetson>:positronic/<you>/runs/real_1 runs/
```

At the end: `ros2 control set_controller_state arm_controller inactive`, then close the launch.

## Compare

```bash
python3 docs/03_verification/sim_to_real/tools/compare_runs.py runs/sim_2 runs/sim_3 runs/sim_4 \
  --plot runs/sim_vs_sim.png        # noise floor of Gazebo
python3 docs/03_verification/sim_to_real/tools/compare_runs.py runs/real_1 runs/real_2 runs/real_3
                                    # repeatability of the arm
python3 docs/03_verification/sim_to_real/tools/compare_runs.py runs/sim_2 runs/real_1 runs/real_2 \
  runs/real_3 --plot runs/real_vs_sim.png
```

The first folder is the baseline A; every other one is compared with it (B - A). Runs are lined
up where the reference leaves the home pose, using the header stamps (simulation time in Gazebo,
the Jetson's clock on the arm), so the two worlds' clocks never have to agree. A `_FAILED` run or
runs made with different `trajectory.yaml` files print a warning.

Bags are ignored by git (`/runs/`). Commit the numbers and the figures (`../figures/`), not the
bags.

## If something goes wrong

| Message | Cause |
|---|---|
| `ROS_DOMAIN_ID is '17', but sim uses 18.` | the shell was opened without `ROS_DOMAIN_ID=18` |
| `No message on /joint_states for 30 s` | the launch is not running, still starting, or on another domain |
| `REFUSED trajectory.yaml:` | a position or a speed is outside the limits; the lines below say which |
| `ERROR: the reference never leaves the home pose` | the trajectory did not run in that bag (cancelled before it started) |
