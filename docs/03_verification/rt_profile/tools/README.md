# Real-time profile run (#39)

Measures the period and the jitter of the control loop on the Jetson, and the latency from a
command to the first motion the arm reports. Three runs, one tool each way:

| Criterion (#39) | How it is measured |
|---|---|
| Timestamps from `write()` over a 10 minute run, histogram, mean, p99 and max | `idle` run: one row per controller cycle, `analyze_rt.py` prints the histogram and the statistics |
| Command to motion latency of the servos, 20 samples, mean and max | `steps` run: 20 commanded steps on one joint, latency from the `write()` that carried each step to the first cycle whose position moved |
| Load test: CPU saturated during a 2 minute run, period and jitter against the idle run | `load` run: every core busy, same measurement, side-by-side table |
| `rt_profile_v1.md` with the plots and the numbers | the `--figure-dir` pngs and the printed blocks go into [`../rt_profile_v1.md`](../rt_profile_v1.md) |

| File | What it is |
|---|---|
| `run_rt_profile.sh` | Runs the three runs on the Jetson. `idle` and `load` are read-only; `steps` moves one joint. |
| `rt_record.py` | ROS 2 node: one row per `/joint_states` message into `cycles.csv`, and with `--steps N` the commanded steps into `steps.csv`. |
| `analyze_rt.py` | No ROS 2: reads the csv files, prints period, jitter, late cycles, a histogram and the latency, writes the pngs and the Markdown table. |
| `test_analyze_rt.py` | Unit tests of the analysis, no ROS 2 and no arm needed: `python3 -m pytest docs/03_verification/rt_profile/tools` |

## What a cycle stamp is, and what it is not

`write()` is called once per `controller_manager` cycle. `joint_state_broadcaster` publishes
once per cycle too, with `header.stamp` set to the time the cycle started, so **one
`/joint_states` message is one `write()`**, and the interval between two cycle stamps is the
interval between two `write()` calls. That is what the period and the jitter here are measured
on, with no change to the driver.

What this does not give is the offset inside a cycle: `write()` runs a few hundred microseconds
after the stamp, after `read()` and after the controllers' `update()`. The offset cancels out of
every period, and it is far below the latency measured in the `steps` run, so it is reported as
a known limit rather than measured. `analyze_rt.py` also prints the **arrival** period, measured
when the recorder received the message: that one carries DDS and the recorder's own scheduling,
and is always the noisier of the two. A `periods` problem that shows on arrival but not on the
cycle stamps was not the loop.

## Before you start

- **The driver runs at a known rate.** `driver.launch.py` defaults to `update_rate:=30`
  (`jetson_controllers.yaml`, fixed by #23). Pass the same number to `analyze_rt.py --rate`.
- **For the `steps` run only:** the arm must be in **fresh mode 1**, or streamed commands queue
  up instead of replacing each other (#20). Set it **before** the driver takes the port, in the
  same shell where you then launch it:
  ```bash
  python3 -c "from pymycobot import MyCobot280; a=MyCobot280('/dev/ttyTHS1', 1000000); \
    a.power_on(); a.set_fresh_mode(1); print('fresh mode', a.get_fresh_mode())"
  ```
  The runner cannot check this: while the driver holds the port, nothing else can open it.
- **Nothing else on `/dev/ttyTHS1`.** The driver locks the port (`TIOCEXCL`).
- **For the `steps` run only:** the `positronic_arm` hardware component must be **active**.
  `driver.launch.py` brings it up active, but since #36 an inactive component makes `write()`
  return without sending anything, so the steps would move nothing. The runner checks this and
  stops; to set it by hand:
  ```bash
  ros2 control list_hardware_components
  ros2 control set_hardware_component_state positronic_arm active
  ```

## Safety

- `idle` and `load` only subscribe. No controller that can move the arm is spawned, so the arm
  cannot move during the two long runs.
- `steps` moves **one joint**, by `DELTA_RAD` (0.0873 rad, 5 deg) and back, 20 times, at the
  driver's fixed `SEND_SPEED` of 30. Bench clear, hand near the power switch. The runner prints
  a banner and a 5 s countdown before the first command, and `rt_record.py` commands the pose
  the arm was resting at as its last message, whatever the last step asked for.
- `load` saturates every core for 2 minutes. The control loop is **not** running with real-time
  priority (the container logs `Could not enable FIFO RT scheduling policy`), so the loop will
  miss cycles during this run. That is the point of the test. Keep the arm still: do not run
  `load` and `steps` together.

## Procedure

**1. Build and open two shells on the Jetson** (from your laptop):

```bash
make nano-build
make nano-shell            # shell 1, inside the container, with /dev/ttyTHS1
ssh -t nano docker exec -it positronic-arm bash   # shell 2
```

**2. Start the driver** (shell 1; for the `steps` run, set fresh mode first, see above):

```bash
ros2 launch positronic_bringup driver.launch.py update_rate:=30
```

**3. The 10 minute run and the load run** (shell 2, read-only, nothing moves):

```bash
export OUT=/ws/log/rt_$(date +%Y-%m-%d_%H%M)
bash docs/03_verification/rt_profile/tools/run_rt_profile.sh idle
bash docs/03_verification/rt_profile/tools/run_rt_profile.sh load
```

**4. The latency run** (shell 2, **the arm moves**; bench clear):

```bash
ros2 run controller_manager spawner forward_position_controller
bash docs/03_verification/rt_profile/tools/run_rt_profile.sh steps
```

**5. Analyse** (shell 2, or on your laptop after step 6):

```bash
python3 docs/03_verification/rt_profile/tools/analyze_rt.py \
  "$OUT"/idle/cycles.csv "$OUT"/load/cycles.csv "$OUT"/steps/cycles.csv \
  --rate 30 --figure-dir docs/03_verification/rt_profile/figures \
  --report "$OUT"/table.md
```

The pngs need matplotlib. Without it the tool prints the same histogram as text and says so;
run step 5 again on your laptop to get the pngs.

**6. Copy the results to your laptop.** `make nano-sync` (and `nano-build`, `nano-shell`)
mirror your laptop onto the Jetson with `rsync --delete`, but `log` is one of its excludes, so
the run folder under `/ws/log` survives a sync. Copy it anyway: the pngs are committed from
your laptop, and nothing else backs the Jetson up.

```bash
scp -r nano:positronic/<you>/log/<run folder> .
```

The csv files are not committed (`*.csv` runs are kept by the operator). Commit the pngs and
`rt_profile_v1.md`.

**7. Fill** [`../rt_profile_v1.md`](../rt_profile_v1.md): every `<...>` in it is a number or a
name only the run can give.

## Reading the result

- **`late 2x` is 0 on `idle` and not on `load`:** the expected outcome. The loop keeps time
  when the Jetson is quiet and misses cycles when it is not, because nothing reserves the CPU
  for it.
- **`late 2x` is not 0 on `idle`:** compare with the #23 run at the same rate
  (`../joint_states_rate/joint_states_rate.md`, max 41.9 ms at 30 Hz). Something else was
  running on the Jetson, or the rate is too high for this image.
- **`cycle` is steady but `arrival` is not:** the loop was fine and the messages were late on
  the way. Nothing to fix in the driver.
- **`latency ... no usable sample`:** the steps commanded less than the motion threshold, or the
  arm never moved. Check fresh mode 1, that the `positronic_arm` component was active and that
  `forward_position_controller` was active, and raise `DELTA_RAD`.
- **Latency around one serial turn plus the servo's own lag:** expected. A command waits up to
  `moving_period_ms` (50 ms) in the serial thread before it reaches the wire, see
  `../write_sweep/README.md`; the wire itself costs under 1 ms.
