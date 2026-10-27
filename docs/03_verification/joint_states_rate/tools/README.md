# /joint_states acceptance run (#23)

Publishes the arm's joints on `/joint_states` from the Jetson container with the real serial
backend, records 10 minutes, and checks the recording against the acceptance criteria of #23:

| Criterion (#23) | How it is checked |
|---|---|
| Rate equal to the value fixed by #20 | `check_joint_states.py --rate <#20 value>`: mean rate within 5 % |
| 10 minutes without a period above 2x nominal | `topic_hz.log` (ros2 topic hz) and the bag: every period between two messages |
| Joint names equal to the URDF | every message carries exactly the six revolute joints of the URDF |
| Values in radians | every position is finite and inside the URDF joint limits, and the first positions match `pymycobot get_angles` (degrees) |

| File | What it is |
|---|---|
| `record_joint_states.sh` | Records the bag and the `ros2 topic hz` log for 10 min 30 s. Read-only. |
| `check_joint_states.py` | Checks a bag, prints PASS/FAIL per criterion, writes a Markdown table. |
| `test_check_joint_states.py` | Unit tests of the checks, no ROS 2 needed: `python3 -m pytest docs/03_verification/joint_states_rate/tools` |

## Before you start

- **The plugin reads the backend parameters.** `backend`, `serial_port` and `baud_rate` in
  `on_init`, the names used by `positronic_description/urdf/positronic_real.urdf.xacro`.
- **The rate.** #20 measured `get_angles` alone at 101 Hz on average (one call up to 21 ms) but
  did not fix a read rate. The working value is 50 Hz, explained in
  `positronic_bringup/config/jetson_controllers.yaml`. Pass another one with
  `update_rate:=<hz>` on the launch and `--rate <hz>` on the checker.

## Safety

From the issue: arm powered, no motion commands, bench clear.

- The launch is read-only: the plugin has no command interface, so no controller can move the
  arm. Do not start any other controller or script that sends `send_angles` during the run.
- One process on `/dev/ttyTHS1` at a time. Close any `pymycobot` session, capture script or
  other container that opens the port. (The boot service container does not open it.)
- If the arm is powered off, `get_angles` gets no reply. `power_on` locks the servos where they
  are: hold the arm while you send it.

## Procedure

**1. Build and open a shell on the Jetson** (from your laptop):

```bash
make nano-build
make nano-shell            # shell 1, inside the container, with /dev/ttyTHS1
```

**2. Reference angles** (shell 1, before the launch, so nothing else holds the port):

```bash
python3 -c "from pymycobot import MyCobot280; print(MyCobot280('/dev/ttyTHS1', 1000000).get_angles())"
```

Note the six angles (degrees). They prove the driver publishes radians. Do not move the arm
afterwards.

**3. Start the driver** (shell 1):

```bash
ros2 launch positronic_bringup driver.launch.py update_rate:=<#20 rate>
```

**4. Quick look, then record** (shell 2, from your laptop):

```bash
ssh -t nano docker exec -it positronic-arm bash
ros2 control list_hardware_components        # PositronicArm, active
ros2 control list_controllers                # joint_state_broadcaster, active
ros2 topic echo --once /joint_states         # six URDF names, small values in radians
bash docs/03_verification/joint_states_rate/tools/record_joint_states.sh
```

It runs for 10 min 30 s and prints where the results went
(`/ws/joint_states_runs/<date>_<time>/`).

**5. Check** (shell 2):

```bash
python3 docs/03_verification/joint_states_rate/tools/check_joint_states.py \
  /ws/joint_states_runs/<run>/bag --rate <#20 rate> \
  --reference-deg '<the list from step 2>' \
  --report /ws/joint_states_runs/<run>/report.md
```

Exit code 0 means every criterion passed.

**6. Copy the results to your laptop now.** `make nano-sync` (and `nano-build`, `nano-shell`)
mirror your laptop onto the Jetson with `rsync --delete`, which deletes the run folder:

```bash
scp -r nano:positronic/<you>/joint_states_runs/<run> .
```

Bags are ignored by git (`*.mcap`, `*.db3`, `rosbag2_*/`). Commit the report and the evidence
document, not the bag.

**7. Fill** `docs/03_verification/joint_states_rate/docs/joint_states_rate.md` with the report, the last lines of
`topic_hz.log` and `run_info.txt`.

## Reading the result

- `periods` fails but `info header` is steady: the control loop kept time and the messages were
  late on the way (DDS, CPU load from recording). Fails with a slow header too: the control loop
  itself stalled (look at `read()` and the serial thread).
- `warn start`: the first messages were all zeros. That is `SerialArm`'s start pose, published
  before the first serial reply (see the limitations below).

## Known limitations (to discuss in #22)

- **Publish rate is not sample rate.** The broadcaster publishes once per control cycle, from
  the latest sample of the serial thread. The thread polls on its own clock: one `get_angles`
  round trip (about 11 ms in the 2026-10-16 capture) plus `SerialArm::POLL_PERIOD` (10 ms), so
  roughly 45 to 50 new samples per second. With `update_rate` above that, `/joint_states` keeps
  its rate but repeats values, and `ros2 topic hz` cannot see it. The #23 criteria measure the
  publish rate; making the poll period follow `update_rate` belongs to #22.
- **Start pose before the first reply.** `SerialArm` returns all zeros until the first reply,
  and the plugin publishes them as a real pose for a few cycles. Waiting for the first sample
  before `on_activate` succeeds would fix it in #22.
