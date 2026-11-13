# Heartbeat link campaign (#45)

Measures how long the mission heartbeat can go missing on its way from the commanding laptop to
the Jetson, to choose from data:

- the **heartbeat rate** of the mission node, and
- the **heartbeat timeout** of the safety supervisor (`heartbeat_timeout_ms`).

**Result: heartbeat at 25 Hz, `heartbeat_timeout_ms` 300.** A dead mission node is detected
within 320 ms. Measured on 2026-11-12 and 13; details in Results.

## Decision rule

Fixed before the runs, so the data decides and nobody tunes the rule to the result:

1. **No false halts.** The timeout is at least **1.5 × the longest link gap** measured in any
   phase, rounded up to the next 50 ms. A stall of the link delays every heartbeat in flight
   whatever the rate, so the longest gap of any phase bounds every rate. (Amended after the
   first run, which compared each rate only with its own phases.)
2. **Heartbeat periods.** The timeout is at least **4 heartbeat periods**, so it never hangs on
   one or two lost packets.
3. **Fastest detection.** Among the rates tested, take the one that gives the **smallest
   timeout**. On a tie, the lower rate (less traffic on a shared Wi-Fi).
4. **Confirmation.** The chosen pair runs 15 minutes with the supervisor active and zero halts.
   If it halts, take the next 50 ms and confirm again.

A heartbeat gap at the same moment as a gap over 100 ms on `/joint_states` (published on the
Jetson itself, at 30 Hz) is the Jetson or the recorder stalling, not the link: the analysis
counts it apart and the rule leaves it out.

## Results

Lab Wi-Fi, Jetson Wi-Fi power saving off, one heartbeat publisher on the domain,
`supervised.launch.py backend:=fake` on the Jetson.

| Run | Heartbeats sent / received | Median gap | p99.9 | Longest gap | Gaps over 100 ms |
|---|---|---|---|---|---|
| 10 Hz quiet, 10 min | 6 000 / 6 000 | 100 ms | 114 ms | 157 ms | (every gap is 100 ms) |
| 25 Hz quiet, 10 min | 15 000 / 15 000 | 40 ms | 54 ms | 88 ms | 0 |
| 50 Hz quiet, 10 min | 30 000 / 30 000 | 20 ms | 31 ms | 63 ms | 0 |
| 25 Hz busy, 15 min | 22 500 / 22 500 | 40 ms | 72 ms | 155 ms | 0 over 200 ms |

"Busy" means a `ros2` command-line tool started on the laptop every 30 s, as a person working
next to the arm would. Not one heartbeat was lost: the link delays, it does not drop.
`/joint_states` on the Jetson never had a gap over 63 ms, so no gap was the Jetson stalling.

One more gap counts: on 2026-11-12, with the real mission node at 25 Hz and the supervisor at
200 ms, the supervisor halted after 7 minutes on a gap of **at least 204 ms** (it halts as soon
as the limit passes, so the full length of that gap is unknown).

**Decision.** The rule on the campaign runs gives 1.5 × 157 = 236, so 250 ms, at 25 Hz (25 and
50 Hz tie; 10 Hz needs 400 ms for its 4 periods). 250 ms would cover the 204 ms gap by only
1.2 times, so the team chose **300 ms**, about 1.5 times it, and kept 25 Hz (4 periods are
160 ms). This is a judgement on top of the rule, made because the 204 ms gap came from the
real mission node. Test 1 of `docs/03_verification/safety_supervisor/` (5 minutes on the arm,
no false halt) checks it again.

**Confirmation.** 15 minutes at 25 Hz with the supervisor active at 300 ms and the laptop busy:
zero halts, 22 500 / 22 500 heartbeats.

**Discarded runs.** In the first busy phase and in the confirmation run, the script started
each `ros2` tool from inside the publishing process, which stalled the publisher: longest gaps
170 ms and 268 ms. Rerun with the tools in a shell process of their own: 155 ms (the busy row
above). The confirmation still counts as one (the supervisor did not halt even with those
stalls), but its gaps are left out of the rule.

**Detection time.** Timeout plus up to 20 ms (the supervisor's check period): 320 ms. Then the
hardware-inactive request is answered within 1 to 2 ms, and the whole halt takes 42 to 67 ms
(fake backend, `docs/03_verification/safety_supervisor/`). 320 ms is 20 ms over the #45
criterion "halt within 300 ms": the criterion is to be rewritten with these numbers.

**Conditions the 300 ms depends on.**

- The mission node publishes at **25 Hz**. It still publishes at 10 Hz in `positronic_mission`
  (3 periods in 300 ms instead of 4): changing it is a follow-up in that package.
- The Jetson's **Wi-Fi power saving is off**. With it on, gaps of 0.2 to 0.43 s came in every
  10 s window and the supervisor halted by itself within seconds (preliminary measurements
  below). It comes back on after every reboot; making it permanent is a follow-up.

## How to run it again

Two scripts run the whole campaign by themselves, about 45 minutes:

- `tools/heartbeat_source.py`, in the laptop dev container, **stands in for the mission node**:
  it publishes the heartbeat (same message, same QoS) and changes the rate on its own. It
  refuses to start while another node publishes the heartbeat.
- `tools/record_gaps.py`, in the Jetson container, records every heartbeat and every
  `/joint_states` message, follows the phases by itself, and at the end prints the tables and
  the recommended rate and timeout (the rule above, in `tools/analyze_gaps.py`).

| Phase | Rate | Laptop during the phase | Minutes |
|---|---|---|---|
| `10hz_quiet` | 10 Hz | nothing | 10 |
| `25hz_quiet` | 25 Hz | nothing | 10 |
| `25hz_busy` | 25 Hz | a `ros2` tool every 30 s, in a shell of its own | 10 |
| `50hz_quiet` | 50 Hz | nothing | 10 |

Each heartbeat carries its phase name, so the two machines need no shared clock.

**Before**, from a laptop host terminal (outside any container): stop every mission node on the
ROS domain, and check the Jetson's Wi-Fi power saving.

```bash
ssh nano 'iwconfig wlan0 | grep -i "power management"'    # must say off
ssh -t nano 'sudo iwconfig wlan0 power off'               # if it says on
make nano-build
make nano-shell
```

**The campaign.** In the Jetson container that `make nano-shell` opened:

```bash
source install/setup.bash
ros2 launch positronic_bringup supervised.launch.py backend:=fake > /tmp/supervised.log 2>&1 &
cd /ws/docs/03_verification/heartbeat_link/tools
python3 record_gaps.py
```

When it says "Waiting for heartbeat_source.py", in a laptop dev container (`make shell`):

```bash
source install/setup.bash
python3 docs/03_verification/heartbeat_link/tools/heartbeat_source.py
```

Do not use `ros2` on the laptop during the quiet phases. The Jetson container prints the
report at the end.

**Confirmation**, with the rate R and timeout T recommended. In the Jetson container:

```bash
ros2 param set /safety_supervisor heartbeat_timeout_ms T
ros2 lifecycle set /safety_supervisor cleanup
ros2 lifecycle set /safety_supervisor configure
python3 record_gaps.py --out confirm.csv
```

In the laptop dev container:

```bash
python3 docs/03_verification/heartbeat_link/tools/heartbeat_source.py --confirm R --minutes 15
```

Once the source prints its first phase, in a second laptop dev container:

```bash
source install/setup.bash
ros2 lifecycle set /safety_supervisor activate
```

It passes when the report says "Supervisor halts recorded: 0" and
`ros2 lifecycle get /safety_supervisor` still says `active [3]`.

**After: copy the recordings to the laptop before any other `make nano-*` command.** Those
commands copy the laptop's repo over the Jetson's with `--delete`, which erases files that exist
only on the Jetson. From a laptop host terminal:

```bash
rsync -a "nano:positronic/$USER/docs/03_verification/heartbeat_link/tools/*.csv" \
  docs/03_verification/heartbeat_link/tools/
```

## Raw data

The recordings are a few megabytes each, so they are not in git (`.gitignore` in this folder)
and are attached to the pull request that added this folder (#45):

| File | Run |
|---|---|
| `heartbeat_2026-11-12_2333.csv` | the campaign; its busy phase is a discarded run |
| `heartbeat_2026-11-13_0053_busy.csv` | the busy rerun, tools in their own process |

The confirmation's recording was lost (erased on the Jetson by a `make nano-*` sync before it
was copied); its numbers above come from the report printed at the end of the run.

Columns: `phase,channel,t_ns`. `channel` is `heartbeat`, `joint_states` or `halt` (a supervisor
halt); `t_ns` is the Jetson's monotonic clock. `python3 tools/analyze_gaps.py FILE.csv` prints
the report again.

## Preliminary measurements (2026-11-12, `ros2 topic hz`)

Why this campaign exists. Not used in the decision: `ros2 topic hz` reports only the largest
gap of each 10 s window, and its first window includes its own start-up.

| Condition | Largest gap per 10 s window | Supervisor (200 ms) |
|---|---|---|
| 10 Hz, power saving on, `ros2` tools running | 0.20 to 0.42 s, in every window | halted 11 s after activation |
| 10 Hz, power saving on, quiet | 0.20 to 0.43 s, in every window | (not active) |
| 10 Hz, power saving off, quiet | 0.10 to 0.12 s; twice 0.20 s (one lost heartbeat) | (not active) |
| 25 Hz, power saving off | 0.12 to 0.15 s, once 0.34 s (likely the tool starting) | halted once after 7 min (the 204 ms gap) |

## Not done yet

- The same measurement with the commanding laptop on a cable, or on a native Ubuntu laptop
  instead of a VM: the stalls come from tools starting on the laptop, so a shorter limit may be
  possible.
- The same campaign with the real mission node, once it publishes at 25 Hz.
