# Architecture v1

**Status:** small informational draft that grows with the project. Not accepted until one reviewer from each block has signed off (see [Review](#review)).

## 1. Terminology: real-time vs low-latency

Use these terms exactly, in docs, code comments and reviews.

| Term | Meaning in this project |
|---|---|
| **Real-time** | Correctness depends on meeting a deadline, and the worst case is bounded and guaranteed. Hard: a miss is a failure. Firm: a late result is worthless. Soft: a late result is degraded. |
| **Low-latency** | Delay is small in the typical case. No upper bound is guaranteed. Reported as percentiles (p50, p95, p99, max). |
| **Period / rate** | How often a loop is scheduled (33 ms = 30 Hz). |
| **Deadline** | When a result is needed. Missing it is counted, not hidden. |
| **Stall** | When a response from the arm takes more than 0.45s. A success never took more than 0.1s while testing. |
| **Jitter** | Variation of actual start time against the schedule. Reported as lateness (actual start minus intended start), p99. |
| **Staleness (age)** | Time since a value (command or state) was produced. |

**This system is low-latency with soft deadlines. It is not real-time.** The host runs a general-purpose Linux kernel, and the serial calls were measured blocking for up to 1.5 s (see section 6). Nothing in this stack may be described as "real-time" or "hard real-time". Say "30 Hz command stream", "p99 write call <= 11 ms" or "soft deadline" instead. If the project later adopts an RT-patched kernel, revisit this section. That would not bound firmware or serial stalls.

## 2. Layers

```
+--------------------------------------------------------------+
| L4  Code quality, measurement             (Block C)          |
+--------------------------------------------------------------+
| L3  Planning / simulation                 (Block B)          |
+--------------------------------------------------------------+
| L2  Driver / I/O, ROS interface           (Block A)          |
+--------------------------------------------------------------+
| L1  Serial link + firmware                (/dev/ttyTHS1)     |
+--------------------------------------------------------------+
| L0  Arm hardware                          (myCobot 280)      |
+--------------------------------------------------------------+
```

Rules:
- Block C acts as a quality control for all blocks. It measures the arm (#20) but ships no code that talks to it.
- Each layer talks only to the layer directly below it, through ROS interfaces.
- Silence does not stop the arm (measured): if commands stop mid-motion, the arm finishes the move to the last commanded target.
- Only L2 touches the serial port.
- L2 converts firmware units (degrees) to ROS units (radians) at its boundary.

## 3. Nodes

**TODO**: Nodes are not fully implemented yet, this part will be filled once the node architecture is better set up.

## 4. Node graph

**TODO**: Nodes are not fully implemented yet, this part will be filled once the node architecture is better set up.

## 5. QoS

**TODO**: Nodes are not fully implemented yet, this part will be filled once the node architecture is better set up.

QoS will filled for the topics once we have working nodes. Can be checked using `ros2 topic info --verbose <topic>`.

| Topic | From to | Reliability | Durability | History | Deadline |
|---|---|---|---|---|---|

## 6. Draft timing budget

All numbers are **low-latency, soft-deadline** targets. Evidence is in `docs/03_verification/serial_measurement/docs/serial_link.md`.

### Per loop

| Loop | Owner | Rate / period | Budget | Evidence |
|---|---|---|---|---|
| Trajectory sampling (JointTrajectoryController in controller_manager) | A | update_rate of the controller_manager | undefined | MoveIt (B) sends the whole trajectory once. The controller samples it; the serial write in write() is the 30 Hz stream below |
| Driver write tick | A | 30 Hz / 33 ms | write call <= 11 ms at p99; lateness p99 < 0.6 ms | measured, write-only (fresh mode 1, 5 runs): write p50 about 9 ms, p99 10.6 to 10.8 ms, max 19 to 24 ms, lateness p99 < 0.6 ms, 0 stalls in 18,005 writes. With reads in the stream the write rate was lower (see port budget) |
| Driver read tick | A | 2 Hz / 500 ms (on read + write) | 20 to 70 ms when not stalled | measured, interleaved with 30 Hz writes: 48 of 137 reads stalled (35%), the rest took about 20 to 70 ms. Read alone: p50 9.3 ms, p99 16 ms, no stalls |
| Link health | A | every serial call | A call over 450 ms is a stall. It is seen only when the call returns, 0.5 to 1.5 s after it started | measured: no call took 100 to 450 ms (4 interleaved runs, and the write runs in serial_link.md). Stalls took 0.52 to 1.08 s in the interleaved runs, up to 1.5 s in write-only runs |
| Stop request | A | on demand | needs to have priority or a thread | not measured. Worst case is the remaining time of a stalled call (up to 1.5 s) plus the `stop()` call itself |

### Serial port budget (the constraint that matters)

**(Proposal)**
Reads and writes share one port, owned by one I/O thread that issues every call in order (two threads on the port are not allowed: calls could take each other's reply), a scheduler to handle that could also be possible. A fixed-schedule scheduler sends a write every 33 ms and places a read after the write on chosen ticks.

Measured with that scheduler (30 Hz writes, fresh mode 1, 120 s, one run each, read issued right after the write, of different Hz intervals each run):

| Read every N writes | Read rate | Reads stalled | Achieved write rate |
|---|---|---|---|
| 1 | 30 Hz | 51% (144 of 281) | 2.3 Hz |
| 2 | 15 Hz | 37% (105 of 287) | 4.8 Hz |
| 3 | 10 Hz | 50% (120 of 240) | 6.0 Hz |
| 6 | 5 Hz | 69% (135 of 195) | 9.7 Hz |
| 15 | 2 Hz | 35% (48 of 137) | 16.9 Hz |

None of the runs was clean (target: 0 stalls and 30 Hz writes). Every read either finished in about 20 to 70 ms or took about 0.55 s (a stall), and the share that stalled did not follow the read rate. Each stalled read blocks the port for about 0.55 s, which skips about 16 write ticks, so the achieved write rate falls even at 2 Hz. A stalled read was never followed by a stalled write. The scheduler itself kept time (lateness p99 below 0.6 ms, write p50 6 to 9 ms). The write-only result (30 Hz, no stalls) still holds.

**(Proposal)**

**Draft decision: 2 Hz reads, not confirmed.** It is the lowest rate tested and the placeholder only. It does not meet the bar, and with this scheduler it would cost about 13 Hz of write rate. Confirming a read rate needs a different schedule, because the stalls appear when a read follows a write closely. Candidates, none tested yet: (a) give the read its own tick instead of the write, so it comes a full period after the previous write; (b) insert a fixed delay between the write and the read; (c) pause writes for a window around the read. Acceptance for any candidate: 0 stalls and 30 Hz writes over at least 5 runs of 120 s. If none passes, the fallback is to read the state only between moves and to treat the commanded setpoint as the state while streaming.

### End-to-end (excluding stalls)

| Path | Worst case, no stall | Comment |
|---|---|---|
| Setpoint published to bytes written | about 45 ms (derived) | up to 33 ms waiting for the next tick, up to 11 ms write. Reads can delay the write by about 0.55 s when one stalls, see below |

### Stalls (not covered by the budgets above)

Calls were measured blocking for 0.5, 1.0 and 1.5 s (probably a 0.5 s reply timeout hit once, twice or three times). What was measured, in fresh mode 1:

- 30 Hz writes: no stalls in 5 runs of 120 s (18,005 writes).
- 50 Hz writes: 4 to 27 stalls per 120 s run, in bursts that recur every 5 to 7 writes. Usable, but not the default.
- About 40 Hz writes: 2.8 to 8.5% of writes stalled in 5 of 5 runs.
- A read right after a write: stalled in 54% of tight write-then-read pairs, and in 35 to 69% of the reads when paced into a 30 Hz stream (every rate from 2 to 30 Hz, one run each). Each stalled read is a single 0.5 s unit and costs about 16 write ticks.
- The first read after the command stream stops: stalls about 0.5 s in 9 of 10 trials at 30 Hz, so the driver cannot see the arm for about 0.5 s after the last write.

## Review

Acceptance requires one reviewer from each block.

| Block | Reviewer | Date | Status | Comments |
|---|---|---|---|---|
| A, Motion and safety | | | pending | |
| B, Planning and simulation | | | pending | |
| C, Quality and measurement | | | pending | |
