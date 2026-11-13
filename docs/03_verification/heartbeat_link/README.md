# Heartbeat link campaign (#45)

How long the mission heartbeat can go missing between the commanding laptop and the Jetson, to
choose the heartbeat rate and the supervisor's `heartbeat_timeout_ms` from data.

**Result: heartbeat at 25 Hz, `heartbeat_timeout_ms` 300.** A dead mission node is detected
within 320 ms (timeout + 20 ms check period).

## Rule (fixed before the runs)

Timeout ≥ 1.5 × the longest link gap of any phase, rounded up to 50 ms, and ≥ 4 heartbeat
periods. Among the rates tested, the smallest timeout wins; on a tie, the lower rate.

## Results (2026-11-12 and 13, lab Wi-Fi, Jetson Wi-Fi power saving off)

| Run | Heartbeats sent / received | Median gap | p99.9 | Longest gap |
|---|---|---|---|---|
| 10 Hz quiet, 10 min | 6 000 / 6 000 | 100 ms | 114 ms | 157 ms |
| 25 Hz quiet, 10 min | 15 000 / 15 000 | 40 ms | 54 ms | 88 ms |
| 50 Hz quiet, 10 min | 30 000 / 30 000 | 20 ms | 31 ms | 63 ms |
| 25 Hz, `ros2` tools started every 30 s, 15 min | 22 500 / 22 500 | 40 ms | 72 ms | 155 ms |

No heartbeat was lost: the link delays, it does not drop. Separately, with the real mission
node, the supervisor (then at 200 ms) halted once on a gap of **at least 204 ms**.

**Decision.** The rule on these runs gives 250 ms at 25 Hz. 250 ms covers the 204 ms gap by
only 1.2 times, so the team chose **300 ms** (about 1.5 times) and kept 25 Hz. A 15-minute run
with the supervisor active at 300 ms had zero halts.

**It depends on:** the mission node at **25 Hz** (still 10 Hz in `positronic_mission`; on the
arm, 10 Hz gave a false halt and 25 Hz none, see `../safety_supervisor/`), and the Jetson's
**Wi-Fi power saving off** (`sudo iwconfig wlan0 power off`, lost at reboot; with it on, gaps
of 0.2-0.4 s came every few seconds).

The raw recordings are attached to the pull request; the scripts that made them are on branch
`45-verification-tools`.