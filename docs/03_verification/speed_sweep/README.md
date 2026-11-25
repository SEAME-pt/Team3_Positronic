# Firmware speed of `send_angles` on the real arm (#133)

Every `send_angles` carries a speed byte (1 to 100). It was fixed at 30 since the first motion.
This sweep makes it a driver parameter, `send_speed` (static: read in `on_init`, fixed for the
life of the driver, default 30), and measures what the arm does at each value, in three tip
setups. Arm runs 2026-11-24 and 2026-11-25, Jetson `nano`, firmware 7.3, fresh mode 1.

## What the byte does

Seen on the wire with the #32 tap (`tools/tap_speed.py`), every byte from 5 to 100:

- **Goal speed of a moving servo = 17 × byte steps/s, floor 300.** 17 steps/s is 1.5 deg/s, so the
  byte is a **speed cap**: 26.4 deg/s for any byte ≤ 17, then 1.5 deg/s per unit (45 at 30, 75 at
  50, 105 at 70, 150 at 100). Joints that don't move get 300 at every byte. Acceleration register
  50 on every servo, at every byte.
- The #33 trajectories peak at 25-28 deg/s at the joint, under or at the cap even at byte 5, so
  the byte can't change them. Where it shows: how fast a jump runs (a big step between two
  targets), not a trajectory the controller interpolates.
- A stop freezes the target. What moves after it is the servo's own lag, the same at every byte.

## Results

Base trajectories (J1 + J2), worst |error| over all joints per run, deg, mean (min-max) of 3
runs (`waypoint_error.py`, "max abs" row); all goals SUCCEEDED, no JTC abort. Stops: J1 at
12-17 deg/s, stopped about 1 s into the move (#36 method), travel after the stop frame and time
to still, from the tap, 5 per mode. `-`: not run at that byte (cut plan below).

| Setup | Byte | T3 10 deg/s | T6 15 deg/s | T4 20 deg/s | Stop travel, deg | Stop to still, ms |
|---|---|---|---|---|---|---|
| 1, camera + gripper | 30 | 2.52 (2.46-2.56) | 3.41 (3.26-3.68) | 4.03 (3.98-4.10) | 1.74 (1.32-2.02) | 147 (126-163) |
| | 50 | 2.58 (2.55-2.63) | 3.31 (3.17-3.55) | 4.11 (4.05-4.18) | 1.95 (1.85-2.03) | 164 (152-185) |
| | 70 | - | - | - | 1.85 (1.41-2.03) | 165 (133-195) |
| | 90 | - | - | - | 1.99 (1.41-2.54) | 168 (145-192) |
| | 100 | 2.51 (2.48-2.53) | 3.27 (3.23-3.29) | 4.08 (4.00-4.17) | 1.85 (1.32-2.11) | 156 (124-184) |
| 2, gripper | 5 | 2.60 (2.48-2.75) | 3.40 (3.29-3.50) | 4.07 (4.01-4.19) | 1.99 (1.93-2.02) | 157 (143-185) |
| | 30 | 2.53 (2.52-2.55) | 3.24 (3.20-3.29) | 4.10 (3.97-4.35) | 1.95 (1.41-2.28) | 173 (139-209) |
| | 50 | - | - | - | 1.97 (1.85-2.10) | 161 (144-189) |
| | 70 | 2.63 (2.54-2.69) | 3.25 (3.22-3.29) | 4.22 (3.96-4.70) | 2.02 (1.85-2.20) | 154 (143-178) |
| 3, bare | 30 | 2.44 (2.39-2.47) | 3.26 (3.19-3.29) | 4.09 (3.93-4.18) | 2.05 (1.93-2.37) | 162 (137-196) |
| | 50 | 2.50 (2.49-2.50) | 3.37 (3.20-3.68) | 4.15 (4.08-4.27) | 1.90 (1.41-2.28) | 165 (137-193) |
| | 70 | 2.48 (2.39-2.59) | 3.24 (3.21-3.29) | 4.34 (3.93-4.56) | - | - |

Setup 1's 30 and 50 come from the same runs with the `adaptive` branch in fixed mode (same bytes on
the wire, same driver base).

- **Byte:** no effect from 5 to 100. Differences between bytes are 0.1-0.2°, inside one mode's
  run-to-run spread. The stop travel stays 1.3-2.5° at every byte, still after 125-210 ms.
- **Load:** no effect on these trajectories either. Bare, gripper and camera + gripper are within
  0.1-0.2° of each other. The tip shows as a constant J4 offset (~0.9° bare, ~2.3° with the
  camera), not as tracking error on J1 and J2.
- **Wrist** (T11, T12, T13: J3-J6 at 10/15/17.5 deg/s), setup 2, byte 30: 3.5 (3.2-3.8), 3.9
  (3.7-4.2), 4.0 (3.9-4.0), all SUCCEEDED.
- **Latency** (#39 steps, J1 5° from rest), setup 1: `write()` to first motion 195 ms (167-201)
  at 30, 200 ms (200-201) at 50. A byte ≤ 17 starts a step slower (+17 ms to first motion, +33 ms
  to 50 % of the step), because the cap is the 26.4 deg/s floor.
- **Fresh mode:** the driver sends fresh mode 1 before every `power_on`, and the Atom acknowledges
  it (`1 sent, 1 acknowledged` at every launch of both days; read back 1 after the driver alone).
  Whether mode 1 survives a power cut of the Atom was not tested: the Atom has no switch of its
  own, so the driver setting it every time is the answer.
- **Temperature:** no servo trend with the byte. What heats the servos is the pose, not the speed:
  with the tip held tilted J4 went 42 → 60 °C in 11 min (camera + gripper, the overheat cut
  fired), and J5 44 → 63 °C in 50 min standing with a target 1.2° off where it settles. Between
  goals the run waits at a rest pose where the tip balances (J4 39.8° with camera + gripper, 35.0°
  with the gripper, ~0° bare; J5 locked where it settles). With it, the hottest servo in a mode
  stayed at 45-53 °C with a tip and 45-50 °C bare (J1, which sat at 41-46 °C all day).

## Recommendation

- **`send_speed` default stays 30.** Nothing we measured gets better above it, and a higher byte
  only raises the cap on how fast a jump between two targets runs (45 deg/s at 30, 105 at 70). Below
  18 every byte is the same, 26.4 deg/s.
- **`joint_limits.yaml` (#41): the byte gives no reason to change the limits.** The working limit
  comes from the controller's tracking, not the byte: on the base ~4° worst error at 20 deg/s (T4)
  with any tip, so #33's 15 deg/s stays; the wrist only at 30 (above), ~4° at 17.5 deg/s.
- **One rest pose, for camera + gripper (the setup we keep): J4 39.8°, the other joints 0, J5
  locked where it settles.** There the tip balances, so J4 holds no load; with J4 at 0 or the tip
  tilted, J4 and J5 heat until the 60 °C cut powers the arm off and it drops. J5 is released and
  locked again after the arm reaches it, because a target even 1° off where J5 settles keeps it
  pushing (44 → 63 °C in 50 min). The values for the other tips are under Temperature.

## Not measured

Cut on 11-24 (Felipe and the Manager) to what changes a decision, from ~10 h of arm time to ~4 h:
T8 and T10 (they abort at 30 in #33), the wrist above 30, the ramp time from the bags, the
10-minute trajectory loop for temperature, latency above 50 (the steps at setup 2 byte 70 left no
`steps.csv`), and the error-against-speed figure. The byte sits above every speed these
trajectories ask for, so none of these is expected to change the answer.

## How it was run

`tools/arm_run_static.sh` in the `positronic-arm` container on the Jetson, one stage per call,
one person at the switch the whole time. Per byte: `up` (fresh mode, tap, driver with
`send_speed:=<byte>`, bag), `start` (rest pose), `first` (5° on J1, the first move of every
byte), `stops`, `base`, `wrist`, `steps`, `down`. Bytes only step up, and above 30 `up` refuses
unless the byte before it has 5 stops in the same setup, each ≤ 4° (`tools/step_up_check.py`);
above 70 the byte is typed twice. `up` refuses a servo above 45 °C, and `cool` waits for it. Every
mode keeps `summary.txt`, `tap.csv`, the bag and the goal outputs; the raw data stay outside the
repo (about 1 GB of taps and bags).
