# 0008. send_angles speed is a parameter, default 30

2026-11-26, proposed, Felipe Suassuna (issue #133).

## Context

Every `send_angles` carries a speed byte, 1 to 100. The driver fixed it at 30 since the first
motion, and #33 measured the tracking only there. #133 asked what the arm does at 50 and 70.

Seen on the wire (#32 tap), the byte is a **speed cap**: 1.5 deg/s per unit, never below
26.4 deg/s (45 deg/s at 30, 105 at 70). The servos' acceleration register stays 50 at every byte.
The #33 trajectories peak at 25-28 deg/s at the joint, so the cap never binds on them.

Measured on the arm, 2026-11-24 to 26, Jetson `nano`, firmware 7.3, three tips (camera +
gripper, gripper, bare):

| | 30 | 50 | 70 |
|---|---|---|---|
| Stop travel at ~10-15 deg/s, J1, 5 runs | 1.3-2.3° | 1.9-2.7° | 1.9-2.3° |
| Base T3/T6/T4 worst error (all 3 tips) | 2.4-4.4°, all pass | same within 0.2° | same within 0.2° |
| Wrist T11/T12/T13, camera + gripper | 3.4-4.1°, 9/9 pass | 3 of 9 and 1 of 9 abort (J4) | 2 of 9 abort (J4) |
| T8/T10 (#33's limit runs) | 6/6 pass, 4.2-4.5° | 4/6 | 6/6 |
| `write()` to first motion, 10 steps | 195 ms | 200 ms | 197 ms |
| Segment start to 90 % speed (J1) | 200-267 ms at every byte from 5 to 90 | | |

- The wrist aborts are J4 just over the JTC state tolerance, 0.089-0.095 rad against 0.087.
  One run at 50 started cold (J4 33 °C) and still aborted, so it is the byte, not heat.
- Heat follows the pose, not the byte: in a T6 loop J4 rose ~1.7 °C/min at 30 and at 50, from a
  tilted tip.
- Stops at 50 and 70 log a J1 state tolerance error ~300 ms after the cancel (none at 30). No
  goal aborts from it.
- Fresh mode (#20 saw streaming collapse in mode 0): the Atom has no switch of its own, so
  whether mode 1 survives its power cut couldn't be tested.

## Decision

- `send_speed` is a hardware parameter in the `<ros2_control>` block, 1 to 100, **default 30**,
  read in `on_init` and fixed for the life of the driver.
- The driver sends fresh mode 1 before every `power_on`.

## Alternatives

- **No parameter, 30 hard-coded:** nothing today needs another byte. But a faster move later
  (above 45 deg/s) would need a driver change instead of a URDF line, and #44/#53 want the speed
  used in a run on record.
- **Default 50 or 70:** nothing we measured got better, and the wrist started aborting on J4.
- **Fresh mode only when it reads 0:** one more request and reply before every activation, to
  save one frame.

## Consequences

- Stop distance, latency and ramp don't depend on the byte, so a higher byte adds no stop risk.
- Above 30, fast wrist moves can abort on J4. Raising the byte means re-running the wrist
  trajectories first.
- The working limits stay #33's: ~15 deg/s on the base, ~17.5 deg/s on the wrist at 30
  (#41 has no reason to change `joint_limits.yaml` for the byte).
- Every activation sends one more frame (`FE FE 03 16 01 FA`) and logs how many were acked.

## Revisit when

- A task needs a joint faster than 45 deg/s: then the byte must go up, and the wrist runs above
  decide by how much.
- The JTC tolerances change (#41), since the J4 aborts sit 0.1-0.5° past today's 0.087 rad.
