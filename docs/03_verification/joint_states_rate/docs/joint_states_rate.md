# Evidence: /joint_states at the measured rate from the Jetson container (#23)

> Template. Fill every `<...>` after the run described in
> `docs/03_verification/joint_states_rate/tools/README.md`, then delete this note.

## Verdict

| Criterion (#23) | Result | Measured |
|---|---|---|
| Rate equal to the value fixed by #20 (<#20 rate> Hz) | <PASS/FAIL> | mean <...> Hz |
| 10 min without a period above 2x nominal (<2x period> ms) | <PASS/FAIL> | max <...> ms, <...> periods above the limit |
| Joint names equal to the URDF | <PASS/FAIL> | <...> |
| Values in radians | <PASS/FAIL> | worst joint <...>: <...> deg published vs <...> deg from pymycobot |

## Setup

| | |
|---|---|
| Date | <YYYY-MM-DD HH:MM> |
| Operator | <name> |
| Arm | <A1/...> |
| Commit | <git rev-parse --short HEAD on the laptop that ran make nano-build> |
| Image | positronic-dev:nano, built <date> |
| Launch | `ros2 launch positronic_bringup driver.launch.py update_rate:=<rate>` |
| Nominal rate | <rate> Hz, from <link to #20 evidence> |
| Duration | <...> s recorded |
| Reference (pymycobot get_angles, deg) | <[...]> |

Safety: arm powered, no motion commands, bench clear, nothing else on `/dev/ttyTHS1`.

## Checker report

Output of `check_joint_states.py ... --report report.md`:

<paste report.md>

Full console output, including the `info header` and `warn start` lines:

```text
<paste>
```

## ros2 topic hz

Last lines of `topic_hz.log`:

```text
<paste>
```

## Run information

`run_info.txt` (update rate, hardware component state, first message):

```text
<paste>
```

## Observations

<Anything unusual during the run: stalls and when they happened, warnings in the launch output,
leading all-zero messages, CPU load. Link the bag location if it is kept somewhere.>
