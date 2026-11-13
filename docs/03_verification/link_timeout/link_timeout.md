# Link timeout on the arm (#35)

Raw output of `hil_link_timeout`, the hardware check of the driver's 200 ms serial link timeout.
How it works, how to run it safely and what the numbers mean are in the driver README,
`src/positronic_hardware/README.md`, section "Link timeout (#35)".

| File | Run |
|---|---|
| `2026-11-10_hil_link_timeout.csv` | 6 trials, J1 ±5° at 30 % speed, `VTIME = 0`. Last bytes to timeout 194.0–201.3 ms; the arm finished its last target every time |

Columns: trial, ms from the last bytes delivered to the timeout, J1 target, J1 at the cut, J1 at
the end (degrees).
