# Test review (#46)

What the unit tests of each package really check, what they miss, and what we decided to do
about each gap. Coverage says which lines ran; this review says whether the behaviour was
checked.

## How a package is reviewed

1. One row per behaviour: is the normal case tested, the failure path (bad input, timeout, lost
   link), the edges (exactly at a limit, just past it, NaN, ±inf, empty, oversized)?
2. Look at the tests themselves: does each one assert something that matters?
3. Each finding gets a decision:
   - **fix now**: a safety path or a failure path with no test (an `issue` in `CLAUDE.md`
     terms). Done in #46.
   - **later**: useful, not blocking. Listed in a follow-up issue.
   - **new issue**: not a missing test but missing behaviour (a feature).

## Coverage at the start (2026-11-17, `make coverage`)

| Package | Coverage | Gate |
|---|---:|:---:|
| positronic_protocol | 100 % | PASS |
| positronic_hardware | 92.2 % | PASS |
| positronic_safety | 86.0 % | PASS |
| positronic_bringup | 82.4 % | PASS |
| positronic_mission | 75.3 % | FAIL |
| positronic_moveit_config | 56.2 % | FAIL |

`positronic`, `positronic_interfaces`, `positronic_description` and `positronic_hil` have no
code that coverage counts.

## Packages

- [positronic_protocol](positronic_protocol.md)
- [positronic_hardware](positronic_hardware.md)
- [positronic_safety](positronic_safety.md)

## Carried to other packages

Questions a review raised that belong to another package's review.

| From | To | Question | Answer |
|---|---|---|---|
| protocol F1 | hardware | Does `read()` reject a reading that is impossible (outside the joint range, or too far from the last one)? | No: hardware H2, new issue |
| protocol F2 | hardware | `decode_ack` is never called outside the protocol tests: the acks are not read. Is a refused command noticed? | No: hardware H3, new issue |

## Status

| Package | Review | Fix now | Later | New issue |
|---|---|---|---|---|
| positronic_protocol | Done 2026-11-17 | – | F1 test, F2 README note, F3 (with the acks issue), F4 cleanup | – |
| positronic_hardware | Done 2026-11-17 | H1 shutdown test, H4 `power_on` failure test | H5 (top), H1 crash note, H4 `write()` while inactive | H2 reading plausibility, H3 read the acks |
| positronic_safety | In progress 2026-11-18 | | | |
| positronic_mission | | | | |
| positronic_moveit_config | | | | |
| positronic_bringup | | | | |
