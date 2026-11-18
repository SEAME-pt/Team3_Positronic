# positronic_protocol: test review

Reviewed 2026-11-17. Coverage 100 % (131 of 131 lines). Tests: `src/positronic_protocol/test/`,
many of them checked against the recording `test/data/frames_20261016_124447.txt`.

**What the package does:** turns commands into bytes (encode), finds the arm's replies in the
received bytes (decode), and converts radians to and from the wire unit (0.01°). It never
opens the serial port.

## Behaviours

| Behaviour | Normal | Failure path | Edges | Verdict |
|---|:-:|:-:|:-:|---|
| Encode the 5 commands without data | ✅ | n/a, cannot fail | n/a | Good, also checked against the recording |
| Encode `send_angles` | ✅ | ✅ speed 0, 101, 255 refused | ✅ speed 1 and 100, int16 min/max, negative angles | Good |
| Radians → 0.01° | ✅ | ✅ NaN, +inf, −inf, 1e300 | ✅ ±327.67° accepted, ±327.68° refused, rounding | Very good |
| 0.01° → radians | ✅ | n/a | ✅ every wire value round-trips | Very good |
| Six joints at once | ✅ | ✅ one bad joint fails all six | ✅ first and last joint | Good |
| Decode the angles reply | ✅ | ✅ wrong footer, wrong length, request echo, null buffer | ✅ empty, split at every byte, chunks of every size, `FE FE` inside the data | Excellent |
| Decode an ack | ✅ | ⚠️ only "ack of another command" | ⚠️ never split across reads | Weaker than the angles path |
| Constants and types | | | | ⚠️ some tests only repeat the constants |

## Findings

### F1. No checksum: a fake frame with the right shape is accepted

**What:** the reply has no checksum. `find_frame` (`src/decode.cpp`) accepts any
`FE FE 0E 20 <12 bytes> FA`. Servo traffic shares the line, so noise that happens to have this
shape is returned as real joint angles. No test shows this; the tests only check that real
frames are found.

**Why it matters:** ros2_control would believe the arm is somewhere it is not. Example: the arm
is at 0°, a fake frame says 200°. The controller sees an error of −200° and commands a large
correction for an error that does not exist. The arm moves, or the safety supervisor halts it,
because of a lie in the data. Unlikely, not impossible.

**Lesson:** a test that passes on real data does not prove the code rejects bad data. Also ask
"what if the input lies?"

**Where to catch it:** not in the protocol (it cannot tell a real frame from a fake one with
the same shape), but in `positronic_hardware` `read()`, the last place before ROS sees the
value. Two plausibility checks:

1. **Range check:** a reading outside the joint's URDF limits (plus a margin) is impossible.
   Cheap, the hardware package already loads the limits. Only catches large lies.
2. **Jump check:** a joint cannot move faster than its URDF velocity limit, so
   `max_jump = max_velocity × time_between_reads × margin`. Catches small lies too.

**What to do with a rejected reading:** drop it and keep the last good value (one noisy frame
should not stop the arm); after N in a row (e.g. 3), report an error and halt.

**Implications and edge cases the feature must handle and test:**
- First reading after start: no previous value, only the range check applies.
- A real fast move must not be rejected: the margin has to be chosen and justified.
- Arm moved by hand with the servos released.
- NaN or ±inf in the time between reads.
- The jump check needs the real time between reads, so the poll thread's timing matters.
- Dropping a reading means ROS sees a value a little old: the "N in a row" limit bounds how old.

**Decision:**
- A test in the protocol that documents the limit (`FakeFrameWithValidShapeIsAccepted`):
  **later**.
- The plausibility checks in hardware: **new issue**, decided after the hardware review (check
  first what `read()` already does).

### F2. The package translates, it does not protect the arm

**What:** `encode_send_angles` accepts any angle up to ±327.67° and any speed from 1 to 100.
`decode_ack` returns the status byte (1 = OK) and does not decide what a 0 means. This is the
right design: the protocol translates, another package judges. So the protocol tests are right
not to test joint limits.

**Who protects instead (checked 2026-11-17):**

| Protection | Where | Status |
|---|---|---|
| Joint limits | `positronic_hardware`, `check_command` before each write (`src/positronic_hardware.cpp`) | Exists, reviewed with hardware |
| Speed ≤ 30 | `positronic_hardware`, `SEND_SPEED{30}` (`include/positronic_hardware/serial_arm.hpp`) | Fixed at 30 |
| Only caller of `encode_send_angles` | `positronic_hardware/src/serial_arm.cpp` | Yes, no other caller |
| Ack status read | `decode_ack` is never called outside the protocol tests | ⚠️ acks ignored, see the hardware review |

**Why it matters:** a node that called `encode_send_angles` directly would skip both the joint
limit check and the speed limit, and the safety supervisor, which watches the hardware path.
A test cannot stop someone from calling a function; documentation and review can.

**Lesson:** when a package does not check something, find who does and check it there.

**Decision:**
- No new test in the protocol.
- One sentence in `src/positronic_protocol/README.md`: "Only `positronic_hardware` may call
  `encode_send_angles`; it holds the limit and speed checks." **Later.**
- Ignored acks: carried to the hardware review.

### F3. The ack path has fewer edge tests than the angles path

**What:**

| Case | Angles | Ack |
|---|:-:|:-:|
| Clean frame | ✅ | ✅ |
| Noise before it | ✅ | ✅ |
| Wrong command, echo of our request | ✅ | ✅ |
| Bad footer | ✅ | ❌ |
| Split across two reads (`NEED_MORE_DATA`) | ✅ | ❌ |
| Empty buffer | ✅ | ❌ |

**Why the risk is low:** both functions use the same search (`find_frame`, `src/decode.cpp`),
only the expected frame differs (6 bytes for an ack, 17 for angles). And today nothing outside
the tests calls `decode_ack` (F2), so a bug there cannot reach the arm.

**Why it is still a gap:** "works because the code is shared" is a guess, not a test. A bug that
only shows with short frames would go unnoticed.

**When it starts to matter:** the day `positronic_hardware` reads the acks. A wrongly decoded
ack could then hide a refused command.

**Decision:** **later**, tied to the ack work. If the hardware review leads to a "read the
acks" issue, its acceptance criteria include these three tests (bad footer, split across reads,
empty buffer). Otherwise they go to the general "later" list.

### F4. Some tests prove nothing

**What:**

| Test | What it does | Problem |
|---|---|---|
| `Smoke.HeaderIsVisible` (`test_smoke.cpp`) | `HEADER == 0xFE` | Duplicate of `Types.FrameMarkers` |
| `Smoke.CaptureIsReadable` (`test_smoke.cpp`) | Opens the recording | Duplicate of `RecordedFrames.CaptureLoads` |
| `Types.FrameMarkers`, `CommandIdValues`, `SpeedRange` | Repeat each constant | A wrong value already fails the encode tests against the recording |
| `Types.LargestFrameFits` | Computes `4 + 12 + 1 + 1` and compares to 18 | Calls no code |
| `Types.WireUnitHoldsFullCircle` | An `int16` holds 18000 | Always true |

The smoke tests made sense when the package was empty (does the test setup work?); real tests
replaced them.

**Why it matters (a little):** no bugs, but the suite looks stronger than it is, and the tests
cost reading time.

**Example:** if `send_angles` grew to 40 bytes (more than `MAX_FRAME_SIZE`, 32),
`LargestFrameFits` would still pass, because it never looks at the real frame. What catches it
is the `static_assert(FRAME_SIZE <= MAX_FRAME_SIZE)` in `build_frame` (`src/encode.cpp`): the
build fails, so the mistake cannot reach a test run. `EncodeSendAngles.FrameShape` would also
fail, because it checks the real frame.

**Lesson:** to judge a test, ask "if the code broke, would this test fail?" If not, it protects
nothing.

**Keep:** `Types.CommandIdsAppearInCapture` (checks the ids against the real recording),
`Types.DefaultsAreEmpty`, the `ByteAccess` tests.

**Decision:** **later**, small cleanup: delete `test/test_smoke.cpp`, remove the five weak tests
from `test/test_types.cpp`. Coverage stays at 100 %.

## Summary

| Finding | Decision |
|---|---|
| F1. A fake frame with the right shape is accepted | Documenting test: later. Plausibility checks in hardware: new issue |
| F2. The package translates, it does not protect | README note: later. Ignored acks: hardware review |
| F3. Ack path has fewer edge tests | Later, tied to the ack work |
| F4. Some tests prove nothing | Later, cleanup |

**Fix now (#46): nothing.** The package is well tested; its main limit (no checksum) cannot be
fixed by a test, and the protection belongs in `positronic_hardware`.
