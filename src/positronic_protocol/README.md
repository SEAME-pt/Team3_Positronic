# positronic_protocol

Encode and decode the serial frames of the Elephant Robotics myCobot 280 (Jetson Nano).

Pure functions: bytes in, bytes out. No serial port, no I/O, no allocation, no exceptions,
so it is safe to call from the ros2_control `read()`/`write()` loop and easy to test
without the arm. The hardware interface (#22) owns the port and the buffer; this library
only builds and finds frames.

## The protocol

Link: `/dev/ttyTHS1`, 1 000 000 baud.

```
FE FE LEN CMD [DATA...] FA        LEN = N + 2 for N data bytes; the frame has LEN + 3 bytes
```

- No checksum on the 280. A reply is recognised by `FE FE LEN CMD` plus `FA` at the right place.
- Angles: signed int16, big-endian, in 0.01 deg (`CentiDegrees`), range +-327.67 deg.
- Speed: one byte, 1 to 100 (percent of the firmware maximum, not a physical unit).

| Command     | ID     | Request                                     | Reply                                  |
|-------------|--------|---------------------------------------------|----------------------------------------|
| power_on    | `0x10` | `FE FE 02 10 FA`                            | `FE FE 03 10 01 FA`                    |
| power_off   | `0x11` | `FE FE 02 11 FA`                            | `FE FE 03 11 01 FA`                    |
| get_angles  | `0x20` | `FE FE 02 20 FA`                            | `FE FE 0E 20` + 6 x int16 + `FA` (17 B)|
| send_angles | `0x22` | `FE FE 0F 22` + 6 x int16 + speed + `FA`    | `FE FE 03 22 01 FA`                    |
| stop        | `0x29` | `FE FE 02 29 FA`                            | `FE FE 03 29 01 FA`                    |

The servo bus (Feetech style, `FF FF id len ...`) shares the UART. Up to ~150 bytes of it
arrive before a get_angles reply, and it keeps flowing while the arm moves. It contains
lone `FE` bytes, so the decoder searches byte by byte.

## API

```cpp
#include "positronic_protocol/decode.hpp"
#include "positronic_protocol/encode.hpp"
#include "positronic_protocol/units.hpp"

namespace pp = positronic_protocol;

pp::Frame request{pp::encode_get_angles()};             // write request.bytes[0..size)
std::optional<pp::Frame> move{pp::encode_send_angles(angles, 20)};  // nullopt if speed is bad

pp::Decoded<pp::JointAngles> reply{pp::decode_get_angles_reply(rx)};
pp::Decoded<std::uint8_t> ack{pp::decode_ack(rx, pp::CommandId::STOP)};

std::optional<pp::JointAngles> wire{pp::to_centidegrees(radians)};  // ROS side -> wire
pp::JointRadians state{pp::to_radians(reply.value)};                // wire -> ROS side
```

Degrees exist only inside the driver; ROS sees radians (REP 103).

### Decode results

| `status`         | Meaning                                         | `consumed`                         |
|------------------|-------------------------------------------------|------------------------------------|
| `FOUND`          | Reply found, `value` is valid                   | End of the reply                   |
| `NEED_MORE_DATA` | The buffer ends with the start of the reply     | Start of that partial reply        |
| `NOT_FOUND`      | Nothing usable                                  | Everything (`rx.size`)             |

`rejected` counts `FE FE` candidates that turned out wrong (other command, bad LEN, bad
footer, the echo of a request). It is for logs and debugging, not for control flow.

### How #22 uses `consumed`

The decoder never changes the buffer. The caller appends what the UART delivered, decodes,
and drops `consumed` bytes from the front. Timeouts and retries are also the caller's job:
the library has no clock.

```cpp
// Sketch for the hardware interface (#22). rx_buffer is a fixed 512-byte buffer.
write_all(port, request.bytes.data(), request.size);
while (!timed_out()) {
  rx_buffer.append(read_available(port));             // whatever arrived, any size
  const auto reply{pp::decode_get_angles_reply(rx_buffer.view())};
  rx_buffer.drop_front(reply.consumed);
  if (reply.status == pp::DecodeStatus::FOUND) {
    return reply.value;                                // done
  }
  // NEED_MORE_DATA or NOT_FOUND: keep reading until the timeout
}
// timeout: resend (pymycobot resends up to 3 times after 0.5 s) or report ERROR
```

## Tests

```bash
make build && make test     # gtests + the full ROS lint set
make tidy                   # clang-tidy, must stay clean
```

| File                        | What it checks                                                     |
|-----------------------------|--------------------------------------------------------------------|
| `test_types.cpp`            | Constants, command ids against the capture, the byte access helper |
| `test_encode.cpp`           | Every request byte for byte, big-endian, speed limits              |
| `test_units.cpp`            | Rounding, round trips, NaN/inf/out of range                        |
| `test_decode.cpp`           | Acks and angles, servo noise, echoes, bad footers, every cut point |
| `test_recorded_frames.cpp`  | The "judge": the library against the capture of 2026-10-16         |

The judge reads `test/data/frames_*.txt` (written by `scripts/capture_frames/`). Each frame
we sent must equal our encoder's output, and our decoder must find in the raw RX the value
pymycobot returned. It also replays the RX in the exact bursts the UART delivered.

Sanitizers (out-of-bounds reads, undefined behaviour):

```bash
colcon build --packages-select positronic_protocol --cmake-args -DPOSITRONIC_SANITIZE=ON
colcon test --packages-select positronic_protocol
```

Coverage (`make doctor` checks the mixin):

```bash
colcon build --packages-select positronic_protocol --mixin coverage-gcc
colcon test --packages-select positronic_protocol
gcovr -r src/positronic_protocol build/positronic_protocol --filter 'src/positronic_protocol/(src|include)/'
```

## What the capture of 2026-10-16 answered

| #  | Question                                          | Answer                                                     |
|----|---------------------------------------------------|------------------------------------------------------------|
| Q1 | Baud                                              | 1 000 000 (capture header)                                 |
| Q2 | send_angles ack: at once or at the end?           | At once, ~3 ms after the request                           |
| Q3 | Servo noise without a request?                    | No: the 10 s idle window is empty                          |
| Q4 | Large angles encode correctly?                    | Yes: 29.92 deg goes out as `0B B0`, 68.73 deg comes back as `1A D9` |
| Q6 | Does the firmware echo our TX?                    | No echo in any RX line                                     |
| Q7 | get_angles with the servos off?                   | No reply at all; pymycobot retries 3 times and returns -1  |
| Q8 | Firmware                                          | get_system_version 7.3, get_modify_version 4               |

A get_angles transaction took ~6.4 ms (mean of 20 reads, last byte after the request).

## Design notes

- **One place reads with a runtime index**: `detail::read_byte()` checks the bounds and
  carries the only `NOLINT`. It uses `NOLINTBEGIN/NOLINTEND` because ament_cpplint rejects
  the single-line `NOLINT(<clang-tidy check>)`.
- **The encoder has no error path**: frame sizes are known at compile time, so a
  `static_assert` proves they fit and `std::get<I>` checks every fixed index.
- **Rounding**: `std::lround`, so 0.29 deg is 29 (pymycobot truncates it to 28).
