<a id="top"></a>
<p align="right">
  <img src="../../assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# 00 · Specifications — Hardware — 08 Command reference

<!-- nav:auto -->
<p align="left">
  <a href="../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="07_serial_protocol.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="09_buttons_and_e_stop.md"><b>Forward &#8594;</b></a>
</p>



Every command in the official serial protocol (62 commands): command byte, message length, bytes sent
and bytes returned.

How the protocol works is in [07_serial_protocol.md](07_serial_protocol.md).

> **Firmware requirement.** Direct serial control needs the latest **Atom firmware** (`atomMain`) on
> the wrist board. The base firmware handles the six motors. On this unit `get_system_version` reports
> **73** and `get_basic_version` returns **−1** (pymycobot 4.0.4b5, class `MyCobot280`). |

---

## 1. Frame format (reminder)

Every message is: `FE FE · length · command · data… · FA`

| Part | Value | Meaning |
| :--- | :--- | :--- |
| Header | `FE FE` | Always these two. |
| Length | `02` – `10` | Counts the command + data + end bytes (not the two `FE`s). |
| Command | `00` – `8F` | The instruction (the tables below). |
| Data | 0–16 bytes | Depends on the command. |
| End | `FA` | Always this. |

A reply returns within **500 ms** and repeats the command byte. Commands marked **—** have **no return
value**.

---

## 2. How numbers are packed

A byte holds 0–255, so larger values are split into **high byte then low byte**. To rebuild:
`value = low + high × 256`.

| Quantity | Sent as (before splitting) | Rebuild then divide by |
| :--- | :--- | :--- |
| Joint angle | degrees × **100** | 100 |
| x / y / z | millimetres × **10** | 10 |
| rx / ry / rz | degrees × **100** | 100 |
| Encoder / potential | raw 0–4096 | 1 |
| Speed / gripper opening | plain 0–100 | 1 |

**Negative numbers.** If the rebuilt value is greater than **33000**, subtract **65536** first:

> `decoded = (raw > 33000 ? raw − 65536 : raw) / scale`

---

## 3. Master command list

`Len` is the length byte in hex. **S** = bytes sent (after the command byte); **R** = bytes returned.

| Opcode | Command | Len | S — send | R — return |
| :---: | :--- | :---: | :--- | :--- |
| **0x10** | Robot power on | `02` | — | — |
| **0x11** | Robot power off & disconnect | `02` | — | — |
| **0x12** | Atom status query | `02` | — | `01` on / `00` off |
| **0x13** | Arm only powers off (Jetson stays on) | `02` | — | — |
| **0x14** | System detection / connection | `02` | — | `01` normal / `00` |
| **0x16** | Refresh-mode switch | `03` | `00` interpolation / `01` refresh | — |
| **0x1A** | Free mode (torque off) | `03` | `00` off / `01` on | — |
| **0x1B** | Is free mode on? | `02` | — | `01` / `00` |
| **0x20** | Read all joint angles | `02` | — | 6× (high, low) |
| **0x21** | Send one joint angle | `06` | joint no., angle H/L, speed | — |
| **0x22** | Send all six joint angles | `0F` | 6× (angle H/L), speed | — |
| **0x23** | Read all coordinates | `02` | — | x,y,z,rx,ry,rz (H,L) |
| **0x24** | Send one coordinate | `06` | axis (1–6), value H/L, speed | — |
| **0x25** | Send all coordinates | `10` | x,y,z,rx,ry,rz (H,L), speed, mode | — |
| **0x26** | Pause program | `02` | — | — |
| **0x27** | Is program paused? | `02` | — | `01` / `00` |
| **0x28** | Resume program | `02` | — | — |
| **0x29** | Stop program | `02` | — | — |
| **0x2A** | Has the point been reached? | `0E` / `0F` | coords **or** angles + `01`/`00` | `01` arrived / `00` not |
| **0x2B** | Is the arm moving? | `02` | — | `01` moving / `00` still |
| **0x30** | Jog — joint, by direction | `05` | joint, direction (0/1), speed | — |
| **0x31** | Jog — joint, to an absolute angle | `06` | joint, angle H/L, speed | — |
| **0x32** | Jog — coordinate, by direction | `05` | axis, direction (0/1), speed | — |
| **0x33** | Jog — coordinate, absolute (stepping) | `06` | axis, value H/L, speed | — |
| **0x3A** | Send one potential (encoder) value | `05` | joint, encoder H/L, speed | — |
| **0x3B** | Read one potential value | `03` | joint | encoder H/L |
| **0x3C** | Send six potential values | `0F` | 6× encoder H/L, speed | — |
| **0x3D** | Read six potential values | `02` | — | 6× encoder H/L |
| **0x41** | Set speed | `03` | speed 0–100 | — |
| **0x4A** | Read a joint's minimum angle | `03` | joint | joint, angle H/L |
| **0x4B** | Read a joint's maximum angle | `03` | joint | joint, angle H/L |
| **0x4C** | Set a joint's minimum angle | `05` | joint, angle H/L | — |
| **0x4D** | Set a joint's maximum angle | `05` | joint, angle H/L | — |
| **0x50** | Check a joint's connection | `03` | joint | joint, `01`/`00` |
| **0x51** | Are all servos powered? | `02` | — | `01` / `00` |
| **0x52** | Write a servo parameter | `05` | joint, data address, data | — |
| **0x53** | Read a servo parameter | `04` | joint, data address | data |
| **0x54** | Set a servo's zero point | `03` | joint | — |
| **0x55** | Brake one motor | `03` | joint | — |
| **0x56** | Power off one motor | `03` | joint | — |
| **0x57** | Power on one motor | `03` | joint | — |
| **0x60** | Set an Atom pin's mode | `04` | pin, `00` in / `01` out | — |
| **0x61** | Write an Atom pin (output) | `04` | pin, `00` low / `01` high | — |
| **0x62** | Read an Atom pin (input) | `03` | pin | pin, level |
| **0x65** | Read the gripper opening | `02` | — | value 0–100 |
| **0x66** | Set the gripper mode | `04` | `00` open / `01` close, speed | — |
| **0x67** | Set the gripper opening | `04` | value, speed | — |
| **0x68** | Set the gripper zero point | `02` | — | — |
| **0x69** | Is the gripper moving? | `02` | — | `01` moving / `00` still |
| **0x6A** | Set the wrist RGB LED | `05` | R, G, B | — |
| **0xA0** | Write a base IO pin | `04` | pin, level | — |
| **0xA1** | Read a base IO pin | `03` | pin | pin, level |
| **0xB1** | Get Wi-Fi account & password | `02` | — | WiFi info |
| **0xB2** | Set the port number | `04` | port H/L | — |
| **0x81** | Set the tool coordinate system | `0E` | x,y,z,rx,ry,rz (H,L) | — |
| **0x82** | Get the tool coordinate system | `02` | — | x,y,z,rx,ry,rz (H,L) |
| **0x83** | Set the world coordinate system | `0E` | x,y,z,rx,ry,rz (H,L) | — |
| **0x84** | Get the world coordinate system | `02` | — | x,y,z,rx,ry,rz (H,L) |
| **0x85** | Set base coordinate system | `03` | `00` base / `01` world | — |
| **0x86** | Get base coordinate system | `02` | — | `00` base / `01` world |
| **0x89** | Set end coordinate system | `03` | `00` flange / `01` tool | — |
| **0x8A** | Get end coordinate system | `02` | — | `00` flange / `01` tool |

---

## 4. Worked examples

Each example is a complete frame (all values in hex).

**Power on:** `FE FE 02 10 FA`

**Go to a joint angle** — joint 1 to 0° at 20 % speed (`06 21 01 00 00 14`):
`FE FE 06 21 01 00 00 14 FA`

**All joints to zero at 30 % speed:**
`FE FE 0F 22 00 00 00 00 00 00 00 00 00 00 00 00 1E FA`

**Read all angles (reply):** `FE FE 0E 20 00 8C 00 3D FF E6 FF 3F 00 AF FF 51 FA`

**Set one coordinate** — X to 200 at speed 20 (`06 24 01 07 D0 14`):
`FE FE 06 24 01 07 D0 14 FA`

**Send all coordinates** — (150.3, −68.7, 101.8, 10.18, 0, −90) at speed 10:
`FE FE 10 25 05 DF FD 51 03 FA 03 FA 00 00 DC D8 0A 01 FA`

**Gripper to 50 % at speed 20:** `FE FE 04 67 32 14 FA`

**Wrist LED red:** `FE FE 05 6A 00 00 FF FA`

**Set speed to 50 %:** `FE FE 03 41 32 FA`

**Enter free mode:** `FE FE 03 1A 01 FA`

---

## 5. Family notes

### 5.1 Axis numbers used by 0x24 / 0x32

| Value | Axis |
| :---: | :--- |
| 1 | x |
| 2 | y |
| 3 | z |
| 4 | rx |
| 5 | ry |
| 6 | rz |

### 5.2 Refresh vs interpolation mode (0x16)

- **Interpolation** (`00`): blended path.
- **Refresh** (`01`): the arm is re-commanded continuously.

### 5.3 Speed ranges

- `0x41` set speed: **0–100**.
- The speed byte inside `0x21/0x22/0x24/0x25` is also **0–100**.

### 5.4 Gripper

- `0x65` returns the opening as **0 (closed) – 100 (open)**.
- `0x66` is the quick open/close; `0x67` moves to a specific opening.
- `0x68` re-zeroes the gripper at its current position.

### 5.5 What "potential / encoder" means

The servos report their position as a raw encoder value (0–4096, with a midpoint). `0x3B/0x3D` read it;
`0x3A/0x3C` write it.

### 5.6 Coordinate systems (frames)

- **Tool** (`0x81/0x82`) — the tool relative to the flange.
- **World** (`0x83/0x84`) — the room reference.
- **Base** (`0x85/0x86`) — base vs world reference.
- **End** (`0x89/0x8A`) — flange vs tool reference.

### 5.7 Occupied pins

On the 280-JN, header pins **3, 8, 10, 27 and 28** are wired internally (I2C, the servo UART and
the module-ID EEPROM); pins **11** and **12** are occupied by internal pads. Use `0xA0`/`0xA1`
(base IO) only on free pins. See [06_pinouts.md](06_pinouts.md).


<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
