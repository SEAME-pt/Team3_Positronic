<a id="top"></a>
<p align="right">
  <img src="../../assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# 00 · Specifications — Hardware — 09 Buttons and E-Stop (REVIEW)

<!-- nav:auto -->
<p align="left">
  <a href="../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="08_command_reference.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="10_safety_and_handling.md"><b>Forward &#8594;</b></a>
</p>



## 1. Stop options

> **Status.** The cell does **not** have an E-stop installed yet. M1 designs and builds it (issues #27/#45/#52). The reference design combines a Category 0 power cut with a software layer (stop options below). |

| Option | Action | Connection | Type | Works if the Jetson is frozen |
| :--- | :--- | :--- | :--- | :---: |
| **1 · Power cut** | Removes motor power | In series on the positive power line | Category 0 | Yes |
| **2 · Serial TX cut** | Stops commands reaching the motors | On the internal serial TX wire | Category 0 | Yes |
| **3 · Jetson power/reset** | Powers off or reboots the computer | Carrier button header | Category 0 | Yes |
| **4 · GPIO button** | Software stop | Base 40-pin header (e.g. GPIO 26, NC to GND) | software-monitored | No |
| **5 · Wrist button** | Software stop | Atom button (GPIO 39, read over serial) | software-monitored | No |

A Category 0 stop removes power immediately. A software-monitored stop works only while the computer
is running.

---

## 2. Option 1 — power cut

A normally-closed mushroom E-stop in series with the positive power line, between the supply and the
DC jack.

- Closed: power flows to the arm.
- Pressed: the switch opens and the motors lose power.

This directly also cuts power to the Jetson unless the Jetson has its own supply. With a separate
Jetson supply, pressing the E-stop cuts the motors and leaves the Jetson running.

<p align="center">
  <img src="../../assets/hardware/base_internal_05_fan_power_cables.png" width="440" alt="Power, fan and indicator cables inside the base">
  <br><sub>Inside the base: the black/red main power wires, the fan and the power-indicator cable.</sub>
</p>

---

## 3. Option 2 — serial TX cut

A normally-closed switch in the internal serial TX wire.

- Closed: commands flow to the motors.
- Pressed: no new position commands reach the motors.

The motors hold their last position with torque until a watchdog trips or the power is cycled.

<p align="center">
  <img src="../../assets/hardware/base_internal_04_servo_cable.png" width="440" alt="Servo cable inside the base">
  <br><sub>The black/red/white servo cable inside the base carries the commands to all six motors.</sub>
</p>

---

## 4. Option 3 — Jetson power/reset

The carrier board has a button header:

- Short press: powers the board on.
- Long hold (about 5 seconds): powers it off.
- Short press on the reset pin: reboots it.

---

## 5. Option 4 — GPIO button

<p align="center">
  <img src="../../assets/hardware/base_interfaces.png" width="500" alt="Base interfaces, including the 40-pin group">
  <br><sub>The 40-pin header is interface ③ on the base.</sub>
</p>

A button on a free pin of the base 40-pin header, for example **GPIO 26** with **GND** as reference,
wired normally-closed to ground.

<p align="center">
  <img src="../../assets/hardware/gpio_circuit.png" width="440" alt="GPIO input wiring example">
  <br><sub>Header wiring for a button.</sub>
</p>

Normally-closed: if the wire is cut, the pin floats high, which reads the same as a press.

| Situation | Pin | Result |
| :--- | :---: | :--- |
| Normal (not pressed) | low | movement allowed |
| Pressed | high | stop |
| Wire cut | high | stop |

On this JetPack (4.6.1 / L4T R32.6.1) the supported GPIO library is **`Jetson.GPIO`**.

---

## 6. Option 5 — wrist button

<p align="center">
  <img src="../../assets/hardware/end_effector_atom.png" width="400" alt="The Atom at the wrist, with its button">
  <br><sub>The Atom at the wrist (②) has a button on GPIO 39.</sub>
</p>

The Atom's button (GPIO 39) is read over the serial bus. The Atom is M5Stack Atom ESP32 (`atomMain`); firmware `get_system_version = 73` (pymycobot 4.0.4b5, class `MyCobot280`). |


<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
