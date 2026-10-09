# Hardware Pinouts: Jetson Nano & M5Stack Atom

## 1. Jetson Nano (40-Pin Header - myCobot Custom Base)

**CRITICAL ORIENTATION NOTE:**
Due to how the Jetson Nano is mounted inside the plastic enclosure of the myCobot 280, the standard 40-pin header is exposed in a way where **Pin 1 and Pin 2 are on the FAR RIGHT**.
You must read the physical pins from **RIGHT to LEFT** to follow standard documentation. The sticker on the case uses BCM (Raspberry Pi style) logic numbering.

### Visual Representation of the Sticker (Reading Right-to-Left)

```text
       <-- Pin 39 .............................................................. Pin 01 <-- (Top Row)
       <-- Pin 40 .............................................................. Pin 02 <-- (Bottom Row)

[ Left side of robot ]                                                               [ Right side of robot ]
-----------------------------------------------------------------------------------------------------------------
| GND | 26 | 19 | 13 | 06 | 05 | 00 | GND | 11 | 09 | 10 | 3.3 | 22 | 27 | 17 | GND | 04 | 03 | NC | 3.3 | Top
|  21 | 20 | 16 | GND| 12 | GND| 01 |  07 | 08 | 25 | GND|  24 | 23 | GND| 18 |  NC | NC | GND| 5V |  5V | Bottom
-----------------------------------------------------------------------------------------------------------------
```

### Detailed Electrical & Hardware Pinout Map

| Physical Pin | Label on Board | Internal / Hardware Function | Default State / Usage Notes |
| :---: | :---: | :--- | :--- |
| **1** | **3.3** | 3.3V Power Supply | **DO NOT USE FOR SIGNAL**. Max 1.5A combined with Pin 17. |
| **2** | **5V** | 5V Power Supply | **DO NOT USE FOR SIGNAL**. Max 2.5A combined with Pin 4. |
| **3** | **NC** | I2C_2_SDA (GPIO 02) | **Disconnected from header**. Used internally for arm comms. |
| **4** | **5V** | 5V Power Supply | **DO NOT USE FOR SIGNAL**. |
| **5** | **03** | I2C_2_SCL (GPIO 03) | Has internal 1.8k pull-up resistor. Use with caution. |
| **6** | **GND** | Ground | Standard Ground. |
| **7** | **04** | AUD_MCLK (GPIO 04) | Standard GPIO. |
| **8** | **NC** | UART_2_TX (GPIO 14) | **Disconnected**. Critical serial TX to the robotic arm. |
| **9** | **GND** | Ground | Standard Ground. |
| **10** | **NC** | UART_2_RX (GPIO 15) | **Disconnected**. Critical serial RX from the robotic arm. |
| **11** | **17** | UART_2_RTS (GPIO 17) | Standard GPIO. |
| **12** | **18** | I2S_4_BCLK (GPIO 18) | Standard GPIO. |
| **13** | **27** | SPI_2_SCK (GPIO 27) | Standard GPIO. |
| **14** | **GND** | Ground | Standard Ground. |
| **15** | **22** | GPIO 22 | Standard GPIO. |
| **16** | **23** | SPI_2_CS1 (GPIO 23) | Standard GPIO. |
| **17** | **3.3** | 3.3V Power Supply | **DO NOT USE FOR SIGNAL**. |
| **18** | **24** | SPI_2_CS0 (GPIO 24) | Standard GPIO. |
| **19** | **10** | SPI_1_MOSI (GPIO 10) | Standard GPIO. |
| **20** | **GND** | Ground | Standard Ground. |
| **21** | **09** | SPI_1_MISO (GPIO 09) | Standard GPIO. |
| **22** | **25** | SPI_2_MISO (GPIO 25) | Standard GPIO. |
| **23** | **11** | SPI_1_SCK (GPIO 11) | Standard GPIO. |
| **24** | **08** | SPI_1_CS0 (GPIO 08) | Standard GPIO. |
| **25** | **GND** | Ground | Standard Ground. |
| **26** | **07** | SPI_1_CS1 (GPIO 07) | Standard GPIO. |
| **27** | **00** | I2C_1_SDA (GPIO 00) | Standard GPIO. |
| **28** | **01** | I2C_1_SCL (GPIO 01) | Standard GPIO. |
| **29** | **05** | GPIO 05 | Standard GPIO. |
| **30** | **GND** | Ground | Standard Ground. |
| **31** | **06** | GPIO 06 | Standard GPIO. |
| **32** | **12** | GPIO 12 | Standard GPIO. |
| **33** | **13** | GPIO 13 | Standard GPIO. |
| **34** | **GND** | Ground | Standard Ground. |
| **35** | **19** | I2S_4_LRCK (GPIO 19) | Standard GPIO. |
| **36** | **16** | UART_2_CTS (GPIO 16) | Standard GPIO. |
| **37** | **26** | SPI_2_MOSI (GPIO 26) | Standard GPIO. |
| **38** | **20** | I2S_4_SDIN (GPIO 20) | Standard GPIO. |
| **39** | **GND** | Ground | Standard Ground. |
| **40** | **21** | I2S_4_SDOUT (GPIO 21) | Standard GPIO. |

## 2. M5Stack Atom (ESP32 - End Effector)
The Atom sits at the tip of the robot arm. It communicates with the base (Jetson) through a serial cable routed inside the arm joints, and provides control for the Adaptive Gripper.

| Interface | Label on Board | Mapped To (ESP32) | Function in myCobot |
| :--- | :--- | :--- | :--- |
| **Grove Port (Side)**| GND | GND | Ground for Gripper/Sensor |
| **Grove Port (Side)**| 5V | 5V | Power for Gripper/Sensor |
| **Grove Port (Side)**| G26 | GPIO 26 | Data / PWM signal for Gripper |
| **Grove Port (Side)**| G32 | GPIO 32 | Data / PWM signal for Gripper |
| **Bottom Header** | 5V0 (Printed as 7VO) | Power Out | 5V Power supply from the arm base |
| **Bottom Header** | GND | GND | Ground |
| **Bottom Header** | 3V3 | 3.3V | 3.3V Power Out |
| **Bottom Header** | G22 | GPIO 22 | Free / Internal routing |
| **Bottom Header** | G19 | GPIO 19 | Free / Internal routing |
| **Bottom Header** | G23 | GPIO 23 | Free / Internal routing |
| **Bottom Header** | G33 | GPIO 33 | Free / Internal routing |

**Notes on the Atom:**
* The M5Stack Atom is purely a slave device. It runs micro-ROS or a basic C firmware (MyStudio) that listens for serial commands from the Jetson Nano to open/close the Gripper.
