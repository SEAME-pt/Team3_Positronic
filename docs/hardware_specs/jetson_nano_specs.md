# myCobot 280 Jetson Nano - Full Hardware & Reconnaissance Specs

## 1. System & Architecture

| Component | Status / Specification | Verification Command | Description & Notes |
| :--- | :--- | :--- | :--- |
| **Operating System** | Ubuntu 20.04.6 LTS (Focal Fossa) | `cat /etc/os-release` | Base OS running on the Cobot. Dictates supported ROS distributions. |
| **Kernel & Architecture** | Linux 4.9.253-tegra (aarch64) | `uname -a` | ARM 64-bit architecture. **Note:** Docker images and binaries must target `arm64/aarch64`. |
| **JetPack Version** | L4T R32.6.1 (JetPack 4.6.1) | `cat /etc/nv_tegra_release` | NVIDIA SDK bundle. Contains specific GPU drivers for this board. |
| **CUDA Version** | 10.2 (`/usr/local/cuda-10.2`) | `ls -la /usr/local/ \| grep cuda` | GPU acceleration toolkit. **Note:** AI/PyTorch models must be compatible with CUDA 10.2. |
| **Power Mode** | MAXN (Mode 0) - Max Perf | `nvpmodel -q` | Jetson is running at maximum power, unlocking all cores. **Note:** Requires stable power supply; avoid downgrading to 5W mode. |
| **Live Monitoring** | `jtop` is running as a service | `jtop` | Real-time dashboard for Jetson vitals. **Action:** Use during M1 to monitor thermal/RAM limits. |

## 2. ROS Ecosystem & Environment

| Component | Status / Specification | Verification Command | Description & Notes |
| :--- | :--- | :--- | :--- |
| **ROS 2 Version** | Galactic (Native) | `ls -la /opt/ros/galactic` | Modern robotics middleware. **Decision:** Standardize on ROS 2 Galactic as it matches the OS natively. |
| **ROS 1 Version** | Noetic (Native) | `ls -la /opt/ros/noetic` | Legacy middleware. **Note:** Ignore for Module 1. |
| **ROS Env Variables** | None are set currently | `env \| grep -i ROS` | ROS 2 environment is not loaded by default. **Requirement:** Add `source /opt/ros/galactic/setup.bash` to `~/.bashrc` or Docker entrypoints. |

## 3. Compute Constraints & Storage

| Component | Status / Specification | Verification Command | Description & Notes |
| :--- | :--- | :--- | :--- |
| **CPU** | 4 Cores (Cortex-A57) @ 1.48 GHz | `lscpu` | Low-power CPU. **Note:** Keep C++ drivers efficient and avoid heavy single-thread blocking. |
| **Total RAM** | 4 GB (3.9 GiB usable) | `free -h` | Shared memory between CPU and GPU. |
| **Idle RAM Available** | ~1.2 GB | `free -h` | Very limited free memory. **Constraint:** Do NOT run Gazebo or Isaac Sim on the Jetson. Simulation must run on host laptops. |
| **System Swap** | 2 GB (zram) | `cat /proc/meminfo \| grep -i swap` | Compressed RAM used as emergency memory. **Note:** Filling RAM will force swap usage, severely degrading real-time control. |
| **Storage (Free)** | 24 GB on `/dev/mmcblk1p1` | `df -h /` | Space for code and datasets. **Note:** Monitor usage; ROS bag recordings fill this quickly. |

## 4. Hardware Devices & Peripherals

| Component | Status / Specification | Verification Command | Description & Notes |
| :--- | :--- | :--- | :--- |
| **Arm Serial Ports** | `/dev/ttyTHS1`, `/dev/ttyTHS2` | `ls -la /dev/ttyTHS*` | Hardware UART pathways to the arm motors. **Configuration:** Map ROS 2 driver to one of these ports. |
| **Camera Port** | Mapped to `/dev/video0` | `ls -la /dev/video*` | Eye-in-hand or fixed Realtek camera. **Configuration:** Set this path in the OpenCV / ROS Camera node. |
| **Camera Formats** | **MJPG:** 30fps at 2592x1944<br>**YUYV:** 3fps at 2592x1944 | `v4l2-ctl -d /dev/video0 --list-formats-ext` | MJPG compresses video; YUYV is raw but slow. **Requirement:** Force camera node to use **MJPG** for low latency visual servoing (30fps). |
| **I2C / Gripper** | Buses 0 and 1 are empty | `i2cdetect -y 0` and `-y 1` | No direct I2C sensors detected on Jetson headers. **Architecture:** The Adaptive Gripper is controlled via the Arm's serial interface. |

## 5. Security, Networking & Background Services

| Component | Status / Specification | Verification Command | Description & Notes |
| :--- | :--- | :--- | :--- |
| **User Permissions** | User `er` is in `dialout` | `groups` | **Verified:** Direct access to Serial ports is available without `sudo`. |
| **Open Ports** | SSH (22), VNC (5900) open | `sudo ss -tuln` | Remote access services. **Workflow:** Remote development via SSH (VSCode) or VNC is supported natively. |
| **Wi-Fi Connection** | Connected to `SEAME` (5GHz) | `iwconfig` | Very strong signal (-49 dBm). **Note:** Stable enough for M1 link-loss watchdog testing. |
| **Wi-Fi Power Mgmt** | `on` | `iwconfig` | Power saving mode is active. **Troubleshooting:** Run `sudo iwconfig wlan0 power off` if ROS 2 DDS messages drop or show high latency. |

## 6. Tooling & Development (Missing / Present)

| Component | Status / Specification | Verification Command | Description & Notes |
| :--- | :--- | :--- | :--- |
| **C/C++ & Make** | gcc 8.4.0, CMake 3.16.3 | `gcc --version` | Standard build tools. **Status:** Ready for compiling ROS 2 C++ packages. |
| **Docker** | Docker 24.0.7 is installed | `docker --version` | Container engine. **Status:** Ready for running isolated ROS 2 environments. |
| **Docker Compose** | **Missing** | `docker compose version` | Defines multi-container apps. **Requirement:** Run `sudo apt install docker-compose-plugin` before attempting `make image`. |
