<a id="top"></a>
<p align="right">
  <img src="../assets/brand/logo.png" width="460" alt="myCobot 280 for Jetson Nano">
</p>

# 00 · Specifications


<!-- nav:auto -->
<p align="left">
  <a href="README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="software/README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="hardware/README.md"><b>Forward &#8594;</b></a>
</p>

The ground for the project: what the physical unit is and what the software stack is.

| Sub-tree | What you find |
| :--- | :--- |
| [`hardware/`](hardware/README.md) | The myCobot 280 (Jetson Nano): arm, base computer, gripper, camera, pinouts, serial protocol, command reference, safety and maintenance. |
| [`software/`](software/README.md) | The software stack: ROS 2, ros2_control, MoveIt 2/RViz2, Gazebo, robot description, languages and build, Docker, serial/firmware, testing, CI, monitoring, perception. |

## At a glance

| | |
| :--- | :--- |
| **Robot** | myCobot 280 for Jetson Nano — 6 DOF, ~250 g payload, 280 mm reach, ±0.5 mm repeatability |
| **Base computer** | NVIDIA Jetson Nano 4 GB (L4T R32.6.1 / JetPack 4.6.1, Ubuntu 20.04.6, kernel 4.9.253-tegra, CUDA 10.2, Python 3.8.10) — `/dev/ttyTHS1` @ 1,000,000 baud |
| **Wrist computer** | M5Stack Atom ESP32 (`atomMain`), firmware `get_system_version = 73` (pymycobot 4.0.4b5, class `MyCobot280`) |
| **Gripper** | Adaptive, 20–45 mm opening, ~150 g gripping force |
| **Camera** | `/dev/video0`, MJPG preferred |
| **Runtime** | ROS 2 Jazzy, ros2_control, MoveIt 2, Gazebo; C++ and Python; Docker |
| **Notes** | No e-stop installed yet (M1). Joint limits: J1 ±168, J2 ±140, J3 ±150, J4 ±150, J5 −155/+160, J6 ±180 deg. 40-pin reads right-to-left (BCM), pin 1 far right. |

## Hardware

| # | Note | Contents |
| :---: | :--- | :--- |
| — | [hardware/README.md](hardware/README.md) | Quick facts, glossary, manual contradictions |
| 01 | [01_overview.md](hardware/01_overview.md) | Parts, ports, base interior, wrist |
| 02 | [02_arm_parameters.md](hardware/02_arm_parameters.md) | Sizes, joints, reach, geometry |
| 03 | [03_jetson_nano.md](hardware/03_jetson_nano.md) | The computer in the base |
| 04 | [04_gripper.md](hardware/04_gripper.md) | The adaptive gripper |
| 05 | [05_camera.md](hardware/05_camera.md) | The USB camera |
| 06 | [06_pinouts.md](hardware/06_pinouts.md) | Base 40-pin header and wrist ports |
| 07 | [07_serial_protocol.md](hardware/07_serial_protocol.md) | Serial frame format and encoding |
| 08 | [08_command_reference.md](hardware/08_command_reference.md) | All 62 opcodes |
| 09 | [09_buttons_and_e_stop.md](hardware/09_buttons_and_e_stop.md) | Stop-button options |
| 10 | [10_safety_and_handling.md](hardware/10_safety_and_handling.md) | Safety rules, transport, storage |
| 11 | [11_maintenance.md](hardware/11_maintenance.md) | Maintenance schedule and calibration |

## Software

| # | Note | Contents |
| :---: | :--- | :--- |
| 01 | [01_ros.md](software/01_ros.md) | What ROS is; ROS 1 vs ROS 2; concepts; DDS and QoS |
| 02 | [02_ros2_control.md](software/02_ros2_control.md) | The real-time control framework (hardware interface, controllers) |
| 03 | [03_moveit_and_rviz.md](software/03_moveit_and_rviz.md) | Motion planning (MoveIt 2) and visualisation (RViz2) |
| 04 | [04_gazebo.md](software/04_gazebo.md) | The simulator; SDF; sim-to-real; alternatives |
| 05 | [05_robot_description.md](software/05_robot_description.md) | URDF, xacro, SDF and TF2 — describing the robot |
| 06 | [06_languages_and_build.md](software/06_languages_and_build.md) | C++, Python, CMake, colcon, ament |
| 07 | [07_docker.md](software/07_docker.md) | Containers, images, Compose, the project's images |
| 08 | [08_serial_and_firmware.md](software/08_serial_and_firmware.md) | pymycobot, myStudio, the arm and Atom firmware |
| 09 | [09_testing_and_quality.md](software/09_testing_and_quality.md) | GoogleTest, pytest, clang-format, clang-tidy, cppcheck, coverage |
| 10 | [10_ci_and_git.md](software/10_ci_and_git.md) | Git, GitHub, GitHub Actions, the hardware-in-the-loop runner |
| 11 | [11_monitoring.md](software/11_monitoring.md) | jtop, tegrastats, ROS 2 command-line tools, remote access |
| 12 | [12_perception_and_ai.md](software/12_perception_and_ai.md) | OpenCV, PyTorch, ONNX, TensorRT, CUDA (Module 2) |
| 13 | [13_networking_and_security.md](software/13_networking_and_security.md) | DDS/QoS, Zenoh bridge, namespaces, SROS 2, signed OTA, threat model, heartbeats (Module 3) |

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
