# SEA:ME 2.0 · Software Engineering for Automation & Mobility Ecosystems

Welcome to the technical content repository of SEA:ME 2.0, the 2026/2027 edition of the program run at 42 Porto.

SEA:ME 2.0 is a 26-week software engineering program followed by a 12-week Corporate Capstone with industry partners. You will build the software that makes real machines move, see, decide and cooperate: micro-drones, a bipedal humanoid, a 6-DOF robotic arm and an autonomous ground vehicle.

## Why Physical AI, and why now

For the last decade most AI lived behind a screen: recommendations, chat, images. Physical AI is the next step. It is AI that perceives the world through sensors, reasons about it and acts on it through motors, wheels and propellers, in real time and under real constraints.

Three things make this the right moment:

- **Compute at the edge.** Boards the size of a credit card now run neural networks at tens of TOPS. A drone or a small robot can perceive on its own, without a datacenter.
- **Open, industrial-grade software.** ROS 2, PX4, Nav2, MoveIt 2 and physics simulators are open source and used in production. The same stack you learn here runs in factories, hospitals and inspection fleets.
- **Foundation models that understand scenes.** Vision-language models (VLM) and vision-language-action models (VLA) let a robot follow an instruction like "pick the damaged part" instead of hand-coded rules.

The hard problems in Physical AI are software problems: real-time control, perception under a latency and power budget, safe behaviour when something fails, and integration between many agents. Those are the problems this program trains you to solve.

## What you will learn

| Area | Technologies |
|---|---|
| Core programming | Modern C++ (17/20), Python 3, CMake, colcon, Git flow, code review |
| Robotics middleware | ROS 2, DDS with explicit QoS, Zenoh bridge, micro-ROS |
| Real-time layer | FreeRTOS / ThreadX on microcontrollers, flight controllers, ISR discipline, jitter profiling |
| Embedded platform | Embedded Linux, cross-toolchains, custom images, signed OTA updates |
| Simulation | Gazebo, Webots, NVIDIA Isaac Sim, URDF/SDF modelling, sim-to-real gap quantification |
| Perception & Edge AI | OpenCV, PyTorch, ONNX, INT8 quantization, NVIDIA TensorRT, Hailo SDK, VLM / VLA models |
| Spatial & planning | TF2, SLAM, Nav2, MoveIt 2, MAVLink / MAVSDK, kinematics |
| Learning from demonstration | Kinesthetic teaching, Dynamic / Probabilistic Movement Primitives |
| Software quality | GoogleTest, pytest, clang-tidy, cppcheck, coverage gates, GitHub Actions, self-hosted hardware-in-the-loop runners |
| Safety & security | Hazard analysis, safe states, fault injection, SROS 2, threat modelling, secure boot |

## Program structure

| Phase | Dates | Folder |
|---|---|---|
| Sprint 0 · Onboarding & Baseline | 6 Oct – 16 Oct 2026 | Tooling, Git, ROS 2 basics, team formation, hardware teardown |
| Module 1 · Architecture, Real-Time Control & Simulation | 19 Oct – 11 Dec 2026 | [M1_Architecture-RT-Control-Sim](M1_Architecture-RT-Control-Sim/) |
| Module 2 · Perception, Edge AI & Spatial Computing | 14 Dec 2026 – 26 Feb 2027 | [M2_Perception-EdgeAI-SpacialCompute](M2_Perception-EdgeAI-SpacialCompute/) |
| Module 3 · Distributed Multi-Agent Operations | 1 Mar – 30 Apr 2027 | [M3_MultiAgent-Ops](M3_MultiAgent-Ops/) |
| Module 4 · Corporate Capstone | 10 May – 30 Jul 2027 | [M4_CorporateCapstone](M4_CorporateCapstone/) |

Breaks: Christmas (21 Dec – 3 Jan), Carnival (8 – 12 Feb), Easter (22 – 26 Mar).

### How the program works

- **Same milestones, platform-specific proof.** Every module sets the same goals for all teams. A drone proves them differently from an arm, so each module lists the expected demo per hardware class.
- **Simulation first, then hardware.** Every behaviour is shown in simulation before it touches a real machine, and you measure the gap between the two.
- **Measured, not merely working.** Deliverables carry numbers: end-to-end latency, control loop jitter, localization drift, detector accuracy after quantization, test coverage.
- **Teams, with individual evaluation.** Work happens in teams with sprint demos and retrospectives. You are evaluated on technical skills, contributions and collaboration.

## The hardware

Four platform classes, all real, all in the lab. Details, applications and challenges are in [00_Hardware-Classes](00_Hardware-Classes/).

| Platform | What it is | Core skills |
|---|---|---|
| [Aerial Micro-Drone](00_Hardware-Classes/01_Drone/) | 2.5-inch cinewhoop quadcopter with flight controller, optical flow and video downlink | Real-time stability, GPS-denied state estimation, failsafe under link loss |
| [Stationary 6-DOF Arm](00_Hardware-Classes/02_Robotic-Arm/) | myCobot 280 with NVIDIA Jetson Nano in the base and adaptive gripper | Forward/inverse kinematics, motion planning, visual servoing, learning from demonstration |
| [Bipedal Humanoid](00_Hardware-Classes/03_Humanoid/) | Hiwonder AiNex, Raspberry Pi and serial-bus servos, ROS-based | High-DOF coordination, balance and gait, kinematic chains |
| [Autonomous Ground Vehicle](00_Hardware-Classes/04_Car/) | 1/10-scale car with Raspberry Pi 5, Hailo AI accelerator and an RTOS microcontroller | SLAM, navigation, sensor fusion, perception latency under motion |

## What you will be able to do at the end

By the end of the program you will have a working physical system, a public demonstration and a portfolio of measured deliverables. Concretely, you will be able to:

- **Build.** Design and implement a distributed autonomous application in C++ and Python on ROS 2, with a real-time layer on a microcontroller or flight controller, validated in Gazebo, Webots or Isaac Sim.
- **Perceive.** Train, quantize, compile and deploy AI models, including VLM / VLA models, to edge accelerators, and state their accuracy, latency and power budgets.
- **Assure.** Write a hazard analysis for your system, implement safe states, and prove resilience under fault injection and link loss.
- **Integrate.** Negotiate an interface contract with peer teams and deliver a heterogeneous multi-agent scenario that survives losing a peer.
- **Operate.** Run CI/CD pipelines with static analysis, unit tests, coverage gates and self-hosted hardware-in-the-loop verification.
- **Communicate.** Defend architecture decisions to engineers and demonstrate a working physical system to non-technical stakeholders.

These map directly to job profiles in robotics, automotive, aerospace, logistics and industrial automation: robotics software engineer, embedded AI engineer, autonomy engineer, systems integration engineer.

## Repository layout

```
00_Hardware-Classes/     The four platforms: key aspects, applications, challenges
M1_Architecture-RT-Control-Sim/
M2_Perception-EdgeAI-SpacialCompute/
M3_MultiAgent-Ops/
M4_CorporateCapstone/
```

## Useful links

- ROS 2: https://docs.ros.org
- PX4: https://px4.io · ArduPilot: https://ardupilot.org
- Gazebo: https://gazebosim.org · Webots: https://cyberbotics.com · Isaac Sim: https://developer.nvidia.com/isaac/sim
- Modern Robotics (kinematics, dynamics, planning): https://modernrobotics.northwestern.edu
- OpenCV: https://opencv.org · Hugging Face robotics: https://huggingface.co
- ThreadX: https://threadx.io · Eclipse SDV: https://eclipsesdv.org
