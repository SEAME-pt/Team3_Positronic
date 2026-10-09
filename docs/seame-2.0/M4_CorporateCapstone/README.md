# Module 4 · Corporate Capstone & Partner Ecosystem

**10 May – 30 July 2027 · 12 weeks**

## Introduction

The Capstone is where the program meets industry. Teams embed with a corporate partner and solve a real problem with the stack built in Modules 1 to 3: the same middleware, the same edge AI pipeline, the same safety and multi-agent discipline, now against acceptance criteria written by an engineer who needs the result.

By the end of the module you demonstrate a working physical and simulated system against signed partner acceptance criteria and defend your architecture in an individual oral exam.

## Motivation

Everything before this point was measured by us. Now it is measured by someone who will use it. Partners bring constraints that do not exist in a lab: regulation, weather, dust, network policies, people walking through the workspace, and a deadline. Learning to deliver under those constraints, and to explain your decisions to both engineers and non-technical stakeholders, is the last competence of the program.

Financially, the Capstone has a variable component negotiated with each partner. Details are covered in the kickoff session.

## How it works

1. **Project selection.** Each team chooses one of the seven projects below. Partners are confirmed per project.
2. **Acceptance criteria.** Team and partner write and sign the acceptance criteria in the first two weeks: what the system must do, how it is measured, and in which environment.
3. **Delivery.** Sprint cadence continues, with partner reviews. Simulation first, then the partner's environment or a faithful stand-in in the lab.
4. **Defense.** Final demo to the partner and an individual oral defense of the architecture.

## The seven projects

| # | Project | Platform | Technical stack | Potential partners |
|---|---|---|---|---|
| 1 | **Toll Portals & High-Voltage Power Line Inspection.** Autonomous drone flyby of toll gantries and power line towers, detecting corrosion, insulator damage and cable sag. | Drone | C++20, ROS 2 (DDS QoS), PyTorch → ONNX → TensorRT (INT8), Yocto Linux, Gazebo (PX4 SITL), Zenoh bridge, OpenCV, GoogleTest | E-Redes, REN, Via Verde, Ascendi, Brisa, Kapsch TrafficCom, Yunex Traffic (Siemens) |
| 2 | **Highway, Railway, Bridge & Tunnel Safety Patrol.** Ground vehicle plus drone patrol for track anomalies, tunnel cracks, bridge joint displacement and emergency signalling faults. | Car + Drone | C++17, ROS 2, Zephyr RTOS, micro-ROS, VLA model, signed OTA, Webots | Infraestruturas de Portugal, CP, Metro Lisboa / Porto, Brisa O&M, CEiiA, Salvador Caetano |
| 3 | **Quality Gate & Programming by Demonstration for Injection-Moulded Parts.** Multimodal edge AI quality control at press exit and a 6-DOF arm taught by kinesthetic demonstration for rapid part sorting. | Arm | Python 3, PyTorch, Hailo-8 SDK (INT8), OpenCV, DMP / ProMP, MoveIt 2, ROS 2, Docker, C++17, VLM / VLA, Webots | Simoldes Plásticos, Iberomoldes, Bosch Car Multimedia, Continental Advanced Antenna, Celoplás |
| 4 | **High-Bay Warehouse & Logistics Inventory Drone.** GPS-denied flight in narrow 12 m vertical aisles for barcode reading and stock counting. | Drone | C++20, NVIDIA Isaac Sim, Gazebo, TensorRT, OpenCV, Buildroot Linux, pytest | Luís Simões, Torrestir, MC Sonae, Jerónimo Martins, CTT |
| 5 | **Hospital UV-C Disinfection AMR & Medical Operations.** Autonomous UV-C sanitization robot for operating rooms with contextual human detection and emergency radiation cutoff. | Car (+ Arm) | C++17, FreeRTOS, micro-ROS, ROS 2 lifecycle nodes, Nav2, MoveIt 2, SROS 2, PyTorch / TensorRT INT8 VLM, Docker, clang-tidy, cppcheck | Grupo CUF, Luz Saúde, Grupo Lusíadas, SNS, Egor Clean, ISS Facility Services |
| 6 | **Solar Farm & Precision Agriculture Aerial Survey.** Photovoltaic hotspot detection at scale, crop health (NDVI) and automated field mapping. | Drone | Python 3, PyTorch, OpenCV, INT8 quantization, C++17, CMake, GoogleTest, Gazebo (PX4 SITL), QGroundControl | EDP Renováveis, Greenvolt, Galp Energia, Sovena, The Navigator Company, Beyond Vision, Edisoft |
| 7 | **Coastal Beach Safety, Environmental & Maritime Surveillance.** Coastal patrol drone for beach safety monitoring, swimmer distress detection and marine litter tracking. | Drone | C++20, ROS 2, PyTorch → TensorRT (INT8), MAVLink / MAVSDK, OpenCV, Yocto Linux, Zenoh bridge, Gazebo | Autoridade Marítima Nacional, ISN, Portos de Portugal, Tekever, CEiiA |

## Cross-cutting topics

These run through the Capstone and are assessed in the defense:

- **Safety and assurance.** Hazard analysis, FMEA, safe-state design, fault injection. Regulatory literacy: ISO 10218 / ISO/TS 15066 (industrial robots), ISO 26262 (automotive), DO-178C (aerospace), EASA / ANAC drone rules.
- **Cybersecurity.** Threat modelling, SBOM hygiene, SROS 2, signed OTA with secure boot, ISO/SAE 21434 and UNECE R155 for automotive.
- **Systems engineering.** Requirements traceability from Sprint 0 to the Capstone acceptance criteria.
- **Professional practice.** Code reviews, technical documentation, public demonstrations.

## Goals and deliverables

| Deliverable | Evidence |
|---|---|
| Signed acceptance criteria | Written with the partner in the first two weeks, versioned in the repo |
| Working system | Physical and simulated system demonstrated against the acceptance criteria, in the partner's environment or a documented stand-in |
| Measured results | The metrics defined in the criteria (detection accuracy, latency, coverage, drift, uptime) reported with method |
| Safety and security package | Hazard analysis, safe states demonstrated under fault injection, threat model, signed update path |
| Technical documentation | Architecture, interfaces, deployment and operation guide a partner engineer can follow |
| Individual oral defense | Each student defends the architecture and their own contribution |
| Public demonstration | Working system shown to technical and non-technical stakeholders |
