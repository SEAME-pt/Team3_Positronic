# Module 2 · Perception, Edge AI & Spatial Computing

**14 December 2026 – 26 February 2027 · 8 weeks (Christmas and Carnival breaks included)**

## Introduction

Module 1 gave your platform a reliable body. Module 2 gives it eyes and a sense of place. You will calibrate sensors, build datasets, train and quantize models, deploy them to edge accelerators, and close a control loop on what the robot sees. You will also localize the robot in its environment, and, on the arm, teach it a task by demonstration.

By the end of the module an accelerated model runs on your hardware at the target frame rate and drives behaviour, with a published latency and power budget.

## Motivation

A model that scores well on a laptop is not a perception system. On a robot it must run on a small, power-limited board, within a latency the control loop can tolerate, on a camera that was calibrated by you, with timestamps that line up with the IMU. Getting that right is most of the work of an edge AI engineer, and it is what separates a demo from a product.

The second theme is spatial reasoning. A robot that cannot answer "where am I, and where is the thing I care about" cannot execute a goal. The third is learning from demonstration: for manipulation tasks it is often faster to show a robot than to program it.

## What you will learn

- **Sensor calibration.** Camera intrinsics and extrinsics, IMU calibration, hand-eye calibration, time synchronization across sensors.
- **Data engineering and edge compilation.** Dataset capture and versioning, training in PyTorch, export to ONNX, INT8 quantization, acceleration with NVIDIA TensorRT or the Hailo SDK.
- **Learning from demonstration.** Kinesthetic teaching, trajectory recording, Dynamic and Probabilistic Movement Primitives, policy learning for manipulation.
- **VLM / VLA integration.** Vision-language models for semantic scene understanding, vision-language-action models for contextual task execution, first in simulation.
- **Spatial computing.** TF2 frames, SLAM, localization, relocalization, basic navigation with Nav2.
- **Profiling.** Full sensor-to-actuation latency and power consumption, published against the platform budget.

## Reference architecture

```
 sensors --> calibration & sync --> perception (quantized model on accelerator)
                                          |
                                          v
                              state estimation / localization (TF2)
                                          |
                                          v
                              planning / control  -->  Module 1 interfaces --> actuators
```

Every stage gets a timestamp. The difference between the first and the last is your end-to-end latency, and you will publish it.

### Aerial Micro-Drone

- **Perception:** the 5.8 GHz video stream arrives on the ground computer as a webcam. A quantized detector or a VLM pipeline runs there (GPU or Hailo) and outputs target coordinates.
- **State estimation:** optical flow plus IMU for velocity and position; visual-inertial odometry from the camera for 3D, GPS-denied localization.
- **Control:** targets become setpoints through the Module 1 flight interface. The loop latency includes the video link and the Wi-Fi uplink.

### Stationary 6-DOF Arm

- **Perception:** camera with hand-eye calibration; a pose estimator for the parts in the workspace; VLM for part classification (good / damaged).
- **Visual servoing:** the pose estimate corrects the target pose in a closed loop before and during the grasp.
- **Learning from demonstration:** move the arm by hand, record joint trajectories, encode them as DMP / ProMP, replay with a new start and goal.
- **Compute split:** decide what runs on the Jetson Nano and what runs on a host, and measure both.

### Bipedal Humanoid

- **Perception:** head camera feeding an onboard classifier or detector, quantized for the Raspberry Pi, driving behaviour (approach, avoid, grasp).
- **State estimation:** gait odometry fused with the IMU; drift measured against ground truth.
- **VLA:** a natural-language command mapped to a navigate / retrieve / return sequence, validated in simulation first.

### Autonomous Ground Vehicle

- **Perception:** camera into a quantized detector on the Hailo-8 at frame rate; obstacle and lane information feed the Nav2 costmap.
- **Localization:** 2D SLAM in simulation and in the arena; relocalization after the car is picked up and moved.
- **Navigation:** Nav2 with an Ackermann-aware planner and controller; dynamic obstacle avoidance.

## Goals and deliverables

### Common to all platforms

| Deliverable | Evidence |
|---|---|
| Calibration report | Intrinsics, extrinsics, IMU parameters, time offsets, with the procedure in the repo |
| Dataset and training pipeline | Versioned dataset, reproducible training script, model card |
| Quantized model | Accuracy before and after INT8 quantization, size and frame rate on the target accelerator |
| Latency and power budget | Sensor-to-actuation latency per stage and power draw, published against the platform budget |
| Portability proof | The perception pipeline running on a second hardware class, with the differences documented |
| Sprint demo and retrospective | Every two weeks |

### Per-platform demos

| Platform | Localization and state estimation | Edge inference in the loop | Autonomous goal |
|---|---|---|---|
| Drone | 3D visual-inertial odometry in a GPS-denied space, drift reported | VLM or detector pipeline on the video stream producing target coordinates | Waypoint inspection sweep of a marked structure |
| Arm | End-effector pose from forward kinematics verified against an external measurement | Pose estimator feeding the visual servoing loop | Pick-and-place sequence with tight alignment; a demonstrated trajectory learned and replayed under workspace variations |
| Humanoid | Gait odometry fused with the IMU, drift within the stated bound | Onboard classifier driving behaviour | Navigate to a site, retrieve an object on a VLA command, return |
| Car | 2D SLAM in simulation and arena; relocalization after displacement | Accelerated detector on the Hailo-8 at target frame rate | Dynamic obstacle avoidance on a factory or road style course |
