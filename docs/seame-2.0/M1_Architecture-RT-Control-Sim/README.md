# Module 1 · Architecture, Real-Time Control & Simulation

**19 October – 11 December 2026 · 8 weeks**

## Introduction

Before a robot can see or decide, it needs a software skeleton that moves it reliably. Module 1 builds that skeleton: a layered architecture on ROS 2, a real-time layer close to the motors, a physics simulation to develop against, and a CI/CD pipeline that tests on real hardware.

By the end of the module your platform executes commanded motion in simulation and on hardware, your pipeline is green with a hardware-in-the-loop runner, and your emergency stop works when we inject faults.

## Motivation

Most robot failures in the field are not algorithm failures. They are timing, integration and state-handling failures: a control loop that jitters, a node that starts before its dependency, a message that arrives late, a stop command that is ignored during a mode transition. Industry solves this with discipline: explicit lifecycles, explicit timing budgets, tests that run on the target, and simulation to iterate fast without breaking hardware.

This module teaches that discipline on your platform, while the code is still small enough to get it right.

## What you will learn

- **High-level control abstraction.** ROS 2 lifecycle nodes, velocity setpoints, spatial goal poses, action servers.
- **Real-time layer.** FreeRTOS / ThreadX task design on a microcontroller, ISR discipline, loop jitter profiling, microcontroller-to-ROS bridge (micro-ROS or custom). For the drone, the flight controller plays this role.
- **Software quality and CI/CD.** Unit tests (GoogleTest, pytest), static analysis (clang-tidy, cppcheck), coverage gates above 80%, self-hosted hardware-in-the-loop runners that flash and test the target.
- **Simulation.** Gazebo for fast physics iteration, Webots for mechanisms, NVIDIA Isaac Sim for photorealistic and contact-rich validation. Quantifying the sim-to-real gap.

## Reference architecture

All platforms follow the same layering. What changes is what sits in each layer.

```
+---------------------------------------------------------------+
|  Mission / behaviour        state machine, goals, safe states  |
+---------------------------------------------------------------+
|  ROS 2 middleware           lifecycle nodes, topics, actions,  |
|                             explicit QoS, TF2                  |
+---------------------------------------------------------------+
|  Hardware abstraction       driver / bridge node: one clean    |
|                             interface, sim and real behind it  |
+---------------------------------------------------------------+
|  Real-time layer            MCU with RTOS, or flight controller|
|                             deterministic loops, watchdogs     |
+---------------------------------------------------------------+
|  Sensors and actuators                                         |
+---------------------------------------------------------------+
```

Rules that apply to every platform:

- The real-time layer never depends on Linux being alive. If the link dies, it enters a safe state on its own.
- The hardware abstraction exposes the same interface in simulation and on hardware. Mission code must not know which one it is talking to.
- Every node that can move the robot is a lifecycle node. Emergency stop forces a transition that cuts actuation.

### Aerial Micro-Drone

- **Real-time layer:** the onboard flight controller runs the attitude and rate loops. Do not touch them.
- **Hardware abstraction:** a ROS 2 "flight interface" lifecycle node talking to the flight controller over the ESP8266 Wi-Fi bridge. It publishes telemetry (attitude, battery, link quality) and accepts velocity / attitude setpoints and mode changes.
- **Simulation:** Gazebo with PX4 SITL behind the same flight interface.
- **Safety:** link-loss watchdog in the interface node and in the flight controller failsafe. The radio kill switch stays with a safety pilot.

### Stationary 6-DOF Arm

- **Real-time layer:** the arm's internal servo controller. You command joint targets over serial.
- **Hardware abstraction:** a ROS 2 driver node publishing joint states and exposing a joint trajectory controller (ros2_control). MoveIt 2 on top for Cartesian goal poses.
- **Simulation:** Gazebo or Webots loaded with the arm URDF and the same controllers.
- **Safety:** hardwired emergency stop in the cell. The driver node halts on stop, on joint limits and on communication timeout.

### Bipedal Humanoid

- **Real-time layer:** the servo bus controller board.
- **Hardware abstraction:** a servo bus driver node (joint states, temperatures, voltage), an IMU node, and a gait node exposing a parameterized walk / turn interface.
- **Mission layer:** a state machine with explicit states: stand, walk, turn, crouch (safe state).
- **Simulation:** Webots or Gazebo with the humanoid model.
- **Safety:** instability detection from the IMU triggers the crouch transition.

### Autonomous Ground Vehicle

- **Real-time layer:** microcontroller with FreeRTOS or ThreadX running the motor PWM, steering servo, encoder and ultrasonic tasks with a watchdog.
- **Bridge:** micro-ROS or a custom CAN / serial bridge exposing `cmd_vel`, wheel odometry and sensor readings.
- **ROS 2 side:** Raspberry Pi 5 running the vehicle lifecycle node, camera node, odometry and path following.
- **Simulation:** Gazebo with an Ackermann vehicle model.
- **Safety:** link-loss watchdog on the microcontroller triggers controlled deceleration without help from Linux.

## Goals and deliverables

### Common to all platforms

| Deliverable | Evidence |
|---|---|
| Architecture document | Layer diagram, node graph, message contracts, timing budget per loop |
| Green CI/CD pipeline | Build, unit tests, static analysis, coverage above 80%, hardware-in-the-loop job on a self-hosted runner |
| Real-time profile | Measured loop period and jitter on the real-time layer, plotted |
| Sim-to-real report | Same commanded motion in simulation and on hardware, with the measured difference |
| Emergency stop under fault injection | Live demo: stop is honoured while we kill a node, drop the link, or feed bad data |
| Sprint demo and retrospective | Every two weeks |

### Per-platform demos

| Platform | Commanded motion | Safe state |
|---|---|---|
| Drone | Streams velocity / attitude setpoints through the flight interface and holds position, in Gazebo (PX4 SITL) and on hardware in the cage | Link loss leads to hover or controlled landing |
| Arm | Reaches a Cartesian goal pose through MoveIt 2, in simulation and on hardware, with the pose error reported | Emergency stop honoured through the lifecycle node; halt on joint limit |
| Humanoid | Walks and turns on command with a parameterized gait, in simulation and on hardware | Instability detected from the IMU leads to a crouch |
| Car | Follows a given path within a stated lateral error, in Gazebo and in the arena | Link loss leads to controlled deceleration from the microcontroller |
