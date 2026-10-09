# 04 · Autonomous Ground Vehicle

A 1/10-scale car with a real-time motor controller and an AI accelerator on top. The car class is where SLAM, navigation and perception while moving come together on the platform SEA:ME has used since its first cohorts.

## The platform

The unit is a **PiRacer-class 1/10 chassis** with the compute refreshed over the last cohorts. The exact 2026/2027 kit list will be confirmed at Sprint 0; the baseline is:

| Component | Role |
|---|---|
| Raspberry Pi 5 (16 GB) + NVMe SSD | Main Linux computer. Runs ROS 2, navigation and the perception pipeline. |
| Raspberry Pi AI HAT+ (Hailo-8, 26 TOPS) | Edge AI accelerator for quantized vision models. |
| Microcontroller (STM32-class / AZ3166) running an RTOS | Real-time layer: motor PWM, steering servo, encoder and ultrasonic reads. Talks to the Pi over CAN or serial. |
| CAN bus shield | Automotive-style bus between the microcontroller and the Pi. |
| Camera, ultrasonic distance sensor, speed sensor | Perception and odometry inputs. A range sensor may be added for mapping. |
| DC motor + steering servo | Ackermann drive: the car steers like a car, not like a differential robot. |
| Display | Instrument cluster style dashboard for state and telemetry. |

## Key aspects

- **Real-time motor control on the microcontroller.** Motor and steering commands run on an RTOS task with deterministic timing, bridged to ROS 2 with micro-ROS or a custom bridge.
- **SLAM and navigation.** Build a 2D map, localize in it, relocalize after being moved, and follow paths with Nav2 under Ackermann constraints.
- **Sensor fusion.** Wheel speed, IMU and camera combined into an odometry estimate good enough for navigation.
- **Perception under motion.** Detectors that run at frame rate on the Hailo accelerator, with latency measured against the vehicle speed.
- **Simulation first.** Gazebo with an Ackermann vehicle model, then the real car in the arena.

## Example applications

- Infrastructure patrol: railway, tunnel, bridge and highway anomaly detection together with a drone.
- Logistics: navigate to a workstation where an arm picks items; transport components between waypoints (Module 3 scenarios).
- Hospital disinfection AMR with human-presence failsafe.
- Perimeter guard: ground interception of targets located by an aerial unit.

## Expected challenges

- **Ackermann kinematics.** Standard planners assume the robot can turn in place. Yours cannot.
- **Mapping with limited sensors.** Camera-based mapping is harder than lidar-based. Choose sensors and algorithms deliberately.
- **Latency under motion.** A detection that arrives 200 ms late describes where the obstacle was. Measure and budget the whole chain.
- **Two computers, one system.** Timing, message contracts and failure handling between the RTOS and Linux sides are where most bugs live.
- **Lighting and floor.** SLAM and vision are sensitive to lighting changes and reflective floors. Control the arena, then test outside it.
