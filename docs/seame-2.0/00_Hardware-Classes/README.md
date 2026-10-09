# Hardware Classes

SEA:ME 2.0 runs on four classes of physical hardware. Each team is assigned one platform for Sprint 0 through Module 2, then works across platforms in the multi-agent scenarios of Module 3.

| Platform | Unit | Compute | Actuation | Where it operates |
|---|---|---|---|---|
| [01 · Aerial Micro-Drone](01_Drone/) | Pilotix Brook 2.5-inch cinewhoop, 4S | Onboard flight controller + ground computer over Wi-Fi and 5.8 GHz video link | 4 brushless motors | Netted cage indoors, or outdoors |
| [02 · Stationary 6-DOF Arm](02_Robotic-Arm/) | Elephant Robotics myCobot 280 Jetson Nano | NVIDIA Jetson Nano in the base | 6 servo joints + adaptive gripper | Fixed on a bench, guarded workspace |
| [03 · Bipedal Humanoid](03_Humanoid/) | Hiwonder AiNex | Raspberry Pi | Serial-bus servos (HX-35 series), IMU | Flat floor, soft mat |
| [04 · Autonomous Ground Vehicle](04_Car/) | 1/10-scale PiRacer-class car | Raspberry Pi 5 + Hailo AI HAT+, RTOS microcontroller | DC drive + servo steering | Ground arena, controlled lighting |

## Why four different platforms

Each class exposes a different family of problems:

- **Drone:** hard real-time stability and state estimation in 3D without GPS. The compute is mostly off-board, so link latency and loss are part of the design.
- **Arm:** kinematics, precise motion in a 3D workspace, and learning tasks from human demonstration.
- **Humanoid:** many joints that must move together, balance, and gait. Falls are part of the job.
- **Car:** mapping, navigation and perception while moving, on a real-time motor control loop.

The milestones are the same for everyone. What changes is the evidence each platform can show:

| Milestone | Drone | Humanoid | Arm | Car |
|---|---|---|---|---|
| Commanded motion via middleware | Velocity / attitude setpoints, position hold | Parameterized gait: walk, turn on command | Cartesian goal pose reached | Follow a path within a lateral error bound |
| Localization & state estimation | 3D visual-inertial odometry, GPS-denied | Gait odometry fused with IMU, bounded drift | End-effector pose from FK verified externally | 2D SLAM, relocalize after displacement |
| Edge inference in the loop | VLM pipeline on the video stream | Onboard classifier driving behaviour | Pose estimator feeding visual servoing | Accelerated vision detector on the AI HAT |
| Autonomous goal execution | Waypoint inspection sweep | Navigate, retrieve object via VLA command, return | Pick-and-place with tight alignment | Dynamic obstacle avoidance |
| Safe state & degraded mode | Link loss: hover / controlled landing; low battery: return | Instability detected: crouch | E-stop via lifecycle node, halt on limits | Link loss: controlled deceleration |
| Multi-agent participation | Publish target coordinates in a shared frame | Accept and execute a task handed off by a peer | Serve docking / loading with visual alignment | Transport between peer-defined waypoints |

## Shared lab equipment

- **3D printer:** Bambu Lab P2S with PLA filament, for mounts, guards, camera holders and replacement parts.
- **Simulation workstations:** Linux laptops with RTX-class GPUs for Gazebo, Webots and Isaac Sim.
- **Ground arena:** flat floor with reconfigurable obstacles and controlled lighting for the car and humanoid.
- **Flight cage:** netted area for drone flights indoors.
- **CI infrastructure:** dedicated network with self-hosted runners attached to real hardware for hardware-in-the-loop tests.
- **Spares and battery station:** spare servos, frames and batteries, plus a safe LiPo charging station.

## Safety rules that apply to every platform

- Complete the safety induction in Sprint 0 before touching hardware: electrical basics, LiPo handling, cage and guarded-cell rules.
- Every platform has a hardware kill switch or emergency stop. Your software must honour it, and it always wins over your code.
- Test in simulation first. Bring a behaviour to hardware only after it works in sim and you understand the failure modes.
- Charge and store LiPo batteries only in the designated station and bags.
- Log every crash, fall or hardware fault in the team's issue tracker. Spares are limited.
