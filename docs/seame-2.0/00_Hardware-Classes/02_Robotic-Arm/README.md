# 02 · Stationary 6-DOF Arm

A desktop collaborative robot arm. The arm class is where you learn kinematics, motion planning, visual servoing and how to teach a robot a task by moving it with your hands.

## The platform

The unit is the **Elephant Robotics myCobot 280 Jetson Nano** with the **myCobot Adaptive Gripper**. The class fleet has three units.

| Component | Role |
|---|---|
| 6 servo joints | 280 mm working radius, about 250 g payload. Each joint reports its angle; the arm exposes a serial protocol for joint and Cartesian commands. |
| NVIDIA Jetson Nano (in the base) | Onboard Linux computer with GPU. Runs the arm driver and can run inference. Check its JetPack / Ubuntu version before picking a ROS 2 distribution; running ROS 2 in a container or on an external host is a valid design. |
| Adaptive gripper | Two-finger gripper that conforms to object shape. Enough for sorting small parts. |
| Camera (eye-in-hand or fixed) | Provides the visual feedback for pose estimation and visual servoing. Mounts are 3D-printed in the lab. |
| Guarded workspace + emergency stop | The arm is small but fast enough to pinch. The cell has a hardwired stop that your lifecycle node must honour. |

## Key aspects

- **Forward and inverse kinematics.** From joint angles to end-effector pose and back. You will verify your FK against external measurements.
- **Motion planning.** MoveIt 2 with a URDF of the arm, collision objects and joint limits. Cartesian goal poses, not hand-tuned joint angles.
- **Visual servoing.** A camera-based pose estimator closes the loop on the object, correcting for placement error.
- **Learning from demonstration.** The arm can be moved by hand in free-drive mode. Recorded trajectories are encoded as Dynamic or Probabilistic Movement Primitives and replayed under workspace variations.
- **Simulation first.** Gazebo or Webots with the arm's URDF, then the real unit.

## Example applications

- Quality gate at the exit of an injection-moulding press: multimodal inspection and rapid part sorting, programmed by demonstration.
- Pick-and-place at a logistics workstation, receiving natural-language orders through a VLA model.
- Docking, loading and unloading requests from a mobile robot (Module 3 scenarios).
- Mould exchange sequences validated in simulation with strict state transitions.

## Expected challenges

- **Precision limits.** Hobby-grade servos have backlash and limited repeatability. Visual feedback is what makes tasks reliable.
- **Hand-eye calibration.** The camera-to-arm transform must be calibrated, and it drifts if a mount moves.
- **Compute constraints.** The Jetson Nano is old and small. Decide what runs onboard and what runs on a host, and measure it.
- **Collision awareness.** The workspace contains the gripper, the parts, fixtures and sometimes another robot. Planning scenes must reflect reality.
- **Payload and thermal limits.** Holding a load at full reach heats the servos. Design tasks that respect the limits.
