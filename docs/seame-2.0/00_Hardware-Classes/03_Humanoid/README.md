# 03 · Bipedal Humanoid

A desktop-sized walking robot. The humanoid class is where many joints must move together, balance is never free, and the robot falls if the gait is wrong.

## The platform

The unit is the **Hiwonder AiNex**, a ROS-based bipedal humanoid built around a Raspberry Pi. The class fleet has three units plus spare servos.

| Component | Role |
|---|---|
| Raspberry Pi (onboard) | Runs Linux, the servo controller interface, the camera pipeline and the vendor ROS stack. |
| Serial-bus servos (Hiwonder HX-35HM / HX-35H, HX-12H for smaller joints) | High-torque joints with magnetic encoders on a shared bus. Each servo reports position and temperature. Spares are in stock because servos wear. |
| IMU | Body orientation and angular rates, used by the balance controller and for gait odometry. |
| Camera | Head-mounted camera for object detection and visual navigation. |
| Battery | Onboard pack. Servo torque drops with voltage, so monitor it. |
| Vendor software | Inverse-kinematics gait generator and ROS packages. Expect to bridge or port them to ROS 2. |

## Key aspects

- **High-DOF coordination.** Over twenty joints must follow a coordinated trajectory at the same time. This is where kinematic chains and timing discipline matter.
- **Balance and gait.** A parameterized gait (step length, turn rate, speed) driven by the IMU. Stability is measured, not assumed.
- **Gait odometry.** Position estimated from steps and fused with the IMU. Drift must stay within defined bounds.
- **Safe state.** When instability is detected, the robot crouches to a low, stable pose. That transition must be fast and reliable.
- **Simulation first.** Webots or Gazebo with the humanoid model, then the real unit on a soft mat.

## Example applications

- Search and rescue: a drone locates an asset, the humanoid navigates to it, retrieves it and carries it to safety (Module 3 scenario).
- Object retrieval on a natural-language command interpreted by a VLA model.
- Task handoff from a peer agent: accept, execute, report.

## Expected challenges

- **Falls.** They will happen. Protect the hardware with a soft mat, test near the ground, and make the crouch reflex work early.
- **Servo thermal and voltage limits.** Long walks heat the servos and the battery sags. Read the telemetry and back off.
- **Calibration.** Servo zero offsets drift after a fall or a replacement. Keep a calibration procedure in the repo.
- **Limited onboard compute.** The Raspberry Pi runs the gait and the camera. Heavy inference may need to be offloaded or quantized.
- **Vendor stack.** The shipped software targets an older ROS version. Bridging it cleanly is real integration work.
