# 0003. MoveIt runs on laptops; the Jetson builds only NANO_PACKAGES

2026-10-28, accepted, Felipe Suassuna (issue #82), approved by the PR reviewers.

## Context

M1 needs MoveIt 2 planning for the real arm. The Jetson Nano in the arm's base has 4 GB of RAM and a
small disk, and M2 adds the camera and perception on it. The full `ros-jazzy-moveit` package also
brings MoveIt's RViz plugins and its Setup Assistant (Qt).

`make nano-build` used to build every package in `src/` in the Jetson image. With the slim `runtime`
image (ADR 0001), any package that needs MoveIt or Gazebo to compile (a mission node using
`MoveGroupInterface`, for example) would break the Jetson build, even though the Jetson never runs it.

## Decision

- **The Jetson runs** the driver (`positronic_hardware`), `controller_manager` and the joint trajectory
  controller: the loop that talks to the servos.
- **Laptops run** MoveIt (`move_group`), RViz and Gazebo. MoveIt sends each planned trajectory once,
  over DDS, to the controller on the Jetson.
- **The Jetson builds only `NANO_PACKAGES`** (Makefile, today `positronic_hardware positronic_bringup`),
  plus their dependencies, through `colcon build --packages-up-to`. CI builds the same list in the
  `runtime` image (`make nano-packages`).

## Alternatives

- **MoveIt in the Jetson image:** works (the old Jetson image had it), but brings Qt and most of RViz
  onto a 4 GB machine, makes the Jetson image as slow to build as before, and puts planning next to the
  control loop for no gain: the trajectory still runs on the Jetson either way.
- **Only `move_group` and the planners on the Jetson** (not the full package): smaller, but still
  planning memory on the Jetson, and every laptop-side tool (RViz, the MoveIt config launch files)
  would be split across two machines.
- **Keep building all of `src/` on the Jetson:** every laptop-only package would have to compile
  without MoveIt or Gazebo, or the Jetson build breaks.
- **`COLCON_IGNORE` files in laptop-only packages:** they would hide those packages on laptops too.

## Consequences

- A Wi-Fi hiccup delays the next plan but never stalls the control loop, which stays on the Jetson.
- The clocks of laptops and the Jetson must agree (NTP). MoveIt rejects `/joint_states` whose
  timestamps look old.
- If a laptop dies mid-motion, the Jetson's controller still finishes the trajectory it already has.
  The e-stop must never depend on a laptop.
- Moving the real arm with MoveIt needs a laptop on the same network and `ROS_DOMAIN_ID`.
- A new package the Jetson must run goes into `NANO_PACKAGES`; laptop-only packages
  (`positronic_moveit_config`, a mission node, Gazebo launch files) stay out and may depend on anything.

## Revisit when

- The arm must plan with no laptop around (a standalone demo).
- The Wi-Fi between the laptops and the Jetson proves too unreliable for demos or M3.
- MoveIt Servo or visual servoing in M2 needs a control rate that only the Jetson can keep.
