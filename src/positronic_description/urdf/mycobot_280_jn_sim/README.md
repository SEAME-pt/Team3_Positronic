<a id="top"></a>
<p align="right">
  <img src="../../../docs/assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# positronic_description — urdf — mycobot_280_jn_sim

<!-- nav:auto -->
<p align="left">
  <a href="../../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="../README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="meshes/README.md"><b>Forward &#8594;</b></a>
</p>

The myCobot 280 JN simulation model (URDF + meshes).

## What this is

The clean, Gazebo-ready URDF variants (`*_gripper`, `*_camera`, `*_no_base`) and the mesh assets.

## Files in this folder

| File | What it is |
| :--- | :--- |
| `mycobot_280_jn*.urdf` | URDF variants (with/without gripper, camera, base). |
| `meshes/` | Visual and collision meshes. |
| `gen_jn_sim.py` | Upstream helper script. |

## Changes to the upstream files

- Zero of joint 6 (`joint6output_to_joint6`): the origin carries a pitch of 0.7854 (45 deg) in
  all eight files. Measured on the real arm with every joint at zero: the flange matched the
  upstream model's zero only with joint 6 at -45 deg, and at zero the real gripper sits 45 deg
  from the links while the upstream model's is parallel. `test_joint6_zero.py` keeps the eight
  files equal and at that value.
- Gripper mounting: the gripper fits the flange on the diagonal of the holes, in two ways 180
  deg apart. Mount it as the model has it (`docs/03_verification/joint6_zero`, Gazebo at zero),
  or the camera ends up on the wrong side of the model. The camera variants with the gripper
  carry this mount (`1.5708 0 3.1416`); the two gripper-only variants still have the upstream one.
- Camera mounting (#97): the camera module sits in line with the finger line, lens past the end
  of one finger, on the real arm and in the four camera variants
  (`joint6output_to_camera_flange`, `rpy="1.5708 0 3.1416"`), with the same `camera_link` frame
  in all four (gripper at the bottom of the image; the real raw image matches it once turned 90
  deg counterclockwise). Mount the real camera as in `docs/03_verification/camera_mount`.
  `test_camera_mount.py` keeps the four files equal and checks the mount.

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
