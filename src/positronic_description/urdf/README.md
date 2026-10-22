<a id="top"></a>
<p align="right">
  <img src="../../../docs/assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# positronic_description — urdf

<!-- nav:auto -->
<p align="left">
  <a href="../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="../launch/README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="mycobot_280_jn_sim/README.md"><b>Forward &#8594;</b></a>
</p>

</div>

The arm URDF, its meshes, and the xacro that prepares it for Gazebo.

## What this is

`mycobot_280_jn_sim/` holds the plain URDFs and meshes of the myCobot 280 Jetson Nano, from
Elephant Robotics. `positronic_gazebo.urdf.xacro` includes `mycobot_280_jn.urdf` (arm only) and adds
the `<ros2_control>` tag (6 joints, position command and state) and the `<gazebo>` plugin. The
Gazebo launch in `positronic_bringup` runs it with `controllers_file:=` set to the controllers yaml.

## Files in this folder

| File | What it is |
| :--- | :--- |
| `positronic_gazebo.urdf.xacro` | The xacro entry point for Gazebo and ros2_control. |
| `mycobot_280_jn_sim/` | URDFs, meshes and the generator script. |

## See also

- [docs/00_specs/software/05_robot_description.md](../../../docs/00_specs/software/05_robot_description.md)

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
