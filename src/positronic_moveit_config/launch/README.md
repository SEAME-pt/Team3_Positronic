<a id="top"></a>
<p align="right">
  <img src="../../../docs/assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# Source — positronic_moveit_config — launch

<!-- nav:auto -->
<p align="left">
  <a href="../../../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="../config/README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="../scripts/README.md"><b>Forward &#8594;</b></a>
</p>

Two entry points: one starts the arm in Gazebo with MoveIt on top, the other attaches MoveIt to an arm that is already running.

## Files in this folder

| File | What it starts |
| :--- | :--- |
| `gazebo_moveit.launch.py` | Includes `positronic_bringup/gazebo.launch.py` (Gazebo, `robot_state_publisher`, the `/clock` bridge and the controllers), then `move_group.launch.py` on the simulation clock. |
| `move_group.launch.py` | Builds the MoveIt configuration with `MoveItConfigsBuilder` from `positronic_description/urdf/positronic_gazebo.urdf.xacro` and the files in `config/` (SRDF, kinematics, joint limits, controllers, OMPL). Starts `move_group`, the planning-scene loader (`planning_scene.py`) and RViz2. |

`move_group.launch.py` assumes `robot_state_publisher` and a controller manager with `arm_controller` are already up. It does not start Gazebo, the arm driver or the controllers itself.

## Usage

```bash
ros2 launch positronic_moveit_config gazebo_moveit.launch.py
ros2 launch positronic_moveit_config move_group.launch.py
```

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
