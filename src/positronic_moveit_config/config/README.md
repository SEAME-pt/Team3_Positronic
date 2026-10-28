<a id="top"></a>
<p align="right">
  <img src="../../../docs/assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# Source — positronic_moveit_config — config

<!-- nav:auto -->
<p align="left">
  <a href="../../../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="../README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="../launch/README.md"><b>Forward &#8594;</b></a>
</p>

Everything `move_group.launch.py` loads into MoveIt for the myCobot 280 JN. The robot description itself comes from `positronic_description/urdf/positronic_gazebo.urdf.xacro`.

## Files

| File | What it is | What it is for |
| :--- | :--- | :--- |
| `positronic.srdf` | Semantic robot description | Defines the `arm` chain from `g_base` to `joint6_flange`, the `gripper` group with its mimic joints, the `gripper_ee` end effector, the `home`/`ready`/`open`/`closed` states and the collision-disable pairs. |
| `kinematics.yaml` | Inverse kinematics | Configures the KDL plugin for `arm` with a 0.005 search resolution, 0.05 s timeout and three attempts. |
| `joint_limits.yaml` | Planning limits | Caps every arm joint at 1.0 rad/s and 2.0 rad/s² and defaults to 20% velocity and acceleration scaling. The gripper joints get the same limits so time parameterisation can run. |
| `ompl_planning.yaml` | OMPL pipeline | RRTConnect as the default planner and RRTstar as the alternative for `arm`, with time-optimal parameterisation on the result. |
| `moveit_controllers.yaml` | Controller mapping | Maps `arm_controller` to `FollowJointTrajectory` and `gripper_action_controller` to a `GripperCommand` action. |
| `moveit.rviz` | RViz2 layout | Opens the grid, the robot model, the TF frames and the MotionPlanning panel. |
| `planning_scene.yaml` | Workcell obstacles | Defines the bench, four walls and roof in `g_base`, matching `positronic_bringup/worlds/workcell.sdf`. Loaded by `scripts/planning_scene.py`. |

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
