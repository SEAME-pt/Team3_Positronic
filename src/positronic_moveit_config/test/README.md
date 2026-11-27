<a id="top"></a>
<p align="right">
  <img src="../../../docs/assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# Source — positronic_moveit_config — test

<!-- nav:auto -->
<p align="left">
  <a href="../../../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="../scripts/README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="../../positronic_multiagent/README.md"><b>Forward &#8594;</b></a>
</p>

The pytest suite for the planning-scene loader, and a check of `positronic_bringup`'s `mission.launch.py`. It runs without a robot and without `move_group`; the integration check that a goal inside the bench is refused is a manual procedure.

## Files

| File | What it is | What it is for |
| :--- | :--- | :--- |
| `test_planning_scene.py` | pytest checks for `scripts/planning_scene.py` | Loads the shipped YAML, verifies that every entry becomes a valid `CollisionObject` with one box primitive and one pose in `g_base`, checks the bench top at z = 0, the work-box inner faces just beyond the 0.28 m reach, the roof, and that `remove=true` switches every operation to `REMOVE`. |
| `test_planning_scene_node.py` | pytest checks of the loader node against a fake `move_group` service | Runs the loader as in `main()` against a fake `/apply_planning_scene` in its own rclpy context, with a service name unique to the process. |
| `test_mission_launch.py` | pytest check of positronic_bringup's `mission.launch.py` | Builds the mission node (nothing launched) and checks it gets `robot_description_planning` from `joint_limits.yaml`: every value, and the 0.3 velocity scaling (#214). Here, not in bringup, because this package already depends on bringup. |

Run it through colcon:

```bash
colcon test --packages-select positronic_moveit_config
colcon test-result --verbose
colcon test-result --all --verbose
```

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
