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

The pytest suite for the planning-scene loader. It runs without a robot and without `move_group`; the integration check that a goal inside the bench is refused is a manual procedure.

## Files

| File | What it is | What it is for |
| :--- | :--- | :--- |
| `test_planning_scene.py` | pytest checks for `scripts/planning_scene.py` | Loads the shipped YAML, verifies that every entry becomes a valid `CollisionObject` with one box primitive and one pose in `g_base`, checks the bench top at z = 0, the work-box inner faces just beyond the 0.28 m reach, the roof, and that `remove=true` switches every operation to `REMOVE`. |
| `test_thermal_park.py` | pytest checks for `scripts/thermal_park.py` | Reads the driver's temperature keys from `/diagnostics`; one hot reading alone does not park, 1 s hot parks once per hot spell, a missing reading stops the clock, parks again only after every servo cooled under `rearm_below_c`; J1 and J6 kept, all six joints needed; the move time is the furthest joint at the speed; the trajectory is one point with the six joints; arrival needs every joint within the tolerance; the stop comes at arrival or after the deadline; the supervisor stop is called when its service is there, only warned about when not, skipped with `stop_after_park` off; the rest pose matches the `rest` state of the SRDF. |

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
