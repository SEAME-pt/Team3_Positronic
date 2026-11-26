<a id="top"></a>
<p align="right">
  <img src="../../../docs/assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# Source — positronic_moveit_config — scripts

<!-- nav:auto -->
<p align="left">
  <a href="../../../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="../launch/README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="../test/README.md"><b>Forward &#8594;</b></a>
</p>

The helpers that run alongside `move_group`: one teaches MoveIt the workcell that the robot is bolted into, the other parks the arm when a servo gets hot.

## Files in this folder

| File | What it is | What it is for |
| :--- | :--- | :--- |
| `planning_scene.py` | `planning_scene_loader` node, run once per launch | Reads `config/planning_scene.yaml` and sends the bench, four walls and roof to `/apply_planning_scene` as a diff. It retries until the service is available, accepts `scene_file`, `remove` and `timeout_sec` parameters, and exits after reporting success or failure. With `remove:=true` it deletes the same objects. |
| `thermal_park.py` | `thermal_park` node, runs with `move_group` | Reads the servo temperatures the driver publishes on `/diagnostics` (`temperature_c/<joint>`, once a second). When one servo stays at `warn_c` (63 degC) for `hot_for_s` (1 s), publishes one trajectory on `/arm_controller/joint_trajectory` to the `rest` pose of the SRDF, at `speed_rad_s` (0.078). J1 and J6 stay where they are. **No collision check**: it skips MoveIt, whose workcell model refuses poses that are free on the bench; keep the path clear. Once at `rest` (or after the move time plus `arrive_margin_s`), calls `/safety_supervisor/stop` (`stop_after_park`): the arm holds its pose with torque and takes no goal until activated again, which the driver allows only under the cut minus 5. Without a supervisor it warns and the arm stays parked, active. One park per hot spell: again only after every servo went under `rearm_below_c` (60 degC). Off with `thermal_park:=false`. |

The launch file starts it automatically with `planning_scene:=true`. The same YAML drives the collision checks in the guard, so a goal refused in RViz is also refused on hardware.

Run it through ros2:

```bash
source install/setup.bash
ros2 run positronic_moveit_config planning_scene.py --ros-args -p remove:=true
```

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
