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

The one helper that runs alongside `move_group`: it teaches MoveIt the workcell that the robot is bolted into.

## Files in this folder

| File | What it is | What it is for |
| :--- | :--- | :--- |
| `planning_scene.py` | `planning_scene_loader` node, run once per launch | Reads `config/planning_scene.yaml` and sends the bench, four walls and roof to `/apply_planning_scene` as a diff. It retries until the service is available, accepts `scene_file`, `remove` and `timeout_sec` parameters, and exits after reporting success or failure. With `remove:=true` it deletes the same objects. |

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
