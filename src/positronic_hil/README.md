<a id="top"></a>
<p align="right">
  <img src="../../docs/assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# Source — positronic_hil

<!-- nav:auto -->
<p align="left">
  <a href="README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="test/README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="test/README.md"><b>Forward &#8594;</b></a>
</p>

Hardware-in-the-loop tests: the real driver on the Jetson against the real arm. They run only with `POSITRONIC_HIL=1`, started by `.github/workflows/hil.yml`; everywhere else they are skipped.

## Files

| File | What it is | What it is for |
| :--- | :--- | :--- |
| `package.xml` | Package manifest | Declares the test dependencies: `positronic_bringup`, `controller_manager_msgs`, `control_msgs`, `rclpy` and the rest. |
| `CMakeLists.txt` | Build file | Registers `test/test_hil.py` and `test/test_hil_trajectory.py` as the ctest tests `test_hil` and `test_hil_trajectory`, plus the ROS linters. |
| `test/` | The HIL tests | See [test/README.md](test/README.md). |

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
