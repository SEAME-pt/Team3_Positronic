<a id="top"></a>
<p align="right">
  <img src="../../../docs/assets/brand/logo.png" width="460" alt="Team Positronic — myCobot 280 Jetson Nano">
</p>

# Source — positronic_hil — test

<!-- nav:auto -->
<p align="left">
  <a href="../README.md"><b>Home</b></a> &nbsp;|&nbsp;
  <a href="../README.md"><b>&#8592; Back</b></a> &nbsp;|&nbsp;
  <a href="../README.md"><b>Forward &#8594;</b></a>
</p>

The pytest suite on the real arm. Each test file starts `positronic_bringup/driver.launch.py`, runs, and stops it. The servos are powered (`power_on` at every activate) and receive commands, but every target is the pose the arm already has, so no joint should move; a test fails if one moves more than 0.5 deg. Keep the arm's reach clear while they run.

## Files

| File | What it is | What it is for |
| :--- | :--- | :--- |
| `conftest.py` | Shared fixture and helpers | Skips every test without `POSITRONIC_HIL=1`, starts and stops the driver, writes its output to `<test>.driver.log` next to the JUnit XML, and gives the tests a `Probe` for `/joint_states` and the `controller_manager` services. |
| `test_hil.py` | Driver checks | `test_read_and_hold`: 60 s of `/joint_states` at 28 Hz or more, no gap over 0.2 s, no NaN, drift under 0.5 deg. `test_stop_and_resume`: three stop/resume cycles in the supervisor order, then drift under 0.5 deg. |
| `test_hil_trajectory.py` | MoveIt path check | `test_hold_trajectory`: a `FollowJointTrajectory` goal on the current pose through `arm_controller` ends `SUCCESSFUL` and drift stays under 0.5 deg. |

<!-- nav:auto -->
<p align="left">
  <a href="#top"><b>Top</b></a>
</p>
