## How to run
- You will need 5 terminals inside the dev container
- Enter the docker container `docker exec -it <container_name> bash`
- `make build` and `source install/setup.bash`
- Source every terminal
- Set `export ROS_DOMAIN_ID=<number>` in every terminal. So teammates nodes don't interfere.
- Small fix on the display if RViz fail's to open. Exit with flag -6.
```
- Host machine:
    xhost +local:docker

- on container:
    echo $DISPLAY

- if empty. Check $DISPLAY on host (usually :0 or :1)
- export that in the container `export DISPLAY=:1`
```

## Heartbeat Table
```
0 IDLE
1 INACTIVE
2 MOVING
3 STOPPED
4 ERROR
```

## RViz Visualization
- The grey color is the current state of the arm, read from /joint_states. Always matches gazebo.
- The orange robot is the goal state of RViz own planning panel. We don't use that for the mission node, we talk to move_group directly.
- To hide it: Displays-> MotionPlanning -> Planning Request -> uncheck Query Goal State.
- To hide the TF Arrows: Displays -> uncheck TF.

### Terminal 1
- Starts gazebo + rviz + moveit
  `ros2 launch positronic_moveit_config gazebo_moveit.launch.py`

### Terminal 2
- Starts the mission_node from the launch script
`ros2 launch positronic_bringup mission.launch.py use_sim_time:=true`

### Terminal 3
- Activate the mission node
`ros2 lifecycle set /mission_node configure`
`ros2 lifecycle set /mission_node activate`

- Set it to listen to the heartbeat topic
`ros2 topic echo /mission/heartbeat --qos-reliability best_effort --field state`

### Terminal 4
- This is used to send pose goals to the mission_node
- Below are some examples of poses and their expected results
- The expected codes are shown in this terminal after the action ended

---

#### Case 1. Send a goal to pose A and shows the feedback.
##### Expected code 0 and success true.
```
ros2 action send_goal --feedback /mission/go_to_pose positronic_interfaces/action/GoToPose \
"{target_pose: {header: {frame_id: g_base}, pose: {
position: {x: 0.1683, y: -0.0428, z: 0.3764},
orientation: {x: -0.43534, y: 0.69380, z: -0.01436, w: 0.57352}}}}"
```
#### Case 2. Goal to pose B. Midway, run the cancel command from terminal 5.
##### Expected result CANCELED, code 3.
```
ros2 action send_goal --feedback /mission/go_to_pose positronic_interfaces/action/GoToPose \
"{target_pose: {header: {frame_id: g_base}, pose: {
position: {x: 0.1479, y: 0.0909, z: 0.3764},
orientation: {x: -0.67115, y: 0.46950, z: 0.21012, w: 0.53383}}}}"
```

#### Reset the Arm, because it stays in STOPPED State.
```
ros2 service call /mission/reset positronic_interfaces/srv/StateTrigger "{}"
```

#### Case 3. Goal to pose C. From terminal 5 send a stop signal midway.
##### Expected result ABORTED, code 4.
```
ros2 action send_goal --feedback /mission/go_to_pose positronic_interfaces/action/GoToPose \
"{target_pose: {header: {frame_id: g_base}, pose: {
position: {x: 0.0865, y: -0.1505, z: 0.3764},
orientation: {x: -0.13080, y: 0.80856, z: -0.23656, w: 0.52265}}}}"
```

#### Stop command. Terminal 5
```
ros2 service call /mission/stop positronic_interfaces/srv/StateTrigger "{}"
```

#### Reset the Arm, because it stays in STOPPED State.
```
ros2 service call /mission/reset positronic_interfaces/srv/StateTrigger "{}"
```

#### Case 4. Goal to pose B on terminal 4, right after, goal to pose A on terminal 5.
```
Case 2 command. Terminal 4

then

Case 1 command. Terminal 5
```
##### Second goal REJECTED

#### Case 5. Goal to pose A with a modified z value (1.0 meter). Out of reach. Planning fails.
##### Expected code 1. Arm stays still and in IDLE State. No reset needed.
```
ros2 action send_goal --feedback /mission/go_to_pose positronic_interfaces/action/GoToPose \
"{target_pose: {header: {frame_id: g_base}, pose: {
position: {x: 0.1683, y: -0.0428, z: 1.0},
orientation: {x: -0.43534, y: 0.69380, z: -0.01436, w: 0.57352}}}}"
```

#### Case 6. Disable the node and send a goal to pose A. Needs to be Rejected. Activate the node after.
##### Heartbeat Stops while Inactive.
```
ros2 lifecycle set /mission_node deactivate

ros2 action send_goal --feedback /mission/go_to_pose positronic_interfaces/action/GoToPose \
"{target_pose: {header: {frame_id: g_base}, pose: {
position: {x: 0.1683, y: -0.0428, z: 0.3764},
orientation: {x: -0.43534, y: 0.69380, z: -0.01436, w: 0.57352}}}}"

ros2 lifecycle set /mission_node activate
```

### Terminal 5
- It's the control terminal
- Cancel, Emergency stop, Reset the node

- This sends a signal to the cancel service
`ros2 service call /mission/go_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"`

- This sends a signal to the emergency stop service
`ros2 service call /mission/stop positronic_interfaces/srv/StateTrigger "{}"`

- This resets the node to IDLE state after an ERROR or STOPPED
`ros2 service call /mission/reset positronic_interfaces/srv/StateTrigger "{}"`
