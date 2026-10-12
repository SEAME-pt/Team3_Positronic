## The models
- There are 8 different models available
- They can be without the base or with the base
```
- Regular model
    280_jn
    280_jn_no_base

- Model with camera
    280_jn_camera
    280_jn_camera_no_base

- Model with gripper
    280_jn_gripper
    280_jn_gripper_no_base

- Model with gripper and camera
    280_jn_gripper_camera
    280_jn_gripper_camera_no_base
```

## Installation
- place mycobot_description inside the src directory
- ros2_ws can be any name you choose
```
mkdir -p ros2_ws/src
mv mycobot_description ~/ros2_ws/src
```

- colcon build is always executed from the root of the directory
- build the package and source the workspace
```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## How to use the model
### Gazebo
- Terminal 1
```
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix mycobot_description)/share:$GZ_SIM_RESOURCE_PATH
gz sim -v 4 -r empty.sdf
```

- Terminal 2
```
ros2 run ros_gz_sim create -file $(ros2 pkg prefix mycobot_description)/share/mycobot_description/urdf/mycobot_280_jn_sim/mycobot_280_jn.urdf -name mycobot 
```

### RViz
```
ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix mycobot_description)/share/mycobot_description/urdf/mycobot_280_jn_sim/mycobot_280_jn.urdf
```

### RViz extra adjustments
- Expand the `Global Options` tab
- In `Fixed Frame`, select `world`

- Expand Robot Model
- Change `Alpha` to 1

- Disable TF to see easier

