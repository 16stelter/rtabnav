# RTABNav

This project sets up a Nav2 navigation stack for Ros2 Jazzy using a RTABMap obstacle map as a global costmap. 
The setup has been tested on a Unitree Go2, as well as a Gazebo simulation of the LEO rover. Configs for both scenarios are included.

## How to use

To use this project, first build it with colcon. If you are using a real robot, make sure that it can communicate with your device (unless you are running the entire stack on the robot). RTABNav requires odometry and pointcloud data as inputs and sends cmd_vel as outputs. To start the stack, execute

```
ros2 launch rtabnav rtabnav.launch.py 
```

You can provide the following parameters:

* params_file: path to the parameter file to be loaded
* remap_file: path to the file specifying necessary topic remappings
* rviz_config_file: path to the rviz config to be loaded
* namespace: the namespace of the robot 
* use_sim_time: whether the project runs in simulation
