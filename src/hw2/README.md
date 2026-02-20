# Olivia Gehrke HW2
This is the ROB 599 Homework 2 code for Olivia Gehrke. The obstacle avoidance being used is Artificial Potential Field.

## Launching Worlds
There are two different launch commands based on the world used.

### Open World Launch
```
ros2 launch hw2 hw2_stage.launch.py world:=open config:= open
```

### Building Interior World Launch
```
ros2 launch hw2 hw2_stage.launch.py world:=simple_inside config:=simple_inside
```

## Launching Controller
The controller will need to be launched in a separate terminal using the command line below. The waypoints are prewritten in the waypoint1.yaml file within the hw2 folder.
```
ros2 launch hw2 controller.launch.py
```

## Thanks
I like to thank all people behind [Stage](https://github.com/rtv/Stage) and [ros-simulation/stage_ros](https://github.com/ros-simulation) for the simulation and the first ROS bridge as well as the people for the initial ROS2 implementations [ShengliangD/stage_ros2](https://github.com/ShengliangD/stage_ros2) and [n0nzzz/stage_ros2](https://github.com/n0nzzz/stage_ros2). The work was used as base for my implementation. Which differs to the previous ROS interfaces in its modularity.
