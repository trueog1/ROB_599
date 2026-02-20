# Olivia Gehrke HW3
This is the ROB 599 Homework 3 code for Olivia Gehrke.
## General Process
To run this code certain steps need to occur in a specific order.

First the map code has to be launched. This involves opening 3 terminals and running the stage code, the controller code, and the map code. Once the map is completed kill all 3 nodes to save the data. The images can be found in the maps folder.

Then run the path planning node in one terminal. Kill the node once a message about the saved waypoints yaml is shown in the terminal. 

Then change the waypoints yaml file in the controller code to waypoint.yaml and then open 2 terminals. Run stage in one terminal and the controller in a second one. 

## Launching Worlds
There are two different launch commands based on the world used. The simple world is recommened for this homework as it has more obstacles to avoid.

### Open World Launch
```
ros2 launch hw3 hw3_stage.launch.py world:=open config:=open
```

### Building Interior World Launch
```
ros2 launch hw3 hw3_stage.launch.py world:=simple_inside config:=simple_inside
```

## Launching Controller
The controller will need to be launched in a separate terminal using the command line below. The waypoints are prewritten in the waypoint1.yaml file within the hw2 folder.
```
ros2 launch hw3 controller3.launch.py
```
## Launching the Map Generation Software
To generate the map, run the line below in your terminal. 
```
ros2 launch hw3 map.launch.py
```
## Launching the Path Planner
To generate the path, run the line below the in a terminal. 
```
ros2 launch hw3 pathplan.launch.py
```

## Thanks
I like to thank all people behind [Stage](https://github.com/rtv/Stage) and [ros-simulation/stage_ros](https://github.com/ros-simulation) for the simulation and the first ROS bridge as well as the people for the initial ROS2 implementations [ShengliangD/stage_ros2](https://github.com/ShengliangD/stage_ros2) and [n0nzzz/stage_ros2](https://github.com/n0nzzz/stage_ros2). The work was used as base for my implementation. Which differs to the previous ROS interfaces in its modularity.
