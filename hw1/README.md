# Homework 1

## Description

Write a node that gets pose of one turtle and makes another turtle moves to it

## Usage 

#### With launch file

```
source devel/setup.bash
roslaunch turtle_commander run_turtles.launch
```
#### OR by using following commands
```
rosrun turtlesim turtlesim_node
rosservice call /spawn "{x: 2.0, y: 2.0, theta: 2.0, name: 'turtle_chaser'}"
rosrun turtlesim turtle_teleop_key
source devel/setup.bash
rosrun turtle_commander chasing.py
```
