# Lab4 - Navigate to Goal
## Introduction
The purpose of this lab is to navigate to pre-defined waypoints by introducing Dead Reckoning, Path Following, Obstacle Avoidance and leveraging our past knowledge about Blending Controllers. 

## Configuration
* (Please add your email into `./manifest.xml` and delete this line)


## Build & Run
* Copy package into workspace
```
cp -r team_x_navigate_to_goal ${YOUR_CATKIN_WORKSPACE}/src
```
* Build the workspace
```
cd catkin_ws && catkin_make
```
* Run the launch file for turtlebot initialization
```
roslaunch team_x_navigate_to_goal turtlebot.launch
```
* Run the launch file for lab4
```
roslaunch team_x_navigate_to_goal navigate.launch
```
## Demo
### [Simulation](https://youtu.be/aUl2uStMr-A)

### [Physical World](https://youtu.be/pzAxc8eWV7s)
