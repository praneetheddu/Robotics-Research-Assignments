# Lab4 Navigate to goal
This is the lab4 package for Gatech ECE/CS7785.

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

