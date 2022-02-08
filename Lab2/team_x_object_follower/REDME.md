# Lab2 Object Follower
This is the lab2 package for Gatech ECE/CS7785.

## Configuration
* (Please add your email into `./manifest.xml` and delete this line)
* Modify the burger model by following the `../../Auxiliary/turtlebot3_sim_update-master.zip/README.md`


## Build & Run
* Copy package into workspace
```
cp -r team_x_object_follower ${YOUR_CATKIN_WORKSPACE}/src
```
* Build the workspace
```
cd catkin_ws && catkin_make
```
* Run the turtlebot simulation 
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
```
* Run the real robot
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
```
* Run the launch file to run our codes
```
roslaunch team_x_object_follower object_follow.launch
```
* If everything is good, the robot will turn right and left alternately


## Note
* Current code is a demo that doesn't use any object detect algorithm, instead, I use a sin function to simulate different object positions.
* My PC met some problems when using cv-bridge in python3, so I change the python version to 2.7 for current code
* The default python version for ROS is python2, if you want to run the node in python3, please config your `catkin` with `catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m`
