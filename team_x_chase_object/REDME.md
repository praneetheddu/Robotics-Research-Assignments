# Lab3 Object Follower
This is the lab3 package for Gatech ECE/CS7785.

## Configuration
* (Please add your email into `./manifest.xml` and delete this line)


## Build & Run
* Copy package into workspace
```
cp -r team_x_chase_object ${YOUR_CATKIN_WORKSPACE}/src
```
* Build the workspace
```
cd catkin_ws && catkin_make
```
* Run the launch file for simulation
```
roslaunch team_x_chase_object object_chase.launch sim:=true
```
* Run the launch file for real robot
```
roslaunch team_x_chase_object object_chase.launch
```

