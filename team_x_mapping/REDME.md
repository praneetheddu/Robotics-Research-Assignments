# Lab5 Mapping Navigation
This is the lab5 package for Gatech ECE/CS7785.

## Configuration
* (Please add your email into `./manifest.xml` and delete this line)
* Create gazebo environment by following the `etc/7785_Lab5_Gazebo_Files-master.zip/README.md`


## Build & Run
* Copy package into workspace
```
cp -r team_x_mapping ${YOUR_CATKIN_WORKSPACE}/src
```
* Build the workspace
```
cd catkin_ws && catkin_make
```
### mapping
* Run the turtlebot simulation 
```
roslaunch team_x_mapping mapping.launch sim:=true 
```
* Run the turtlebot3 robot
```
roslaunch team_x_mapping mapping.launch
```
### navigation
* Run the turtlebot simulation (default map name: map/map.yaml)
```
roslaunch team_x_mapping navigation.launch sim:=true 
```
* Run the turtlebot3 robot
```
roslaunch team_x_mapping navigation.launch
```


## Note

