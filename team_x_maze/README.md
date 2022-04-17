# Lab6 Maze Challenge
This is the lab6 package for Gatech ECE/CS7785.

## Configuration
* (Please add your email into `./manifest.xml` and delete this line)
* Create gazebo environment by following the `etc/7785_Lab5_Gazebo_Files-master.zip/README.md`
## Map definition
  ----------
  |        |                 ^  front
  |        |                 |
  |   M    |      left   <-     ->   right
  |   A    |                 |
  |   P    |                 v  back
  |        |
  ----------

## Build & Run
* Copy package into workspace
```
cp -r team_x_maze ${YOUR_CATKIN_WORKSPACE}/src
```
* Build the workspace
```
cd catkin_ws && catkin_make
```
### Mapping
* Run the turtlebot simulation 
```
roslaunch team_x_maze mapping.launch
```
* Run the turtlebot3 robot to do the mapping
on the robot:
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
on your pc:
```
roslaunch team_x_maze mapping_real.launch
```

To save the map
```
rosrun map_server map_saver -f ~/catkin_ws/src/team_x_maze/map/sim_maze
rosrun map_server map_saver -f ~/catkin_ws/src/team_x_maze/map/maze
```
### Navigation
* Run the turtlebot simulation (default map name: map/sim_maze.yaml)
```
roslaunch team_x_maze navigation.launch sim:=true 
```
* Run on the turtlebot3 robot (default map name: map/maze.yaml)
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
roslaunch team_x_maze navigation.launch
```
* Run the lab6
```
roslaunch team_x_maze maze.launch

```

## Note

To publish a destination in CLI
```
 rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped  '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}'

```


