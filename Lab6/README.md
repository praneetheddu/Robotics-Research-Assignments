# Lab6 - Final Challenge: Maze Navigation
![Header Image](https://user-images.githubusercontent.com/69251304/164844511-468e37e7-05df-4af6-bb35-293ba6961cf1.png)
## Introduction
There are two parts to this lab:
<br/>**Part 1:** Image Classification <br/>
**Part 2:** Maze
## Part 1 Image Classification
### Background
There are several road signs located within the final maze which guides the robot to reach its destination. However, the robot needs to recognize what sign is displayed within its view. This part utilizes supervised learning and adopts KNN model with given dataset to develop a classification model to later use during the maze challenge. Our model predicts roadsigns with `98.3%` accuracy.

## Requirements: 
* set python 2.7 as interpreter
* opencv minimum version 4.2.0

## Testing Classifier
Run `python knn.py`
Confusion matrix will be printed in Terminal at the end of testing execution

## Part 2 Maze Navigation
### Background
We can now deploy our trained model within our application for sign classificaiton. Compressed image data is passed to classifier which is gathered by Raspi Camera. Robot initially starts in front of a sign with a given heading (which can be set within our application). Based on our current prediction, the way point is accumulated by reading LiDAR's distance data minus certain threshold distance (This is the distance that robot needs to maintain between itself and the wall. This value is chosen based on our testing). Eventually, the robot reaches the final destination if its able to classify properly and stay within the path.


## Requirements: 
* set python 2.7 as interpreter
* opencv minimum version 4.2.0

## Build & Run
* Copy package into workspace
```
cp -r team_x_maze ${YOUR_CATKIN_WORKSPACE}/src
```
* Build the workspace
```
cd catkin_ws && catkin_make
```

### navigation
* Run the turtlebot simulation (default map name: map/sim_maze.yaml)
```
roslaunch team_x_maze navigation.launch sim:=true 
```
* Run the turtlebot3 robot (default map name: map/maze.yaml)
```
roslaunch team_x_maze navigation.launch
```
* Run the lab6
```
roslaunch team_x_maze goal.launch heading:=${Robot's heading}

```

### Kidnapped Robot Case
In a scenario, where the robot couldn't identify a sign or is lost within the maze, we implement a scan search technique. This technique allows the robot to scan with 15 degree increments in both direction until its able to find a sign and adjust itself accordingly. The robot will then continue with its mission.