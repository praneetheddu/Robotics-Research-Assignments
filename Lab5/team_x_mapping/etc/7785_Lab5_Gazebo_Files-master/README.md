# Lab 5 and Final Project Maze

***This should all be done on your computer as it is only meant for the simulator!***

## Adding a Model of the Maze from Lab 5 to your Gazebo Environment

 1. Copy all of the folders in the `mazeFiles` directory you downloaded.
 2. Paste them into the directory `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models`
 3. Copy `turtlebot_maze.launch` file in the `mazeFiles` directory you downloaded.
 4. Paste it into the directory `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch`
 5. Copy `7785maze.world` file in the `mazeFiles` directory you downloaded.
 6. Paste it into the directory `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds`
 7. Open the command terminal and navigate to the directory `~/catkin_ws`
 8. run `catkin_make`

## Launching the Envinroment

 1. roscore
> Note: Make sure your ~/.bashrc file is updated to consider your computer the rosmaster and host.
 2. run `roslaunch turtlebot3_gazebo turtlebot3_maze.launch`

Gazebo should launch with a maze environment similar to lab 5 and the final with the turtlebot inside.
