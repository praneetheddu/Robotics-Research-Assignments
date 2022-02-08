
# Adding a Camera to the Turtlebot3 Gazebo Model

***To enable the Turtlebot3 Gazebo model to simulate a camera, a few files must be changed and added. If you wreck your Turtlebot3 files or the Gazebo files associated with them, repull the `turtlebot3` directory if you pulled the github repository directly or reinstall the debian package to fix the error.***

***This should all be done on your computer as it is only meant for the simulator!***

## Altering the Turtlebot3 Burger Gazebo Model

Copy all the files in the gazeboModel directory you downloaded from this GIT repository.

If you used GIT or downloaded the Turtlebot3 repository directly into your catkin_ws, copy the provided paste the files into the following directoy,\
`~/catkin_ws/src/turtlebot3/turtlebot3_description/urdf`

Alternatively if you installed the turtlebot3 files using debian packages (e.g. you used the command `sudo apt-get install ros-melodic-turtlebot3` to install everything), paste the files into this directory,\
`/opt/ros/melodic/share/turtlebot3_description/urdf`

The file turtlebot3_burger.gazebo.xacro contains the simulated camera model parameters at the bottom of the file you may want to edit.
 
 **Camera FPS:** `<update_rate>30.0</update_rate>`  
 **Resolution:**  
`<image>`  
`<width>1280</width>`  
`<height>960</height>`  
`<format>R8G8B8</format>`  
`</image>`  
 **Node name:** `<cameraName>raspicam_node</cameraName>`  
**Topic name:** `<imageTopicName>image/compressed</imageTopicName>`  

You should now be all set to run the simulation environment containing a turtlebot with a simulated rpi_camera.

## Checking the Camera

For simulation, make sure your ~/.bashrc file is updated to consider your computer the rosmaster and host. This can be done by using your computer's IP address or replacing the IP address with `localhost`.

 1. run `roscore`
 2. run `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
> Gazebo should launch an environment that looks like a turtle with the turtlebot inside.

You're all set! You can run teleop to move the robot around and access the camera view using rqt_image_view.