#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Final Lab Demo
Integrate Multi-modal sensing and Navigation into robot 
to complete a maze by following signs

Author: Yinuo Wang, Praneeth Eddu
Date: 04/14/2022
'''
"""
TODO(✔️ ❌) :  
1. Integrate KNN model into ROS to classify signs using Raspi camera ❌
2. Integrate KNN model into ROS to classify signs using Sim camera ✔️ 
3. Use LiDAR to determine distance to walls ✔️
4. Leverage distance and signage data to publish goals using move_base simple goal ❌
5. Integrate camera in simulation environment ✔️
6. Check for target sign of completion ❌
7. Develop a failsafe in-case robot gets lost ❌
"""

# ROS 
import rospy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
from geometry_msgs.msg import Point

# Python
import sys
import numpy as np
import knn 
import cv2
from cv_bridge import CvBridge

# Global variables
sim = True
bridge = CvBridge() # Bridge converts the image from ros to openCV
DEBUG = False

sign_dict = {
    -1: "NA",
    0: "wall",
    1: "left", 
    2: "right", 
    3: "do not enter", 
    4: "stop", 
    5: "goal"
}
sign = -1
dist_to_wall = 0

"Get distance to wall"
def obs_callback(msg):
    global dist_to_wall
    dist_to_wall = msg.x
    if (dist_to_wall > 200):
        dist_to_wall = 0
    #if DEBUG:
    rospy.loginfo_throttle(1, "Distance to wall = %2.2f m", dist_to_wall)

"Get compressed image and predict sign"
def predict_image(CompressedImage):
    imgBGR = bridge.compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
    # Predict sign here
    sign = knn.predict(imgBGR, debug=DEBUG)
    if DEBUG:
        rospy.loginfo_throttle(1, "Sign predicted = %s", sign_dict[sign])


def init():
    rospy.loginfo("CV Version = %s", cv2. __version__)
    rospy.loginfo("Python Version = %s", sys.version)
    if sim:
        rospy.Subscriber("/turtlebot_burger/camera/image_raw/compressed", CompressedImage, predict_image, queue_size=1, buff_size=2**24)
    else:
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, predict_image, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/object_pos", Point, obs_callback)
    rospy.init_node('navigate_maze', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
