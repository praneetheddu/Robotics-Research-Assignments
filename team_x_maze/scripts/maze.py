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
6. Check for target sign of completion ✔️
7. Develop a failsafe in-case robot gets lost ❌
8. Define the actions when detecting "not enter" and "stop" sign ❌
"""

# ROS 
import rospy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
from geometry_msgs.msg import Point
from team_x_maze.msg import Wall
from actionlib_msgs.msg import GoalStatusArray

# Python
import sys
import numpy as np
import knn
import cv2
from cv_bridge import CvBridge
import math
import time

###########################################
#            Global variables            ##
###########################################
sim = True
bridge = CvBridge()  # Bridge converts the image from ros to openCV
DEBUG = False
# DEBUG = True

# sign
sign_dict = {
    -1: "NA",
    0: "Wall",
    1: "Left",
    2: "Right",
    3: "Do Not Enter",
    4: "Stop",
    5: "Goal"
}
sign = 0
dist_to_wall = [0, 0, 0, 0]

# pos in map frame
cur_pos = [0, 0, 0]
cur_ang = 0
# heading define
#  ----------                 ^  front
#  |        |                 |
#  |   M    |      left   <-     ->   right
#  |   A    |                 |
#  |   P    |                 v  back
#  |        |
#  ----------
left_heading = [0, 0, 0.707, 0.707]
right_heading = [0, 0, -0.707, 0.707]
forward_heading = [0, 0, 0, 1]
backward_heading = [0, 0, 1, 0]

heading = [left_heading, forward_heading, right_heading, backward_heading]
cur_heading = 0  # left

# goal
pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
goal = PoseStamped()
goal_offset = 0.26
initial = True


"""
Check the robot arrival status
"""


def check_arrive():

    global cur_pos
    global cur_ang
    global goal

    ang = qua2rpy(heading[cur_heading])
    # check if arrive, please keep following parameters are the same as them in dwa_local_planner_params.yaml
    if abs(goal.pose.position.x - cur_pos[0]) < 0.10 and abs(goal.pose.position.y - cur_pos[1]) < 0.10 and (abs(
            ang - cur_ang) < 10 or abs(ang - cur_ang) > 350):  # degree (0-360)
        return True
    # rospy.loginfo("%f, %f, %f",abs(goal.pose.position.x - cur_pos[0]),abs(goal.pose.position.y - cur_pos[1]),abs(ang - cur_ang))
    return False


"""
set next goal in map frame
"""


def set_goal():
    global goal
    global heading
    global cur_heading
    global cur_pos
    global dist_to_wall

    if check_arrive() or initial:
        goal.header.frame_id = "map"
        rospy.loginfo("%s Sign Detected", sign_dict[sign])

        dist = dist_to_wall
        if sign_dict[sign] is "Goal":
            rospy.loginfo("Arrive Final Goal!")
        elif sign_dict[sign] is "Left":
            dist[1] -= goal_offset
            # set goal position
            if cur_heading == 0 or cur_heading == 2:  # left/right
                goal.pose.position.x = cur_pos[0] + dist[1] if cur_heading == 2 else cur_pos[0] - dist[1]
                goal.pose.position.y = cur_pos[1]
            else:  # forward/backward
                goal.pose.position.y = cur_pos[1] + dist[1] if cur_heading == 1 else cur_pos[1] - dist[1]
                goal.pose.position.x = cur_pos[0]

            # set goal heading direction
            cur_heading = 3 if cur_heading - 1 < 0 else cur_heading - 1
            # goal.pose.orientation = heading[cur_heading]

        elif sign_dict[sign] is "Right":
            dist[2] -= goal_offset
            # set goal position
            if cur_heading == 0 or cur_heading == 2:  # left/right
                goal.pose.position.x = cur_pos[0] + dist[2] if cur_heading == 0 else cur_pos[0] - dist[2]
                goal.pose.position.y = cur_pos[1]
            else:  # forward/backward
                goal.pose.position.y = cur_pos[1] + dist[2] if cur_heading == 3 else cur_pos[1] - dist[2]
                goal.pose.position.x = cur_pos[0]

            # set goal heading direction
            cur_heading = 0 if cur_heading + 1 > 3 else cur_heading + 1
            # goal.pose.orientation = heading[cur_heading]
        elif sign_dict[sign] is ("Do Not Enter" or "Stop"):
            goal.pose.position.x = cur_pos[0]
            goal.pose.position.y = cur_pos[1]

        goal.pose.orientation.x = heading[cur_heading][0]
        goal.pose.orientation.y = heading[cur_heading][1]
        goal.pose.orientation.z = heading[cur_heading][2]
        goal.pose.orientation.w = heading[cur_heading][3]

        rospy.loginfo("New Goal: (%f, %f), (%f,%f,%f,%f)", goal.pose.position.x, goal.pose.position.y,
                      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
                      goal.pose.orientation.w)


"""
Publish goal in 1HZ
"""


def pub_goal():
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        set_goal()
        pub.publish(goal)
        rate.sleep()


""" Get distance to wall """


def obs_callback(msg):
    # msg = Wall()
    # float32 front
    # float32 left
    # float32 right
    # float32 back

    global dist_to_wall
    dist_to_wall[0] = msg.front
    dist_to_wall[1] = msg.left
    dist_to_wall[2] = msg.right
    dist_to_wall[3] = msg.back
    for i in range(4):
        if dist_to_wall[i] > 9999:
            dist_to_wall[i] = 0
    # if DEBUG:
    # rospy.loginfo_throttle(1, "Distance to wall = %2.2f m", dist_to_wall[2])


""" Get compressed image and predict sign """


def predict_image(CompressedImage):
    global sign
    imgBGR = bridge.compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
    # Predict sign here
    sign = knn.predict(imgBGR, debug=DEBUG)
    # if DEBUG:
    # rospy.loginfo_throttle(1, "Sign predicted = %s", sign_dict[sign])


""" quaternion to euler angles """


def qua2rpy(qua):
    x = qua[0]
    y = qua[1]
    z = qua[2]
    w = qua[3]
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    # left -- yaw = 180-360
    # right -- yaw = 0-180
    yaw = y * 180 / math.pi
    if yaw < 0:
        yaw = yaw + 360

    return yaw


""" get odometry data """


def odom_callback(msg):

    global cur_pos
    global cur_ang
    global initial
    rospy.sleep(0.5)
    # just run once because we cannot get robot initial position in the map frame
    if initial:
        cur_pos[0] = (msg.pose.pose.position.x)
        cur_pos[1] = (msg.pose.pose.position.y)
        cur_pos[2] = (msg.pose.pose.position.z)

        odom_qua = [0, 0, 0, 0]
        odom_qua[0] = (msg.pose.pose.orientation.x)
        odom_qua[1] = (msg.pose.pose.orientation.y)
        odom_qua[2] = (msg.pose.pose.orientation.z)
        odom_qua[3] = (msg.pose.pose.orientation.w)
        cur_ang = qua2rpy(odom_qua)

        set_goal()
        initial = False


""" update pose data in map frame"""


def map_callback(msg):

    global sign
    global cur_pos
    global cur_ang
    cur_pos[0] = (msg.feedback.base_position.pose.position.x)
    cur_pos[1] = (msg.feedback.base_position.pose.position.y)
    cur_pos[2] = (msg.feedback.base_position.pose.position.z)

    map_qua = [0, 0, 0, 0]
    map_qua[0] = (msg.feedback.base_position.pose.orientation.x)
    map_qua[1] = (msg.feedback.base_position.pose.orientation.y)
    map_qua[2] = (msg.feedback.base_position.pose.orientation.z)
    map_qua[3] = (msg.feedback.base_position.pose.orientation.w)
    cur_ang = qua2rpy(map_qua)


def init():

    if sim:
        rospy.Subscriber("/turtlebot_burger/camera/image_raw/compressed", CompressedImage, predict_image, queue_size=1,
                         buff_size=2 ** 24)
    else:
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, predict_image, queue_size=1,
                         buff_size=2 ** 24)
    rospy.Subscriber("/object_pos", Wall, obs_callback)
    rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, map_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.init_node('navigate_maze', anonymous=True)
    pub_goal()
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
