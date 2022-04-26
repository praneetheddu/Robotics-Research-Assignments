#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-5 Navigate to goals
# Yinuo Wang, Praneeth Erwin Eddu
# Mar 23, 2022


# todo: Current publish goals in "odom" frame, not sure if we must publish in "map" frame



#  ros
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback
#
import numpy as np
import math
import time

###################################
## VARIABLE DECLARATION AND SETUP
###################################
# way points
wp_list = []
wp_num = 0
wp_idx = 0

# odometry
odom_pos = [0,0,0]
odom_ang =  0
timestamp = 0

# map
map_pos = [0,0,0]
map_ang = 0


pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
goal = PoseStamped()
start_time = time.time()
"""
quaternion to euler angles
"""


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
    # global odom_pos
    # global odom_ang
    # global goal
    #
    # odom_pos[0] = (msg.pose.pose.position.x)
    # odom_pos[1] = (msg.pose.pose.position.y)
    # odom_pos[2] = (msg.pose.pose.position.z)
    #
    # odom_qua = [0,0,0,0]
    # odom_qua[0] = (msg.pose.pose.orientation.x)
    # odom_qua[1] = (msg.pose.pose.orientation.y)
    # odom_qua[2] = (msg.pose.pose.orientation.z)
    # odom_qua[3] = (msg.pose.pose.orientation.w)
    # odom_ang = qua2rpy(odom_qua)

    update_goal()

""" update pose data in map frame"""


def map_callback(msg):
    global map_pos
    global map_ang
    global goal

    map_pos[0] = (msg.feedback.base_position.pose.position.x)
    map_pos[1] = (msg.feedback.base_position.pose.position.y)
    map_pos[2] = (msg.feedback.base_position.pose.position.z)

    map_qua = [0,0,0,0]
    map_qua[0] = (msg.feedback.base_position.pose.orientation.x)
    map_qua[1] = (msg.feedback.base_position.pose.orientation.y)
    map_qua[2] = (msg.feedback.base_position.pose.orientation.z)
    map_qua[3] = (msg.feedback.base_position.pose.orientation.w)
    map_ang = qua2rpy(map_qua)

    # update_goal()

""" 
get waypoint goal 
"""


def get_waypoints():
    global wp_list
    global wp_num
    # Uncomment line below when running on robot
    with open('/home/allen/catkin_ws/src/team_x_mapping/global_waypoints.txt', 'r') as f:
    # with open('/home/allen/catkin_ws/src/team_x_mapping/global-wp-2.txt', 'r') as f:
    # with open('/home/pran/catkin_ws/src/Robotics-Research-Assignments/team_x_mapping/global_waypoints.txt', 'r') as f:
        lines = f.readlines()
        f.close()
    wp_num = len(lines)
    for i in range(wp_num):
        wp = lines[i].split(", ")
        for j in range(len(wp)):
            wp[j] = float(wp[j])
        wp_list.append(wp)




"""
Check the robot arrival status
"""

def check_arrive():
    global wp_list
    global wp_idx
    global wp_num
    global odom_pos
    global odom_ang
    global map_pos
    global map_ang

    current_pos = map_pos
    current_ang = map_ang

    if wp_idx >= wp_num:
        return False
    # compute the yaw angular
    ang = qua2rpy([wp_list[wp_idx][3],wp_list[wp_idx][4],wp_list[wp_idx][5],wp_list[wp_idx][6]])

    # check if arrive, please keep following parameters are the same as them in dwa_local_planner_params.yaml
    if  abs(wp_list[wp_idx][0] - current_pos[0]) < 0.10 and \
        abs(wp_list[wp_idx][1] - current_pos[1]) < 0.10 and \
        abs(wp_list[wp_idx][2] - current_pos[2]) < 0.03 and \
        abs(ang - current_ang) < 4: # degree (0-360)

        return True

    return False

"""
Update way point goal
"""

def update_goal():
    global wp_idx
    global wp_list
    global wp_num
    global start_time

    if check_arrive() is True:
        rospy.loginfo("Goal %d has arrived", wp_idx)
        # update way point
        start_time = time.time()       
        wp_idx = wp_idx + 1
        if wp_idx >= wp_num:
            rospy.loginfo("TASK FINISHED! NO MORE WAY POINTS.")

    #  if still have way points in the list
    if wp_idx < wp_num:
        # update goal
        if (time.time() - start_time > 2):
            rospy.loginfo_throttle(3, "Adding new goal!")
            goal.header.frame_id = "map"
            # goal.header.frame_id = "odom"
            goal.pose.position.x = wp_list[wp_idx][0]
            goal.pose.position.y = wp_list[wp_idx][1]
            # goal.pos.position.z = wp_list[wp_idx][2]
            goal.pose.orientation.x = wp_list[wp_idx][3]
            goal.pose.orientation.y = wp_list[wp_idx][4]
            goal.pose.orientation.z = wp_list[wp_idx][5]
            goal.pose.orientation.w = wp_list[wp_idx][6]
        else:
            rospy.loginfo_throttle(1, "Waiting for 2 seconds!")


"""
Publish goal in 1HZ
"""
def pub_goal():
    global goal
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(goal)
        rate.sleep()




def init():
    # get way points
    get_waypoints()
    # print(wp_list)
    # ROS Node
    rospy.init_node('go_to_goal', anonymous=True)

    sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    sub2 = rospy.Subscriber('/move_base/feedback',MoveBaseActionFeedback, map_callback)
    # sub = rospy.Subscriber('/move_base/status', GoalStatusArray, check_arrive)
    pub_goal()

    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
