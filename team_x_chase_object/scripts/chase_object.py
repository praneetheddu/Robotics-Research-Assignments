#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-3 Publish the robot desired linear and angular velocity cmd to low level motors
# Yinuo Wang, Praneeth Erwin Eddu
# Feb 03, 2022

#  ros
import math

import rospy
# from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import time
import numpy as np

###################################
## VARIABLE DECLARATION AND SETUP
###################################
MAX_SPEED = 1.5
MAX_LINEAR_SPEED = 0.1
MIN_SPEED = 0.15
BETA = 1.35
# Linear Distance Config
desire_dis = 0.25
kp_dis = 0.6
ki_dis = 0.1
kd_dis = 0.05
total_dis = 0
pre_dis = 0
dt = 0.01
dis_x = 0
dis_y = 0

# Angular Distance Config
desired_ang = 0
desired_ang_window = 0.03
kp_ang = 2.8  #4.0
ki_ang = 0.15
kd_ang = 0.5
total_ang = 0
pre_ang = 0
ang = 0
ranges = []
# Object Config
ball_found = False

# LiDAR config
window = 6
TOLERANCE_THRESHOLD = 0.05

# Other
current_time = time.time()
"""
Use appropriate window
"""
def set_window_size(angle):
    window_size = 1
    if abs(angle) >= 0 and abs(angle) < 8:
        window_size = 5
    elif abs(angle) >= 8 and abs(angle) < 16:
        window_size = 3
    elif abs(angle) >= 16: 
        window_size = 2
    return window_size

"""
subscribe the object position and update the commands
"""
def pos_callback(data):
    rate = rospy.Rate(10)  # 10hz
    global ang
    global ball_found

    if data.x > 1200 or data.y > 1200:
        # Ball is not detected
        ball_found = False
        ang = 0
    else:
        ball_found = True
        ang = (data.x - 180) / 360
        # ang = 0 # Uncomment to distance controller only
        rospy.loginfo_throttle(2.3, "Ball's location = %2.2f degrees", ang)

"""
Use LiDAR readings for PID control
"""


def lidar_callback(msg):
    global window
    global dis_x
    global ball_found
    global desire_dis
    global ang
    global BETA
    # ball_found = True # Uncomment to measure distance only
    if not ball_found:
        # If no ball is detected, set distance to  0
        dis_x = desire_dis
        # pid_controller(stop=True)
        pass
    else:
        if (any(msg.ranges) < 2):

            left = msg.ranges[0: 179]
            right = msg.ranges[180: 359]
            front = right + left
            # print(front)
            # dis_x = (sum(msg.ranges[len(msg.ranges) - (window // 2):len(msg.ranges)]) + sum(msg.ranges[: window // 2])) / window
            index = int(ang/3.14159*180)
            '''
            dis_x = sum(front[179+index-5: 179+index+5]) / 11
            '''
            # ----------------- New Code below ----------------------------
            if abs(index) > 12:
                BETA = 0.7
            else:
                BETA = 1.35 
            thresh = set_window_size(index)
            dis_x = sum(front[179+index-thresh: 179+index+thresh]) / (2*thresh + 1)
            rospy.loginfo_throttle(1.0, "index = %2.2f degrees, distance = %2.2f m", index, dis_x)
            # dis_x = (msg.ranges[0] + msg.ranges[1] + msg.ranges[359]) / 3
            # try to filt wrong recoganization
            # ------------------------------------------------------------ 
            if dis_x > 1.0 or dis_x is 0.0:
                dis_x = desire_dis
                ball_found = False
                rospy.loginfo("no ball found")
            else:
                rospy.loginfo_throttle(0.5, "Distance = %2.2f m", dis_x)



def pid_controller(stop=False):
    # prepare the commands
    cmd = Twist()

    if stop:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return
    global ball_found
    global dis_x
    global dis_y
    global ang
    global total_dis
    global pre_dis
    global total_ang
    global pre_ang
    global MAX_SPEED
    global desire_dis
    global desired_ang
    global desired_ang_window
    global kp_dis
    global kd_dis
    global kp_ang
    global kd_ang
    global MIN_SPEED
    global MAX_LINEAR_SPEED
    global BETA

    # set publish rate (control frequency)
    rate = rospy.Rate(100)  # 100hz
    while not rospy.is_shutdown():
        if not ball_found:
            cmd.linear.x = 0
            cmd.linear.y = 0
            cmd.angular.z = 0
            rospy.loginfo("no ball found")
        else:
            # distance pid
            # dis = math.sqrt(pow(dis_x, 2) + pow(dis_y, 2))
            diff_dis = dis_x - desire_dis
            total_dis = total_dis + (diff_dis * dt)
            P = (kp_dis * diff_dis)
            I = (ki_dis * total_dis)
            D = kd_dis * ((diff_dis - pre_dis) / dt)
            v = P + D
            pre_dis = diff_dis
            
            # rospy.loginfo_throttle(1, " vel-cmd %f,  object_dis %f", v, dis_x)

            # angular pid
            diff_ang = desired_ang - ang
            if abs(diff_ang) <= desired_ang_window:
                w = 0
            else:
                total_ang += diff_ang * dt
                P = kp_ang * diff_ang
                I = ki_ang * total_ang
                D = kd_ang * ((diff_ang - pre_ang) / dt)
                w = P + D
                pre_ang = diff_ang
            # rospy.loginfo_throttle(3, " ang-cmd %f,  object_ang %f", w, ang)

            # limit the cmd range
            if v > 0:
                cmd.linear.x = min(MAX_LINEAR_SPEED, v)
            else:
                cmd.linear.x = max(-MAX_LINEAR_SPEED, v) * BETA
            if w >= 0:
                cmd.angular.z = min(MAX_SPEED, w)
            else:
                cmd.angular.z = max(-MAX_SPEED, w)
            rospy.loginfo_throttle(1, " vel-cmd %f,  object_dis %f", cmd.linear.x, dis_x)
            rospy.loginfo_throttle(3, " ang-cmd %f,  object_ang %f", cmd.angular.z, ang)

        # publish the angular velocity cmd
        pub.publish(cmd)
        # make sure the messages are published in 100hz
        rate.sleep()

"""
create a ros node to subscribe the object position
and call the callback function to update the vel commands
"""
def init():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('chase_object', anonymous=True)

    # position and angular subscriber
    # rospy.Subscriber('/object_pos', Pose2D, pos_callback)
    rospy.Subscriber('/imageLocation', Point, pos_callback, queue_size = 1)
    sub = rospy.Subscriber('/scan', LaserScan, lidar_callback)
    # command publisher
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    # run the PID controller
    pid_controller(stop=False)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
