#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-4 Subscribe obstacle position and publish the robot desired linear and angular velocity cmd to low level motors
# Yinuo Wang, Praneeth Erwin Eddu
# Mar 08, 2022

#  ros
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import time
import numpy as np
import math

###################################
## VARIABLE DECLARATION AND SETUP
###################################
# following variables are defined in robot frame
obs_x = 0
obs_y = 0
obs_ang = 0
# following variables are defined in world frame
odom_x = 0
odom_y = 0
odom_ang = 0

"""quaternion to euler angles"""


def qua2rpy(qua):
    x = qua[0]
    y = qua[1]
    z = qua[2]
    w = qua[3]

    # r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # p = math.asin(2 * (w * y - z * z))
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))

    # left -- yaw>0
    # right -- yaw<0
    yaw = y * 180 / math.pi

    return yaw


""" get odometry data """

def odom_callback(msg):
    global odom_x
    global odom_y
    global odom_ang
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y

    odom_qua = []
    odom_qua.append(msg.pose.pose.orientation.x)
    odom_qua.append(msg.pose.pose.orientation.y)
    odom_qua.append(msg.pose.pose.orientation.z)
    odom_qua.append(msg.pose.pose.orientation.w)
    # print(odom_qua)
    odom_ang = qua2rpy(odom_qua)
    print(odom_ang)


"""get obstacle position in the robot frame"""
def obs_callback(msg):
    global dis_x
    global dis_y
    global ang
    dis_x = msg.x
    dis_y = msg.y
    ang = msg.z


# def pid_controller(stop=False):


def init():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('go_to_goal', anonymous=True)

    sub1 = rospy.Subscriber('/obs_pos', Point, obs_callback)
    sub2 = rospy.Subscriber('/odom', Odometry, odom_callback)
    # command publisher
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    # run the PID controller
    # pid_controller(stop=False)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
