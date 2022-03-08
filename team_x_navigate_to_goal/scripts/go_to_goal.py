#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-4 Subscribe obstacle position and publish the robot desired linear and angular velocity cmd to low level motors
# Yinuo Wang, Praneeth Erwin Eddu
# Mar 08, 2022

#  ros
import math

import rospy
# from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import time
import numpy as np

###################################
## VARIABLE DECLARATION AND SETUP
###################################


def odom_callback(msg):




def obs_callback(msg):



def pid_controller(stop=False):




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
    pid_controller(stop=False)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
