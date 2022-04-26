#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-3 Publish the distance and direction of the object in the robot coordinate
# Yinuo Wang, Praneeth Erwin Eddu
# Feb 19, 2022

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
import numpy as np
import math
t = 0

def lidar_callback(data):
    global t

    state = Pose2D()
    # state.x = t
    state.x = 0.2*math.sin(t)+0.3
    state.y = 0
    state.theta = 0.1* math.sin(t)
    t += 0.1
    pub.publish(state)



def Init():

    # Initializate the node and gives a name, in this case, 'find_ball'
    rospy.init_node('get_object', anonymous=True)
    # Creates the node, the publisher, and subscribes to the compressedImage.
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

    # Create a publisher that will be publishing Geometric message Points
    global pub
    pub = rospy.Publisher('/object_pos', Pose2D, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    try:
        Init()
    except rospy.ROSInterruptException:
        pass


