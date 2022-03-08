#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-4 Use lidar data to find obstacles in front of robot
# Yinuo Wang, Praneeth Erwin Eddu
# Feb 03, 2022
import numpy as np
# ros
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

import time


####################################
## VARIABLES DECLARATION AND SETUP
####################################

half_window_size = 5
# obstacle_found = True

pub = rospy.Publisher('/obs_pos', Point, queue_size=5)
def lidar_callback(msg):

    # global obstacle_found
    global half_window_size
    # global obs_dis
    global pub
    obs_msg = Point()

    if (any(msg.ranges) < 2):
        # the range we care about is the semi-cycle in front of robot
        left = msg.ranges[0:89]
        right = msg.ranges[270: 359]
        front = right + left

        obs = []
        # using sliding window to detect any possible obstacles
        for index in range(half_window_size, len(front)-half_window_size-1):
            dis_x = sum(front[index - half_window_size: index + half_window_size]) / (2 * half_window_size + 1)
            if dis_x < 0.5:
                obs.append(index)
        # print(obs)
        real_obs = []
        #   try to denoise, if obstacles are too small, it more likes noise rather than real obstacle
        if len(obs) > 5:
            for i in range(1,len(obs)):
                if obs[i] - obs[i-1] <= 3: # these two points represent the same obstacle
                    real_obs.append(obs[i])

            if len(real_obs) > 2: # or we think it's still noise
                sum_obs = 0
                for j in real_obs:
                    sum_obs = sum_obs + front[j]
                obs_dis = sum_obs / len(real_obs)
                center_index = len(obs) / 2

                angular_z = 1.0 - center_index / (len(front)/2) # if angular <0 -- right, angular >0 --left
                obs_msg.x = obs_dis * np.cos(angular_z)
                obs_msg.y = obs_dis * np.sin(angular_z)
                obs_msg.z = angular_z
                print(obs_dis)
                # rospy.loginfo_throttle("Found Obstacle, Distance = %2.2f m", obs_dis)
            else:
                # obstacle_found = False
                obs_msg.x = 0
                obs_msg.y = 0
                obs_msg.z = 0
                rospy.loginfo_throttle("no obstacles found")
        else:
            # obstacle_found = False
            obs_msg.x = 0
            obs_msg.y = 0
            obs_msg.z = 0
            rospy.loginfo_throttle("no obstacles found")

    pub.publish(obs_msg)

"""
create a ros node to detect obstacle position
publish the obstacle position in the world coordinate
"""
def init():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('get_object_range', anonymous=True)
    # position and angular subscriber

    sub = rospy.Subscriber('/scan', LaserScan, lidar_callback)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
