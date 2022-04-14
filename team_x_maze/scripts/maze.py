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
2. Use LiDAR to determine distance to walls ❌
3. Leverage distance and signage data to publish goals using move_base simple goal ❌
4. Integrate camera in simulation environment ❌
5. Check for target sign of completion ❌
6. Develop a failsafe in-case robot gets lost ❌
"""

# ROS 
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback

# Python
import numpy as np

def init():
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
