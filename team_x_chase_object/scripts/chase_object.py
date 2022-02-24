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

###################################
## VARIABLE DECLARATION AND SETUP
###################################
MAX_SPEED = 1.0
MAX_LINEAR_SPEED = 0.08
MIN_SPEED = 0.15
# Linear Distance Confif
desire_dis = 0.30
kp_dis = 0.15
ki_dis = 0.1
kd_dis = 0.10
total_dis = 0
pre_dis = 0
dt = 0.01
dis_x = 0
dis_y = 0

# Angular Distance Config
desired_ang = 0
desired_ang_window = 0.03
kp_ang = 0.67
ki_ang = 0.15
kd_ang = 0.40
total_ang = 0
pre_ang = 0
ang = 0

# Object Config
ball_found = False

# LiDAR config
window = 6
TOLERANCE_THRESHOLD = 0.05

# Other
current_time = time.time()
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
        ang = (data.x - 240) / 360
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
    # ball_found = True # Uncomment to measure distance only 
    if not ball_found:
	# If no ball is detected, set distance to  0
        dis_x = desire_dis
        # pid_controller(stop=True)
        pass
    else:
        if (any(msg.ranges) < 2):
	    # dis_x = (sum(msg.ranges[len(msg.ranges) - (window // 2):len(msg.ranges)]) + sum(msg.ranges[: window // 2])) / window
	    dis_x = (msg.ranges[0] + msg.ranges[1] + msg.ranges[359]) / 3
            # dis_x = msg.ranges[0]
            rospy.loginfo_throttle(0.5, "Distance = %2.2f m", dis_x)
    


def pid_controller(stop=False):
    # prepare the commands
    cmd = Twist()
    
    if stop:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return

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


    # set publish rate (control frequency)
    rate = rospy.Rate(100)  # 100hz
    while not rospy.is_shutdown():
        # distance pid
        # dis = math.sqrt(pow(dis_x, 2) + pow(dis_y, 2))
        diff_dis = dis_x - desire_dis
        total_dis = total_dis + (diff_dis * dt)
        P = (kp_dis * diff_dis)
        I = (ki_dis * total_dis)
        D = kd_dis * ((diff_dis - pre_dis) / dt)
        v = P + D 
        pre_dis = diff_dis
        rospy.loginfo_throttle(1, " vel-cmd %f,  object_dis %f", v, dis_x)

        # angular pid
        diff_ang = desired_ang - ang
        if abs(diff_ang) <= desired_ang_window:
	    diff_ang = 0
        
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
            cmd.linear.x = max(-MAX_LINEAR_SPEED,v)
        if w > 0:
            cmd.angular.z = min(MAX_SPEED, w)
        else:
            cmd.angular.z = max(-MAX_SPEED, w)

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
