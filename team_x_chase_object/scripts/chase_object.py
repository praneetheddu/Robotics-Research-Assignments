#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-3 Publish the robot desired linear and angular velocity cmd to low level motors
# Yinuo Wang, Praneeth Erwin Eddu
# Feb 03, 2022

#  ros
import math

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

###################################
## VARIABLE DECLARATION AND SETUP
###################################
desire_dis = 0.3
desire_ang = 0

kp_dis = 1
ki_dis = 0.1
kd_dis = 0.1

kp_ang = 1
ki_ang = 0.05
kd_ang = 0.05

dt = 0.01
total_dis = 0
pre_dis = 0
total_ang = 0
pre_ang = 0

dis_x = 0
dis_y = 0
ang = 0
is_ball_present = False
"""
subscribe the object position and update the commands
"""
def pos_callback(data):
    # Message Type [Pose2D]:
    # This expresses a position and orientation on a 2D manifold.
    # float64 x
    # float64 y
    # float64 theta
    global dis_x
    global dis_y
    global ang
    global is_ball_present
    dis_x = data.x
    dis_y = data.y
    ang = data.theta

"""
Use LiDAR readings for PID control
"""
def lidar_callback(msg):
    if (any(msg.ranges) < 2):
        rospy.loginfo_throttle(1, "Distance = %2.2f m", msg.ranges[0])

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
    # set publish rate (control frequency)
    rate = rospy.Rate(100)  # 100hz
    while not rospy.is_shutdown():
        # distance pid
        # dis = math.sqrt(pow(dis_x, 2) + pow(dis_y, 2))
        diff_dis = dis_x - desire_dis
        total_dis = total_dis + diff_dis * dt
        v = kp_dis * diff_dis + ki_dis * total_dis + kd_dis * (diff_dis - pre_dis) / dt
        pre_dis = diff_dis
        rospy.loginfo_throttle(5, " vel-cmd %f,  object_dis %f", v, dis_x)

        # angular pid
        diff_ang = desire_ang - ang
        total_ang += diff_ang * dt
        w = kp_ang * diff_ang + ki_ang * total_ang + kd_ang * (diff_ang - pre_ang) / dt
        pre_ang = ang
        rospy.loginfo_throttle(5, " ang-cmd %f,  object_ang %f", w, ang)\

        # limit the cmd range
        if v > 0:
            cmd.linear.x = min(0.22, v)
        else:
            cmd.linear.x = max(-0.22,v)
        if w > 0:
            cmd.angular.z = min(1.0, w)
        else:
            cmd.angular.z = max(-1.0, w)

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
    rospy.Subscriber('/object_pos', Pose2D, pos_callback)
    sub = rospy.Subscriber('/scan', LaserScan, lidar_callback)
    # command publisher
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    # run the PID controller
    pid_controller(stop=True)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass

