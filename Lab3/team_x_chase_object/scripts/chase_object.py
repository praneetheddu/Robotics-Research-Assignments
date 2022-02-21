#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-3 Publish the robot desired linear and angular velocity cmd to low level motors
# Yinuo Wang, Praneeth Erwin Eddu
# Feb 03, 2022

#  ros
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

pos = [0, 0, 0]
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

"""
subscribe the object position and update the commands
"""
def pos_callback(data):
    # set publish rate (cannot influence the subscribe rate)
    rate = rospy.Rate(10)  # 10hz
    # rospy.loginfo("object position: %f, %f, %f", data.x, data.y, data.z)
    pos[0] = data.x
    pos[1] = data.y
    pos[2] = data.z
    # prepare the command
    cmd = Twist()
    # angular vel range from -2~2
    if pos[0] > 640:
        cmd.angular.z = (640-pos[0])/320
        rospy.loginfo("object is on the right, z-vel:%f", cmd.angular.z)

    elif pos[0] <= 640:
        cmd.angular.z = (640-pos[0])/320
        rospy.loginfo("object is on the left, z-vel:%f", cmd.angular.z)

    # publish the angular velocity cmd
    pub.publish(cmd)
    # make sure the messages are published in 10hz
    rate.sleep()

"""
create a ros node to subscribe the object position
and call the callback function to update the vel commands
"""
def robot_controller():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('controller', anonymous=True)

    rospy.Subscriber('/object_pos', Point, pos_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    robot_controller()

