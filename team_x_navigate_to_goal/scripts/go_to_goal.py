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
# from chase_object import pid_controller
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

MAX_SPEED = 1.5
MAX_LINEAR_SPEED = 0.10
MIN_SPEED = 0.15
BETA = 1.35

# Linear Controller Config
desire_dis = 0.25
kp_dis = 0.6
ki_dis = 0.1
kd_dis = 0.05
total_dis = 0
pre_dis = 0
dt = 0.01

# waypoints
wp_list = []
wp = ((0 , 0))
wp_reached = True
wp_ind = 0

# Angular Distance Config
desired_ang = 0
desired_ang_window = 0.03
kp_ang = 0.5  #2.8
ki_ang = 0.15
kd_ang = 0.5
total_ang = 0
pre_ang = 0
ang = 0
ranges = []
# Object Config
heading_reached = True
init = True

# LiDAR config
window = 6
TOLERANCE_THRESHOLD = 0.05

# Other
current_time = time.time()

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
    linear_controller(stop=False)


"""get obstacle position in the robot frame"""
def obs_callback(msg):
    global dis_x
    global dis_y
    global ang
    dis_x = msg.x
    dis_y = msg.y
    ang = msg.z


""" Get waypoints from txt file and store it in a list"""
def get_waypoints():
    global wp_list
    global wp
    # Uncomment line below if running code on robot
    # with open('/home/burger/catkin_ws/src/team_x_navigate_to_goal/wayPoints.txt', 'r') as f:
    with open('/home/pran/catkin_ws/src/Robotics-Research-Assignments/team_x_navigate_to_goal/wayPoints.txt', 'r') as f:        
        wp_list = f.readlines()
        f.close()


def linear_controller(stop=False):
    cmd = Twist()
    if stop:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return
    
    # Get waypoints 
    global wp
    global wp_list
    global wp_reached
    global wp_ind
    
    # odom
    global odom_x
    global odom_y
    global odom_ang

    # linear dist
    global total_dis

    # other
    global init
    global heading_reached
    
    if wp_reached and heading_reached:
        # Once the way point is reached, cue the next waypoint
        if wp_ind < len(wp_list):
            coordinates = wp_list[wp_ind].split(" ")
            wp = (float(coordinates[0]) , float(coordinates[1]))
            rospy.loginfo("Coordinates recieved: (%2.2f , %2.2f)", 
                            wp[0], wp[1])
            
            # Turn in place to orient to the next waypoint
            if not init:
                if abs(odom_x - 90) > 45:
                    turn_value = odom_x - 90.0
                elif abs(odom_y - 90) > 45:
                    turn_value = odom_y - 90.0
                rospy.loginfo("Turning in place with turn angle = %f", turn_value)
                angular_controller(turn_value, stop=False)
                heading_reached = False
                
                wp_reached = False
            else:
                init = False

    # PID
    # pos_x = odom_msg.pose.pose.position.x
    # pos_y = odom_msg.pose.pose.position.y
    diff_dis = wp[0] - odom_x
    
    # If waypoint is reached, move to the next waypoint
    if diff_dis < 0.01:
        wp_reached = True
        if wp_ind < len(wp_list):
            rospy.loginfo("Waypoint %d reached!", 
                            wp_ind + 1)
            wp_ind += 1
    else:
        wp_reached = False
    
    if not wp_reached:
        rate = rospy.Rate(100)
        total_dis += (diff_dis * dt)
        pre_dis = diff_dis
        P = kp_dis * diff_dis
        I = ki_dis * total_dis
        D = kd_dis * ((diff_dis - pre_dis) / dt)
        v = P + D
        
        if v > 0:
            cmd.linear.x = min(MAX_LINEAR_SPEED, v)
        else:
            cmd.linear.x = max(-MAX_LINEAR_SPEED, v) * BETA
        rospy.loginfo_throttle(1, " vel-cmd: %f,  x-pos: %f, y-pos: %f", 
                                cmd.linear.x, odom_x, odom_y)
        pub.publish(cmd)
        rate.sleep()
    else:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

def angular_controller(diff_angle, stop=False):
    cmd = Twist()
    rate = rospy.Rate(100)
    # Get waypoints 
    global wp
    # odom
    global odom_x
    global odom_y
    global odom_ang
    
    # PID
    # global diff_ang
    global total_ang
    global pre_ang
    global heading_reached

    
    if abs(90 - odom_ang) > 5:
        heading_reached = False
        diff_ang = desired_ang - 90
        total_ang += diff_ang * dt
        P = kp_ang * diff_ang
        I = ki_ang * total_ang
        D = kd_ang * ((diff_ang - pre_ang) / dt)
        w = P + D
        pre_ang = diff_ang
        rospy.loginfo_throttle(1, "Robot is turning in-place")
        if w >= 0:
            cmd.angular.z = min(MAX_SPEED, w)
        else:
            cmd.angular.z = max(-MAX_SPEED, w)   
    else:
        cmd.angular.z = 0
        rospy.loginfo_throttle(1, "Turning Complete. Robot is headed to next waypoint")
        heading_reached = True
    pub.publish(cmd)
    rate.sleep()

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
    
    # spin() # simply keeps python from exiting until this node is stopped
    get_waypoints()
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
