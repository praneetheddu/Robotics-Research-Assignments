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

MAX_SPEED = 1.
MAX_LINEAR_SPEED = 0.10
MIN_SPEED = 0.15
BETA = 1.5 #1.35

# waypoints
wp_list = []
wp = ((0, 0))
wp_reached = False
wp_ind = 0

# Angular Distance Config
desired_ang = 0
desired_ang_window = 0.03
kp_ang = 2  # 4.0
ki_ang = 0.15
kd_ang = 0.6
total_ang = 0
pre_ang = 0
target_ang = 0
ang = 0
ranges = []
# Other
current_time = time.time()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
cmd = Twist()
"""quaternion to euler angles"""

'''
GTWP = Go to Waypoint
TURN = Turn in place
AO = Avoid obstacle
'''
MODES = ['GTWP', 'TURN', 'AO']
MODE = MODES[0]

def qua2rpy(qua):
    x = qua[0]
    y = qua[1]
    z = qua[2]
    w = qua[3]

    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))

    # left -- yaw = 180-360
    # right -- yaw = 0-180
    yaw = y * 180 / math.pi
    if yaw < 0:
        yaw = yaw + 360

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
    odom_ang = qua2rpy(odom_qua)
    # rospy.loginfo_throttle(2, "odom angle = %f", odom_ang)

def get_waypoints():
    global wp_list
    global wp
    # Uncomment line below when running on robot
    # with open('/home/burger/catkin_ws/src/team_x_navigate_to_goal/wayPoints.txt', 'r') as f:
    with open('/home/pran/catkin_ws/src/Robotics-Research-Assignments/team_x_navigate_to_goal/scripts/wayPoints.txt', 'r') as f:
        wp_list = f.readlines()
        f.close()

get_waypoints()
coordinates = wp_list[wp_ind].split(" ")
wp = (float(coordinates[0]), float(coordinates[1]))
rospy.loginfo("Current Position: (%2.2f, %2.2f)", odom_x, odom_y)
rospy.loginfo("Coordinates recieved: (%2.2f , %2.2f)",
              wp[0], wp[1])

def linear_controller(stop=False):
    global cmd
    if stop:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return
    
    global MODE
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
    global pre_dis

    # angular target
    global target_ang
    flag = 0
    if wp_reached == True:
        if wp_ind < len(wp_list):
            rospy.loginfo("Waypoint %d reached!",
                            wp_ind)
            # Once the way point is reached, cue the next waypoint
            coord = wp_list[wp_ind].split(" ")
            wp = (float(coord[0]), float(coord[1]))
            rospy.loginfo("Current Position: (%2.2f, %2.2f)", odom_x, odom_y)
            rospy.loginfo("Next Coordinates recieved: (%2.2f , %2.2f)",
                            wp[0], wp[1])
            MODE = MODES[1] # Change to rotate in-place
            if abs(wp[0] - odom_x) > abs(wp[1] - odom_y):
                flag = 0
            else:
                flag = 1
        else:
            break
    
    if MODE == 'GTWP':
        diff_dis = 0
        
        if flag == 0:
            diff_dis = abs(wp[0] - odom_x)
        elif flag == 1:
            diff_dis = abs(wp[1] - odom_y)
        
        total_dis += (diff_dis * dt)
        pre_dis = diff_dis
        P = kp_dis * diff_dis
        I = ki_dis * total_dis
        D = kd_dis * ((diff_dis - pre_dis) / dt)
        # PD control 
        v = P + D
        # Ceil to a MAX Speed
        if v > 0:
            cmd.linear.x = min(MAX_LINEAR_SPEED, v)
        else:
            cmd.linear.x = max(-MAX_LINEAR_SPEED, v) * BETA
        
        # WP reached
        if abs(diff_dis) < 0.005:
            cmd.linear.x = 0
            wp_reached = True
            wp_ind += 1
            cmd.angular.z = 0

def run_events():
    global MODE
    global cmd
    rate = rospy.Rate(100)
    while not rospy.is_shutdown(): 
        if MODE = 'GTWP':
            rospy.loginfo_throttle(5, "Mode = GTWP, Driving to Way Point")
            linear_controller(stop=False)
        elif MODE = 'TURN':
            rospy.loginfo_throttle(5, "Mode = TURN, Turning in Place!")

        pub.publish(cmd)
        rate.sleep()

def init():
    # ROS Node
    rospy.init_node('go_to_goal', anonymous=True)

    sub1 = rospy.Subscriber('/obs_pos', Point, obs_callback)
    sub2 = rospy.Subscriber('/odom', Odometry, odom_callback)

    run_events()
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass