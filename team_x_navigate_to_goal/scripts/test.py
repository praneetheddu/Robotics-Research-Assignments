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

MAX_SPEED = 1.
MAX_LINEAR_SPEED = 0.10
MIN_SPEED = 0.15
BETA = 1.5 #1.35

# Linear Controller Config

kp_dis = 0.8
ki_dis = 0.1
kd_dis = 0.05
total_dis = 0
pre_dis = 0
dt = 0.01

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
# Object Config
ball_found = False

# LiDAR config
window = 6
TOLERANCE_THRESHOLD = 0.05

# Other
current_time = time.time()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
cmd = Twist()
"""quaternion to euler angles"""


def qua2rpy(qua):
    x = qua[0]
    y = qua[1]
    z = qua[2]
    w = qua[3]

    # r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # p = math.asin(2 * (w * y - z * z))
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
    # print(odom_qua)
    odom_ang = qua2rpy(odom_qua)
    # rospy.loginfo_throttle(2, "odom angle = %f", odom_ang)


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
    with open('/home/burger/catkin_ws/src/team_x_navigate_to_goal/wayPoints.txt', 'r') as f:
        wp_list = f.readlines()
        f.close()
    # print(len(wp_list))


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
    while not rospy.is_shutdown():
        # If waypoint is reached, move to the next waypoint

        # diff = abs(wp[0] - odom_x) + abs(wp[1] - odom_y)

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
                if abs(wp[0] - odom_x) > abs(wp[1] - odom_y):
                    flag = 0
                else:
                    flag = 1
            else:
                break
        # elif wp_reached == False:


        if wp_reached:

                # Turn in place to orient to the next waypoint
                turn_value = 0
                if abs(odom_x - wp[0]) > 0.05:
                    turn_value = 90.0
                elif abs(odom_y - wp[1]) > 0.05:
                    turn_value = 90.0
                rospy.loginfo("Turning in place with turn angle = %f", turn_value)
                start_ang = odom_ang
                angular_controller(turn_value, start_ang, stop=False)
                # rospy.loginfo("Finished turn")
                wp_reached = False

        # PID
        # pos_x = odom_msg.pose.pose.position.x
        # pos_y = odom_msg.pose.pose.position.y


        elif not wp_reached:
            rate = rospy.Rate(100)

            diff_dis = 0
            if flag == 0:
                diff_dis = abs(wp[0] - odom_x)
            elif flag == 1:
                diff_dis = abs(wp[1] - odom_y)
            # diff_dis = (wp[0] - odom_x) + (wp[1] - odom_y)

            # print(diff_dis)
            # print(flag)

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

            if abs(diff_dis) < 0.005:
                cmd.linear.x = 0
                wp_reached = True
                wp_ind += 1
            cmd.angular.z = 0
            rospy.loginfo_throttle(1, " vel-cmd: %f,  x-pos: %f, y-pos: %f",
                                   cmd.linear.x, odom_x, odom_y)
            pub.publish(cmd)
            rate.sleep()

    # cmd.linear.x = 0.0
    # cmd.angular.z = 0.0
    # pub.publish(cmd)


def angular_controller(diff_angle, start_ang, stop=False):
    global cmd
    # Get waypoints
    global wp
    # odom
    global odom_x
    global odom_y
    global odom_ang

    # PID
    # global diff_ang
    global target_ang
    global pre_ang
    global wp_reached
    target_ang = start_ang + diff_angle
    flag = 0
    if target_ang > 360:
        target_ang = target_ang - 360
        odom_ang = odom_ang - 360
        flag = 1

    while abs(odom_ang - target_ang) >= 1:
        rate = rospy.Rate(100)

        if flag ==1 and odom_ang>target_ang:
            odom_ang = odom_ang - 360
        rospy.loginfo_throttle(1, "current %.2f, target %.2f", odom_ang, target_ang)
        diff_ang = target_ang - odom_ang
        # total_ang += diff_ang * dt
        P = kp_ang * diff_ang
        # I = ki_ang * total_ang
        D = kd_ang * ((diff_ang - pre_ang) / dt)
        w = P + D  # + obstacle_controller()
        pre_ang = diff_ang
        if w >= 0:
            cmd.angular.z = min(MAX_SPEED, w)
        else:
            cmd.angular.z = max(-MAX_SPEED, w)
        cmd.linear.x = 0
        # rospy.loginfo_throttle(1, "odom-x %.2f odom-y %.2f odom-ang %.2f", odom_x, odom_y, odom_ang)

        pub.publish(cmd)
        rate.sleep()


def obstacle_controller():
    global obs_x
    global obs_y
    global obs_ang
    k1 = 2
    k2 = 1
    k3 = 1
    w = k1 * (0.3 - obs_x) + k2 * (0.3 - obs_y) + k3 * (60 * math.pi / 180 - obs_ang)
    return w


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
    # global pub
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    # run the PID controller
    # pid_controller(stop=False)

    # spin() # simply keeps python from exiting until this node is stopped
    # get_waypoints()

    linear_controller(stop=False)

    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
