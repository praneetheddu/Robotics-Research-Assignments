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

MAX_SPEED = 0.7  # 0.4
MAX_LINEAR_SPEED = 0.12  # 0.09
MIN_SPEED = 0.12
BETA = 1.5  # 1.35

# waypoints
wp_list = []
wp = ((0, 0))
wp_reached = False
wp_ind = 0

# Linear Controller Config
kp_dis = 0.8
ki_dis = 0.1
kd_dis = 0.05
total_dis = 0
pre_dis = 0
dt = 0.01
W_a = 1

# Angular Distance Config
desired_angle = 0
desired_ang_window = 0.01  # 0.01
kp_ang = 1.23  # 1.15 # 1.5  # 4.0
ki_ang = 0.15
kd_ang = 0.6
total_ang = 0
pre_ang = 0
target_ang = 0
ang = 0
ranges = []
GAMMA = 2.1  # 6.2
W_d = 1

# object
dis_x = 0
dis_y = 0
ang = 0

# Other
current_time = time.time()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
cmd = Twist()
flag = 0
greater_360 = 0
toggle = False

TURN_ANGLES = np.array([90, 180, 270])
'''
GTWP = Go to Waypoint
TURN = Turn in place
'''
MODES = ['GTWP', 'TURN']
MODE = MODES[0]

"""quaternion to euler angles"""


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


""" get waypoint goal """


def get_waypoints():
    global wp_list
    global wp
    # Uncomment line below when running on robot
    with open('/home/burger/catkin_ws/src/team_x_navigate_to_goal/wayPoints.txt', 'r') as f:
        # with open('/home/pran/catkin_ws/src/Robotics-Research-Assignments/team_x_navigate_to_goal/scripts/wayPoints.txt', 'r') as f:
        wp_list = f.readlines()
        f.close()


get_waypoints()
coordinates = wp_list[wp_ind].split(" ")
wp = (float(coordinates[0]), float(coordinates[1]))
rospy.loginfo("Current Position: (%2.2f, %2.2f)", odom_x, odom_y)
rospy.loginfo("Coordinates recieved: (%2.2f , %2.2f)",
              wp[0], wp[1])


def obs_callback(msg):
    global dis_x
    global dis_y
    global ang
    dis_x = msg.x
    dis_y = msg.y
    ang = msg.z


def goal_controller(stop=False):
    global cmd
    if stop:
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return

    global MODE
    global flag

    # Get waypoints
    global wp
    global wp_list
    global wp_reached
    global wp_ind

    # odom
    global odom_x
    global odom_y
    global odom_ang

    # linear control
    global total_dis
    global pre_dis
    global W_a

    # angular control
    global target_ang
    global desired_angle
    global desired_ang_window
    global pre_ang
    global W_d
    global kp_ang
    global kd_ang
    global toggle

    if wp_reached == True:
        if wp_ind < len(wp_list):
            rospy.loginfo("Waypoint %d reached!", wp_ind - 1)
            # Once the way point is reached, cue the next waypoint
            coord = wp_list[wp_ind].split(" ")
            wp = (float(coord[0]), float(coord[1]))
            rospy.loginfo("Current Position: (%2.2f, %2.2f)", odom_x, odom_y)
            rospy.loginfo("Next Coordinates recieved: (%2.2f , %2.2f)",
                          wp[0], wp[1])
            MODE = MODES[1]  # Change to rotate in-place
            if abs(wp[0] - odom_x) > abs(wp[1] - odom_y):
                flag = 0
            else:
                flag = 1
        else:
            return

    if MODE == 'GTWP':
        diff_dis = 0
        diff_ang = 0
        cur_coord = wp_list[wp_ind].split(" ")
        cur_wp = (float(cur_coord[0]), float(cur_coord[1]))
        if wp_ind > 0:
            pre_coord = wp_list[wp_ind - 1].split(" ")
            pre_wp = (float(pre_coord[0]), float(pre_coord[1]))
        else:
            pre_wp = (0.0, 0.0)

        if flag == 0:  # move in x direction
            diff_dis = abs(wp[0] - odom_x)
            # Keep robot on track on y-axis when moving in x-dir
            diff_ang = (wp[1] - odom_y)

            if cur_wp[0] - pre_wp[0] < 0:  # negative direction of x
                diff_ang = -1 * diff_ang

        elif flag == 1:  # move in y direction
            diff_dis = abs(wp[1] - odom_y)
            # Keep robot on track on x-axis when moving in y-dir
            diff_ang = (wp[0] - odom_x)

            if cur_wp[1] - pre_wp[1] > 0:  # positive direction of y
                diff_ang = -1 * diff_ang

        # PD controller for linear velocity
        pre_dis = diff_dis
        P = kp_dis * diff_dis
        D = kd_dis * ((diff_dis - pre_dis) / dt)
        v = P + D
        lin_vel = 0
        # Ceil to a MAX Speed
        if v > 0:
            lin_vel = min(MAX_LINEAR_SPEED, v)
        else:
            lin_vel = max(-MAX_LINEAR_SPEED, v) * BETA

        w_avoid, obj_on_left = avoid_controller()

        # PD controller for angular velocity
        if abs(diff_ang) <= desired_ang_window:

            w = 0
            d_odom_ang = odom_ang
            if odom_ang < 360 and odom_ang > 350:
                d_odom_ang = 0
            rospy.loginfo_throttle(1, "Within Angle window----------------------------------")
            rospy.loginfo_throttle(1, "Curr heading: %2.2f, desired heading: %2.2f ****", odom_ang, desired_angle)
            if toggle:
                diff_ang = d_odom_ang - desired_angle
            else:
                diff_ang = desired_angle - d_odom_ang
            if diff_ang >= 10:  # go right side
                # if desired_angle - d_odom_ang >= 10:
                rospy.loginfo_throttle(1, "***************Turn****************")
                turn_controller_2(desired_angle, odom_ang)
                return
        else:
            P = kp_ang * diff_ang
            D = kd_ang * ((diff_ang - pre_ang) / dt)
            w = P + D
            pre_ang = diff_ang
        W_a = 1
        W_d = 1

        if toggle:
            # right
            ang_vel = w + w_avoid
        else:
            # left
            ang_vel = w - w_avoid
        # Compute blending weights for obstacle avoidance
        if abs(w_avoid) > 0:
            # Moving away from obstaces in front
            W_a = 1 - np.exp(-GAMMA * abs(w_avoid))
            W_d = 1 - W_a

        rospy.loginfo_throttle(1, "W_D = %2.2f , W_A = %2.2f",
                               W_d, W_a)
        rospy.loginfo_throttle(1, "Diff ang = %2.2f , Current heading = %2.2f",
                               diff_ang, odom_ang)
        rospy.loginfo_throttle(1, "odom reading_x = %2.2f , odom reading_y = %2.2f, ang_vel = %2.2f",
                               odom_x, odom_y, ang_vel)

        cmd.angular.z = W_a * ang_vel
        cmd.linear.x = W_d * lin_vel

        # WP reached
        if abs(diff_dis) < 0.01:
            cmd.linear.x = 0
            cmd.angular.z = 0

            wp_reached = True
            wp_ind += 1


def avoid_controller():
    global dis_x
    global dis_y
    global ang
    obj_on_left = False
    k1 = 0.48  # 0.2 + 0.2
    k2 = 0.38  # 0.1 + 0.2
    k3 = 0.38  # 0.1 + 0.2

    w = 0
    if dis_x != 0 and dis_y != 0 and ang != 0:  # found obstacles
        if dis_x > 1000 and dis_y > 1000 and ang > 1000:
            # obstacle detected on left side
            obj_on_left = True
        else:
            w = k1 * (0.3 - obs_x) + k2 * (0.3 - obs_y) + k3 * (60 * math.pi / 180 - obs_ang)
            rospy.loginfo_throttle(1, "Obstacle detected!, w = %2.2f", w)

    return -w, obj_on_left


''' Matching robot's heading with desired angle '''


def turn_controller_2(diff_angle, start_ang, stop=False):
    global cmd
    global MODE
    # Get waypoints
    global wp

    # odom
    global odom_x
    global odom_y
    global odom_ang

    # PID
    global target_ang
    global pre_ang
    global wp_reached
    global greater_360

    rate = rospy.Rate(100)
    while abs(odom_ang - diff_angle) >= 0.5:

        if greater_360 == 1:
            odom_ang = odom_ang - 360
            greater_360 = 0
        diff_ang = diff_angle - odom_ang
        P = kp_ang * diff_ang
        D = kd_ang * ((diff_ang - pre_ang) / dt)
        w = P + D
        pre_ang = diff_ang
        if w >= 0:
            cmd.angular.z = min(MAX_SPEED, w)
        else:
            cmd.angular.z = max(-MAX_SPEED, w)
        cmd.linear.x = 0
        rospy.loginfo_throttle(1, "target: %.2f  odom-ang %.2f", diff_angle, odom_ang)

        pub.publish(cmd)
        rate.sleep()
    cmd.angular.z = 0
    wp_reached = False
    pub.publish(cmd)

    rospy.loginfo_throttle(1, "$$$$$$$$$$ Finished Turning! $$$$$$$$$$$")
    MODE = MODES[0]  # Finished turning

""" State Machine """

def run_events():
    global MODE
    global cmd
    global flag
    global odom_ang
    global odom_x
    global odom_y
    global greater_360
    global desired_angle
    global toggle

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if MODE == 'GTWP':
            rospy.loginfo_throttle(5, "Mode = GTWP, Driving to Way Point")
            goal_controller(stop=False)
            pub.publish(cmd)

        elif MODE == 'TURN':
            rospy.loginfo_throttle(2, "Current angle = %2.2f", odom_ang)

            turn_angle = 90
            greater_360 = 0

            odom_ang_temp = odom_ang
            if odom_ang > 345:
                odom_ang_temp = 0
            desired_angle = odom_ang_temp + turn_angle
            if desired_angle > 360:
                greater_360 = 1
            desired_angle_arr = abs(TURN_ANGLES - desired_angle)
            desired_angle_ind = np.where(desired_angle_arr == np.min(desired_angle_arr))
            desired_angle = TURN_ANGLES[desired_angle_ind[0][0]]

            rospy.loginfo_throttle(2, "Desired angle = %2.2f, current_angle =  %2.2f", desired_angle, odom_ang)
            turn_controller_2(desired_angle, odom_ang)
            rospy.loginfo_throttle(1, "heading after Turning = %2.2f",
                                   odom_ang)
            toggle = not toggle

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
