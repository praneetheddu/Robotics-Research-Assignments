#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Lab-2 Object_find
# Yinuo Wang, Praneeth Erwin Eddu
# Feb 03, 2022

import numpy as np
# ros
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
# opencv
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
"""
subscribe the camera data and detect the object position
"""
def image_callback(img_msg):
    try:
        # rospy.loginfo("received image of type: %s", img_msg.format)
        cv_image = bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
        print(cv_image.shape)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


"""
create a ros node to subscribe the camera image and publish object position
"""
def object_finder():
    # init the node
    rospy.init_node('object_finder', anonymous=True)
    # subscribed Topic
    sub = rospy.Subscriber("/raspicam_node/image/compressed/compressed", CompressedImage, image_callback)
    # init the published topic
    pub = rospy.Publisher('/object_pos', Point, queue_size=10)
    # set publish rate (cannot influence the subscribe rate)
    rate = rospy.Rate(10)  # 10hz
    t = 0
    while not rospy.is_shutdown():
        # the camera image format:
        #     ** FPS: ** ` < update_rate > 30.0 < / update_rate > `
        #     ** Resolution: **
        #     ` < width > 1280 < / width > `
        #     ` < height > 960 < / height > `
        #     ` < format > R8G8B8 < / format > `
        #     ** Node name: ** ` < cameraName > raspicam_node < / cameraName > `
        #     ** Topic name: ** ` < imageTopicName > image/compressed < / imageTopicName > `

        # Simulate the detected object position with sin & cos
        t = t + 0.1
        msg = Point()
        msg.x = (np.sin(t) + 1.0) * 640.0
        msg.y = (np.cos(t) + 1.0) * 480.0
        msg.z = 0.0
        rospy.loginfo("object pos: x-%f y-%f z-%f", msg.x, msg.y, msg.z)
        # publish the object position
        pub.publish(msg)
        # make sure the messages are published in 10hz
        rate.sleep()


if __name__ == '__main__':
    try:
        object_finder()
    except rospy.ROSInterruptException:
        pass
