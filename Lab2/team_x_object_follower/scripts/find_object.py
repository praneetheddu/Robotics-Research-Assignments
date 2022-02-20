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
from imutils.object_detection import non_max_suppression
import imutils
import time

bridge = CvBridge()
# load template
# template = cv.imread("/home/allen/catkin_ws/src/team_x_object_follower/template.jpg", 0)
template = cv.imread("/home/burger/catkin_ws/src/team_x_object_follower/template.jpg", 0)

if template is None:
    print('Could not load temp.')

threshold = 0.82
object_x = 320
object_y = 240
pub = rospy.Publisher('/object_pos', Point, queue_size=1)

"""
subscribe the camera data and detect the object position
"""
def image_callback(img_msg):
    global object_x, object_y
    object_x = 320
    object_y = 240
    rate = rospy.Rate(10)  # 10hz
    # rospy.sleep(1)
    diamond_found = False
    try:
        # rospy.loginfo("received image of type: %s", img_msg.format)
        img_rgb = bridge.compressed_imgmsg_to_cv2(img_msg, "rgb8")


        img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)

        rects = []
	msg = Point()
	re_template = template
        # modify the template size in 0.1-1.5 scale of original size
        # for scale in np.linspace(0.1, 1.5, 50):
            # # resize the template
            # re_template = imutils.resize(template, width=int(template.shape[1] * scale),
                                         # height=int(template.shape[0] * scale))

            # # store template width and height
        rw, rh = re_template.shape[::-1]

            # execute the match function
		
	res = cv.matchTemplate(img_gray, re_template, 5)

	# get the match result with a threshold

	(loc_y, loc_x) = np.where(res >= threshold)
	# There usually are many duplicated rectangles for the same target
	for pt in zip(loc_x, loc_y):
	    rects.append((pt[0], pt[1], pt[0] + rw, pt[1] + rh))
	
        if rects is None:
	    # no template found. don't move the robot
	    rospy.info("no object found")
            if (last_stamp - time.time() > 0.8):
                msg.x = 320
 	        msg.y = 240
	        msg.z = 0
	        pub.publish(msg)
	        return
        diamond_found = True
        # merge near rectangles to a single one
        pick = non_max_suppression(np.array(rects))

        # draw the rectangles and output their coordinates
        for (start_x, start_y, end_x, end_y) in pick:
            # cv.rectangle(img_gray, (start_x, start_y), (end_x, end_y), (0, 0, 255), 1)
            object_x = start_x
            object_y = start_y
            break
        #     # the camera image format:
        #     #     ** FPS: ** ` < update_rate > 30.0 < / update_rate > `
        #     #     ** Resolution: **
        #     #     ` < width > 640 < / width > `
        #     #     ` < height > 480 < / height > `
        #     #     ` < format > R8G8B8 < / format > `
        #     #     ** Node name: ** ` < cameraName > raspicam_node < / cameraName > `
        #     #     ** Topic name: ** ` < imageTopicName > image/compressed < / imageTopicName > `

        #    Simulate the detected object position with sin & cos
        msg.x = object_x
        msg.y = object_y
        msg.z = 0
        # rospy.loginfo("object pos: x-%f y-%f z-%f", msg.x, msg.y, msg.z)
        # publish the object position
        pub.publish(msg)
        last_stamp = time.time()
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    rate.sleep()

"""
create a ros node to subscribe the camera image and publish object position
"""
def object_finder():
    global object_x,object_y
    # init the node
    rospy.init_node('object_finder', anonymous=True)
    # subscribed Topic
    sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, image_callback, queue_size=1, buff_size=52428800)

    # set publish rate (cannot influence the subscribe rate)
    # rate = rospy.Rate(1)  # 10hz
    # t = 0
    # while not rospy.is_shutdown():
    #     # the camera image format:
    #     #     ** FPS: ** ` < update_rate > 30.0 < / update_rate > `
    #     #     ** Resolution: **
    #     #     ` < width > 640 < / width > `
    #     #     ` < height > 480 < / height > `
    #     #     ` < format > R8G8B8 < / format > `
    #     #     ** Node name: ** ` < cameraName > raspicam_node < / cameraName > `
    #     #     ** Topic name: ** ` < imageTopicName > image/compressed < / imageTopicName > `
    #
    #     # Simulate the detected object position with sin & cos
    #     # t = t + 0.1
    #     # msg = Point()
    #     # msg.x = (np.sin(t) + 1.0) * 640.0
    #     # msg.y = (np.cos(t) + 1.0) * 480.0
    #     # msg.z = 0.0

    rospy.spin()


if __name__ == '__main__':
    try:
        object_finder()
    except rospy.ROSInterruptException:
        pass
