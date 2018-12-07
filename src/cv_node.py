#!/usr/bin/env python

import rospy
import numpy as np
from scipy import signal
from sensor_msgs.msg import Image
from std_msgs.msg import UInt64, UInt64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2

BLUE_FILTER_TOPIC = '/cv_node/blue_data'
RED_FILTER_TOPIC = '/cv_node/red_data'


# Applies a filter to images received on a specified topic and publishes the filtered image
class CVNode:
    # Initialize the filter
    def __init__(self, sub_topic, pub_topic_red, pub_topic_blue):
        self.sub = rospy.Subscriber(sub_topic, Image, self.apply_filter_cb, queue_size=5)
        self.pub_red = rospy.Publisher(pub_topic_red, Image, queue_size=1)
        self.pub_blue = rospy.Publisher(pub_topic_blue, Image, queue_size=1)
        self.pub_saw_red = rospy.Publisher(RED_FILTER_TOPIC, UInt64, queue_size=1)
        self.pub_saw_blue = rospy.Publisher(BLUE_FILTER_TOPIC, UInt64MultiArray, queue_size=1)
        self.bridge = CvBridge()

    def apply_filter_cb(self, msg):
        in_image = None
        try:
            in_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        img = cv2.cvtColor(in_image, cv2.COLOR_BGR2HSV)
        # height, width, channels = img.shape
        # print height, width, channels

        # define range of red color in HSV NOT THE BEST ON THE FLOOR
        lower_red = np.array([0, 50, 50])
        upper_red= np.array([14, 255, 255])
        mask_red0 = cv2.inRange(img, lower_red, upper_red)

        lower_red = np.array([160, 50, 50])
        upper_red= np.array([180, 255, 255])
        mask_red1 = cv2.inRange(img, lower_red, upper_red)

        # define range of blue color in HSV WORKED WELL on my computer not on car
        # trying for RGB with blue since it is being difficult
        lower_blue = np.array([95, 105, 20])
        upper_blue = np.array([115, 255, 255])
        # lower_blue = np.array([0,   0, 200 ])
        # upper_blue = np.array([65, 65, 255])

        # mask for the colors
        mask_blue = cv2.inRange(img, lower_blue, upper_blue)
        mask_red = (mask_red0 + mask_red1)

        mask_red_blur = cv2.GaussianBlur(mask_red, (11, 11), 0 )
        mask_blue_blur = cv2.GaussianBlur(mask_blue, (11, 11), 0)

        try:
            self.pub_red.publish(self.bridge.cv2_to_imgmsg(mask_red_blur, encoding="passthrough"))
            self.pub_blue.publish(self.bridge.cv2_to_imgmsg(mask_blue_blur, encoding="passthrough"))
        except CvBridgeError as e:
            print e


if __name__ == '__main__':

    rospy.init_node('cv_node', anonymous=True)

    # Populate params with values passed by launch file
    filter_path = rospy.get_param("~filter_path", None)
    sub_topic = rospy.get_param("~sub_topic", None)
    pub_topic_red = rospy.get_param("~pub_topic_red", None)
    pub_topic_blue = rospy.get_param("~pub_topic_blue", None)

    f = CVNode(sub_topic, pub_topic_red, pub_topic_blue)
    rospy.spin()
