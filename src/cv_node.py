#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
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
        self.pub_saw_red = rospy.Publisher(RED_FILTER_TOPIC, Float64, queue_size=1)
        self.pub_saw_blue = rospy.Publisher(BLUE_FILTER_TOPIC, Float64, queue_size=1)
        self.bridge = CvBridge()

    def apply_filter_cb(self, msg):
        in_image = None
        try:
            in_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        img = cv2.cvtColor(in_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0,50, 50])
        upper_red= np.array([14, 255, 255])
        mask_red0 = cv2.inRange(img, lower_red, upper_red)

        lower_red = np.array([160,50, 50])
        upper_red= np.array([180, 255, 255])
        mask_red1 = cv2.inRange(img, lower_red, upper_red)

        mask_red = (mask_red0 + mask_red1)

        # define range of blue color in HSV
        lower_blue = np.array([95, 105, 20])
        upper_blue = np.array([115, 255, 255])

        # mask for the colors
        mask_blue = cv2.inRange(img, lower_blue, upper_blue)

        thresh_blue = cv2.threshold(mask_blue, 127, 255, cv2.THRESH_BINARY)[1]
        #    thresh = cv2.threshold(mask_blue,60,255,cv2.THRESH_BINARY)[1]

        cnts_blue = cv2.findContours(thresh_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

        thresh_red = cv2.threshold(mask_red, 127, 255, cv2.THRESH_BINARY)[1]
        #    thresh = cv2.threshold(mask_blue,60,255,cv2.THRESH_BINARY)[1]

        cnts_red = cv2.findContours(thresh_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

        red_radius = 0.0
        blue_radius = 0.0
        blue_angle = 0.0
        angle_red = 0.0

        if len(cnts_blue) > 5:
            maxcontour = max(cnts_blue, key=cv2.contourArea)
            ((x, y), blue_radius) = cv2.minEnclosingCircle(maxcontour)
            center = (int(x), int(y))
            blue_angle = -1.0 * np.arctan2(x - 320, 480 - y)

            print 'Blue angle:%f Blue radius:%f ' % (blue_angle, blue_radius),

            # only proceed if the radius meets a minimum size
            if blue_radius > 10:
                # draw the circle and centroid on the frame, then update the list of tracked points
                cv2.circle(in_image, center, int(blue_radius), (0, 255, 255), 2)
                cv2.circle(in_image, center, 5, (0, 0, 255), -1)

        for c in cnts_blue:
            cv2.drawContours(in_image, [c], -1, (0, 255, 0), 1)  # draws the Conture lines
            cv2.drawContours(thresh_blue, [c], -1, (0, 255, 0), 1)

        if 20.0 < blue_radius < 220.0:
            self.pub_saw_blue.publish(blue_angle)
        else:
            self.pub_saw_blue.publish(-99.99)

        if len(cnts_red) > 5:
            maxcontour = max(cnts_red, key=cv2.contourArea)
            ((x, y), red_radius) = cv2.minEnclosingCircle(maxcontour)
            center = (int(x), int(y))
            angle_red = -1.0 * np.arctan2(x - 320, 480 - y)

            print 'Red angle:%f Red radius:%f ' % (angle_red, red_radius),

            # only proceed if the radius meets a minimum size
            if red_radius > 10:
                # draw the circle and centroid on the frame, then update the list of tracked points
                cv2.circle(in_image, center, int(red_radius), (0, 0, 255), 2)
                cv2.circle(in_image, center, 5, (0, 255, 0), -1)
        print ""
        for c in cnts_red:
            cv2.drawContours(in_image, [c], -1, (0, 255, 0), 1)  # draws the Conture lines
            cv2.drawContours(thresh_red, [c], -1, (0, 255, 0), 1)

        if red_radius > 40:
            self.pub_saw_red.publish(angle_red)
        else:
            self.pub_saw_red.publish(-99.99)

        # Display the resulting frame
        try:
            self.pub_red.publish(self.bridge.cv2_to_imgmsg(in_image, encoding="passthrough"))
            # self.pub_blue.publish(self.bridge.cv2_to_imgmsg(in_image, encoding="passthrough"))
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
