#!/usr/bin/env python
#coding:utf-8

import rospy
import sys
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/output_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.ImageCallback)
        
        self.path = "/home/amsl/ros_catkin_ws/src/my_cv_bridge_tutorial/sample_picture/"
        
        self.count = 0

    def ImageCallback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        height, width, channels= cv_image.shape[:3]
        print "height : ", height, "\twidth : ", width, "\tchannels : ", channels

        #　赤色抽出
        #  color_min = np.array([150, 100, 150])
        #  color_max = np.array([180, 255, 255])
        
        #　緑色抽出
        color_min = np.array([40, 100, 150])
        color_max = np.array([80, 255, 255])

        color_mask = cv2.inRange(hsv_image, color_min, color_max)
        cv_image2 = cv2.bitwise_and(cv_image, cv_image, mask = color_mask)
        
        canny_image = cv2.Canny(cv_image2, 50, 200)


        contours, hierarchy = cv2.findContours(canny_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #  print len(contours)
        
        area = np.array([])

        for i in xrange(len(contours)):
            area = np.append(area, cv2.contourArea(contours[i]))
        max_area_index = np.argmax(area)
        print "max_area_index : ", max_area_index

        M = cv2.moments(contours[max_area_index])

        #  print "M : ", M

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        print "cx : ", cx
        print "cy : ", cy

        print "count : ", self.count
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(canny_image, "mono8"))
        abs_path = self.path + "img%d.png" % self.count
        if self.count % 100 == 0:
            cv2.imwrite(abs_path, cv_image)

        self.count += 1

        cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)

        cv_half_image = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)
        cv_half_image2 = cv2.resize(cv_image2, (0, 0), fx=0.5, fy=0.5)
        cv_half_image3 = cv2.resize(canny_image, (0, 0), fx=0.5, fy=0.5)
        
        cv2.imshow("Origin Image", cv_half_image)
        cv2.imshow("Result Image", cv_half_image2)
        cv2.imshow("Edge Image", cv_half_image3)

def main(args):
    ic = image_converter()

    rospy.init_node("image_converter")

    cv2.namedWindow("Origin Image")
    cv2.namedWindow("Result Image")
    cv2.namedWindow("Edge Image")
    cv2.startWindowThread()

    rospy.spin()

if __name__=="__main__":
    main(sys.argv)


