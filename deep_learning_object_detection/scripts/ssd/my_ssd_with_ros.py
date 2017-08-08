#!/usr/bin/env python
#coding:utf-8
#coment : 
#2017025---> cannot recognizing Multiple number of people. only single

import argparse

import rospy
import sys
import cv2 as cv
import numpy as np
import skimage.io
import skimage.draw
from skimage.transform import resize

from std_msgs.msg import String, Header, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import chainer
from chainer import serializers
from chainer import cuda

from utils.prior import prior

import utils.ssd_net
import utils.draw

import time

import copy

parser = argparse.ArgumentParser(description='detect object')
parser.add_argument('--gpu', type=int, default=-1, help='Choose gpu(0) or cpu(-1) mode')
args = parser.parse_args()
xp = chainer.cuda.cupy if chainer.cuda.available and args.gpu >= 0 else np


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/result_image", Image, queue_size=1)
        self.bbox_pub = rospy.Publisher("/bbox_array", Int32MultiArray, queue_size=1)
        self.header_pub = rospy.Publisher("/zed/rgb/header/before_process", Header, queue_size=1)
        self.bridge = CvBridge()
        #  self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.ImageCallback)
        self.image_sub = rospy.Subscriber("/zed/rgb/image_raw_color", Image, self.ImageCallback)
        
        self.model_path = "/home/amsl/ros_catkin_ws/src/master_thesis/deep_learning_object_detection/data/ssd/ssd.model"

        self.model = None

    def ImageCallback(self, msg):
        start = time.time()
        img_header = Header()
        img_header = msg.header
        print "img_header : "
        print img_header
        self.header_pub.publish(img_header)
        #  print "msg.header : ", msg.header
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e
        
        height, width, channels= cv_image.shape[:3]
        print "height : ", height, "\twidth : ", width, "\tchannels : ", channels
        orig_image = cv.resize(cv_image, (width/2, height/2))
        resize_image = cv.resize(cv_image, (300, 300))
        
        #  orig_image = copy.deepcopy(cv_image)
        #  orig_image = copy.deepcopy(resize_image)
        print('orig_image:', orig_image.shape)
        
        mean = np.array([104, 117, 123])
        resize_image = cv.cvtColor(resize_image, cv.COLOR_BGR2RGB)
        resize_image = resize_image.astype(np.float32)

        img = resize_image - mean[::-1]
        img = img.transpose(2, 0, 1)[::-1]

        #  print "img : ", img
        
        start = time.time()

        x = None
        if args.gpu >= 0:
            img_gpu = cuda.to_gpu(np.array([img], dtype=np.float32))
            x = chainer.Variable(img_gpu)
        else:
            img_cpu = np.array([img], dtype=np.float32)
            x = chainer.Variable(img_cpu)

        self.model(x, 1, 1, 1, 1)

        prior = self.model.mbox_prior.astype(np.float32)
        loc = cuda.to_cpu(self.model.mbox_loc.data[0])
        conf = cuda.to_cpu(self.model.mbox_conf_softmax_reshape.data[0])

        elapsed_time = time.time() - start
        print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")


        cand = utils.draw.detect(prior, loc, conf)
        #  utils.draw.draw(image, cand, args.name)
        result, x1, y1, x2, y2, bbox_pub_flag = utils.draw.cv_draw(orig_image, cand)
        print "cand : ", cand
        if bbox_pub_flag:
            print "x1 : ", x1, "\t y1 : ", y1, "\t x2 : ", x2, "\t y2 : ", y2

            bbox_array = Int32MultiArray()
            bbox_array.data = [x1, y1, x2, y2]
            self.bbox_pub.publish(bbox_array)
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))


        #  cv.imshow("Origin Image", orig_image)
        #  cv.imshow("Result Image", result)

    def get_model(self):
        self.model = utils.ssd_net.SSD()
        serializers.load_npz(self.model_path, self.model)

        if args.gpu >= 0:
            print "--- GPU mode ---"
            self.model.to_gpu(args.gpu)
        else:
            print "--- CPU mode ---"

        print "model load complete!!"


def main(args):
    ic = image_converter()
    ic.get_model()

    rospy.init_node("my_sdd_with_ros")
    print "Here we go!!!!"
    #  cv.namedWindow("Origin Image")
    #  cv.namedWindow("Result Image")
    #  cv.startWindowThread()

    rospy.spin()


if __name__=="__main__":
    main(sys.argv)


