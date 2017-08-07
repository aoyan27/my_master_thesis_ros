#!/usr/bin/env python

import argparse

import numpy as np
import skimage.io
import skimage.draw
from skimage.transform import resize

import cv2 as cv

if __name__=="__main__":
    image_path = "/home/amsl/ros_catkin_ws/src/master_thesis/my_cv_bridge_tutorial/scripts/ssd/sample_image/dog.jpg"
    ski_image = skimage.img_as_float(skimage.io.imread(image_path, as_grey=False)).astype(np.float32)
    ski_image = ski_image * 255
    print "ski_image : ", ski_image
    ski_image = ski_image.transpose(2, 0, 1)[::-1]
    print "ski_image(after) : ", ski_image
    
    cv_image = cv.imread(image_path)
    print "cv_image : ", cv_image
    #  print type(cv_image)
    cv_image = cv.imread(image_path).astype(np.float32)
    cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
    print "cv_image(after) : ", cv_image
    cv_image = cv_image.transpose(2, 0, 1)[::-1]

    #  print "ski_image : ", ski_image
    print "cv_image : ", cv_image
