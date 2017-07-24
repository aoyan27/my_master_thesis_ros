#!/usr/bin/env python
#coding:utf-8

import argparse

import rospy
import sys
import cv2 as cv
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import chainer
from chainer import serializers
from chainer.cuda import to_gpu
from models.cpu_nms import cpu_nms as nms
from models.faster_rcnn import FasterRCNN
from models.vgg16 import VGG16Prev

import copy

parser = argparse.ArgumentParser()
parser.add_argument('--nms_thresh', type=float, default=0.3)
parser.add_argument('--conf', type=float, default=0.8)
parser.add_argument('--gpu', type=int, default=-1)
args = parser.parse_args()
xp = chainer.cuda.cupy if chainer.cuda.available and args.gpu >= 0 else np


CLASSES = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor')
PIXEL_MEANS = np.array([[[102.9801, 115.9465, 122.7717]]])


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/result_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.ImageCallback)
        
        self.model_path = "/home/amsl/chainer-faster-rcnn/data/VGG16_faster_rcnn_final.model"

        self.model = None
        
        self.get_model_flag = True

    def ImageCallback(self, msg):
        if self.get_model_flag:
            self.model = self.get_model(gpu=args.gpu)
            if chainer.cuda.available and args.gpu >= 0:
                self.model.to_gpu(args.gpu)
            self.get_model_flag = False

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e
        
        height, width, channels= cv_image.shape[:3]
        print "height : ", height, "\twidth : ", width, "\tchannels : ", channels
        
        orig_image = copy.deepcopy(cv_image)
        print('orig_image:', orig_image.shape)
        img, im_scale = self.img_preprocessing(orig_image, PIXEL_MEANS)
        img = np.expand_dims(img, axis=0)
        print('img:', img.shape, im_scale)
        if args.gpu >= 0:
            img = to_gpu(img, device=args.gpu)
        
        with chainer.using_config('enable_backprop', False):
            img = chainer.Variable(img)
        img_info = chainer.Variable(np.array([[img.shape[2], img.shape[2]]]))
        cls_score, bbox_pred = self.model(img, img_info)
        cls_score = cls_score.data

        if args.gpu >= 0:
            cls_score = chainer.cuda.cupy.asnumpy(cls_score)
            bbox_pred = chainer.cuda.cupy.asnumpy(bbox_pred)
        result = self.draw_result(orig_image, im_scale, cls_score, bbox_pred,
                         args.nms_thresh, args.conf)
        

        cv.imshow("Origin Image", cv_image)
        cv.imshow("Result Image", result)


    def get_model(self, gpu):
        model = FasterRCNN(trunk_class=VGG16Prev)
        model.rcnn_train = False
        model.rpn_train = False
        serializers.load_npz(self.model_path, model)

        return model

    def img_preprocessing(self, orig_img, pixel_means, max_size=1000, scale=600):
        img = orig_img.astype(np.float32, copy=True)
        img -= pixel_means
        im_size_min = np.min(img.shape[0:2])
        im_size_max = np.max(img.shape[0:2])
        im_scale = float(scale) / float(im_size_min)
        if np.round(im_scale * im_size_max) > max_size:
            im_scale = float(max_size) / float(im_size_max)
        img = cv.resize(img, None, None, fx=im_scale, fy=im_scale,
                        interpolation=cv.INTER_LINEAR)

        return img.transpose([2, 0, 1]).astype(np.float32), im_scale

    def draw_result(self, out, im_scale, clss, bbox, nms_thresh, conf):
        CV_AA = 16
        for cls_id in range(1, 21):
            _cls = clss[:, cls_id][:, np.newaxis]
            _bbx = bbox[:, cls_id * 4: (cls_id + 1) * 4]
            dets = np.hstack((_bbx, _cls))
            keep = nms(dets, nms_thresh)
            dets = dets[keep, :]

            inds = np.where(dets[:, -1] >= conf)[0]
            for i in inds:
                x1, y1, x2, y2 = map(int, dets[i, :4] / im_scale)
                cv.rectangle(out, (x1, y1), (x2, y2), (0, 0, 255), 2, CV_AA)
                ret, baseline = cv.getTextSize(
                    CLASSES[cls_id], cv.FONT_HERSHEY_SIMPLEX, 0.8, 1)
                cv.rectangle(out, (x1, y2 - ret[1] - baseline),
                             (x1 + ret[0], y2), (0, 0, 255), -1)
                cv.putText(out, CLASSES[cls_id], (x1, y2 - baseline),
                           cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1, CV_AA)

        return out




def main(args):
    ic = image_converter()

    rospy.init_node("image_converter")

    cv.namedWindow("Origin Image")
    cv.namedWindow("Result Image")
    cv.startWindowThread()

    rospy.spin()


if __name__=="__main__":
    main(sys.argv)


