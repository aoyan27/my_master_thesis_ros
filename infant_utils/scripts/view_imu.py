#!/usr/bin/env python
#coding:utf-8

import rospy 
from sensor_msgs.msg import Imu
import tf

import math

class ViewImuData:
    def __init__(self):
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imuCallback)

    def imuCallback(self, msg):
        quaternion = msg.orientation
        e = tf.transformations.euler_from_quaternion((\
                quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        print "Euler : "
        print "Roll : %.4f, Pitch : %.4f, Yaw : %.4f" \
                % (math.degrees(e[0]), math.degrees(e[1]), math.degrees(e[2]))


def main():
    rospy.init_node("view_imu")
    vid = ViewImuData()

    print "Here we go!!"

    rospy.spin()


if __name__ == "__main__":
    main()
