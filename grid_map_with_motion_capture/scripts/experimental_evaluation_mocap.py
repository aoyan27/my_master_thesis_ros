#!/usr/bin/env python
#coding:utf-8

import rospy 
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud, PointCloud2
from nav_msgs.msg import Odometry

import numpy as np

import math
import sys
import time

class MocapExperiment:
    def __init__(self):
        self.my_traj_sub \
                = rospy.Subscriber("/my_agent_velocity", Marker, self.myTrajCallback)
        self.enemy_traj_sub \
                = rospy.Subscriber("/other_agents_velocity", MarkerArray, self.otherTrajCallback)
        self.tiny_sub = rospy.Subscriber("/tinypower/odom", Odometry, self.tinyCallback)

        #  self.my_traj_pub = rospy.Publisher("/my_traj", Marker, queue_size=1)
        #  self.other_traj_pub = rospy.Publisher("/other_traj", Marker, queue_size=1)
        self.my_traj_pub = rospy.Publisher("/my_traj/dwa", Marker, queue_size=1)
        self.other_traj_pub = rospy.Publisher("/other_traj/dwa", Marker, queue_size=1)

        self.color_list = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), \
                           ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)]

        self.sub_my_velocity = Marker()
        self.sub_other_velocity = MarkerArray()
        self.sub_my_velocity_flag = False
        self.sub_other_velocity_flag = False

        self.my_traj = Marker()
        self.set_init_traj_parameter(self.my_traj, 0)
        self.other_traj = Marker()
        self.set_init_traj_parameter(self.other_traj, 1)

        self.clearance_list = []
        self.velocity_list = []
        
        self.my_traj_sub_flag = False
        self.other_traj_sub_flag = False

        self.start_flag = False
        self.first_flag = True

        self.start_time = None

    def tinyCallback(self, msg):
        print "tiny_vel : ", msg.twist.twist.linear.x
        if msg.twist.twist.linear.x != 0.0:
            self.start_flag = True
            if self.first_flag:
                self.start_time = time.time()
                self.first_flag = False
        else:
            self.start_flag = False

    def myTrajCallback(self, msg):
        #  print "==== my agent ===="
        self.sub_my_velocity = msg
        print "self.sub_my_velocity : ", self.sub_my_velocity.scale.x
        self.sub_my_velocity_flag = True

    def otherTrajCallback(self, msg):
        #  print "===== other agents ====="
        self.sub_other_velocity = msg
        self.sub_other_velocity_flag = True

    def set_init_traj_parameter(self, traj, id):
        traj.ns = "traj_%d " % id
        traj.color = self.color_list[id]
        traj.type = Marker.LINE_STRIP
        traj.action = Marker.ADD
        traj.scale.x = 0.05
        traj.lifetime = rospy.Duration()

    def calc_dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y - b.y)**2)

    def append_traj(self, header):
        if self.sub_my_velocity_flag and self.sub_other_velocity_flag:
            if self.start_flag:
                self.sub_my_velocity_flag = False
                self.sub_other_velocity_flag = False

                self.my_traj.header = header
                self.other_traj.header = header

                self.my_traj.points.append(self.sub_my_velocity.pose.position)
                self.other_traj.points.append(self.sub_other_velocity.markers[0].pose.position)

                self.velocity_list.append(self.sub_my_velocity.scale.x)

                self.clearance_list.append(self.calc_dist(self.my_traj.points[-1], \
                                                          self.other_traj.points[-1]))

                print "appended trajectory!!"
                self.my_traj_pub.publish(self.my_traj)
                self.other_traj_pub.publish(self.other_traj)
            else:
                print "Finish !!"
                if len(self.clearance_list) != 0:
                    print "self.clearance_list : ", self.clearance_list
                    print "min_clearance : ", min(self.clearance_list) - 0.434

                    print "self.velocity_list : ", self.velocity_list
                    print "average_velocity : ", sum(self.velocity_list) / len(self.velocity_list)

                    goal_time = time.time() - self.start_time
                    print ("Goal Time:{0}".format(goal_time) + "[sec]")

                    sys.exit(1)
                


def main():
    rospy.init_node("experimental_evaluation_mocap")

    me = MocapExperiment()


    loop_rate = rospy.Rate(20)

    print "Here we go!!"

    while not rospy.is_shutdown():
        header = Header(frame_id="/input_map", stamp=rospy.Time.now())

        me.append_traj(header)

        loop_rate.sleep()


if __name__ == "__main__":
    main()
