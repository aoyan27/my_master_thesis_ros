#!/usr/bin/env python
#coding:utf-8

import argparse

import rospy
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
import tf

import numpy as np

import sys
import os
import copy
import math
import time
from datetime import datetime
import atexit

import pickle

parser = argparse.ArgumentParser(description='This script is make_pedestrian_dataset(raw)...')

parser.add_argument('-l', '--locate', default='test', type=str, \
        help='Please set location parameter')

args = parser.parse_args()
print args


class CreateDatasetLocalMapTrajectory:
    def __init__(self):
        self.track_vel_sub = \
            rospy.Subscriber("/velocity_arrows", MarkerArray, self.velocityCallback)
        self.local_map_real_sub = \
            rospy.Subscriber("/local_map_real", OccupancyGrid, self.localMapCallback)
        
        self.view_trajs_pub = rospy.Publisher("/view_trajs", MarkerArray, queue_size=1)

        np.random.seed(5)
        self.max_human = 30000
        self.color_list = [ColorRGBA(r=np.random.rand(), g=np.random.rand(), b=np.random.rand(), \
                                     a=1.0) for i in xrange(self.max_human)]

        self.trajectories = MarkerArray()
        self.trajectories.markers = [Marker() for i in xrange(len(self.color_list))]
        self.trajs_velocity_vector_list = [[] for i in xrange(len(self.color_list))]
        self.reset_trajectories_and_velocities()

        self.trajs_length_list = [0 for i in xrange(len(self.color_list))]
        self.trajs_velocity_vector_length_list = [0 for i in xrange(len(self.color_list))]

        self.dt = 0.05
        self.run_time_threshold = 7200.0 / self.dt
        self.callback_count = 0    #  データセット作成する時間を計るをためのパラメータ(30秒保存)


        self.file_path \
            = "/home/amsl/ros_catkin_ws/src/master_thesis/make_pedestrian_dataset/datasets/raw/"
        self.abs_path = None
        self.start_time = datetime.now()
        self.date = self.start_time.strftime('%Y_%m_%d')
        #  print "date : ", date
        if not os.path.exists(self.file_path+self.date):
            os.makedirs(self.file_path+self.date)
        if not os.path.exists(self.file_path+self.date+"/"+args.locate):
            os.makedirs(self.file_path+self.date+"/"+args.locate)
        if len(os.listdir(self.file_path+self.date+"/"+args.locate)) != 0:
            sys.stderr.write('\033[31mError!! This directory already has some files!!\033[0m\n')
            sys.exit(1)
        else:
            self.abs_path = self.file_path + self.date + "/" + args.locate + "/"
        print "self.abs_path : ", self.abs_path
        self.local_map_and_trajectories_data = {}
        self.scenario_count = 0

        self.dummy_position = Point(x=1000.0, y=1000.0, z=1000.0)
        self.dummy_velocity_vector = [0.0, Quaternion()]

        self.grid_map_list = []
        self.grid_map = None
        self.local_map_flag = False

        self.frame_id = "/velodyne"

        self.trajs_no_update_count_list = [0 for j in xrange(len(self.color_list))]

        self.range_constraint = np.array([5.0, 5.0])



    def set_save_data(self):
        self.local_map_and_trajectories_data['map'] = self.grid_map_list
        self.local_map_and_trajectories_data['traj'] = self.trajectories
        self.local_map_and_trajectories_data['velocity'] = self.trajs_velocity_vector_list
        #  print " self.trajectories : ",  self.trajectories.markers[0:3]
        #  print "self.trajs_velocity_vector_list : ", self.trajs_velocity_vector_list[0:3]

    def save_dataset(self, no_pedestrian=False):
        print "Now Saving!!!"
        self.set_save_data()
        finish_time = datetime.now()
        date_time = finish_time.strftime('%Y_%m_%d_%H_%M_%S')
        #  print "date_time : ", date_time
        delta_time = finish_time - self.start_time
        total_time = delta_time.total_seconds()
        print "Total time : ", total_time, "[s]"
        filename = None
        if not no_pedestrian:
            filename = self.abs_path +  "%s_scenario_%d_total_%f_raw_dataset.pkl" \
                    % (date_time, self.scenario_count, total_time)
        else:
            filename = self.abs_path +  "_%s_scenario_%d_total_%f_raw_dataset.pkl" \
                    % (date_time, self.scenario_count, total_time)

        print "File : ", filename
        #  with open(filename, mode='wb') as f:
            #  pickle.dump(self.local_map_and_trajectories_data, f)


    def localMapCallback(self, msg):
        #  print "========================== localMapCallback ================================"
        self.local_map_flag = True
        cell_size = msg.info.resolution
        rows = msg.info.height
        cols = msg.info.width
        map_data = msg.data

        self.grid_map = np.asarray(map_data, dtype=np.uint8).reshape((rows, cols))
        #  print "grid_map : "
        #  print self.grid_map
        #  self.show_grid_map(self.grid_map)

    def show_grid_map(self, grid_map):
        if not grid_map.ndim == 2:
            rospy.logerr('Error occurred!! Check grid dimentions!!')
        else:
            vis_grid_map = copy.deepcopy(grid_map)

            index = np.where(vis_grid_map == 100)
            vis_grid_map[index] = 1
            for row in vis_grid_map:
                print "|",
                for i in row:
                    print "%2d" % i,
                print "|"
        
    def velocityCallback(self, msg):
        print "============= velocity subscrube ================="
        if not self.local_map_flag:
            print "============= No Subscribe local map!!! ==========="
        else:
            if self.callback_count <= self.run_time_threshold:
                print "################## Creating datasets ######################"
                print "######## Callback Time : %.3f [s] ########" % (self.callback_count*self.dt)

                self.grid_map_list.append(self.grid_map)
                self.get_trajectories(msg)
                self.callback_count += 1
            else:
                print "################ Finish creating datasets ###############"
                self.save_dataset()
                self.callback_count = 0



    def reset_trajectories_and_velocities(self):
        del self.trajectories.markers[:]
        self.trajectories.markers = [Marker() for i in xrange(len(self.color_list))]

        self.trajs_velocity_vector_list = [[] for i in xrange(len(self.color_list))]

        for i in xrange(len(self.trajectories.markers)):
            self.trajectories.markers[i].type = Marker.LINE_STRIP
            self.trajectories.markers[i].action = Marker.ADD
            self.trajectories.markers[i].scale.x = 0.1
            self.trajectories.markers[i].lifetime = rospy.Duration(5.0)
            #  self.trajectories.markers[i].lifetime = rospy.Duration()

    def reset_all_variables(self):
        self.reset_trajectories_and_velocities()
        self.trajs_length_list = [0 for i in xrange(len(self.color_list))]
        self.trajs_velocity_vector_length_list = [0 for i in xrange(len(self.color_list))]
        
        self.trajs_no_update_count_list = [0 for j in xrange(len(self.color_list))]
        
        self.grid_map_list = []

    def set_trajectory_parameter(self, num_id, time_stamp, point, velocity_vector, append=False):
        self.trajectories.markers[num_id].header.frame_id = self.frame_id
        self.trajectories.markers[num_id].header.stamp = time_stamp
        self.trajectories.markers[num_id].ns = "human_%d" % num_id
        self.trajectories.markers[num_id].id = num_id
        self.trajectories.markers[num_id].color = self.color_list[num_id]

        if not append:
            self.trajectories.markers[num_id].points[-1] = point
            self.trajs_velocity_vector_list[num_id][-1] = velocity_vector
        else:
            self.trajectories.markers[num_id].points.append(point)
            self.trajs_velocity_vector_list[num_id].append(velocity_vector)
    

    def get_trajectories(self, human_trajs):
        start = time.time()
        num_human = len(human_trajs.markers)
        print "num_human : ", num_human
        if num_human >= len(self.color_list):
            num_humans = len(self.color_list) - 1
            print "num_humans(max) : ", num_humans
        
        time_stamp = rospy.Time.now()
        #  for i in xrange(self.max_human):
            #  if self.trajs_length_list[i] == 0:
                #  #  print "!!!!!!!!!!! Set init dummy point and velocity_vector !!!!!!!!!!"
                #  self.set_trajectory_parameter(i, time_stamp, \
                                              #  self.dummy_position, \
                                              #  self.dummy_velocity_vector, append=True)
            #  else:
                #  before_position = self.trajectories.markers[i].points[-1]
                #  before_velocity_vector = self.trajs_velocity_vector_list[i][-1]

                #  #  if before_position == self.dummy_position:
                #  self.set_trajectory_parameter(i, time_stamp, \
                                              #  self.dummy_position, \
                                              #  self.dummy_velocity_vector, append=True)
                #  else:
                    #  self.trajs_no_update_count_list[i] += 1
                    #  print " self.trajs_no_update_count_list[i] : ",  self.trajs_no_update_count_list[i]
                    #  if self.trajs_no_update_count_list[i] < 2:
                        #  self.set_trajectory_parameter(i, time_stamp, \
                                                      #  self.dummy_position, \
                                                      #  self.dummy_velocity_vector, append=True)

        for i in xrange(num_human):
            human_id = human_trajs.markers[i].id
            print "------------------ human_id : ", human_id, "-------------------"
            """
            認識された人の位置、速度ベクトル、方位を格納
            """
            position = human_trajs.markers[i].pose.position
            print "position : "
            print position
            velocity = human_trajs.markers[i].scale.x
            #  print "velocity : ", velocity
            orientation = human_trajs.markers[i].pose.orientation
            #  print "orientation : "
            #  print orientation
            velocity_vector = [velocity, orientation]
            print "velocity_vector : "
            print velocity_vector

            range_flag = np.array([math.fabs(position.x), math.fabs(position.y)]) \
                    <= self.range_constraint
            print "range_flag : ", range_flag 
            if range_flag.all():
                if human_id < self.max_human:
                    self.trajs_no_update_count_list[i] = 0
                    #  if self.trajs_length_list[i] == 0:
                    self.set_trajectory_parameter(human_id, time_stamp, position, velocity_vector, append=True)
                    print "append!!"
                    #  else:
                        #  self.set_trajectory_parameter(human_id, time_stamp, position, velocity_vector)
                    #  if self.trajs_length_list[i] == 0:
                        #  self.set_trajectory_parameter(human_id, time_stamp, position, velocity_vector, append=True)
                    #  else:
                        #  self.set_trajectory_parameter(human_id, time_stamp, position, velocity_vector)



        for j in xrange(self.max_human):
            self.trajs_length_list[j] = len(self.trajectories.markers[j].points)
            self.trajs_velocity_vector_length_list[j] = len(self.trajs_velocity_vector_list[j])
        #  print "self.trajs_length_list : ", self.trajs_length_list
        #  print "self.trajs_velocity_vectorlength_list : ", self.trajs_velocity_vector_length_list
        
        #  print "self.trajectories : ", self.trajectories
        
        self.get_vis_trajs(human_trajs)
        self.view_trajs_pub.publish(self.vis_trajs)

        elapsed_time = time.time() - start
        print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")

    def get_vis_trajs(self, human_trajs):
        num_humans = len(human_trajs.markers)
        self.vis_trajs = MarkerArray()
        self.vis_trajs.markers = [Marker() for i in xrange(num_humans)]

        for i in xrange(num_humans):
            human_id = human_trajs.markers[i].id
            self.vis_trajs.markers[i] = self.trajectories.markers[human_id]


def main():
    rospy.init_node("make_pedestrian_datasets")

    cdlt = CreateDatasetLocalMapTrajectory()
    print "Here we go!!!!"
    
    #  loop_rate = rospy.Rate(20)
    #  print 'Registering'
    #  atexit.register(cdlt.save_dataset)
    #  print 'Registered'

    
    rospy.spin()

if __name__ == "__main__":
    main()
