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
from datetime import *

import pickle

parser = argparse.ArgumentParser(description='This script is make_pedestrian_dataset(raw)...')

parser.add_argument('-l', '--locate', default='test', type=str, \
        help='Please set location parameter')

args = parser.parse_args()
print args


class CreateDatasetLocalMapTrajectory:
    def __init__(self):
        self.track_vel_sub = \
            rospy.Subscriber("/kalman_filter/human_velocity", MarkerArray, self.velocityCallback)
        self.local_map_real_sub = \
            rospy.Subscriber("/local_map_real", OccupancyGrid, self.localMapCallback)
        
        self.view_trajs_pub = rospy.Publisher("/view_trajs", MarkerArray, queue_size=1)
        self.vis_view_trajs_pub = rospy.Publisher("/view_trajs/vis", MarkerArray, queue_size=1)

        self.human_exist = False

        self.color_list = \
                [ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0), ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), \
                 ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), \
                 ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0), ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0), \
                 ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)]

        #  self.color_list = \
                #  [ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0), ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), \
                 #  ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), \
                 #  ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0), ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0), \
                 #  ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0), ColorRGBA(r=0.5, g=0.5, b=0.0, a=1.0), \
                 #  ColorRGBA(r=0.5, g=0.0, b=0.5, a=1.0), ColorRGBA(r=0.0, g=0.5, b=0.5, a=1.0)]
        self.file_path \
            = "/home/amsl/ros_catkin_ws/src/master_thesis/make_pedestrian_dataset/datasets/raw/"
        self.abs_path = None
        self.date = datetime.now().strftime('%Y_%m_%d')
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

        self.trajectories = MarkerArray()
        self.trajectories.markers = [Marker() for i in xrange(len(self.color_list))]
        
        self.trajs_length_list = [0 for i in xrange(len(self.color_list))]
        self.trajs_velocity_vector_length_list = [0 for i in xrange(len(self.color_list))]

        self.dt = 0.05
        self.velocity_threshold = 5.0    #  歩行者としての速度の上限

        self.callback_count = 0    #  データセット作成する時間を計るをためのパラメータ(30秒保存)
        #  self.run_time_threshold = 1.0 / self.dt
        self.run_time_threshold = 10.0 / self.dt
        #  self.run_time_threshold = 20.0 / self.dt
        
        self.callback_count_break = 0    #  休憩時間を計るためのパラメータ(1秒)
        self.break_time_threshold = 0.5 / self.dt

        self.no_update_count_threshold = 0.05 / self.dt

        self.distance_threshold = (self.velocity_threshold * self.dt + 0.02) * 1.5

        self.dummy_position = Point(x=1000.0, y=1000.0, z=1000.0)
        self.dummy_velocity_vector = [0.0, Quaternion()]

        self.trajs_velocity_vector_list = [[] for i in xrange(len(self.color_list))]
        
        self.range_constraint = np.array([5.0, 5.0])
        
        self.trajs_no_update_count_list = [0 for j in xrange(len(self.color_list))]


        self.grid_map_list = []
        self.grid_map = None
        self.local_map_flag = False

        self.create_traj = False

        self.scenario_trajs_length_list = None

        self.frame_id = "/velodyne"

        self.reset_flag = True


    def set_save_data(self):
        self.local_map_and_trajectories_data['map'] = self.grid_map_list
        self.local_map_and_trajectories_data['traj'] = self.trajectories
        self.local_map_and_trajectories_data['velocity'] = self.trajs_velocity_vector_list

    def save_dataset(self, no_pedestrian=False):
        print "Now Saving!!!"
        self.set_save_data()

        date_time = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        #  print "date_time : ", date_time
        filename = None
        if not no_pedestrian:
            filename = self.abs_path +  "%s_scenario_%d_raw_dataset.pkl" \
                    % (date_time, self.scenario_count)
        else:
            filename = self.abs_path +  "_%s_scenario_%d_raw_dataset.pkl" \
                    % (date_time, self.scenario_count)

        print "File : ", filename
        with open(filename, mode='wb') as f:
            pickle.dump(self.local_map_and_trajectories_data, f)


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
        #  print "========================== velocityCallback ================================"
        #  print "msg : ", msg
        #  print len(msg.markers)

        self.create_traj = False

        if len(msg.markers) == 0:
            print "!!!!!!!!!!!!!!!!!!! No pedestrians !!!!!!!!!!!!!!!!!!!!!!!!"
            print "************************ Reset *************************"
            if self.reset_flag and self.human_exist:
                save_trajs_flag_list = np.array(self.scenario_trajs_length_list) \
                        != np.array([0 for i in xrange(len(self.color_list))])
                if save_trajs_flag_list.any():
                    self.scenario_count += 1
                    self.save_dataset(no_pedestrian=True)

            self.human_exist = False		
            self.reset_all_variables(msg)
        else:
            if not self.local_map_flag:
                print "============= No Subscribe local map!!! ==========="
            else:
                self.human_exist = True
                self.reset_flag = True
                self.callback_count += 1
                
                print "self.callback_count : ", self.callback_count
                if self.callback_count <= self.run_time_threshold:
                    print "################## Creating datasets ######################"
                    print "############ Run Time : %.3f [s] ########" \
                            % (self.callback_count*self.dt)
                    
                    self.scenario_trajs_length_list = self.create_trajectories(msg)
                    self.grid_map_list.append(self.grid_map)
                    print "len(self.grid_map_list) : ", len(self.grid_map_list)

                    #  if self.reset_flag:
                        #  print "********** Reset(no pedestrians in the range) **********"
                        #  self.reset_all_variables(msg)
                 	
                    #  細かい軌道のやつを排除しようとした
                    #  temp_flag_list = np.array(self.scenario_trajs_length_list) \
                            #  < np.array([20 for i in xrange(len(self.color_list))])
                    #  if temp_flag_list.all():
                        #  print "********* Reset!! **************"
                        #  self.callback_count = 0
                else:
                    print "################ Finish creating datasets ###############"
                    print "***************** Reset ********************"
                    print "##### Break Time : %.3f [s] ####" % (self.callback_count_break*self.dt)
                    self.callback_count_break += 1
                    
                    if self.callback_count_break > self.break_time_threshold:
                        self.scenario_count += 1
                        self.save_dataset()
                                
                        self.reset_all_variables(msg)


    def reset_all_variables(self, msg):
        self.reset_flag = False

        self.reset_trajectories(msg)
        self.trajs_length_list = [0 for i in xrange(len(self.color_list))]
        self.trajs_velocity_vector_list = [[] for i in xrange(len(self.color_list))]
        
        self.trajs_no_update_count_list = [0 for j in xrange(len(self.color_list))]
        
        self.grid_map_list = []
        
        self.callback_count_break = 0
        self.callback_count = 0


    def reset_trajectories(self, human_trajs):
        del self.trajectories.markers[:]
        self.trajectories.markers = [Marker() for i in xrange(len(self.color_list))]
        
        for i in xrange(len(self.trajectories.markers)):
            self.trajectories.markers[i].type = Marker.LINE_STRIP
            self.trajectories.markers[i].action = Marker.ADD
            self.trajectories.markers[i].scale.x = 0.30
            self.trajectories.markers[i].lifetime = rospy.Duration()
    
    def calc_predict_position(self, before_position, before_velocity_vector, dt):
        q = (before_velocity_vector[1].x, before_velocity_vector[1].y, \
                before_velocity_vector[1].z, before_velocity_vector[1].w)
        linear = before_velocity_vector[0]
        #  print "linear : ", linear
        angular = tf.transformations.euler_from_quaternion(q)[2]
        #  print "angular : ", angular
        predict_position = Point()
        predict_position.x = before_position.x + linear*math.cos(angular)*dt
        predict_position.y = before_position.y + linear*math.sin(angular)*dt
        predict_position.z = before_position.z
        #  #  print "predict_position : "
        #  #  print predict_position

        return predict_position
    
    def calc_dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)

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



    def create_trajectories(self, human_trajs):
        num_humans = len(human_trajs.markers)
        print "num_humans : ", num_humans
        if num_humans >= len(self.color_list):
            num_humans = len(self.color_list)
            print "num_humans(max) : ", num_humans
        
        num_trajs = len(self.color_list)
        print "num_trajs : ", num_trajs
        """
        予測ステップ
        """
        time_stamp = rospy.Time.now()
        for i in xrange(num_trajs):
            #  print "**************** traj_id : ", i, " *************** "
            if self.trajs_length_list[i] == 0:
                #  print "!!!!!!!!!!! Set init dummy point and velocity_vector !!!!!!!!!!"
                self.set_trajectory_parameter(i, time_stamp, \
                                              self.dummy_position, \
                                              self.dummy_velocity_vector, append=True)
            else:
                before_position = self.trajectories.markers[i].points[-1]
                before_velocity_vector = self.trajs_velocity_vector_list[i][-1]
                #  print "before_position : "
                #  print before_position
                #  print "before_velocity_vector : ", before_velocity_vector

                if before_position == self.dummy_position:
                    self.set_trajectory_parameter(i, time_stamp, \
                                                  self.dummy_position, \
                                                  self.dummy_velocity_vector, append=True)
                else:
                    self.trajs_no_update_count_list[i] += 1
                    predict_position \
                            = self.calc_predict_position(before_position, \
                                                         before_velocity_vector, self.dt)
                    #  print "predict_position : "
                    #  print predict_position

                    predict_range_flag = np.array([math.fabs(predict_position.x), \
                                                   math.fabs(predict_position.y)]) \
                                       <= self.range_constraint
                    if predict_range_flag.all():
                        if self.trajs_no_update_count_list[i] \
                                <= self.no_update_count_threshold:
                            self.set_trajectory_parameter(i, time_stamp, \
                                                          predict_position, \
                                                          before_velocity_vector, \
                                                          append=True)
        
        """
	人認識で認識できた人数分ループを回す
        """
        for i in xrange(num_humans):
            print "------------------ human_id : ", i, "-------------------"
            #  print "human_trajs.markers[", i, "].scale.x : ", human_trajs.markers[i].scale.x

            dist_list = [1000.0 for k in xrange(num_trajs)]

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

            """
            認識された人の位置が計測範囲内かを確認
            """
            range_flag = np.array([math.fabs(position.x), math.fabs(position.y)]) \
                    <= self.range_constraint
            if range_flag.all():
                #  速度ベクトルがあまりにも暴れているのは排除したい
                if 0.5 <= velocity and velocity <= self.velocity_threshold:
                    self.create_traj = True

                    traj_position = self.trajectories.markers[i].points[-1]
                    traj_velocity_vector = self.trajs_velocity_vector_list[i][-1]

                    #  """
                    #  もし、対応する軌道メモりの最新の点がダミーなら
                    #  """
                    #  if traj_position != self.dummy_position:
                    """
                    すべての軌道メモりの最新の位置(予測位置)と観測位置から対応する奴を探す
                    """
                    #  print "!!!!!!!!!!!!! No Dummy !!!!!!!!!!!!"
                    for j in xrange(num_trajs):
                        #  print "************ index_trajs : ", j, " *************** "
                        traj_position_ = self.trajectories.markers[j].points[-1]
                        traj_velocity_vector_ = self.trajs_velocity_vector_list[j][-1]
                        #  print "traj_position_ : ", traj_position_

                        if traj_position_ != self.dummy_position:
                            dist_list[j] = self.calc_dist(traj_position_, position)

                    #  print "dist_list : ", dist_list
                    min_dist = min(dist_list)
                    #  print "min_dist : ", min_dist
                    index_min_dist = np.argmin(np.asarray(dist_list))

                    #  print "index_min_dist : "
                    #  print index_min_dist

                    #  print "self.distance_threshold : ", self.distance_threshold
                    """
                    もっとも近くしきい値以内の予測地点を持つ軌道メモりと対応付ける
                    """
                    if min_dist <= self.distance_threshold:
                        #  print "!!!!!!!!!!!!!!! ADD !!!!!!!!!!!!!!!!"
                        self.set_trajectory_parameter(index_min_dist, time_stamp, \
                                                      position, velocity_vector)
                        self.trajs_no_update_count_list[index_min_dist] = 0
                    #  else:
                        #  for j in xrange(num_trajs):
                            #  traj_position_ = self.trajectories.markers[j].points[-1]
                            #  traj_velocity_vector_ = self.trajs_velocity_vector_list[j][-1]
                            #  #  print "traj_position_ : ", traj_position_
                            #  if traj_position_ == self.dummy_position:
                                #  self.set_trajectory_parameter(j, time_stamp, \
                                                              #  position, velocity_vector)
                    else:
                        if traj_position == self.dummy_position:
                            #  print "!!!!!!!!!!!!!!!! New !!!!!!!!!!!!!!"
                            #  print "!!!!!!!!!!!!!!! add !!!!!!!!!!!!!!!!"
                            self.set_trajectory_parameter(i, time_stamp, \
                                                          position, velocity_vector)
                            self.trajs_no_update_count_list[i] = 0

            else:
                print "!!!!!!! No pedestrian in range !!!!!!"


        
        for j in xrange(num_trajs):
            self.trajs_length_list[j] = len(self.trajectories.markers[j].points)
            self.trajs_velocity_vector_length_list[j] = len(self.trajs_velocity_vector_list[j])
        print "self.trajs_length_list : ", self.trajs_length_list
        print "self.trajs_velocity_vectorlength_list : ", self.trajs_velocity_vector_length_list

        self.view_trajs_pub.publish(self.trajectories)


        return self.trajs_length_list


def main():
    rospy.init_node("make_pedestrian_datasets")

    cdlt = CreateDatasetLocalMapTrajectory()
    print "Here we go!!!!"
    
    #  loop_rate = rospy.Rate(20)

    
    rospy.spin()

if __name__ == "__main__":
    main()
