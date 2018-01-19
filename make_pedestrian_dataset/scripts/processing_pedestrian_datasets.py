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
import cv2 as cv
import matplotlib.pyplot as plt

import sys
import os
import copy
import math
from datetime import *

import pickle

parser = argparse.ArgumentParser(description='This script is processing_pedestrian_dataset...')

parser.add_argument('-ldp', '--load_dataset_path', \
        default='/make_pedestrian_dataset/datasets/raw/test', \
        type=str, help='Please set load dataset_path')

args = parser.parse_args()
print args

class ProcessingPedestrianDataset:
    def __init__(self):
        self.pedestrian_markers = None
        self.grid_map_list = None

        self.dataset_image_list = None
        self.dataset_state_list = None
        self.dataset_action_list = None
        self.dataset_reward_map_list = None


        self.discreate_action_list = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        self.n_discreate_action = len(self.discreate_action_list)
        self.dirs = \
                {0: '>', 1: 'dr', 2: 'v', 3: 'dl', 4: '<', 5: 'ul', 6: '^', 7: 'ur', 8: '-'}
        self.discreate_movement \
                = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 0)]

        self.height = None
        self.width = None
        self.cell_size = 0.25
        self.origin_x = None
        self.origin_y = None

        self.dummy_position = Point(x=1000.0, y=1000.0, z=1000.0)

        self.pickup_index = None
        self.pedestrian_threshold = 1.0    #  人間の幅を設定

        self.file_path \
            = "/home/amsl/ros_catkin_ws/src/master_thesis/make_pedestrian_dataset/datasets/processing/"
        self.abs_path = None
        self.date = datetime.now().strftime('%Y_%m_%d')
        #  print "date : ", date
        self.load_path = args.load_dataset_path
        self.locate = self.load_path.split('/')[-2]

        if not os.path.exists(self.file_path+self.date):
            os.makedirs(self.file_path+self.date)
        if not os.path.exists(self.file_path+self.date+"/"+self.locate):
            os.makedirs(self.file_path+self.date+"/"+self.locate)
        
        if len(os.listdir(self.file_path+self.date+"/"+self.locate)) != 0:
            sys.stderr.write('\033[31mError!! This directory already has some files!!\033[0m\n')
            sys.exit(1)
        else:
            self.abs_path = self.file_path + self.date + "/" + self.locate + "/"
        print "self.abs_path : ", self.abs_path
        

    def load_raw_dataset(self, filename):
        with open(filename, mode='rb') as f:
            data = None
            data = pickle.load(f)

        self.grid_map_list = data['map']
        _, self.height, self.width = np.asarray(self.grid_map_list).shape
        print "self.height, self.width : ", self.height, self.width
        self.origin_y = self.height * self.cell_size / 2.0
        self.origin_x = self.width * self.cell_size / 2.0
        print "self.origin_y, self.origin_x : ", self.origin_y, self.origin_x

        self.pedestrian_markers = data['traj']
        print "Load %d grid_map_data!!!" % len(self.grid_map_list)
        for i in xrange(len(self.pedestrian_markers.markers)):
            print "Load %d pedestrian_trajectory data!!!" \
                    % len(self.pedestrian_markers.markers[i].points)

    def view_image(self, array, title):
        image = cv.cvtColor(array.astype(np.uint8), cv.COLOR_GRAY2RGB)
        #  print image
        plt.imshow(255 - 255*image, interpolation="nearest")
        plt.title(title)
        #  plt.show()
        plt.pause(0.05)

    def view_image_cui(self, array):
        for row in array:
            print "|",
            for i in row:
                print "%d" % i,
            print "|"

    def grid2image(self, array):
        image = copy.deepcopy(np.asarray(array))
        #  print "image : "
        #  print image
        index = np.where(image==100)
        #  print "index : ", index
        image[index] = 1

        #  self.view_image_cui(image[0])
        #  self.view_image(image[0], 'grid_image')

        return image

    def get_crcl_range(self, state):
        continuous_state = np.asarray(state) * self.cell_size
        #  print "cointinuous_state : ", continuous_state
        min_continuous_state = continuous_state - np.array([self.pedestrian_threshold/2]*2)
        min_continuous_state_shape = min_continuous_state.shape
        #  print min_continuous_state_shape
        min_continuous_state = map(lambda x: 0 if x<0 else x, min_continuous_state.reshape(-1))
        min_continuous_state = np.asarray(min_continuous_state).reshape(min_continuous_state_shape)
        #  print "min_continuous_state : ", min_continuous_state
        
        max_value = (self.width-1) * self.cell_size
        max_continuous_state = continuous_state + np.array([self.pedestrian_threshold/2]*2)
        max_continuous_state_shape = max_continuous_state.shape
        #  print max_continuous_state_shape
        max_continuous_state = map(lambda x: max_value \
                if x>max_value else x, max_continuous_state.reshape(-1))
        max_continuous_state = np.asarray(max_continuous_state).reshape(max_continuous_state_shape)
        #  print "max_continuous_state : ", max_continuous_state

        min_discreate_state = (min_continuous_state / self.cell_size).astype(np.int32)
        #  print "min_discreate_state : ", min_discreate_state
        max_discreate_state = (max_continuous_state / self.cell_size).astype(np.int32)
        #  print "max_discreate_state : ", max_discreate_state

        return min_discreate_state, max_discreate_state
    
    def crcl_pedestrian(self, image_list, state_list, pickup_index):
        #  print len(image_list)
        #  print len(state_list)
        #  print pickup_index
        crcl_image_list = []
        image_list_ = copy.deepcopy(image_list)
        num_state_list_size = len(state_list)
        if num_state_list_size != 0:
            #  print "processing!!"
            for j in xrange(num_state_list_size):
                min_discreate_state, max_discreate_state = self.get_crcl_range(state_list)
                image_list_[pickup_index[j]]\
                        [min_discreate_state[j][0]:max_discreate_state[j][0], \
                         min_discreate_state[j][1]:max_discreate_state[j][1]] = 0
                crcl_image_list.append(image_list_[pickup_index[j]])
                #  print "************* j : ", j, " ************ " 
                #  self.view_image_cui(image_list_[pickup_index[j]])
                #  self.view_image(image_list_[pickup_index[j]], 'grid_image')
                #  print "----------------------=="
                #  self.view_image_cui(image_list[pickup_index[j]])
                #  self.view_image(image_list[pickup_index[j]], 'grid_image')

        return crcl_image_list


    def get_grid_image_list(self, grid_list, state_list, pickup_index):
        image_list = self.grid2image(grid_list)
        num_pedestrians = len(state_list)
        grid_image_list = [[] for i in xrange(num_pedestrians)]
        #  print "grid_image_list : "
        #  print grid_image_list
        for i in xrange(num_pedestrians):
            #  print "================= i : ", i, "================="
            grid_image_list[i] = self.crcl_pedestrian(image_list, state_list[i], pickup_index[i])
        #  print "grid_image_list : "
        #  print grid_image_list
        return grid_image_list


    def continuous2discreate(self, continuous_point):
        #  print "continuous_point : "
        #  print continuous_point.x
        discreate_y = int((continuous_point.y + self.origin_y) /self.cell_size)
        discreate_x = int((continuous_point.x + self.origin_x) /self.cell_size)
        #  print "discreate_y, discreate_x : ", discreate_y, discreate_x

        return [discreate_y, discreate_x]

    def get_state_list(self, marker_array):
        num_pedestrians = len(marker_array.markers)
        state_list = [[] for i in xrange(num_pedestrians)]
        self.pickup_index = [[] for i in xrange(num_pedestrians)]

        discreate_dummy_position = self.continuous2discreate(self.dummy_position)

        for i in xrange(num_pedestrians):
            num_state_list_size = len(marker_array.markers[i].points)
            #  print "marker_array.markers[", i, "] : "
            #  print marker_array.markers[i].points
            single_pedestrian_state_list = None
            no_dummy_state_list = list(filter(lambda x: x!=self.dummy_position, \
                                              marker_array.markers[i].points))
            #  print "no_dummy_state_list : ", no_dummy_state_list
            if len(no_dummy_state_list) != 0:
                no_dummy_index \
                        = np.where(np.asarray(marker_array.markers[i].points) \
                                   == no_dummy_state_list[0])[0][0]
                #  print "no_dummy_index : ", no_dummy_index
                single_pedestrian_state_list \
                    = [self.continuous2discreate(marker_array.markers[i].points[no_dummy_index])] 
                self.pickup_index[i].append(no_dummy_index)
                #  print "single_pedestrian_state_list : "
                #  print single_pedestrian_state_list
                for j in xrange(1, num_state_list_size):
                    current_discreate_point \
                            = self.continuous2discreate(marker_array.markers[i].points[j])
                    #  print "curent_discreate_point : ", current_discreate_point
                    before_discreate_point \
                            = self.continuous2discreate(marker_array.markers[i].points[j-1])
                    #  print "before_discreate_point : ", before_discreate_point
                    if current_discreate_point != before_discreate_point \
                           and current_discreate_point != discreate_dummy_position:
                        #  print "current_point_ : ", current_discreate_point
                        single_pedestrian_state_list.append(current_discreate_point)
                        self.pickup_index[i].append(j)

                    #  print "single_pedrstrian_state_list : "
                    #  print single_pedestrian_state_list
                state_list[i] = single_pedestrian_state_list
        #  print "state_list : "
        #  print state_list

        return state_list

    def get_action(self, current_state, next_state):
        #  print "next_state : ", next_state
        #  print "current_state : ", current_state
        diff_state = np.asarray(next_state) - np.asarray(current_state)
        #  print "diff_state : ", diff_state
        tmp_diff_state = map(lambda x: abs(x), diff_state)

        diff_state_flag = np.asarray(tmp_diff_state) > 1
        #  print "diff_state_flag : ", diff_state_flag
        if diff_state_flag.any():
            index = np.where(diff_state_flag==True)
            diff_state[index] = diff_state[index] / diff_state[index]
        #  print "diff_state : ", diff_state
        return self.discreate_movement.index(tuple(diff_state))
        

    def get_action_list(self, state_list):
        #  print "state_list : ", state_list
        state_list_ = copy.deepcopy(state_list)
        num_pedestrians = len(state_list_)
        action_list = [[] for i in xrange(num_pedestrians)]
        for i in xrange(num_pedestrians):
            single_pedestrian_state_action_list = None
            num_state_list_size = len(state_list_[i])
            if num_state_list_size != 0:
                state_list_[i].append(state_list_[i][-1])
                #  print "state_list_ : ", state_list_
                single_pedestrian_action_list \
                    = [self.get_action(state_list_[i][j], state_list_[i][j+1]) \
                       for j in xrange(num_state_list_size)]
                #  print "single_pedestrian_action_list : "
                #  print single_pedestrian_action_list
                action_list[i] = single_pedestrian_action_list
        #  print "action_list : "
        #  print action_list


        return action_list

    def get_reward_map(self, state_list):
        num_pedestrians = len(state_list)
        reward_map_list = [[] for i in xrange(num_pedestrians)]
        for i in xrange(num_pedestrians):
            num_state_list_size = len(state_list[i])
            if num_state_list_size != 0:
                reward_map = np.zeros((self.height, self.width), dtype=np.uint8)
                goal_index = state_list[i][-1]
                reward_map[tuple(goal_index)] = 1
                #  print "reward_map : "
                #  print reward_map
                #  self.view_image_cui(reward_map)
                #  self.view_image(reward_map, 'reward_map')
                reward_map_list[i] = [reward_map for l in xrange(num_state_list_size)]
        #  print "reward_map_list : "
        #  print reward_map_list
        return reward_map_list


    
    def get_dataset(self):

        self.dataset_state_list = self.get_state_list(self.pedestrian_markers)
        #  print "self.dataset_state_list : "
        #  print  self.dataset_state_list
        self.dataset_action_list = self.get_action_list(self.dataset_state_list)
        #  print "self.dataset_action_list : "
        #  print self.dataset_action_list
        self.dataset_reward_map_list = self.get_reward_map(self.dataset_state_list)
        #  print "self.dataset_reward_map_list : "
        #  print self.dataset_reward_map_list

        self.dataset_image_list \
                = self.get_grid_image_list(self.grid_map_list, \
                                           self.dataset_state_list, \
                                           self.pickup_index)
        #  print "self.dataset_image_list : "
        #  print self.dataset_image_list


    def save_dataset(self, data, filename):
        save_filename = self.abs_path+filename
        print "Now Saveing..."
        print "Path : %s " % save_filename
        print "Save %d datasets !!!!!" % len(data['image'])
        with open(save_filename, mode='wb') as f:
            pickle.dump(data, f)


def main():
    rospy.init_node('processing_pedestrian_dataset')

    ppd = ProcessingPedestrianDataset()

    directory = os.listdir(args.load_dataset_path)
    print directory
    
    if len(directory) != 0:
        ppd.load_raw_dataset(args.load_dataset_path+directory[0])
    
    rows = ppd.height
    cols = ppd.width
    max_samples = 10000
    print "max_samples : ", max_samples
    image_data = np.zeros((max_samples, rows, cols))
    reward_map_data = np.zeros((max_samples, rows, cols))
    state_list_data = np.zeros((max_samples, 2))
    action_list_data = np.zeros(max_samples)

    directory = os.listdir(args.load_dataset_path)
    print directory

    num_sample = 0
    for filename in directory:
        print "-------------- filename : ", filename, "--------------------"
        ppd.load_raw_dataset(args.load_dataset_path+filename)
        ppd.get_dataset()
        
        num_pedestrians = len(ppd.pedestrian_markers.markers)

        ns = 0
        for i in xrange(num_pedestrians):
            print "======================== i : ", i, "========================="
            ns = len(ppd.dataset_state_list[i])
            if ns == 0:
                continue
            print "ns : ", ns
            #  for k in xrange(ns):
                #  ppd.view_image(ppd.dataset_image_list[i][k], 'grid_map')


            image_data[num_sample:num_sample+ns] = ppd.dataset_image_list[i][:]
            reward_map_data[num_sample:num_sample+ns] = ppd.dataset_reward_map_list[i][:]
            state_list_data[num_sample:num_sample+ns] = ppd.dataset_state_list[i][:]
            action_list_data[num_sample:num_sample+ns] = ppd.dataset_action_list[i][:]

            num_sample += ns

    print "num_sample : ", num_sample
    #  for i in xrange(num_sample):
        #  print "*******************************"
        #  ppd.view_image_cui(image_data[i])

    print "++++++++++++++++++++++++++++++++++++++++++++"

    data = {}
    data['image'] = image_data[0:num_sample]
    data['reward'] = reward_map_data[0:num_sample]
    data['state'] = state_list_data[0:num_sample]
    data['action'] = action_list_data[0:num_sample]

    dataset_name ='pedestrian_dataset.pkl'
    ppd.save_dataset(data, dataset_name)


if __name__ == "__main__":
    main()

