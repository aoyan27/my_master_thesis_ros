#!/usr/bin/env python
#coding:utf-8

import argparse

import rospy
import tf

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

import sys
import os
import glob
import copy
import math
from datetime import *

import pickle

parser = argparse.ArgumentParser(description='This script is concatenate_dataset...')

parser.add_argument('-dp', '--dataset_path', \
        default='/make_pedestrian_dataset/datasets/processing/', \
        type=str, help='Please set load dataset_path(all file in the directory concatenate)')

parser.add_argument('-sdn', '--save_dir_name', \
        default='test_concat', \
        type=str, help='dir name(saved concatenate file)')

args = parser.parse_args()
print args

class ConcatenateDataset:
    def __init__(self):
        self.save_file_path \
            = "/home/amsl/ros_catkin_ws/src/master_thesis/make_pedestrian_dataset/datasets/concatenate/"
        self.abs_path = None
        self.date = datetime.now().strftime('%Y_%m_%d')
        #  print "date : ", date
        self.load_path = args.dataset_path

        if not os.path.exists(self.save_file_path+self.date):
            os.makedirs(self.save_file_path+self.date)
        if not os.path.exists(self.save_file_path+self.date+"/"+args.save_dir_name):
            os.makedirs(self.save_file_path+self.date+"/"+args.save_dir_name)
        
        if len(os.listdir(self.save_file_path+self.date+"/"+args.save_dir_name)) != 0:
            sys.stderr.write('\033[31mError!! This directory already has some files!!\033[0m\n')
            sys.exit(1)
        else:
            self.abs_path = self.save_file_path + self.date + "/" + args.save_dir_name + "/"
        print "self.abs_path : ", self.abs_path

        self.all_grid_image = []
        self.all_reward_map = []
        self.all_state_list = []
        self.all_action_list = []

        

    def load_dataset(self, filename):
        with open(filename, mode='rb') as f:
            data = None
            data = pickle.load(f)

        grid_image = data['image']
        reward_map = data['reward']
        state_list = data['state']
        action_list = data['action']
        dataset_size = len(grid_image)
        print "Load %d trajectory_data!!!" % len(grid_image)
        return dataset_size, grid_image, reward_map, state_list, action_list

    def save_dataset(self, data, filename):
        save_filename = self.abs_path+filename
        print "Now Saveing..."
        print "Path : %s " % save_filename
        print "Save %d datasets !!!!!" % len(data['image'])
        with open(save_filename, mode='wb') as f:
            pickle.dump(data, f)

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

    def concatenate_dataset(self, grid_image, reward_map, state_list, action_list):
        if len(self.all_grid_image) != 0:
            if grid_image[0].shape == self.all_grid_image[0].shape:
                self.all_grid_image.extend(grid_image)
                self.all_reward_map.extend(reward_map)
                self.all_state_list.extend(state_list)
                self.all_action_list.extend(action_list)
            else:
                sys.stderr.write('\033[31mError!! Not match image size!!\033[0m\n')

                data = {}
                data['image'] = self.all_grid_image
                data['reward'] = self.all_reward_map
                data['state'] = self.all_state_list
                data['action'] = self.all_action_list
                
                dataset_name ='_map_dataset.pkl'
                self.save_dataset(data, dataset_name)

                sys.exit(1)
        else:
            self.all_grid_image.extend(grid_image)
            self.all_reward_map.extend(reward_map)
            self.all_state_list.extend(state_list)
            self.all_action_list.extend(action_list)


def main():
    rospy.init_node('concatenate_dataset')

    cd = ConcatenateDataset()

    #  directory = os.listdir(args.dataset_path)
    path = args.dataset_path  + '/*'
    dataset_paths = glob.glob(path)
    print "dataset_paths : ", dataset_paths

    for path in dataset_paths:
        print "==================================================="
        #  print "path : ", path
        if os.path.isdir(path):
            files = os.listdir(path)
            #  print "files : ", files
            for file in files:
                dataset_file = path + '/' + file
                print "dataset_file : ", dataset_file
                dataset_size, grid_image, reward_map, state_list, action_list = cd.load_dataset(dataset_file)
                if dataset_size != 0:
                    cd.concatenate_dataset(grid_image, reward_map, state_list, action_list)


    print "concat_data_size : ", len(cd.all_grid_image)
    #  print "cd.all_grid_image : "
    #  print cd.all_grid_image


    data = {}
    data['image'] = np.asarray(cd.all_grid_image)
    data['reward'] = np.asarray(cd.all_reward_map)
    data['state'] = np.asarray(cd.all_state_list)
    data['action'] = np.asarray(cd.all_action_list)

    dataset_name ='concatenate_pedestrian_dataset.pkl'
    cd.save_dataset(data, dataset_name)


if __name__ == "__main__":
    main()

