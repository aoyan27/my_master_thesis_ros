#!/usr/bin/env python
#coding:utf-8

import argparse

import numpy as np
np.set_printoptions(suppress=True, threshold=np.inf)
import cupy as cp
import cv2 as cv
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from progressbar import ProgressBar

import chainer 
from chainer import cuda, Variable, optimizers, serializers
from chainer import Chain
import chainer.functions as F
import chainer.links as L

import copy
import pickle
import time
import sys
import math

from networks.vin import ValueIterationNetwork

import rospy
import tf
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker


class InputDataGenerator:
    def __init__(self, input_image_size=(20, 20)):
        self.input_image_size = input_image_size
        #  self.gridmap_sub = \
                #  rospy.Subscriber("/local_map_real/expand", OccupancyGrid, self.gridMapCallback)
        #  self.gridmap_sub = \
        #          rospy.Subscriber("/local_map_real", OccupancyGrid, self.gridMapCallback)

        #  self.gridmap_sub = \
                #  rospy.Subscriber("/local_map_static", OccupancyGrid, self.gridMapCallback)

        self.gridmap_sub = \
                rospy.Subscriber("/local_map", OccupancyGrid, self.gridMapCallback)

        self.lcl_sub = rospy.Subscriber("/lcl5", Odometry, self.lclCallback)
        self.local_goal_sub = rospy.Subscriber("/local_goal", PoseStamped, self.localGoalCallback)

        self.input_data = None
        self.grid_image = None
        self.reward_map = None
        
        self.grid_map = None
        self.lcl = None
        self.local_goal = None
        self.discreate_local_goal = None

        self.gridmap_sub_flag = False
        self.lcl_sub_flag = False
        self.local_goal_sub_flag = False
    
    def show_gridmap(self, grid_map):
        if not grid_map.ndim == 2:
            rospy.logerr('Error occurred!! Check grid dimentions!!')
        else:
            vis_grid_map = copy.deepcopy(grid_map)
            for row in vis_grid_map:
                print "|",
                for i in row:
                    print "%2d" % i,
                print "|"

    def my_resize_image(self, input_image):
        index = np.where(input_image == 100)
        #  print "index : "
        #  print index
        continuous_y = index[0] * self.grid_map.info.resolution
        continuous_x = index[1] * self.grid_map.info.resolution
        #  print "continuous_y : ", continuous_y
        #  print "continuous_x : ", continuous_x
        rows = self.grid_map.info.height
        resize_resolution = self.grid_map.info.resolution*rows/self.input_image_size[0]
        #  print "resize_resolution : ", resize_resolution
        resize_discreate_y = continuous_y / resize_resolution
        resize_discreate_x = continuous_x / resize_resolution
        #  print "resize_discreate_y : ", resize_discreate_y
        #  print "resize_discreate_x : ", resize_discreate_x
        resize_image = np.zeros(self.input_image_size, dtype=np.uint8)
        resize_image[resize_discreate_y.astype(np.int32), resize_discreate_x.astype(np.int32)] = 1
        #  print "resize_image : "
        #  print resize_image

        return resize_image
    
    def grid2image(self, grid_data):
        image = copy.deepcopy(grid_data)
        index = np.where(image == 100)
        image[index] = 1

        #  resize_image = self.my_resize_image(image)

        #  print image
        image = image.astype(np.uint8)

        resize_image = cv.resize(image.astype(np.uint8), self.input_image_size)
        return resize_image

    def view_image(self, array, title):
        image = cv.cvtColor(array.astype(np.uint8), cv.COLOR_GRAY2RGB)
        #  print image
        plt.imshow(255 - 255*image, interpolation="nearest")
        plt.title(title)
        plt.show()

    def gridMapCallback(self, msg):
        #  print "========================================"
        self.gridmap_sub_flag = True
        #  print "msg(grid_map) : ", msg
        self.grid_map = msg
        #  print "self.grid_map.info : "
        #  print self.grid_map.info

        cell_size = msg.info.resolution
        rows = msg.info.height
        cols = msg.info.width
        map_data = msg.data
        #  print "map_data : ", type(map_data)
        grid_data = np.asarray(map_data).reshape((rows, cols))
        #  print "grid_data : "
        #  self.show_gridmap(grid_data)
        resize_grid_image = self.grid2image(grid_data)
        #  print "resize_grid_image : "
        #  print resize_grid_image
        #  self.view_image(resize_grid_image, 'local_grid_map')
        self.grid_image = resize_grid_image
        #  self.show_gridmap(self.grid_image)


    def lclCallback(self, msg):
        self.lcl_sub_flag = True
        #  print "msg(lcl) : ", msg
        self.lcl = msg

    
    def global2local_transform(self, global_pose):
        global_vector = np.array([global_pose.pose.position.x, global_pose.pose.position.y, 1.0])
        #  print "global_vector : ", global_vector
        translation_matrix = np.eye(3, dtype=np.float32)
        translation_matrix[0, 2] = -1.0 * self.lcl.pose.pose.position.x
        translation_matrix[1, 2] = -1.0 * self.lcl.pose.pose.position.y
        #  print "translation_matrix : "
        #  print translation_matrix
        tmp_vector = np.dot(translation_matrix, global_vector)
        #  print "tmp_vextor : ", tmp_vector
        q = (self.lcl.pose.pose.orientation.x, self.lcl.pose.pose.orientation.y, \
                self.lcl.pose.pose.orientation.z, self.lcl.pose.pose.orientation.w)
        yaw = -1.0*tf.transformations.euler_from_quaternion(q)[2]
        #  print "yaw : ", yaw
        rotation_matrix = np.matrix((
                            (np.cos(yaw), -np.sin(yaw), 0.0),
                            (np.sin(yaw),  np.cos(yaw), 0.0),
                            (         0.0,         0.0, 1.0)))
        #  print "rotation_matrix : "
        #  print rotation_matrix
        local_vector = np.dot(rotation_matrix, tmp_vector)
        #  print "local_vector : ", local_vector
        return [local_vector[0,0], local_vector[0,1]]

    def create_reward_map(self, local_goal):
        rows = self.grid_map.info.height
        resolution = self.grid_map.info.resolution*rows/self.input_image_size[0]
        #  print "resolution : ", resolution

        #  print "local_goal : ", local_goal
        x = local_goal[0] + self.input_image_size[0]*resolution/2
        y = local_goal[1] + self.input_image_size[1]*resolution/2
        #  print "x : ", x
        #  print "y : ", y

        grid_x = int(x / resolution)
        grid_y = int(y / resolution)
        if grid_x < 0:
            grid_x = 0
        if grid_x >= self.input_image_size[0]:
            grid_x = self.input_image_size[0]-1
        if grid_y < 0:
            grid_y = 0
        if grid_y >= self.input_image_size[1]:
            grid_y = self.input_image_size[1]-1

        #  print "grid_x : ", grid_x
        #  print "grid_y : ", grid_y
        self.discreate_local_goal = [grid_y, grid_x]

        reward_map = np.zeros(self.input_image_size, dtype=np.uint8)
        reward_map[grid_y, grid_x] = 1
        #  print "reward_map : "
        #  print reward_map
        return reward_map

    def localGoalCallback(self, msg):
        #  print "------------------------------------------"
        self.local_goal_sub_flag = True
        global_local_goal = msg
        #  print "global_local_goal : ", global_local_goal
        
        local_local_goal = self.global2local_transform(global_local_goal)
        self.local_goal = local_local_goal
        #  print "local_local_goal : ", local_local_goal
        
        self.reward_map = self.create_reward_map(local_local_goal)
        #  self.show_gridmap(self.reward_map)

    
    def cvt_input_data(self):
        #  print "self.grid_image : "
        #  print self.grid_image
        #  print "self.reward_map : "
        #  print self.reward_map
        input_data_ = np.concatenate((np.expand_dims(self.grid_image, 0), \
                np.expand_dims(self.reward_map, 0)), axis=0)
        self.input_data = np.expand_dims(input_data_, 0)


class ValueIterationNetworkAgent:
    def __init__(self, model_path, gpu, input_image_size, mode=1):
        self.mode = mode

        self.model = ValueIterationNetwork(l_q=9, n_out=9, k=25)
        self.load_model(model_path)
        
        self.gpu = gpu
        if self.gpu >= 0:
            cuda.get_device(self.gpu).use()
            self.model.to_gpu()
        
        self.action_list = None
        self.n_action = None
        self.dirs = None
        self.set_action()
        
        self.input_data_ = None
        self.traj_state_list = []
        self.traj_action_list = []

        self.max_challenge_times = input_image_size[0]/2 + input_image_size[1]/2
        #  print "max_challenge_times : ", max_challenge_times

    def set_action(self):
        if self.mode == 0:    # mode=0 : 行動4パターン
            self.action_list = [0, 1, 2, 3, 4]
            self.n_action = len(self.action_list)
            self.dirs = {0: '>', 1: '<', 2: 'v', 3: '^', 4: '-'}
        elif self.mode == 1:    # mode=1 : 行動8パターン
            self.action_list = [0, 1, 2, 3, 4, 5, 6, 7, 8]
            self.n_action = len(self.action_list)
            self.dirs = \
                    {0: '>', 1: '<', 2: 'v', 3: '^', 4: 'ur', 5: 'ul', 6: 'dr', 7: 'dl', 8: '-'}
            self.movement \
                = {0: [0, 1], 1: [0, -1], 2: [1, 0], 3: [-1, 0], \
                   4: [-1, 1], 5: [-1, -1], 6: [1, 1], 7: [1, -1], 8: [0, 0]}

    def load_model(self, filename):
        print "Load {}!!".format(filename)
        serializers.load_npz(filename, self.model)
    
    def get_action(self, input_data, state_data):
        if self.gpu >= 0:
            input_data = cuda.to_gpu(input_data)

        p = self.model(input_data, state_data)
        action = np.argmax(p.data)
        return action

    def move(self, state, action, grid_data, reflect=1):
        grid_range = [grid_data.shape[0], grid_data.shape[1]]

        y, x = state
        next_y, next_x = state
        
        next_y = y + reflect*self.movement[int(action)][0]
        next_x = x + reflect*self.movement[int(action)][1]
        
        out_of_range = False
        if next_y < 0 or (grid_range[0]-1) < next_y:
            #  print "y, out_of_range!!!!"
            next_y = y
            out_of_range = True

        if next_x < 0 or (grid_range[1]-1) < next_x:
            #  print "x, out of range!!!!!"
            next_x = x
            out_of_range = True

        collision = False
        if grid_data[next_y, next_x] == 1:
            #  print "collision!!!!!"
            collision = True
            #  if action == 0 or action == 1:
                #  next_x = x
            #  elif action == 2 or action == 3:
                #  next_y = y

        return [next_y, next_x], out_of_range, collision


    def get_path(self, input_data, state_data, local_goal):
        state_list = []

        self.input_data_ = input_data
        grid_image = input_data[0][0]

        state_data_ = copy.deepcopy(state_data)
        #  print "state_data_ : ", state_data_, type(state_data_)
        state = state_data_[0]
        #  print "state : ", state, type(state)
        state_list.append(list(state))
        #  print "state_list : ", state_list


        for i in xrange(self.max_challenge_times):
            state_list.append(\
                    self.move(state, self.get_action(input_data, state_data_), grid_image)[0])

            state_data_[0] = state_list[-1]
            if state_list[-1] == local_goal:
                break

        self.traj_state_list = state_list


    def view_path(self, input_image, path):
        grid = copy.deepcopy(input_image[0][0])
        for state in path:
            grid[tuple(state)] = 2
        for row in grid:
            print "|",
            for i in row:
                if i==0:
                    print "-",
                elif i==1:
                    print "#",
                elif i==2:
                    print "*",
            print "|"


def main(model_path, gpu):
    rospy.init_node("ros_vin_predict")

    idg = InputDataGenerator()

    next_target_pub = rospy.Publisher("/vin/target_path", Int32MultiArray, queue_size=1)

    agent = ValueIterationNetworkAgent(model_path, gpu, idg.input_image_size)
    
    input_data = None
    #  state = [10, 6]
    state = np.asarray(map(lambda x: x / 2, list(idg.input_image_size)), dtype=np.int32)
    state_data = np.expand_dims(state, 0)
    #  print "state_data : ", state_data
    orientation = 0.0

    loop_rate = rospy.Rate(100)

    ros_next_state = Int32MultiArray()
    layout = MultiArrayDimension()
    layout.size = idg.input_image_size[0]
    ros_next_state.layout.dim.append(layout)

    print "Here we go!!"

    while not rospy.is_shutdown():
        if idg.gridmap_sub_flag and idg.local_goal_sub_flag and idg.grid_image is not None:
            print "*****************************************"
            start_time = time.time()
            idg.cvt_input_data()
            #  print "idg.input_data"
            #  print idg.input_data
            input_data = idg.input_data
            if gpu >= 0:
                input_data = cuda.to_gpu(input_data)
            #  print "state_data : ", state_data
            
            #  action = agent.get_action(input_data, state_data)
            #  print "action : ", action, "(", agent.dirs[int(action)], ")"
            #  next_state, out_of_range, collision \
                    #  = agent.move(state_data[0], action, input_data[0][0])
            #  print "next_state : ", next_state
            #  print "out_of_range : ", out_of_range
            #  print "collision : ", collision
            #  ros_next_state.data = next_state
            #  print "ros_next_state : ", ros_next_state
            #  next_target_pub.publish(ros_next_state)
            
            agent.get_path(input_data, state_data, idg.discreate_local_goal)
            print "agent.traj_state_list : ", agent.traj_state_list
            #  agent.view_path(input_data ,agent.traj_state_list)

            ros_next_state.data = np.asarray(agent.traj_state_list).reshape(-1)
            next_target_pub.publish(ros_next_state)

            elapsed_time = time.time() - start_time
            print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")
            
            #  idg.gridmap_sub_flag = False
            #  idg.local_goal_sub_flag = False

        loop_rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='This script is ros_vin_predict ...')
    parser.add_argument('-g', '--gpu', default=-1, type=int, help='number of gpu device')
    parser.add_argument('-m', '--model_path', \
        default='/home/amsl/ros_catkin_ws/src/master_thesis/ros_value_iteration_networks/models/vin_model_50_epoch_5000_domain_9_action_20x20_40_to_0_n_objects_seed_0/vin_model_46.model', \
        type=str, help="load model path")

    args = parser.parse_args()
    print args
    main(args.model_path, args.gpu)
