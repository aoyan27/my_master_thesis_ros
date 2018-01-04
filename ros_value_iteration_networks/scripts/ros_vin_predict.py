#!/usr/bin/env python
#coding:utf-8

import argparse

import numpy as np
np.set_printoptions(suppress=True, threshold=np.inf)
import matplotlib.pyplot
import cv2 as cv
import matplotlib.pyplot as plt
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

from networks.vin import ValueIterationNetwork

import rospy

import tf
from std_msgs.msg import Int32
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped


class InputDataGenerator:
    def __init__(self, input_image_size=(16, 16)):
        self.input_image_size = input_image_size
        #  self.gridmap_sub = \
                #  rospy.Subscriber("/local_map_real/expand", OccupancyGrid, self.gridMapCallback)
        self.gridmap_sub = \
                rospy.Subscriber("/local_map_real", OccupancyGrid, self.gridMapCallback)
        self.lcl_sub = rospy.Subscriber("/lcl5", Odometry, self.lclCallback)
        self.local_goal_sub = rospy.Subscriber("/local_goal", PoseStamped, self.localGoalCallback)

        self.input_data = None
        self.grid_image = None
        self.reward_map = None
        
        self.grid_map = None
        self.lcl = None
        self.local_goal = None

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
    
    def grid2image(self, grid_data):
        image = copy.deepcopy(grid_data)

        index = np.where(image == 100)
        image[index] = 1
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
    def __init__(self, model_path, gpu, mode=1):
        self.mode = 1

        self.model = ValueIterationNetwork(l_q=9, n_out=9, k=20)
        self.load_model(model_path)

        if gpu >= 0:
            cuda.get_device(gpu).use()
            self.model.to_gpu()
        
        self.action_list = None
        self.n_action = None
        self.dirs = None
        self.set_action()

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

    def load_model(self, filename):
        print "Load {}!!".format(filename)
        serializers.load_npz(filename, self.model)
    
    def get_action(self, input_data, state_data):
        p = self.model(input_data, state_data)
        action = np.argmax(p.data)
        return action

    def move(self, state, action, grid_data, reflect=1):
        grid_range = [grid_data.shape[0], grid_data.shape[1]]
        y, x = state
        next_y, next_x = state
        
        if self.mode == 0:
            if action == 0:
                #  right
                next_x = x + reflect*1
            elif action == 1:
                #  left
                next_x = x - reflect*1
            elif action == 2:
                #  down
                next_y = y + reflect*1
            elif action == 3:
                #  up
                next_y = y - reflect*1
            else:
                #  stay
                next_x = x
                next_y = y
        elif self.mode == 1:
            if action == 0:
                #  right
                next_x = x + reflect*1
            elif action == 1:
                #  left
                next_x = x - reflect*1
            elif action == 2:
                #  down
                next_y = y + reflect*1
            elif action == 3:
                #  up
                next_y = y - reflect*1
            elif action == 4:
                # upper right
                next_x = x + reflect*1
                next_y = y - reflect*1
            elif action == 5:
                # upper left
                next_x = x - reflect*1
                next_y = y - reflect*1
            elif action == 6:
                # down right
                next_x = x + reflect*1
                next_y = y + reflect*1
            elif action == 7:
                # down left
                next_x = x - reflect*1
                next_y = y + reflect*1
            else:
                #  stay
                next_x = x
                next_y = y
        elif self.mode == 2:
            if action == 0:
                #  down
                next_y = y + reflect*1
            elif action == 1:
                #  up
                next_y = y - reflect*1
            elif action == 2:
                # upper right
                next_x = x + reflect*1
                next_y = y - reflect*1
            elif action == 3:
                # upper left
                next_x = x - reflect*1
                next_y = y - reflect*1
            elif action == 4:
                # down right
                next_x = x + reflect*1
                next_y = y + reflect*1
            elif action == 5:
                # down left
                next_x = x - reflect*1
                next_y = y + reflect*1
            else:
                #  stay
                next_x = x
                next_y = y
        #  print "next_x : ", next_x
        #  print "next_y : ", next_y
        
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

    def get_path(self, input_data, state_data):
        state_list = []
        action_list = []

        state_data_ = copy.deepcopy(state_data)
        state = state_data_[0]
        #  print "state : ", state
        #  print input_data
        grid_image = input_data[0][0]
        reward_map = input_data[0][1]
        local_goal_index = np.where(reward_map == 1)
        #  print "local_goal_index : ", local_goal_index
        max_challenge_times = grid_image.shape[0] + grid_image.shape[1]
        #  print "max_challenge_times : ", max_challenge_times
        challenge_times = 0
        resign = False
        

        while tuple(state) != local_goal_index:
            challenge_times += 1
            if challenge_times >= max_challenge_times:
                #  state_list = []
                #  action_list = []
                resign = True
                break
            
            action = self.get_action(input_data, state_data_)

            #  print "action : ", action, " (", self.dirs[action], ")"

            next_state, _, _ = self.move(state, action, grid_image)
            #  print "next_state : ", next_state

            state_list.append(list(state))
            action_list.append(action)

            state_data_[0] = next_state
            state = next_state
        state_list.append(list(state))

        return state_list, action_list, resign

    def show_path(self, input_data, state_data):
        state_list, action_list, resign = self.get_path(input_data, state_data)

        grid_image = input_data[0][0]
        reward_map = input_data[0][1]
        n_local_state = grid_image.shape[0] * grid_image.shape[1]
        local_goal_index = np.where(reward_map == 1)

        vis_path = np.array(['-']*n_local_state).reshape(grid_image.shape)
        index = np.where(grid_image == 1)
        vis_path[index] = '#'
        state_list = np.asarray(state_list)
        for i in xrange(len(state_list)):
            vis_path[tuple(state_list[i])] = '*'
            if tuple(state_list[i]) == local_goal_index:
                vis_path[tuple(local_goal_index)] = 'G'
        vis_path[state_data[0][0], state_data[0][1]] = '$'

        path_data = {}
        path_data['vis_path'] = vis_path
        path_data['state_list'] = state_list
        path_data['action_list'] = action_list

        self.view_path(path_data['vis_path'])

    def view_path(self, path):
        grid = copy.deepcopy(path)
        for row in grid:
            print "|",
            for i in row:
                print "%2c" % i,
            print "|"


def main(model_path, gpu):
    print "Here we go!!"
    idg = InputDataGenerator()

    rospy.init_node("ros_vin_predict")

    agent = ValueIterationNetworkAgent(model_path, gpu)
    
    input_data = None
    #  state = [10, 6]
    state = np.asarray(map(lambda x: x / 2, list(idg.input_image_size)), dtype=np.int32)
    state_data = np.expand_dims(state, 0)
    #  print "state_data : ", state_data
    
    loop_rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if idg.gridmap_sub_flag and idg.local_goal_sub_flag:
            print "*****************************************"
            idg.cvt_input_data()
            #  print "idg.input_data"
            #  print idg.input_data
            input_data = idg.input_data
            if gpu >= 0:
                input_data = cuda.to_gpu(input_data)
            print "state_data : ", state_data
            
            action = agent.get_action(input_data, state_data)
            print "action : ", action, "(", agent.dirs[action], ")"
            
            agent.show_path(input_data, state_data)
            
            idg.gridmap_sub_flag = False
            idg.local_goal_sub_flag = False

        loop_rate.sleep()
        #  rospy.spin()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='This script is ros_vin_predict ...')
    parser.add_argument('-g', '--gpu', default=-1, type=int, help='number of gpu device')
    parser.add_argument('-m', '--model_path', \
            default='/home/amsl/ros_catkin_ws/src/master_thesis/ros_value_iteration_networks/models/vin_model_1.model', type=str, help="load model path")

    args = parser.parse_args()
    print args
    main(args.model_path, args.gpu)
