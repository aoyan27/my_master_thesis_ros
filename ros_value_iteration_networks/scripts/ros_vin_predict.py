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
from std_msgs.msg import Int32
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker


class InputDataGenerator:
    def __init__(self, input_image_size=(20, 20)):
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
        if self.gpu >= 0:
            input_data = cuda.to_gpu(input_data)

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
        next_state = [next_y, next_x]

        return next_state, out_of_range, collision

    def get_path(self, input_data, state_data):
        state_list = []
        action_list = []

        state_data_ = copy.deepcopy(state_data)
        state = state_data_[0]
        #  print "state : ", state
        #  print input_data
        self.input_data_ = input_data
        grid_image = input_data[0][0]
        reward_map = input_data[0][1]
        #  print "reward_map : ", reward_map
        local_goal_index = None
        if isinstance(reward_map, chainer.cuda.ndarray):
            reward_map = cuda.to_cpu(reward_map)
            local_goal_index = np.where(reward_map == 1)
        else:
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
        self.traj_state_list = state_list
        self.traj_action_list = action_list

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

class LocalPlanner:
    def __init__(self, idg, vin_agent, state, orientation):
        self.vis_velocity_vector = self.set_vis_velocity_vector()

        self.goal_radius = 0.1

        self.idg = idg
        #  print "self.idg.grid_map.info.resolution : ", self.idg.grid_map.info.resolution
        self.cell_size \
                = self.idg.grid_map.info.resolution\
                * self.idg.grid_map.info.height / self.idg.input_image_size[0]
        #  print "self.cell_size : ", self.cell_size
        self.vin_agent = vin_agent
        #  print "self.vin_agent.traj_state_list : "
        #  print self.vin_agent.traj_state_list
        #  print "self.vin_agent.input_data_[0][0] : "
        #  print self.vin_agent.input_data_[0][0]
        self.grid = self.vin_agent.input_data_[0][0]
        self.reward_map = self.vin_agent.input_data_[0][1]
        #  print "self.grid : "
        #  print self.grid
        self.rows, self.cols = self.grid.shape
        #  print "self.rows : ", self.rows
        #  print "self.cols : ", self.cols
        self.local_goal_index = None
        self.objects_index = None
        if isinstance(self.reward_map, chainer.cuda.ndarray):
            self.local_goal_index = cp.where(self.reward_map == 1)
            self.objects_index = cp.where(self.grid == 1)
        else:
            self.local_goal_index = np.where(self.reward_map == 1)
            self.objects_index = np.where(self.grid == 1)
        #  print "self.local_goal_index : ", self.local_goal_index
        self.goal = self.discreate2continuous(self.local_goal_index[0], self.local_goal_index[1])
        #  print "self.goal : ", self.goal
        #  print "self.objects_index : ", self.objects_index
        self.objects = np.asarray(self.objects_index).transpose(1, 0)
        #  print "self.objects : ", self.objects, self.objects.shape
        self.continuous_objects = self.get_continuous_objects()
        #  print "self.continuous_objects : ", self.continuous_objects

        #  self.state_ = list(self.discreate2continuous(state[0], state[1]))
        self.state_ = state
        self.orientation_ = orientation


        self.continuous_action_list = None
        self.n_continuous_action = 0
        self.velocity_vector = {}
        self.set_continuous_action()


        self.evaluation_time = 0.5
        self.dt = 0.05

        self.future_traj_position_list = [[] for i in xrange(self.n_continuous_action)]
        self.future_traj_orientation_list = [[] for i in xrange(self.n_continuous_action)]
        self.future_traj_collision_list = [[] for i in xrange(self.n_continuous_action)]
        self.future_traj_out_of_range_list = [[] for i in xrange(self.n_continuous_action)]

        self.continuous_global_path_list = None

        self.evaluation_value = [0 for i in xrange(self.n_continuous_action)]

        self.selected_traj_position_list = None
        self.selected_traj_orientation_list = None


        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.continuous_x_list = []
        self.continuous_y_list = []
    
    def set_vis_velocity_vector(self):
        marker = Marker()
        marker.header.frame_id = "/velodyne"
        marker.ns = "vis_velocity_vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.lifetime = rospy.Duration()

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.scale.y = 0.05
        marker.scale.z = 0.05

        return marker


    def calc_continuous_trajectory(self):
        y = self.state_[0]
        x = self.state_[1]
        theta = self.orientation_
        self.continuous_x_list.append(x)
        self.continuous_y_list.append(y)

    def show_continuous_objectworld(self, global_path=None, local_path=None, selected_path=None):
        start_time = time.time()

        self.ax.cla()
        
        y = self.state_[0]
        x = self.state_[1]
        theta = self.orientation_

        height, width = self.discreate2continuous(self.rows, self.cols)

        self.ax.set_ylim([0, height])
        self.ax.set_xlim([0, width])

        self.ax.set_xticks(np.arange(0, height, self.cell_size))
        self.ax.set_yticks(np.arange(0, width, self.cell_size))

        gca = plt.gca()
        gca.invert_yaxis()

        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        
        self.ax.set_title('Continuous Object World')

        self.ax.grid(True)

        #  障害物エリアを生成
        objects_continuous_y, objects_continuous_x = self.continuous_objects.transpose(1, 0)
        #  self.ax.scatter(objects_continuous_x, objects_continuous_y, s=100, \
                #  color="pink", alpha=0.5, linewidths="2", edgecolors="red")

        #  object_rectangles = [patches.Rectangle(xy=[obs_x, obs_y], \
                #  width=self.cell_size, height=self.cell_size, \
                #  facecolor='pink', alpha=0.5, linewidth="2", edgecolor="red") \
                #  for (obs_x, obs_y) in zip(objects_continuous_x, objects_continuous_y)]
        #  for r in object_rectangles:
            #  self.ax.add_patch(r)
        object_circles = [patches.Circle(xy=[obs_x, obs_y], radius=self.cell_size/2.0, \
                facecolor='pink', alpha=0.5, linewidth="2", edgecolor="red") \
                for (obs_x, obs_y) in zip(objects_continuous_x, objects_continuous_y)]
        for c in object_circles:
            self.ax.add_patch(c)


        #  ゴールエリアを生成
        c = patches.Circle(xy=(self.goal[1], self.goal[0]), radius=self.goal_radius, \
                facecolor='indigo', edgecolor='indigo', alpha=0.5)
        self.ax.add_patch(c)


        #  global_pathを生成
        if global_path is not None:
            tmp_global_path = np.asarray(copy.deepcopy(global_path)).transpose(1, 0)
            self.ax.plot(tmp_global_path[1], tmp_global_path[0], color='darkorange')

        #  local_pathの候補を生成
        if local_path is not None:
            for i in xrange(len(local_path)):
                tmp_local_path = np.asarray(copy.deepcopy(local_path[i])).transpose(1, 0)
                self.ax.plot(tmp_local_path[1], tmp_local_path[0], color='aqua')

        #  selected_pathを生成
        if selected_path is not None:
            tmp_selected_path = np.asarray(copy.deepcopy(selected_path)).transpose(1, 0)
            self.ax.plot(tmp_selected_path[1], tmp_selected_path[0], color='red')

        #  現在のエージェントの位置と方位を生成
        self.ax.scatter(x, y, s=20, color="darkviolet", linewidths="2", edgecolors="darkviolet")
        self.ax.plot([x, x+0.15*math.cos(theta)], [y, y+0.15*math.sin(theta)], \
                color='green', linewidth="3") 
        #  エージェントの軌道を生成
        #  self.continuous_x_list.append(x)
        #  self.continuous_y_list.append(y)
        self.ax.plot(self.continuous_x_list, self.continuous_y_list, color='blue')

        elapsed_time = time.time() - start_time
        print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")

        #  plt.pause(0.05)
        plt.show()

    def set_continuous_action(self):
        #  self.continuous_action_list = [0, 1, 2, 3, 4, 5, 6, 7]
        #  self.continuous_action_list = [0, 1, 2, 3, 4, 5, 6]

        self.continuous_action_list = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.n_continuous_action = len(self.continuous_action_list)
        #  self.velocity_vector \
                #  = {0: [0.1, -10.0], 1: [0.3, -5.0], 2: [0.5, -2.5], \
                   #  3: [0.6, 0.0], \
                   #  4: [0.5, 2.5], 5: [0.3, 5.0], 6: [0.1, 10.0], \
                   #  7: [0.0, 0.0]}

        #  self.velocity_vector \
                #  = {0: [0.5, -3.0], 1: [0.75, -2.5], 2: [1.0, -1.0], \
                   #  3: [1.2, 0.0], \
                   #  4: [1.0, 1.0], 5: [0.75, 2.5], 6: [0.5, 3.0]}

        self.velocity_vector \
            = {0: [0.5, -3.0], 1: [0.6, -2.5], 2: [0.7, -2.0], 3: [0.8, -1.5], 4: [1.0, -1.0], \
               5: [1.2, 0.0], \
               6: [1.0, 1.0], 7: [0.8, 1.5], 8: [0.7, 2.0], 9: [0.6, 2.5], 10: [0.5, 3.0]}

    def discreate2continuous(self, discreate_y, discreate_x):
        continuous_y = discreate_y * self.cell_size
        continuous_x = discreate_x * self.cell_size
        return continuous_y, continuous_x

    def continuous2discreate(self, continuous_y, continuous_x):
        discreate_y = int(continuous_y / self.cell_size)
        discreate_x = int(continuous_x / self.cell_size)
        return discreate_y, discreate_x

    def continuous_move(self, state, orientation, action, grid_range=None):
        if grid_range is None:
            grid_range = [self.rows, self.cols]

        y, x = state
        next_y, next_x = state
        yaw = orientation
        next_yaw = orientation

        linear = self.velocity_vector[action][0]
        angular = self.velocity_vector[action][1]
        #  print "yaw : ", math.degrees(yaw)
        next_yaw = yaw + angular*self.dt
        #  print "next_yaw : ", math.degrees(next_yaw)
        
        #  print "[y, x] :[ ", y, x, "]"
        next_y = y + linear*math.sin(next_yaw)*self.dt
        next_x = x + linear*math.cos(next_yaw)*self.dt
        #  print "[next_y, next_x] :[ ", next_y, next_x, "]"

        out_of_range = False
        if next_y < 0*self.cell_size or (grid_range[0]-1)*self.cell_size < next_y:
            #  print "y, out_of_range!!!!"
            next_y = y
            next_yaw = yaw
            out_of_range = True

        if next_x < 0*self.cell_size or (grid_range[1]-1)*self.cell_size < next_x:
            #  print "x, out of range!!!!!"
            next_x = x
            next_yaw = yaw
            out_of_range = True

        collision = False
        #  print "next_y : ", next_y
        #  print "next_x : ", next_x
        next_discreate_y, next_discreate_x = self.continuous2discreate(next_y, next_x)
        #  print "next_discreate_y : ", next_discreate_y
        #  print "next_discreate_x : ", next_discreate_x
        if self.grid[next_discreate_y, next_discreate_x] != 0:
            continuous_obs_y, continuous_obs_x \
                    = self.discreate2continuous(next_discreate_y, next_discreate_x)
            continuous_obs_y += self.cell_size / 2.0
            continuous_obs_x += self.cell_size / 2.0

            if math.sqrt((next_y-continuous_obs_y)**2 + (next_x-continuous_obs_x)**2) \
                    <= self.cell_size/2.0:
                #  print "collision!!!!!"
                collision = True
            #  if action == 0 or action == 1:
                #  next_x = x
            #  elif action == 2 or action == 3:
                #  next_y = y
        
        return [next_y, next_x], next_yaw, out_of_range, collision
    
    def get_continuous_objects(self):
        n_objects_ = len(self.objects)
        continuous_objects_ = None
        if n_objects_ == 0:
            continuous_objects_ = self.objects
        else:
            tmp_objects = np.asarray(copy.deepcopy(self.objects)).transpose(1, 0)
            objects_continuous_y, objects_continuous_x \
                    = self.discreate2continuous(tmp_objects[0], tmp_objects[1])
            objects_continuous_y += self.cell_size / 2.0
            objects_continuous_x += self.cell_size / 2.0
            continuous_objects_ \
                    = np.array([objects_continuous_y, objects_continuous_x]).transpose(1, 0)
        return continuous_objects_


    def get_future_trajectory(self, position, orientation):
        #  print "position : ", position
        #  continuous_position = list(self.discreate2continuous(position[0], position[1]))
        continuous_position = position
        #  print "continuous_position : ", continuous_position

        #  print "orientation : ", orientation
        for i_action in self.continuous_action_list:
            #  print "i_action : ", i_action
            y, x = continuous_position
            yaw = orientation

            self.future_traj_position_list[i_action].append(continuous_position)
            self.future_traj_orientation_list[i_action].append(orientation)
            self.future_traj_out_of_range_list[i_action].append(False)
            self.future_traj_collision_list[i_action].append(False)


            t = 0.0
            while t <= self.evaluation_time:
                #  print "t : ", t
                next_position, next_orientation, out_of_range, collision \
                        = self.continuous_move([y, x], yaw, i_action)

                self.future_traj_position_list[i_action].append(next_position)
                self.future_traj_orientation_list[i_action].append(next_orientation)
                self.future_traj_out_of_range_list[i_action].append(out_of_range)
                self.future_traj_collision_list[i_action].append(collision)
                y, x = next_position
                yaw = next_orientation
                if out_of_range or collision:
                    #  print "out_of_range or collision !!!"
                    break
                t += self.dt
        #  print "self.future_traj_position_list : "
        #  print self.future_traj_position_list
        #  self.show_continuous_objectworld(local_path=self.future_traj_position_list)

    def transform_global_path_discreate2continuous(self):
        global_state_list = self.vin_agent.traj_state_list
        tmp_global_state = np.asarray(copy.deepcopy(global_state_list)).transpose(1, 0)
        #  print "tmp_global_state : ", tmp_global_state
        global_path_continuous_y, global_path_continuous_x \
                = self.discreate2continuous(tmp_global_state[0], tmp_global_state[1])
        global_path_continuous_y += self.cell_size / 2.0
        global_path_continuous_x += self.cell_size / 2.0
        self.continuous_global_path_list \
                = np.array([global_path_continuous_y, global_path_continuous_x]).transpose(1, 0)
        #  print "self.continuous_global_path_list : "
        #  print self.continuous_global_path_list 
        #  self.show_continuous_objectworld(global_path=self.continuous_global_path_list)

    def calc_dist(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def calc_dist_line_and_point(self, a, b, p):
        u = np.array([b[0]-a[0], b[1]-a[1]])
        v = np.array([p[0]-a[0], p[1]-a[1]])
        L = abs(np.cross(u, v) / np.linalg.norm(u))
        return L

    def check_inverse_global_path_distance(self, final_position, global_state_list):
        # global_pathとの距離の逆数を評価
        #  max_range_y = np.max([global_state_list[0][0], global_state_list[-1][0]])
        #  max_range_x = np.max([global_state_list[0][1], global_state_list[-1][1]])
        #  min_range_y = np.min([global_state_list[0][0], global_state_list[-1][0]])
        #  min_range_x = np.min([global_state_list[0][1], global_state_list[-1][1]])
        #  if [min_range_y, min_range_x] <= final_position \
                #  and final_position <= [max_range_y, max_range_x]:
        if len(global_state_list) < 2:
            L = self.calc_dist(final_position, global_state_list[0])
        else:
            tmp_global_dist_list = []
            for global_position in global_state_list:
                tmp_global_dist_list.append(self.calc_dist(final_position, global_position))

            #  print "tmp_global_dist_list : ", tmp_global_dist_list
            min_index = np.argsort(np.asarray(tmp_global_dist_list))
            #  print "min_index : ", min_index
            
            global_position_1 = global_state_list[min_index[0]]
            global_position_2 = global_state_list[min_index[1]]
            #  print "global_position_1 : ", global_position_1
            #  print "global_position_2 : ", global_position_2
            L = self.calc_dist_line_and_point(global_position_1, global_position_2, final_position)
            #  print "L : ", L
        if L == 0.0:
            L = 0.0001

        inverse_global_path_distance = 1.0 / L
        #  else:
            #  inverse_global_path_distance = 0.0

        return inverse_global_path_distance

    def check_nearest_obstacle_distance(self, final_position):
        #  最近傍の障害物との距離
        tmp_obs_dist_list = []
        for obstacle in self.continuous_objects:
            tmp_obs_dist_list.append(self.calc_dist(final_position, obstacle))
        #  print "tmp_obs_dist_list : ", tmp_obs_dist_list
        max_obs_dist = 0
        if len(tmp_obs_dist_list) != 0:
            max_obs_dist = np.max(tmp_obs_dist_list)
        #  print "max_obs_dist : ", max_obs_dist

        return max_obs_dist

    def check_diffarence_goal_heading(self, final_position, final_orientation):
        #  ゴールと自身の方位の差
        #  print "final_orientation : ", math.degrees(final_orientation)
        goal_orientation = math.atan2(self.goal[0]-final_position[0], \
                                      self.goal[1]-final_position[1])

        #  print "goal_orientation : ", math.degrees(goal_orientation)
        diffarence_goal_heading = np.pi -  math.fabs(goal_orientation - final_orientation)
        #  print "diffarence_goal_heading : ", math.degrees(diffarence_goal_heading)
        
        return diffarence_goal_heading

    def vector_normalize(self, input_vector):
        input_vector = np.asarray(input_vector)
        normal_vector = copy.deepcopy(input_vector)
        norm = np.linalg.norm(input_vector)
        if norm != 0:
            normal_vector = input_vector / norm

        return normal_vector

    def evaluation_local_path(self):
        local_position_list = self.future_traj_position_list
        local_orientation_list = self.future_traj_orientation_list
        global_state_list = self.continuous_global_path_list
        #  print "local_position_list : ", local_position_list
        #  print "local_orientaiton_list : ", local_orientation_list
        self.evaluation_value = [0 for i in xrange(self.n_continuous_action)]

        inverse_global_path_distance = [0 for i in xrange(self.n_continuous_action)]
        nearest_obstacle_distance = [0 for i in xrange(self.n_continuous_action)]
        diffarence_goal_heading = [0 for i in xrange(self.n_continuous_action)]
        velocity_list = [0 for i in xrange(self.n_continuous_action)]

        
        for i_action in self.continuous_action_list:
            #  if  not self.future_traj_out_of_range_list[i_action][-1] \
                    #  and  not self.future_traj_collision_list[i_action][-1]:
            if not self.future_traj_collision_list[i_action][-1]:

                final_position = local_position_list[i_action][-1]

                # global_pathとの距離の逆数を評価
                inverse_global_path_distance[i_action] \
                    = self.check_inverse_global_path_distance(final_position, global_state_list)

                #  最近傍の障害物との距離
                nearest_obstacle_distance[i_action] \
                    = self.check_nearest_obstacle_distance(final_position)

                #  ゴールと自身の方位の差
                final_orientation = local_orientation_list[i_action][-1]
                diffarence_goal_heading[i_action] \
                    = self.check_diffarence_goal_heading(final_position, final_orientation)

            #  自身の速度
            velocity_list[i_action] = self.velocity_vector[i_action][0]


        #  print "inverse_global_path_distance : ", inverse_global_path_distance
        #  print "nearest_obstacle_distance : ", nearest_obstacle_distance
        #  print "diffarence_goal_heading : ", diffarence_goal_heading
        #  print "velocity_list : ", velocity_list
        
        a = 10.0
        b = 20.0
        c = 5.0
        d = 1.0

        self.evaluation_value = a * self.vector_normalize(inverse_global_path_distance) \
                              + b * self.vector_normalize(nearest_obstacle_distance) \
                              + c * self.vector_normalize(diffarence_goal_heading) \
                              + d * self.vector_normalize(velocity_list)
        #  print "self.evaluation_value : "
        #  print self.evaluation_value
        optimal_action = np.argmax(self.evaluation_value)
        #  print "optimal_action : ", optimal_action
        self.selected_traj_position_list = self.future_traj_position_list[optimal_action]
        self.selected_traj_orientation_list = self.future_traj_orientation_list[optimal_action]
        #  self.show_continuous_objectworld(selected_path=self.selected_traj_position_list)
        
        self.vis_velocity_vector.header.stamp = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler\
                (0.0,0.0, self.velocity_vector[optimal_action][1]*self.dt)

        self.vis_velocity_vector.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.vis_velocity_vector.scale.x = self.velocity_vector[optimal_action][0]
        #  print "self.vis_velocity_vector : ", self.vis_velocity_vector

        return optimal_action


def main(model_path, gpu):
    print "Here we go!!"
    idg = InputDataGenerator()

    rospy.init_node("ros_vin_predict")

    vis_vel_vec_pub = rospy.Publisher("/vis_velocity_vector", Marker, queue_size=1)
    action_command_pub = rospy.Publisher("/action_command", Int32, queue_size=1)

    agent = ValueIterationNetworkAgent(model_path, gpu)
    
    input_data = None
    #  state = [10, 6]
    state = np.asarray(map(lambda x: x / 2, list(idg.input_image_size)), dtype=np.int32)
    state_data = np.expand_dims(state, 0)
    #  print "state_data : ", state_data
    orientation = 0.0
    
    loop_rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if idg.gridmap_sub_flag and idg.local_goal_sub_flag and idg.grid_image is not None:
            print "*****************************************"
            idg.cvt_input_data()
            #  print "idg.input_data"
            #  print idg.input_data
            input_data = idg.input_data
            if gpu >= 0:
                input_data = cuda.to_gpu(input_data)
            print "state_data : ", state_data
            
            action = agent.get_action(input_data, state_data)
            #  print "action : ", action, "(", agent.dirs[action], ")"
            
            start_time = time.time()
            agent.show_path(input_data, state_data)
            continuous_state = [4.25, 4.25]

            local_planner = LocalPlanner(idg, agent, continuous_state, orientation)
            local_planner.transform_global_path_discreate2continuous()
            local_planner.get_future_trajectory(continuous_state, orientation)
            continuous_action = local_planner.evaluation_local_path()
            print "continuous_action : ", continuous_action, \
                    local_planner.velocity_vector[continuous_action]
            #  local_planner.show_continuous_objectworld\
                    #  (global_path=local_planner.continuous_global_path_list, \
                     #  local_path=local_planner.future_traj_position_list, \
                     #  selected_path=local_planner.selected_traj_position_list)
            #  continuous_action = 0
            elapsed_time = time.time() - start_time
            print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")

            #  vis_vel_vec_pub.publish(local_planner.vis_velocity_vector)
            action_command_pub.publish(continuous_action)
            
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
