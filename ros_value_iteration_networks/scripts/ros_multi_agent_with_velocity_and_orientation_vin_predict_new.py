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

from networks.multi_agent_vin_with_velocity_and_orientation import ValueIterationNetwork

import rospy
import tf
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray, MultiArrayDimension
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray


class InputDataGenerator:
    def __init__(self, input_image_size=(20, 20), num_human=1):
        self.input_image_size = input_image_size

        #  self.gridmap_sub = \
                #  rospy.Subscriber("/input_grid_map/vin/expand", OccupancyGrid, self.gridMapCallback)
        self.gridmap_sub = \
                rospy.Subscriber("/input_grid_map/vin", OccupancyGrid, self.gridMapCallback)
        self.human_sub = \
                rospy.Subscriber("/other_agents_state", Float32MultiArray, self.humanCallback)
        self.lcl_sub = rospy.Subscriber("/lcl5", Odometry, self.lclCallback)
        self.reward_map_sub = \
                rospy.Subscriber("/reward_map/vin", OccupancyGrid, self.rewardMapCallback)
        self.global_map_sub = \
                rospy.Subscriber("/map", OccupancyGrid, self.globalMapCallback)

        self.input_data = None
        self.grid_image = None
        self.reward_map = None
        
        self.grid_map = None
        self.human = None
        self.lcl = None
        self.global_map = None

        self.gridmap_sub_flag = False
        self.human_sub_flag = False
        self.lcl_sub_flag = False
        self.reward_map_sub_flag = False

        self.resize_resolution = None
        self.resize_rate = None

        self.num_human = num_human

        self.position_data = None
        self.orientation_data = None
        self.velocity_data = None

        self.other_position_data = [[] for i in xrange(self.num_human)]
        self.other_orientation_data = [[] for i in xrange(self.num_human)]
        self.other_velocity_data = [[] for i in xrange(self.num_human)]

        self.action_list = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        self.n_action = len(self.action_list)
        self.dirs = \
                {0: '>', 1: 'dr', 2: 'v', 3: 'dl', 4: '<', 5: 'ul', 6: '^', 7: 'ur', 8: '-'}
        self.movement \
            = {0: [0, 1], 1: [1, 1], 2: [1, 0], 3: [1, -1], \
               4: [0, -1], 5: [-1, -1], 6: [-1, 0], 7: [-1, 1], 8: [0, 0]}
        self.orientation_list = {}
        for i in xrange(self.n_action):
            self.orientation_list[i] = math.atan2(self.movement[i][0], self.movement[i][1])

        self.orientation_res = 2.0*math.pi / float(self.n_action)

    
    def show_gridmap(self, grid_map):
        if not grid_map.ndim == 2:
            rospy.logerr('Error occurred!! Check grid dimentions!!')
        else:
            vis_grid_map = copy.deepcopy(grid_map)
            for row in vis_grid_map:
                print "|",
                for i in row:
                    if i==0:
                        print "-",
                    elif i==1:
                        print "#",
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
        unknown_index = np.where(image == -1)
        image[unknown_index] = 0
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


            

    """
    globalMapCallback
    """
    def globalMapCallback(self, msg):
        self.global_map = msg


    """
    gridMapCallback
    """
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

        self.resize_resolution = cell_size*rows/self.input_image_size[0]
        #  print "resize_resolution : ", self.resize_resolution

        grid_data = np.asarray(msg.data).reshape((rows, cols))
        #  print "grid_data : "
        #  self.show_gridmap(grid_data)
        resize_grid_image = self.grid2image(grid_data)
        #  print "resize_grid_image : "
        #  print resize_grid_image
        #  self.view_image(resize_grid_image, 'local_grid_map')
        self.grid_image = resize_grid_image
        #  self.show_gridmap(self.grid_image)

    """
    lclCallback
    """
    def lclCallback(self, msg):
        self.lcl_sub_flag = True
        #  print "msg(lcl) : ", msg
        self.lcl = msg


    def continuous2discreate(self, continuous_x, continuous_y):
        x = continuous_x + self.input_image_size[0]*self.resize_resolution/2
        y = continuous_y + self.input_image_size[1]*self.resize_resolution/2

        grid_x = int(x / self.resize_resolution)
        grid_y = int(y / self.resize_resolution)

        if grid_x < 0:
            grid_x = 0
        if grid_x >= self.input_image_size[0]:
            grid_x = self.input_image_size[0]-1
        if grid_y < 0:
            grid_y = 0
        if grid_y >= self.input_image_size[1]:
            grid_y = self.input_image_size[1]-1

        return grid_y, grid_x


    def create_reward_map(self, reward_map_raw):
        reward_map_size_raw = len(reward_map_raw)
        #  print "reward_map_size_raw : ", reward_map_size_raw
        self.resize_rate = float(self.input_image_size[0]) / float(reward_map_size_raw)
        #  print "self.resize_rate : ", self.resize_rate
        local_goal_index = np.where(reward_map_raw == 100)
        discreate_local_goal_raw = np.array([local_goal_index[0][0], local_goal_index[1][0]])
        self.discreate_local_goal = (discreate_local_goal_raw * self.resize_rate).astype(np.int32)
        print "self.discreate_lcoal_goal : ", self.discreate_local_goal

        reward_map = np.zeros(self.input_image_size, dtype=np.uint8)
        reward_map[tuple(self.discreate_local_goal)] = 1
        #  print "reward_map : "
        #  print reward_map
        return reward_map

    def rewardMapCallback(self, msg):
        #  print "------------------------------------------"
        self.reward_map_sub_flag = True

        rows = msg.info.height
        cols = msg.info.width
        reward_map_raw = np.asarray(msg.data).reshape((rows, cols))

        self.reward_map = self.create_reward_map(reward_map_raw)
    
    def cvt_input_data(self):
        #  print "self.grid_image : "
        #  print self.grid_image
        #  print "self.reward_map : "
        #  print self.reward_map
        input_data_ = np.concatenate((np.expand_dims(self.grid_image, 0), \
                np.expand_dims(self.reward_map, 0)), axis=0)
        self.input_data = np.expand_dims(input_data_, 0)


    """
    humanCallback
    """
    def humanCallback(self, msg):
        #  print "+++++++++++++++++++++++++++++++++++++++++++++"
        self.human_sub_flag = True
        num_observation = len(msg.data) / 3
        #  print "num_observation : ", num_observation
        if(num_observation >= self.num_human):
            self.human = np.asarray(msg.data).reshape(self.num_human, 3)
        else:
            self.human = np.asarray(msg.data).reshape(num_observation, 3)
        #  print "self.human : "
        #  print self.human

    def generate_my_agent_data(self):
        return_flag = False

        self.position_data \
                = np.asarray(map(lambda x: x / 2, list(self.input_image_size)), dtype=np.int32)
        self.position_data = np.expand_dims(self.position_data, 0)
        print "position : ", self.position_data
        q = (self.lcl.pose.pose.orientation.x, \
             self.lcl.pose.pose.orientation.y, \
             self.lcl.pose.pose.orientation.z, \
             self.lcl.pose.pose.orientation.w)
        yaw = tf.transformations.euler_from_quaternion(q)[2] 
        print "yaw : ", yaw
        action_index = int(yaw / self.orientation_res)
        print "action_index_ : ", action_index
        if action_index < 0:
            action_index = self.n_action-1 + action_index
        #  print "action_index : ", action_index
        self.orientation_data = np.expand_dims(self.orientation_list[action_index], 0)
        print "orientation : ", self.orientation_data

        velocity = self.movement[action_index]
        #  print "velocity : ", velocity
        self.velocity_data = np.expand_dims(velocity, 0)
        print "velocity : ", self.velocity_data
        
        if len(self.position_data) != 0 \
                and len(self.orientation_data) != 0 and len(self.velocity_data) != 0:
            return_flag = True

        return return_flag
    
        
    def get_other_orientation_and_velocity(self, human_orientation):
        yaw = human_orientation
        print "yaw : ", yaw

        action_index = int(yaw / self.orientation_res)
        print "actin_index : ", action_index
        if action_index < 0:
            action_index = self.n_action-1 + action_index
        print "action_index_ : ", action_index
        velocity = self.movement[action_index]
        #  print "velocity : ", velocity

        return self.orientation_list[action_index], velocity

    def generate_other_agent_data(self):
        return_flag = False
        if self.human_sub_flag:
            if len(self.human) > 0:
                for i in xrange(self.human.shape[0]):
                    self.other_position_data[i] \
                            = np.array([self.human[i][1], self.human[i][0]]) * self.resize_rate
                    self.other_position_data[i] = self.other_position_data[i].astype(np.int32)
                    orientation = self.human[i][2]
                    self.other_orientation_data[i], self.other_velocity_data[i]\
                            = self.get_other_orientation_and_velocity(orientation)

                self.other_position_data = np.asarray(self.other_position_data)
                self.other_orientation_data = np.asarray(self.other_orientation_data)
                self.other_velocity_data = np.asarray(self.other_velocity_data)

                print "self.other_position_data : ", self.other_position_data
                print "self.other_orientation_data : ", self.other_orientation_data
                print "self.other_velocity_data : ", self.other_velocity_data
                self.human_sub_flag = False

                return_flag = True

        return return_flag




class ValueIterationNetworkAgent:
    def __init__(self, idg, model_path, gpu, input_image_size, max_times=None):
        self.idg = idg

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
        
        self.traj_state_list = []
        self.traj_continuous_state_list = []
        self.traj_action_list = []
        
        self.max_challenge_times = None
        if max_times is None:
            self.max_challenge_times = input_image_size[0]/2 + input_image_size[1]/2
        else:
            self.max_challenge_times = max_times
        #  print "max_challenge_times : ", max_challenge_times

    def set_action(self):
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
    
    def get_action(self, input_data, \
                   my_position_data, my_velocity_data, my_orientation_data,\
                   other_position_data, other_velocity_data, other_orientation_data):
        if self.gpu >= 0:
            input_data = cuda.to_gpu(input_data)
            my_position_data = cuda.to_gpu(my_position_data)
            my_orientation_data = cuda.to_gpu(my_orientation_data)
            my_velocity_data = cuda.to_gpu(my_velocity_data)
            other_position_data = cuda.to_gpu(other_position_data)
            other_orientation_data = cuda.to_gpu(other_orientation_data)
            other_velocity_data = cuda.to_gpu(other_velocity_data)

        p = self.model(input_data, \
                       my_position_data, my_velocity_data, my_orientation_data, \
                       other_position_data, other_velocity_data, other_orientation_data)
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

    def continuous2discreate(self, x, y, resolution, origin=Pose()):
        discreate_x = int((x + -1.0*origin.position.x) / resolution)
        discreate_y = int((y + -1.0*origin.position.y) / resolution)
        return discreate_y, discreate_x
    
    def discreate2continuous(self, x, y, resolution, origin=Pose()):
        continuous_x = x * resolution - -1.0*origin.position.x
        continuous_y = y * resolution - -1.0*origin.position.y
        return continuous_y, continuous_x


    def get_path(self, input_data, \
                 my_position_data, my_velocity_data, my_orientation_data,\
                 other_position_data, other_velocity_data, other_orientation_data, local_goal):

        state_list = []
        continuous_state_list = []

        grid_image = input_data[0][0]
        #  print "input_data : "
        #  print input_data

        my_position_data_ = copy.deepcopy(my_position_data)
        my_velocity_data_ = copy.deepcopy(my_velocity_data)
        my_orientation_data_ = copy.deepcopy(my_orientation_data)
        #  print "state_data_ : ", state_data_, type(state_data_)
        #  print "my_position_data_ : ", my_position_data_
        #  print "my_velocity_data_ : ", my_velocity_data_
        #  print "my_orientation_data_ : ", my_orientation_data_

        #  print "other_position_data : ", other_position_data
        #  print "other_velocity_data : ", other_velocity_data
        #  print "other_orientation_data : ", other_orientation_data

        #  print "self.idg.grid_map.info : "
        #  print self.idg.grid_map.info
        state = my_position_data_[0]
        continuous_y, continuous_x \
                = self.discreate2continuous(state[1], state[0], \
                                            self.idg.resize_resolution, \
                                            origin=self.idg.grid_map.info.origin)
        continuous_state = [continuous_y, continuous_x]

        state_list.append(list(state))
        #  print "state_list : ", state_list
        continuous_state_list.append(continuous_state)
        #  print "continuous_state_list : ", continuous_state_list

        for i in xrange(self.max_challenge_times):
            action = self.get_action(input_data, \
                    my_position_data_, my_velocity_data_, my_orientation_data_,\
                    other_position_data, other_velocity_data, other_orientation_data)
            print "action : ", action, "(", self.dirs[int(action)], ")"

            state = self.move(state, action, grid_image)[0]
            state_list.append(state)
            continuous_y, continuous_x \
                    = self.discreate2continuous(state[1], state[0], \
                                                self.idg.resize_resolution, \
                                                origin=self.idg.grid_map.info.origin)
            continuous_state = [continuous_y, continuous_x]
            continuous_state_list.append(continuous_state)

            my_position_data_[0] = state_list[-1]
            my_velocity_data_[0] = self.movement[int(action)]
            my_orientation_data_[0] \
                    = math.atan2(self.movement[int(action)][0], self.movement[int(action)][1])
            if tuple(state_list[-1]) == tuple(local_goal):
                break

        self.traj_state_list = state_list
        self.traj_continuous_state_list = continuous_state_list



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
    rospy.init_node("ros_multi_agent_vin_predict")

    idg = InputDataGenerator()

    next_target_pub = rospy.Publisher("/vin/target_path", Int32MultiArray, queue_size=1)
    next_target_continuous_pub \
            = rospy.Publisher("/vin/target_path/continuous", Float32MultiArray, queue_size=1)

    agent = ValueIterationNetworkAgent(idg, model_path, gpu, idg.input_image_size)
    #  agent = ValueIterationNetworkAgent(idg, model_path, gpu, idg.input_image_size, max_times=1)
    
    input_data = None

    loop_rate = rospy.Rate(100)

    ros_next_state = Int32MultiArray()
    layout = MultiArrayDimension()
    layout.size = idg.input_image_size[0]
    ros_next_state.layout.dim.append(layout)

    ros_next_state_continuous = Float32MultiArray()
    layout_continuous = MultiArrayDimension()
    layout_continuous.size = idg.input_image_size[0]
    ros_next_state_continuous.layout.dim.append(layout_continuous)


    print "Here we go!!"


    while not rospy.is_shutdown():
        if idg.gridmap_sub_flag and idg.reward_map_sub_flag and idg.human_sub_flag \
                and idg.lcl_sub_flag and idg.grid_image is not None:
            print "*****************************************"
            start_time = time.time()
            idg.cvt_input_data()
            #  print "idg.input_data"
            #  print idg.input_data
            my_agent_flag = idg.generate_my_agent_data()
            other_agent_flag = idg.generate_other_agent_data()
            
            if my_agent_flag and other_agent_flag:
                agent.get_path(idg.input_data, \
                               idg.position_data, idg.velocity_data, idg.orientation_data,\
                               idg.other_position_data, idg.other_velocity_data, \
                               idg.other_orientation_data, idg.discreate_local_goal)
                print "agent.traj_state_list : ", agent.traj_state_list
                print "agent.traj_continuous_state_list : ", agent.traj_continuous_state_list
                agent.view_path(idg.input_data ,agent.traj_state_list)

                ros_next_state.data = np.asarray(agent.traj_state_list).reshape(-1)
                next_target_pub.publish(ros_next_state)

                ros_next_state_continuous.data \
                        = np.asarray(agent.traj_continuous_state_list).reshape(-1)
                next_target_continuous_pub.publish(ros_next_state_continuous)

                elapsed_time = time.time() - start_time
                print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")
            else:
                print "No human!!"
                ros_next_state.data = []
                next_target_pub.publish(ros_next_state)

            idg.gridmap_sub_flag = False
            idg.reward_map_sub_flag = False
            idg.human_sub_flag = False
            idg.lcl_sub_flag = False


        loop_rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='This script is ros_vin_predict ...')
    parser.add_argument('-g', '--gpu', default=-1, type=int, help='number of gpu device')
    parser.add_argument('-m', '--model_path', \
        default='/home/amsl/ros_catkin_ws/src/master_thesis/ros_value_iteration_networks/models/multi_agent_object_world_vin_model_50_epoch_5000_domain_9_action_20x20_50_to_0_n_objects_2_agents_with_velocity_and_orientation_future_seed_0_relu_adam/multi_agent_vin_model_50.model', \
        type=str, help="load model path")

    args = parser.parse_args()
    print args
    main(args.model_path, args.gpu)
