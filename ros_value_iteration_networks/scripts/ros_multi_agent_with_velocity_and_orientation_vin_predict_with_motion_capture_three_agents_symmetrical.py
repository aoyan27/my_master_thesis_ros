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

#  from networks.multi_agent_vin_with_velocity_and_orientation import ValueIterationNetwork
from networks.multi_agent_vin_symmetrical_net2 import ValueIterationNetwork

import rospy
import tf
from std_msgs.msg import Int32, Int32MultiArray, MultiArrayDimension
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray


class InputDataGenerator:
    def __init__(self, local_goal, input_image_size=(20, 20), num_human=2):
        self.local_goal = local_goal
        self.input_image_size = input_image_size

        #  self.gridmap_sub = \
                #  rospy.Subscriber("/input_grid_map", OccupancyGrid, self.gridMapCallback)
        self.gridmap_sub = \
                rospy.Subscriber("/input_grid_map/expand", OccupancyGrid, self.gridMapCallback)

        self.human_sub = \
                rospy.Subscriber("/other_agents_velocity", MarkerArray, self.extractHumanCallback)
        self.my_agent_sub = \
                rospy.Subscriber("/my_agent_velocity", Marker, self.myAgentCallback)


        self.input_data = None
        self.grid_image = None
        self.reward_map = None
        
        self.grid_map = None
        self.human = None
        self.my_agent = None
        self.discreate_local_goal = None

        self.gridmap_sub_flag = False
        self.human_sub_flag = False
        self.my_agent_sub_flag = False

        self.resize_resolution = None

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

    def extractHumanCallback(self, msg):
        #  print "+++++++++++++++++++++++++++++++++++++++++++++"
        self.human_sub_flag = True
        self.human = msg
        #  print "len(self.human.markers) : ", len(self.human.markers)

    def myAgentCallback(self, msg):
        self.my_agent_sub_flag = True
        self.my_agent = msg


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
    

    def continuous2discreate(self, continuous_x, continuous_y):
        x = continuous_x + self.input_image_size[0]*self.resize_resolution/2
        y = continuous_y + self.input_image_size[1]*self.resize_resolution/2

        grid_x = int(round(x / self.resize_resolution))
        grid_y = int(round(y / self.resize_resolution))

        if grid_x < 0:
            grid_x = 0
        if grid_x >= self.input_image_size[0]:
            grid_x = self.input_image_size[0]-1
        if grid_y < 0:
            grid_y = 0
        if grid_y >= self.input_image_size[1]:
            grid_y = self.input_image_size[1]-1
        #  print "[grid_y, grid_x] : [", grid_y, grid_x, "]"

        return grid_y, grid_x

    def create_reward_map(self, local_goal):
        grid_y, grid_x = self.continuous2discreate(self.local_goal[0], self.local_goal[1])
        #  print "grid_x : ", grid_x
        #  print "grid_y : ", grid_y
        self.discreate_local_goal = [grid_y, grid_x]

        reward_map = np.zeros(self.input_image_size, dtype=np.uint8)
        reward_map[grid_y, grid_x] = 1
        #  print "reward_map : "
        #  print reward_map
        return reward_map
    
    def cvt_input_data(self):
        #  print "self.grid_image : "
        #  print self.grid_image
        self.reward_map = self.create_reward_map(self.local_goal)
        #  print "self.reward_map : "
        #  print self.reward_map
        input_data_ = np.concatenate((np.expand_dims(self.grid_image, 0), \
                np.expand_dims(self.reward_map, 0)), axis=0)
        self.input_data = np.expand_dims(input_data_, 0)

    def generate_my_agent_data(self):
        print "+++++++++++++++++++++++++++++++++++++"
        self.position_data \
                = self.continuous2discreate(self.my_agent.pose.position.x, \
                                            self.my_agent.pose.position.y)
        self.position_data = np.expand_dims(self.position_data, 0)
        print "position : ", self.position_data
        orientation = self.my_agent.pose.orientation
        self.orientation_data, self.velocity_data\
                = self.get_orientation_and_velocity(orientation)
        self.orientation_data = np.expand_dims(self.orientation_data, 0)
        print "orientation : ", self.orientation_data
        self.velocity_data = np.expand_dims(self.velocity_data, 0)
        print "velocity : ", self.velocity_data
        print "+++++++++++++++++++++++++++++++++++++"
        
    def get_orientation_and_velocity(self, human_orientation):
        q = (human_orientation.x, human_orientation.y, human_orientation.z, human_orientation.w)
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        #  print "yaw : ", yaw

        action_index = int(yaw / self.orientation_res)
        if action_index < 0:
            action_index = self.n_action + action_index
        #  print "action_index : ", action_index
        velocity = self.movement[action_index]
        #  print "velocity : ", velocity

        return self.orientation_list[action_index], velocity

    def generate_other_agent_data(self):
        if self.human_sub_flag:
            if len(self.human.markers) > 0:
                print "----------------------------------------"
                for i in xrange(self.num_human):
                    self.other_position_data[i] \
                            = self.continuous2discreate(self.human.markers[i].pose.position.x, \
                                                        self.human.markers[i].pose.position.y)
                    orientation = self.human.markers[i].pose.orientation
                    self.other_orientation_data[i], self.other_velocity_data[i]\
                            = self.get_orientation_and_velocity(orientation)

                    self.other_position_data[i] = np.expand_dims(self.other_position_data[i], 0)
                    self.other_orientation_data[i] = np.expand_dims(self.other_orientation_data[i], 0)
                    self.other_velocity_data[i] = np.expand_dims(self.other_velocity_data[i], 0)

                self.other_position_data = np.asarray(self.other_position_data)
                self.other_orientation_data = np.asarray(self.other_orientation_data)
                self.other_velocity_data = np.asarray(self.other_velocity_data)

                print "self.other_position_data : "
                print self.other_position_data
                print "self.other_orientation_data : "
                print self.other_orientation_data
                print "self.other_velocity_data : "
                print self.other_velocity_data
                self.human_sub_flag = False
                print "----------------------------------------"




class ValueIterationNetworkAgent:
    def __init__(self, model_path, gpu, input_image_size):
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
        self.traj_action_list = []

        self.max_challenge_times = input_image_size[0]/2 + input_image_size[1]/2
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
                   other_position_data, other_velocity_data, other_orientation_data, \
                   other_position_data2, other_velocity_data2, other_orientation_data2):
        if self.gpu >= 0:
            input_data = cuda.to_gpu(input_data)
            my_position_data = cuda.to_gpu(my_position_data)
            my_orientation_data = cuda.to_gpu(my_orientation_data)
            my_velocity_data = cuda.to_gpu(my_velocity_data)
            other_position_data = cuda.to_gpu(other_position_data)
            other_orientation_data = cuda.to_gpu(other_orientation_data)
            other_velocity_data = cuda.to_gpu(other_velocity_data)
            other_position_data2 = cuda.to_gpu(other_position_data2)
            other_orientation_data2 = cuda.to_gpu(other_orientation_data2)
            other_velocity_data2 = cuda.to_gpu(other_velocity_data2)

        p = self.model(input_data, \
                       my_position_data, my_velocity_data, my_orientation_data, \
                       other_position_data, other_velocity_data, other_orientation_data, \
                       other_position_data2, other_velocity_data2, other_orientation_data2)
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


    def get_path(self, input_data, \
                 my_position_data, my_velocity_data, my_orientation_data,\
                 other_position_data, other_velocity_data, other_orientation_data, \
                 other_position_data2, other_velocity_data2, other_orientation_data2, local_goal):

        state_list = []

        grid_image = input_data[0][0]

        my_position_data_ = copy.deepcopy(my_position_data)
        my_velocity_data_ = copy.deepcopy(my_velocity_data)
        my_orientation_data_ = copy.deepcopy(my_orientation_data)
        #  print "state_data_ : ", state_data_, type(state_data_)
        state = my_position_data_[0]
        #  print "state : ", state, type(state)
        state_list.append(list(state))
        #  print "state_list : ", state_list

        found = False
        for i in xrange(self.max_challenge_times):
            action = self.get_action(input_data, \
                    my_position_data_, my_velocity_data_, my_orientation_data_,\
                    other_position_data, other_velocity_data, other_orientation_data, \
                    other_position_data2, other_velocity_data2, other_orientation_data2)

            state_list.append(self.move(state, action, grid_image)[0])

            my_position_data_[0] = state_list[-1]
            my_velocity_data_[0] = self.movement[int(action)]
            my_orientation_data_[0] \
                    = math.atan2(self.movement[int(action)][0], self.movement[int(action)][1])
            if state_list[-1] == local_goal:
                found = True
                break

        self.traj_state_list = state_list

        return found


    def view_path(self, input_image, path, found):
        grid = copy.deepcopy(input_image[0][0])
        for state in path:
            grid[tuple(state)] = 2

        grid[tuple(path[0])] = 3
        if found:
            grid[tuple(path[-1])] = 4

        for row in grid:
            print "|",
            for i in row:
                if i==0:
                    print "-",
                elif i==1:
                    print "#",
                elif i==2:
                    print "*",
                elif i==3:
                    print "$",
                elif i==4:
                    print "G",
            print "|"


def main(model_path, gpu):
    rospy.init_node("ros_multi_agent_vin_predict")

    local_goal = (4.5, 0.0)
    idg = InputDataGenerator(local_goal)

    next_target_pub = rospy.Publisher("/vin/target_path", Int32MultiArray, queue_size=1)

    agent = ValueIterationNetworkAgent(model_path, gpu, idg.input_image_size)
    
    input_data = None


    loop_rate = rospy.Rate(100)

    ros_next_state = Int32MultiArray()
    layout = MultiArrayDimension()
    layout.size = idg.input_image_size[0]
    ros_next_state.layout.dim.append(layout)

    print "Here we go!!"

    while not rospy.is_shutdown():
        if idg.gridmap_sub_flag and idg.my_agent_sub_flag and idg.human_sub_flag \
                and idg.grid_image is not None:
            print "*****************************************"
            start_time = time.time()
            idg.cvt_input_data()
            #  print "idg.input_data"
            #  print idg.input_data
            idg.generate_my_agent_data()
            idg.generate_other_agent_data()
            
            
            found = agent.get_path(idg.input_data, \
                                   idg.position_data, idg.velocity_data, idg.orientation_data,\
                                   idg.other_position_data[0], idg.other_velocity_data[0], \
                                   idg.other_orientation_data[0], \
                                   idg.other_position_data[1], idg.other_velocity_data[1], \
                                   idg.other_orientation_data[1], idg.discreate_local_goal)
            print "agent.traj_state_list : ", agent.traj_state_list
            agent.view_path(idg.input_data ,agent.traj_state_list, found)

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
        default='/home/amsl/ros_catkin_ws/src/master_thesis/ros_value_iteration_networks/models/multi_agent_object_world_vin_model_50_epoch_5000_domain_9_action_20x20_50_to_0_n_objects_3_agents_with_velocity_and_orientation_future_seed_0_symmetrical_net2/multi_agent_vin_model_50.model', \
        type=str, help="load model path")

    args = parser.parse_args()
    print args
    main(args.model_path, args.gpu)
