#!/usr/bin/env python
#coding : utf-8

import numpy as np
np.set_printoptions(precision=3, suppress=True, threshold=np.inf)

import chainer 
from chainer import cuda, Variable, optimizers, serializers
from chainer import Chain
import chainer.functions as F
import chainer.links as L

class ValueIterationNetwork(Chain):
    def __init__(self, n_in=2, l_h=150, l_q=5, n_out=5, k=10):
        super(ValueIterationNetwork, self).__init__(
            conv1 = L.Convolution2D(n_in, l_h, 3, stride=1, pad=1), 
            conv2 = L.Convolution2D(l_h, 1, 1, stride=1, pad=0, nobias=True),

            conv3a = L.Convolution2D(1, l_q, 3, stride=1, pad=1, nobias=True),
            conv3b = L.Convolution2D(1, l_q, 3, stride=1, pad=1, nobias=True),

            l4a = L.Linear(None, 32, nobias=True),
            l4b = L.Linear(None, 32, nobias=True),
            l4c = L.Linear(None, 32, nobias=True),
            l4d = L.Linear(None, 32, nobias=True),
            
            l5a = L.Linear(None, 128, nobias=True),
            l5b = L.Linear(None, 128, nobias=True),
            l5c = L.Linear(None, 128, nobias=True),
            l5d = L.Linear(None, 128, nobias=True),

            l6a = L.Linear(None, 64, nobias=True),
            l6b = L.Linear(None, 64, nobias=True),

            l7 = L.Linear(None, 32, nobias=True),
            l8 = L.Linear(None, n_out, nobias=True),
        )

        self.k = k
    
    def attention(self, q, state_list):
        #  print "q.data : ",
        #  print q.data[0]
        w = np.zeros(q.data.shape)
        #  print "w : ", w.shape
        for i in xrange(len(state_list)):
            #  print "state_list : ", state_list[i]
            w[i, :, int(state_list[i][0]), int(state_list[i][1])] = 1.0
            #  print "w : "
            #  print w[i]

        if isinstance(q.data, cuda.ndarray):
            w = cuda.to_gpu(w)

        #  print q.data.shape
        #  print w.shape
        
        w = Variable(w.astype(np.float32))
        #  print "state_list : "
        #  print state_list[0]
        a = q * w
        #  print "a : "
        #  print a.shape
        a = F.reshape(a, (a.data.shape[0], a.data.shape[1], -1))
        #  print "a() : "
        #  print a.shape

        q_out = F.sum(a, axis=2)
        #  print "q_out : "
        #  print q_out
        #  print q_out.shape
        return q_out


    def __call__(self, input_data, my_state_list, my_velocity_list, my_orientation_list, \
                 other_state_list, other_velocity_list, other_orientation_list):

        input_data = Variable(input_data.astype(np.float32))

        h = F.relu(self.conv1(input_data))
        #  print "h : ", h
        self.r = self.conv2(h)
        #  print "self.r : ", self.r
        #  print "self.r : ", self.r.data.shape

        """
        VI-Module
        """
        q = self.conv3a(self.r)
        #  print "q : ", q.data.shape
        
        self.v = F.max(q, axis=1, keepdims=True)
        #  print "self.v : ", self.v.shape

        for i in xrange(self.k):
            q = self.conv3a(self.r) + self.conv3b(self.v)
            self.v = F.max(q, axis=1, keepdims=True)

        #  print "q(after k) : ", q.shape
        #  print "q(after k) : ", q
        #  print "self.v : ", self.v
        
        q = self.conv3a(self.r) + self.conv3b(self.v)
        q_out = self.attention(q, my_state_list)

        
        """
        my_state
        """
        my_velocity_ = my_velocity_list.astype(np.float32)
        #  print "my_velocity_ : ", my_velocity_
        my_orientation_size = my_orientation_list.shape[0]
        my_orientation_ = my_orientation_list.astype(np.float32).reshape(my_orientation_size, 1)
        #  print "my_orientation_ : ", my_orientation_
        my_state = F.concat((q_out, my_velocity_), axis=1)
        my_state = F.concat((my_state, my_orientation_), axis=1)
        #  print "my_state : ", my_state

        """
        other_state
        """
        other_state_ = other_state_list.astype(np.float32)
        #  print "other_state_ : ", other_state_
        other_velocity_ = other_velocity_list.astype(np.float32)
        #  print "other_velocity_ : ", other_velocity_
        other_orientation_size = other_orientation_list.shape[0]
        other_orientation_ \
                = other_orientation_list.astype(np.float32).reshape(other_orientation_size, 1)
        #  print "other_orientation_ : ", other_orientation_
        other_state = F.concat((other_state_, other_velocity_), axis=1)
        other_state = F.concat((other_state, other_orientation_), axis=1)
        #  print "other_state : ", other_state

        
        """
        Policy
        """
        h1a = F.relu(self.l4a(my_state))
        h1b = F.relu(self.l4b(my_state))

        h1c_1 = F.relu(self.l4c(other_state))

        h1d_1 = F.relu(self.l4d(other_state))

        concat_my_state = F.concat((h1a, h1c_1), axis=1)

        concat_other_state = F.concat((h1b, h1d_1), axis=1)

        h2a = F.relu(self.l5a(concat_my_state))
        h2b = F.relu(self.l5b(concat_my_state))

        h2c_1 = F.relu(self.l5c(concat_other_state))

        h2d_1 = F.relu(self.l5d(concat_other_state))

        concat2_my_state = F.concat((h2a, h2c_1), axis=1)

        concat2_other_state = F.concat((h2b, h2d_1), axis=1)

        h3a = F.relu(self.l6a(concat2_my_state))
        tmp_h3a = F.expand_dims(h3a, axis=0)

        h3b_1 = F.relu(self.l6b(concat2_other_state))
        tmp_h3b_1 = F.expand_dims(h3b_1, axis=0)

        tmp_concat = F.concat((tmp_h3a, tmp_h3b_1), axis=0)
        max_pooling = F.max(tmp_concat, axis=0)

        h4 = F.relu(self.l7(max_pooling))

        y = self.l8(h4)

        return y


    def forward(self, input_data, my_state_list, my_velocity_list, my_orientation_list, \
                other_state_list, other_velocity_list, other_orientation_list, \
                action_list):

        y = self.__call__(input_data, my_state_list, my_velocity_list, my_orientation_list, \
                          other_state_list, other_velocity_list, other_orientation_list)
        #  print "y : ", y
        
        t = Variable(action_list.astype(np.int32))

        return F.softmax_cross_entropy(y, t), F.accuracy(y, t)



