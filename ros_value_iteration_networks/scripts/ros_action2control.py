#!/usr/bin/env python
#coding:utf-8

import rospy
from std_msgs.msg import Int32

from knm_tiny_msgs.msg import Velocity


class ActionCommand2ControlVariable:
    def __init__(self):
        self.action_sub \
                = rospy.Subscriber("/action_command", Int32, self.actionCallback)

        self.control_pub \
                = rospy.Publisher("/tinypower/command_velocity", Velocity, queue_size=1)

        self.continuous_action_list = None
        self.n_continuous_action = 0
        self.velocity_vector = {}
        self.set_continuous_action()

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


    def actionCallback(self, msg):
        print "================================="
        action_command = msg.data
        print "action_command : ", action_command
        print "self.velocity_vector : ", self.velocity_vector[action_command]

        control_variable = Velocity()
        control_variable.header.stamp = rospy.Time.now()
        control_variable.op_linear = self.velocity_vector[action_command][0]
        control_variable.op_angular = self.velocity_vector[action_command][1]

        print "control_variable : ", control_variable
        
        self.control_pub.publish(control_variable)

def main():
    print "Here we go!!"

    ac2cv = ActionCommand2ControlVariable()

    rospy.init_node("ros_action2control")

    rospy.spin()


if __name__ == "__main__":
    main()

