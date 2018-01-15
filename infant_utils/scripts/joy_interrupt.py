#!/usr/bin/env python
#coding:utf-8

import rospy 
from knm_tiny_msgs.msg import Velocity
from sensor_msgs.msg import Joy

class JoyInterrupt:
    def __init__(self):
        self.joy_sub = ropy.Subscriber("/joy", Joy, self.joyCallback)

        self.cmd_vell_sub \
                = rospy.Subscriber("/tinypower/command_velocity". Velocity, cmdVelCallback)

    def joyCallback(self, msg):
        pass

    def cmdVelCallnack(self, msg):
        pass

def main():
    rospy.init_node("joy_interrupt")
    ji = JpyInterrupt()

    print "Here we go!!"

    rospy.spin()


if __name__ == "__main__":
    main()
