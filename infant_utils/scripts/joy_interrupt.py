#!/usr/bin/env python
#coding:utf-8

import rospy 
from knm_tiny_msgs.msg import Velocity
from sensor_msgs.msg import Joy

class JoyInterrupt:
    def __init__(self):
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCallback)

        self.cmd_sub \
                = rospy.Subscriber("/control_command", Velocity, self.cmdCallback)

        self.cmd_vel_pub = rospy.Publisher("/tinypower/command_velocity", Velocity, queue_size=1)
        
        self.cmd_ = Velocity()
        self.cmd_vel_joy = Velocity()
        self.joy_mode = False

    def joyCallback(self, msg):
        self.cmd_vel_joy.header.stamp = rospy.Time.now()
        gain = (1-msg.buttons[8]*0.5) * (1+msg.buttons[10]*0.6);
        self.cmd_vel_joy.op_linear=msg.axes[1]*0.8*gain;
        self.cmd_vel_joy.op_angular=-msg.axes[0]*1.2*gain;
        if msg.buttons[3] == 1:
            self.joy_mode = True
        elif msg.buttons[14] == 1:
            self.joy_mode = False
            

    def cmdCallback(self, msg):
        if not self.joy_mode:
            self.cmd_ = msg
        else:
            self.cmd_ = self.cmd_vel_joy

def main():
    rospy.init_node("joy_interrupt")
    ji = JoyInterrupt()

    print "Here we go!!"

    loop_rate = rospy.Rate(40)
    
    while not rospy.is_shutdown():
        ji.cmd_vel_pub.publish(ji.cmd_)
        loop_rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()
