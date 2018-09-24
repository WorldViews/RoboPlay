#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64MultiArray

class ListnerVilma:
    def __init__(self):
        self.deg_array = rospy.Subscriber('deg_of_servo', Float64MultiArray, self.callback)

    def callback(self, data):
        self.x = data.data
        print self.x

if __name__ == '__main__':
    rospy.init_node('listner')
    myVilma = ListnerVilma()
    rospy.spin()
