#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64MultiArray

TimeDif = 3.0
GripperOpen = 0.70
GripperClose = -0.60

class ServoDeg:

    def __init__(self):
        pub = rospy.Publisher('deg_of_servo',Float64MultiArray,queue_size=10)
        self.start_time = time.time()
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.deg_of_servo = self.cal_deg_of_servo(time.time())
            my_array_for_publishing = Float64MultiArray(data = self.deg_of_servo)
            pub.publish(my_array_for_publishing)
            rate.sleep()


    def cal_deg_of_servo(self, now_time):
        ElapsedTime = now_time - self.start_time
        if ElapsedTime < TimeDif:
            deg_of_servo = [1.21, 1.01, -1.57, 0.00, GripperClose]
        elif TimeDif < ElapsedTime and ElapsedTime < 2 * TimeDif:
            deg_of_servo = [1.21, 1.01, -1.57, 0.82, GripperOpen]
        elif TimeDif < ElapsedTime and ElapsedTime < 3 * TimeDif:
            deg_of_servo = [1.21, 1.01, -1.57, 0.82, GripperClose]
        elif 3 * TimeDif < ElapsedTime and ElapsedTime < 4 * TimeDif:
            deg_of_servo = [1.21, 1.01, -1.57, -0.41, GripperClose]
        elif 4 * TimeDif < ElapsedTime and ElapsedTime < 5 * TimeDif:
            deg_of_servo = [-0.53, 2.45, -0.70, -0.41, GripperClose]
        elif 5 * TimeDif < ElapsedTime and ElapsedTime < 6 * TimeDif:
            deg_of_servo = [-0.53, 2.45, -0.07, -0.41, GripperClose]
        elif 6 * TimeDif < ElapsedTime and ElapsedTime < 7 * TimeDif:
            deg_of_servo = [-0.53, 2.45, 0.90, -0.41, GripperClose]
        elif 7 * TimeDif < ElapsedTime and ElapsedTime < 8 * TimeDif:
            deg_of_servo = [0.95, 0.62, 1.57, -0.70, GripperClose]
        elif 8 * TimeDif < ElapsedTime and ElapsedTime < 9 * TimeDif:
            deg_of_servo = [0.95, 0.62, 1.57, 1.30, GripperClose]
        elif 9 * TimeDif < ElapsedTime:
            deg_of_servo = [0.95, 0.62, 1.57, 1.30, GripperOpen]
        else:
            deg_of_servo = [0.0,0.0,0.0,0.0,0.0]
        return deg_of_servo


if __name__ == '__main__':
    rospy.init_node('array_pub')
    a = ServoDeg()
