#!/usr/bin/env python

import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform

pub = rospy.Publisher("hand_pos", Transform, queue_size = 10)
Center = Transform()
Center.translation.x = 0.0
Center.translation.y = 0.0
Center.translation.z = 0.0
Center.rotation.x = 0.0
Center.rotation.y = 0.0
Center.rotation.z = 1.0
Center.rotation.w = 0.0         #Gripper open and close

Scale = 0.25 / 1000


def callback_ros(data):
    global pub
    global Thumb
    global Scale

    Center.translation.x = -Scale * data.linear.x
    Center.translation.y = Scale * data.linear.z
    Center.translation.z = Scale * data.linear.y
    Center.rotation.x = 0.0
    pub.publish(Center)
    rospy.loginfo("x: %f, y: %f, z: %f", Center.translation.x, Center.translation.y, Center.translation.z)
    #rospy.loginfo("distance: %f", Center.rotation.w)


# Yes, a listener aka subscriber ;) obviously. Listens to: leapmotion/data
def listener():
    rospy.init_node('kinect_sub', anonymous=True)

    global pub
    rospy.Subscriber("kinect_pos", Twist, callback_ros)


    rospy.spin()


if __name__ == '__main__':
    listener()
