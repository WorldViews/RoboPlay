#!/usr/bin/env python

""" For backwards compatibility with the old driver files
                Will be DELETED in the future               """
#Green LED is front


__author__ = 'flier'

import rospy
import math
from leap_motion.msg import leap
from leap_motion.msg import leapros
from geometry_msgs.msg import Transform

pub = rospy.Publisher("hand_pos", Transform, queue_size = 10)
Center = Transform()
Center.translation.x = 0.0
Center.translation.y = 0.0
Center.translation.z = 0.0
Center.rotation.x = 0.0
Center.rotation.y = 0.0
Center.rotation.z = 1.0
Center.rotation.w = 0.0

def callback_ros(data):
    global pub
    global marker_data
    global Center_x
    global Center_y
    global Center_z

    Center.translation.x = -(data.pinky_proximal.z + data.thumb_metacarpal.z)/2/1000
    Center.translation.y = -(data.pinky_proximal.x + data.thumb_metacarpal.x)/2/1000
    Center.translation.z = (data.pinky_proximal.y + data.thumb_metacarpal.y)/2/1000 - 0.15
    Center.rotation.x = - data.ypr.y * math.pi/180*2

    """
    Center.translation.x = 0.00
    Center.translation.y = 0.20
    Center.translation.z = 0.10
    Center.rotation.x = 0
    """
    pub.publish(Center)
    rospy.loginfo("x: %f, y: %f, z: %f, pitch: %f", Center.translation.x, Center.translation.y, Center.translation.z, Center.rotation.x)
    #rospy.loginfo("x: %f", Center.rotation.x)


# Yes, a listener aka subscriber ;) obviously. Listens to: leapmotion/data
def listener():
    rospy.init_node('leap_sub', anonymous=True)

    global pub
    global marker_data
    global Center_x
    global Center_y
    global Center_z
    # rospy.Subscriber("leapmotion/raw", leap, callback)
    rospy.Subscriber("leapmotion/data", leapros, callback_ros)


    rospy.spin()


if __name__ == '__main__':
    listener()
