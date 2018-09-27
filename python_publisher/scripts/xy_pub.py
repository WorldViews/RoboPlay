#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32

hit_pos = [0.15,0.0,-0.07]

def callback(data):
    global hit_pos
    hit_pos[0] = data.x
    hit_pos[1] = data.y
    hit_pos[2] = -0.07
    print(hit_pos)

def listner():
    rospy.Subscriber("hit_pos",Vector3,callback)

def geomPub():
    pub = rospy.Publisher('tap_pos',Vector3,queue_size=10)
    rospy.init_node('geompub',anonymous = True)
    rate = rospy.Rate(30)

    tapped_pos = Vector3()

    while not rospy.is_shutdown():
        listner()

        tapped_pos.x = hit_pos[0]
        tapped_pos.y = hit_pos[1]
        tapped_pos.z = hit_pos[2]

        pub.publish(tapped_pos)
        print(tapped_pos.x)
        rate.sleep()

if __name__ == '__main__':
    try:
        geomPub()
    except rospy.ROSInterruptException:
        pass
