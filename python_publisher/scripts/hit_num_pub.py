#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32

hit_num = 0

def callback(data):
    global hit_num
    hit_num = data.data
    print(hit_num)

def listner():
    rospy.Subscriber("hit_num",Int32,callback)

def geomPub():
    pub = rospy.Publisher('tap_pos',Vector3,queue_size=10)
    rospy.init_node('geompub',anonymous = True)
    rate = rospy.Rate(30)

    a = [0.0, 0.15, -0.07]
    b = [0.15, 0.0, -0.07]
    c = [0.0, -0.15, -0.07]
    flag = 0

    tapped_pos = Vector3()

    while not rospy.is_shutdown():
        listner()

        #print(hit_num)
        if hit_num == 0:
            tapped_pos.x = a[0]
            tapped_pos.y = a[1]
            tapped_pos.z = a[2]
            flag = 1
        elif hit_num == 1:
            tapped_pos.x = b[0]
            tapped_pos.y = b[1]
            tapped_pos.z = b[2]
        else:
            tapped_pos.x = c[0]
            tapped_pos.y = c[1]
            tapped_pos.z = c[2]
            flag = 0

        pub.publish(tapped_pos)
        #print(tapped_pos.z)
        rate.sleep()

if __name__ == '__main__':
    try:
        geomPub()
    except rospy.ROSInterruptException:
        pass
