#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState


hit_num = 0
receive_num = 0
receive_num_old = 0
flag = 0

DownHeight = -0.01
UpHeight = 0.06
Wrist_Ang_Far = 0.85
Wrist_Ang_Near = 1.05

Red_Left = [0.08, -0.23, DownHeight, Wrist_Ang_Far]
Red_Left_Up = [0.08, -0.23, UpHeight, Wrist_Ang_Far]
Yellow = [0.00, -0.23, DownHeight, Wrist_Ang_Far]
Yellow_Up = [0.00, -0.23, UpHeight, Wrist_Ang_Far]
White = [-0.08, -0.23, DownHeight, Wrist_Ang_Far]
White_Up = [-0.08, -0.23, UpHeight, Wrist_Ang_Far]
Red_Right = [-0.08, -0.15, DownHeight, Wrist_Ang_Near]
Red_Right_Up = [-0.08, -0.15, UpHeight, Wrist_Ang_Near]
Green = [0.00, -0.14, DownHeight, Wrist_Ang_Near]
Green_Up = [0.00, -0.14, UpHeight, Wrist_Ang_Near]
Blue = [0.08, -0.14, DownHeight, Wrist_Ang_Near]
Blue_Up = [0.08, -0.14, UpHeight, Wrist_Ang_Near]

RobotAngles = [0.0, 0.0, 0.0, 0.0, 0.0]

L_B2 = 0.07
L_23 = 0.107
L_34 = 0.107
L_4G = 0.1091

def callback(data):
    global hit_num
    global receive_num
    global flag
    receive_num_old = hit_num
    hit_num = data.data
    #rospy.loginfo(hit_num)
    if receive_num_old != hit_num:
        receive_num = receive_num + 1
        flag = 1
        #rospy.loginfo(flag)
    else:
        flag = 0

    #print(hit_num)


def robot_callback(data):
    global RobotAngles

    CurrentStatus = data.position
    Num = len(CurrentStatus)

    if Num == 5:
        RobotAngles = data.position

def listner():
    rospy.Subscriber("hit_num",Int32,callback)
    rospy.Subscriber('joint_states', JointState, robot_callback)



def geomPub():
    global Red_Left
    global Yellow
    global White
    global Red_Right
    global Green
    global Blue
    global receive_num
    global receive_num_old
    global flag

    pub = rospy.Publisher('hand_pos',Transform,queue_size=10)
    rospy.init_node('geompub',anonymous = True)
    rate = rospy.Rate(30)

    Destination = [0,0,0,0]
    Old_Destination = [0,0,0,0]
    tapped_pos = Transform()
    tapped_pos.translation.x = 0
    tapped_pos.translation.y = 0
    tapped_pos.translation.z = 0.38
    tapped_pos.rotation.x = 0
    Gripper_pos = tf.TransformListener()
    trans = [0,0,0]
    now_z = 0.00
    old_z = 0.00

    a = 0
    flags = 0

    while not rospy.is_shutdown():
        listner()


        try:
            (trans,rot) = Gripper_pos.lookupTransform('/base_link','/gripper_link',rospy.Time(0))
            now_z = trans[2]
            #rospy.loginfo(trans)
        except:
            pass



        if hit_num == 0:        #Red_Left
            Destination = Red_Left
        elif hit_num == 1:        #Yellow
            Destination = Yellow
        elif hit_num == 2:        #White
            Destination = White
        elif hit_num == 3:        #Red_Right
            Destination = Red_Right
        elif hit_num == 4:        #Green
            Destination = Green
        elif hit_num == 5:        #Blue
            Destination = Blue

        if (abs(trans[0]-Destination[0]) > 0.01 or abs(trans[1]-Destination[1]) > 0.01) and flags !=2:        #gripper is not above Destination
            #First, up the gripper
            if now_z < 0.025:
                if flags == 1 and flag == 0:
                    tapped_pos.translation.x = Destination[0]
                    tapped_pos.translation.y = Destination[1]
                    if Destination < 3:
                        tapped_pos.translation.z = UpHeight + 0.04
                    else:
                        tapped_pos.translation.z = UpHeight

                    tapped_pos.rotation.x = Destination[3]
                    a = 0.1
                else:
                    flags = 0
                    a = 0.2
                    tapped_pos.translation.x = trans[0]
                    tapped_pos.translation.y = trans[1]
                    if Destination < 3:
                        tapped_pos.translation.z = UpHeight + 0.04
                    else:
                        tapped_pos.translation.z = UpHeight

                    if trans[1] < -0.20:
                        tapped_pos.rotation.x = Wrist_Ang_Far
                    else:
                        tapped_pos.rotation.x = Wrist_Ang_Near
                    rospy.loginfo("now_z: %f", now_z)
                    rospy.loginfo("tapped_pos.translation.z: %f", tapped_pos.translation.z)

                #a = 0
                #rospy.loginfo("up")
            else:      #go to the Destination (x and y only)
                tapped_pos.translation.x = Destination[0]
                tapped_pos.translation.y = Destination[1]
                if Destination < 3:
                    tapped_pos.translation.z = UpHeight + 0.04
                else:
                    tapped_pos.translation.z = UpHeight
                tapped_pos.rotation.x = Destination[3]
                a = 1
                flags = 1
                #rospy.loginfo("x-y move")
                #rospy.loginfo("trans[0]: %f", trans[0])
                #rospy.loginfo("Destination[0]: %f", Destination[0])
        else:           #gripper is above the Destination
            flags = 2
            if Destination == Old_Destination:      #Already pushed the button
                tapped_pos.translation.x = Destination[0]
                tapped_pos.translation.y = Destination[1]
                if Destination < 3:
                    tapped_pos.translation.z = UpHeight + 0.04
                else:
                    tapped_pos.translation.z = UpHeight
                a = 2
                flags = 0
            else:       #Not push the button yet
                tapped_pos.translation.x = Destination[0]
                tapped_pos.translation.y = Destination[1]
                tapped_pos.translation.z = DownHeight
                a = 3
                if now_z > old_z and now_z < 0.015:
                    Old_Destination = Destination
                    if Destination < 3:
                        tapped_pos.translation.z = UpHeight + 0.04
                    else:
                        tapped_pos.translation.z = UpHeight
                    a = 4



        #rospy.loginfo("x: %f", tapped_pos.translation.x)
        #rospy.loginfo("y: %f", tapped_pos.translation.y)
        #rospy.loginfo("z: %f", tapped_pos.translation.z)
        #rospy.loginfo("wrist_ang: %f", tapped_pos.rotation.x)


        old_z = now_z
        #rospy.loginfo(a)
        #Old_Destination =Destination
        #print(tapped_pos.z)
        pub.publish(tapped_pos)
        #receive_num_old = receive_num
        rate.sleep()

if __name__ == '__main__':
    try:
        geomPub()
    except rospy.ROSInterruptException:
        pass
