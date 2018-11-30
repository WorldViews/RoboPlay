#!/usr/bin/env python
import rospy
import csv
import tf
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from datetime import datetime

now = datetime.now()
filename = '{0:%Y%m%d%H%M%S}_Slave.csv'.format(now)
f = open(filename, 'w')
csvWriter = csv.writer(f)
f.write("Time,Arm,Boom,Rotation,Bucket,Grapple,X,Y,Z,Pitch\n")
f.close()
eachdeg = [0.0,0.0,0.0,0.0,0.0]
Gripper_pos = tf.TransformListener()
start_time = 0
flag = 0
trans = [0,0,0]
rot = [0,0,0,0]

def output_callback(data):
    global now
    global f
    global csvWriter
    global eachdeg
    global Gripper_pos
    global start_time

    #rospy.loginfo(data.data)
    #trans = [0,0,0]

    for i in range(5):
        try:
            eachdeg[i] = data.position[i]
        except:
            eachdeg[i] = -100
        #f.write(str(eachdeg[i]) + ',')
        #a = a + 1

    #try:
        #(trans,rot) = Gripper_pos.lookupTransform('/base_link','/gripper_link',rospy.Time(0))
        #rospy.loginfo(trans[2])
        #f.write(str(trans[0]) + ',' + str(trans[1]) + ',' + str(trans[2]) + ',' + str(rot[0]) + ',' + str(rot[1]) + ',' + str(rot[2]) + ',' + str(rot[3]) + '\n')
        #rospy.loginfo(rot)
    #except:
        #pass
        #f.write('\n')


    #rospy.loginfo(eachdeg)
    #csvWriter.writerow(eachdeg)
    #f.close

def listener():
    global start_time
    global flag

    rospy.init_node('slave_output', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, output_callback)

    rate = rospy.Rate(30)

    start_time = rospy.get_time()



    while not rospy.is_shutdown():
        #try:
        f = open(filename, 'a')
        f.write(str(rospy.get_time() - start_time) + ',')
        a=0

        for i in range(5):
            f.write(str(eachdeg[i]) + ',')
            #a = a + 1

        try:
            (trans,rot) = Gripper_pos.lookupTransform('/base_link','/gripper_link',rospy.Time(0))
            #rospy.loginfo(trans[2])
            f.write(str(trans[0]) + ',' + str(trans[1]) + ',' + str(trans[2]) + ',' + str(rot[0]) + ',' + str(rot[1]) + ',' + str(rot[2]) + ',' + str(rot[3]) + '\n')
            #rospy.loginfo(rot)
        except:
            f.write('\n')

        f.close()
        rate.sleep()
        #rospy.loginfo(start_time)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    listener()
