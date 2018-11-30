#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JoyAngles = [0.0, 0.0, 0.0, 0.0, 0.0]
RobotAngles = [0.0, 0.0, 0.0, 0.0, 0.0]
ServoAngles = [0.0, 0.0, 0.0, 0.0, 0.0]

pub = rospy.Publisher('deg_of_servo',Float64MultiArray,queue_size=10)

ServoAngle = Float64MultiArray()

def robot_callback(data):
    global RobotAngles
    global JoyAngles
    global ServoAngle
    global ServoAngles
    global pub

    CurrentStatus = data.position
    Num = len(CurrentStatus)

    if Num == 5:
        RobotAngles = data.position
        try:
            ServoAngles[0] = RobotAngles[0]
        except:
            None

        #Bucket
        try:
            ServoAngles[3] = RobotAngles[1]
        except:
            None

        #Boom
        try:
            ServoAngles[1] = RobotAngles[2]
        except:
            None

        #Swing
        try:
            ServoAngles[2] = RobotAngles[3]
        except:
            None

        #Grapple
        try:
            ServoAngles[4] = RobotAngles[4]
        except:
            None
        ServoAngle.data = ServoAngles
        pub.publish(ServoAngle)


def listener():
    global ServoAngles
    global RobotAngles
    global JoyAngles
    global pub

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/master_robot/joint_states', JointState, robot_callback)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

        rate.sleep()
        #rospy.loginfo(ServoAngles[1])

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    listener()
