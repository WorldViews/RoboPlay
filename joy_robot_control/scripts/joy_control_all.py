#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JoyAngles = [0.0, 0.0, 0.0, 0.0, 0.0]
RobotAngles = [0.0, 0.0, 0.0, 0.0, 0.0]
ServoAngles = [0.0, 0.0, 0.0, 0.0, 0.0]

def joy_callback(data):
    global JoyAngles
    a = 0
    k_arm = 0.2
    k_grapple = 0.3

    JoyAngles[0] = - k_arm * data.axes[1]       #Arm
    JoyAngles[1] = k_arm * data.axes[5]         #Boom
    JoyAngles[2] = k_arm * data.axes[0]         #Swing
    JoyAngles[3] = k_arm * data.axes[2]         #Bucket
    JoyAngles[4] = k_grapple * data.buttons[5] - k_grapple * data.buttons[4]          #Grapple


def robot_callback(data):
    global RobotAngles
    global JoyAngles

    CurrentStatus = data.position
    Num = len(CurrentStatus)

    if Num == 5:
        RobotAngles = data.position


def listener():
    global ServoAngles
    global RobotAngles
    global JoyAngles

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('joint_states', JointState, robot_callback)
    rospy.Subscriber("joy", Joy, joy_callback)


    pub = rospy.Publisher('deg_of_servo',Float64MultiArray,queue_size=10)
    ServoAngle = Float64MultiArray()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        for i in range(5):
            if JoyAngles[i] != 0:

                #Arm
                try:
                    if JoyAngles[0] != 0:
                        ServoAngles[0] = RobotAngles[0] + JoyAngles[0]
                except:
                    None

                #Bucket
                try:
                    if JoyAngles[3] != 0:
                        ServoAngles[3] = RobotAngles[4] + JoyAngles[3]
                except:
                    None

                #Boom
                try:
                    if JoyAngles[1] != 0:
                        ServoAngles[1] = RobotAngles[2] + JoyAngles[1]
                except:
                    None

                #Swing
                try:
                    if JoyAngles[2] != 0:
                        ServoAngles[2] = RobotAngles[3] + JoyAngles[2]
                except:
                    None

                #Grapple
                try:
                    if JoyAngles[4] != 0:
                        ServoAngles[4] = RobotAngles[1] + JoyAngles[4]
                except:
                    None


        #rospy.loginfo("RobotAngles[0] is %f", RobotAngles[0])
        #rospy.loginfo("JoyAngles[0] is %f", JoyAngles[0])
        #rospy.loginfo("ServoAngles[0] is %f", ServoAngles[0])
        ServoAngle.data = ServoAngles
        pub.publish(ServoAngle)
        rate.sleep()
        #rospy.loginfo(ServoAngles[1])

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    listener()
