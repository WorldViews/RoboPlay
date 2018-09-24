#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>//joystick
#include <iostream>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include "sensor_msgs/JointState.h"  //getdata_joint用
#include "gazebo_msgs/LinkStates.h"  //getdata_link用
#include <termios.h>
#include<unistd.h>




int main(int argc, char **argv){
  std_msgs::Float64 gain_data;
  gain_data.data=0;
  int count=0;
  char button;

 ros::init(argc,argv,"publisher_test");
 ros::NodeHandle nh;

 ros::Publisher para_pub =nh.advertise<std_msgs::Float64>("testtest",1000);
 ros::Rate loop_rate(1);

while(ros::ok()){



  gain_data.data=0.0;
  para_pub.publish(gain_data);

  printf("gain=%f\n",gain_data.data);
  printf("%d",count);
  ros::spinOnce();
  loop_rate.sleep();

}

 return 0;

}
