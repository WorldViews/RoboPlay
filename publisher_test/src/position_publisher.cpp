#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Joy.h>//joystick
#include <iostream>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>



int main(int argc, char **argv){
  std_msgs::Float64 x_data,y_data,z_data,roll_data,pitch_data,yaw_data;


 ros::init(argc,argv,"position_publisher");
 ros::NodeHandle x_nh,y_nh,z_nh,roll_nh,pitch_nh,yaw_nh;

 ros::Publisher x_pub =x_nh.advertise<std_msgs::Float64>("x_robot_arm",1000);
 ros::Publisher y_pub =y_nh.advertise<std_msgs::Float64>("y_robot_arm",1000);
 ros::Publisher z_pub =z_nh.advertise<std_msgs::Float64>("z_robot_arm",1000);
 ros::Publisher roll_pub =roll_nh.advertise<std_msgs::Float64>("roll_robot_arm",1000);
 ros::Publisher pitch_pub =pitch_nh.advertise<std_msgs::Float64>("pitch_robot_arm",1000);
 ros::Publisher yaw_pub =yaw_nh.advertise<std_msgs::Float64>("yaw_robot_arm",1000);
 ros::Rate loop_rate(1);

 x_data.data = 0.13;
 y_data.data = 0.0;
 z_data.data = 0.05;
 roll_data.data = -3.14;
 pitch_data.data = 1.30;
 yaw_data.data = -3.12;



while(ros::ok()){
  x_pub.publish(x_data);
  y_pub.publish(y_data);
  z_pub.publish(z_data);
  roll_pub.publish(roll_data);
  pitch_pub.publish(pitch_data);
  yaw_pub.publish(yaw_data);


  ros::spinOnce();
  loop_rate.sleep();

}

 return 0;

}
