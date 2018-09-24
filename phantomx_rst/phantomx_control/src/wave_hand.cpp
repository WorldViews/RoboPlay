#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

float Arr[6];
float Hand_Pos[3];

void X_Callback(const std_msgs::Float64& msg)
{
  Arr[0] = msg.data;
  ROS_INFO("I heard x");
}

void Y_Callback(const std_msgs::Float64& msg)
{
  Arr[1] = msg.data;
  ROS_INFO("I heard x");
}

void Z_Callback(const std_msgs::Float64& msg)
{
  Arr[2] = msg.data;
  ROS_INFO("I heard x");
}

void roll_Callback(const std_msgs::Float64& msg)
{
  Arr[3] = msg.data;
  ROS_INFO("I heard x");
}

void pitch_Callback(const std_msgs::Float64& msg)
{
  Arr[4] = msg.data;
  ROS_INFO("I heard x");
}

void yaw_Callback(const std_msgs::Float64& msg)
{
  Arr[5] = msg.data;
  ROS_INFO("I heard x");
}

void handpos_Callback(const geometry_msgs::Twist& vel)
{
  Hand_Pos[0] = vel.linear.x;
  Hand_Pos[1] = vel.linear.y;
  Hand_Pos[2] = vel.linear.z;
  ROS_INFO("I heard x");
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pickandplacer");
  ros::NodeHandle n;

  ros::NodeHandle x_nh,y_nh,z_nh,roll_nh,pitch_nh,yaw_nh,kinect_nh;

  ros::Subscriber x_sub = x_nh.subscribe("x_robot_arm",1000,X_Callback);
  ros::Subscriber y_sub = y_nh.subscribe("y_robot_arm",1000,Y_Callback);
  ros::Subscriber z_sub = z_nh.subscribe("z_robot_arm",1000,Z_Callback);
  ros::Subscriber roll_sub = roll_nh.subscribe("roll_robot_arm",1000,roll_Callback);
  ros::Subscriber pitch_sub = pitch_nh.subscribe("pitch_robot_arm",1000,pitch_Callback);
  ros::Subscriber yaw_sub = yaw_nh.subscribe("yaw_robot_arm",1000,yaw_Callback);
  ros::Subscriber kinect_sub = kinect_nh.subscribe("/hand_pos",1000,handpos_Callback);


  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroup group("arm");
  moveit::planning_interface::MoveGroup gripper("gripper");

  group.setPoseReferenceFrame("base_link"); // plan w.r.t. base_link frame by default

  ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Endeffector frame: %s", group.getEndEffectorLink().c_str());

  // Set values for your gripper here
  const double gripper_open = 0.7;
  const double gripper_closed = -0.6;

  group.setGoalPositionTolerance(0.04);
  group.setGoalOrientationTolerance(0.1);
  group.allowReplanning(true);

  ROS_INFO("Moving into default configuration");
  group.setJointValueTarget({0,0,0,0,0});
  group.move();

  ros::Duration(3).sleep();

  float old_x, new_x;
  double dis=10;

  ros::Rate loop_rate(33);
  geometry_msgs::Pose robot_pose;
  float servo_val = 0.0;
  while(ros::ok())
  {
    ROS_INFO("loop");
    new_x = Hand_Pos[0];

    if(old_x!=new_x)
    {
      servo_val = -Hand_Pos[0]/130;
      group.setJointValueTarget({0,servo_val,0,0,0});
      group.move();
    }

    old_x=Hand_Pos[0];

    ros::spinOnce();
    loop_rate.sleep();
  }



  return 0;
}
