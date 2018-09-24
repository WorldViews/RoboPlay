#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
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
bool callback;

void handpos_Callback(const geometry_msgs::Twist& vel)
{
  Hand_Pos[0] = vel.linear.z;
  Hand_Pos[1] = vel.linear.x;
  Hand_Pos[2] = vel.linear.y;
  ROS_INFO("I heard x");
  callback=true;
}

void hitpos_Callback(const geometry_msgs::Vector3& vel)
{
  Hand_Pos[0] = vel.x;
  Hand_Pos[1] = vel.y;
  Hand_Pos[2] = vel.z;
  //ROS_INFO("I heard hit_pos");
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "pickandplacer");
  ros::NodeHandle n;

  ros::NodeHandle x_nh,y_nh,z_nh,roll_nh,pitch_nh,yaw_nh,kinect_nh,hit_nh;

  //ros::Subscriber x_sub = x_nh.subscribe("x_robot_arm",1000,X_Callback);
  //ros::Subscriber y_sub = y_nh.subscribe("y_robot_arm",1000,Y_Callback);
  //ros::Subscriber z_sub = z_nh.subscribe("z_robot_arm",1000,Z_Callback);
  //ros::Subscriber roll_sub = roll_nh.subscribe("roll_robot_arm",1000,roll_Callback);
  //ros::Subscriber pitch_sub = pitch_nh.subscribe("pitch_robot_arm",1000,pitch_Callback);
  //ros::Subscriber yaw_sub = yaw_nh.subscribe("yaw_robot_arm",1000,yaw_Callback);
  //ros::Subscriber kinect_sub = kinect_nh.subscribe("/hand_pos",1000,handpos_Callback);
  ros::Subscriber hit_sub = hit_nh.subscribe("hit_pos",1000,hitpos_Callback);

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





  ROS_INFO("Open Gripper");
  gripper.setJointValueTarget({gripper_open});
  gripper.move();

  ros::Duration(3).sleep(); // wait a few seconds here

  ROS_INFO("Close Gripper");
  gripper.setJointValueTarget({gripper_closed});
  gripper.move();

  ros::Duration(3).sleep();

  float old_pos[3], new_pos[3];
  double dis;

  ros::Rate loop_rate(33);
  geometry_msgs::Pose robot_pose;

  Hand_Pos[0] = 0.02;
  Hand_Pos[1] = 0.02;
  Hand_Pos[2] = -0.15;
  old_pos[0]=Hand_Pos[0];
  old_pos[1]=Hand_Pos[1];
  old_pos[2]=Hand_Pos[2];

  while(ros::ok())
  {
    //callback = false;
    //ROS_INFO("loop");

    for(int i=0; i<3; i++)
    {
      new_pos[i] = Hand_Pos[i];
    }

    dis=sqrt((old_pos[0]-new_pos[0])*(old_pos[0]-new_pos[0])+(old_pos[1]-new_pos[1])*(old_pos[1]-new_pos[1])+(old_pos[2]-new_pos[2])*(old_pos[2]-new_pos[2]));

    //if(dis>0.05)
    if(dis>0.05)
    {

      group.setJointValueTarget({0,0,0,0,0});
      group.move();
      ros::Duration(2).sleep();

      //robot_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-3.14,1.30,-3.12);
      robot_pose.position.x = -0.02+Hand_Pos[0];
      robot_pose.position.y = 0.08+Hand_Pos[2];
      robot_pose.position.z = -0.07;



      //robot_pose.position.x = 0.20;
      //robot_pose.position.y = -0.11;
      //robot_pose.position.z = -0.09;

      ROS_INFO("%f",robot_pose.position.x);
      ROS_INFO("%f",robot_pose.position.y);
      ROS_INFO("%f",robot_pose.position.z);

      group.setPositionTarget(robot_pose.position.x,robot_pose.position.y ,robot_pose.position.z);

      moveit::planning_interface::MoveGroup::Plan pour_plan;
      bool success = group.plan(pour_plan);
      group.move();

      ros::Duration(2).sleep();

      for(int i=0; i<3; i++)
      {
        old_pos[i] = Hand_Pos[i];
      }
    }




    //ROS_INFO("%f",Hand_Pos[2]);
    ros::spinOnce();

    loop_rate.sleep();
  }



  return 0;
}
