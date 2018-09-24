#include <ros/ros.h>
#include "tf/transform_datatypes.h"

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

int main( int argc, char** argv )
{
  ros::init(argc, argv, "movit_control");
  ros::NodeHandle n("~");

  // Start a ROS spinning thread (required for processing callbacks while moveit is blocking)
  ros::AsyncSpinner spinner(1);
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



/*
  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.00,-1.57,0.00);
  start_pose2.position.x = 0.10;
  start_pose2.position.y = 0.10;
  start_pose2.position.z = 0.00;
  const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);
*/
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.00,-1.57,0.00);
  target_pose1.position.x = 0.10;
  target_pose1.position.y = 0.10;
  target_pose1.position.z = 0.00;
  group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  group.move();

  ros::Duration(2).sleep();

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  //sleep(2.0);

  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose3 = target_pose1;
  target_pose3.position.x = 0.10;
  target_pose3.position.y = 0.10;
  target_pose3.position.z = 0.05;

  waypoints.push_back(target_pose3);

  target_pose3.position.x = 0.10;
  target_pose3.position.y = -0.10;
  target_pose3.position.z = 0.05;

  waypoints.push_back(target_pose3);


  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  ROS_INFO("a");
  geometry_msgs::Pose target_pose2;
  target_pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.00,-1.57,0.00);
  target_pose2.position.x = 0.10;
  target_pose2.position.y = -0.10;
  target_pose2.position.z = 0.00;
  group.setPoseTarget(target_pose2);

  ROS_INFO("b");
  moveit::planning_interface::MoveGroup::Plan my_plan2;
  success = group.plan(my_plan2);

  ROS_INFO("c");

  ROS_INFO("Visualizing plan 4 %.2f%% acheived", fraction*100);

  group.move();

  ROS_INFO("Demo completed. Waiting for user shutdown (ctrl-c).");
  ros::waitForShutdown();

  return 0;
}
