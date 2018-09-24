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

#include <phantomx_lib/phantomx_interface.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "movit_control");
  ros::NodeHandle n("~");

  phantomx::PhantomXControl robot;

  robot.initialize();

  // Start a ROS spinning thread (required for processing callbacks while moveit is blocking)
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("arm");
  moveit::planning_interface::MoveGroup gripper("gripper");

  group.setPoseReferenceFrame("base_link"); // plan w.r.t. base_link frame by default
//   group.setEndEffectorLink();

  ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Endeffector frame: %s", group.getEndEffectorLink().c_str());

  // Set values for your gripper here
  const double gripper_open = 0.7;
  const double gripper_closed = -0.20;

  group.setGoalPositionTolerance(0.04);
  group.setGoalOrientationTolerance(0.1);
  group.allowReplanning(true);

  ROS_INFO("Moving into default configuration");
  group.setJointValueTarget({0,0,0,0,0});
  group.move(); // we do not test before, just move if possible ;-)
                // move is a blocking call


  // test gripper
  ROS_INFO("Open Gripper");
  gripper.setJointValueTarget({gripper_open});
  gripper.move();

  ros::Duration(2).sleep(); // wait a few seconds here

  for(int i=0;i<5;i++)
  {
    ROS_INFO("Move to a designated place");
    group.setJointValueTarget({0.02,1.12,0.93,0.84,gripper_open});
    group.move();

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Close Gripper");
    gripper.setJointValueTarget({gripper_closed});
    gripper.move();

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Moving into default configuration");
    group.setJointValueTarget({0,0,0,0,0});
    group.move(); // we do not test before, just move if possible ;-)

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Move to another designated place");
    group.setJointValueTarget({-0.72,-1.19,-0.61,-1.24,gripper_closed});
    group.move();


    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Open Gripper");
    gripper.setJointValueTarget({gripper_open});
    gripper.move();

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Moving into default configuration");
    group.setJointValueTarget({0,0,0,0,0});
    group.move(); // we do not test before, just move if possible ;-)

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Move to another designated place");
    group.setJointValueTarget({-0.72,-1.19,-0.61,-1.24,gripper_closed});
    group.move();

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Close Gripper");
    gripper.setJointValueTarget({gripper_closed});
    gripper.move();

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Moving into default configuration");
    group.setJointValueTarget({0,0,0,0,0});
    group.move(); // we do not test before, just move if possible ;-)

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Move to a designated place");
    group.setJointValueTarget({0.02,1.12,0.93,0.84,gripper_open});
    group.move();

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Open Gripper");
    gripper.setJointValueTarget({gripper_open});
    gripper.move();

    ros::Duration(2).sleep(); // wait a few seconds here

    ROS_INFO("Moving into default configuration");
    group.setJointValueTarget({0,0,0,0,0});
    group.move(); // we do not test before, just move if possible ;-)

    ros::Duration(2).sleep(); // wait a few seconds here


    i++;
  }



  ROS_INFO("Demo completed. Waiting for user shutdown (ctrl-c).");
  ros::waitForShutdown();

  return 0;
}
