/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/


#include <phantomx_lib/phantomx_interface.h>



// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "measure_states");
  ros::NodeHandle n("~");


  phantomx::PhantomXControl robot;

  robot.initialize();

  ROS_INFO("Reference frame: %s", "/arm_base_link"); // this is hardcoded atm, todo add as parameter in robot
  ROS_INFO("Endeffector frame: %s", "/gripper_link"); // this is hardcoded atm, todo add as parameter in robot

  //robot.relaxServos();

  ROS_INFO_STREAM("Servos relaxed. Now move the endeffector around... (cancel with crtl-c)");

  ros::Rate r(1);
  while(ros::ok())
  {
//robot.relaxServos();
      ROS_INFO("------------------------------------------------------------------------------------------");
      phantomx::JointVector q;
      robot.getJointAngles(q);
      ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "q: [" << q.transpose() << "]\tgripper: " << robot.getGripperJointAngle());

      Eigen::Affine3d ee_state;
      robot.getEndeffectorState(ee_state);
      phantomx::RpyVector rpy = phantomx::convertRotMatToRpy(ee_state.linear());
      ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "ee pos: [" << ee_state.translation().transpose() << "]\trpy: [" << rpy.transpose() << "]");


      r.sleep();
  }


  return 0;
}
