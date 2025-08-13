/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
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
 */

/** \author Alessandro Di Fava. */

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include <std_msgs/Empty.h>



// Our Action interface type for moving TIAGo's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


// Create a ROS action client to move TIAGo's arm
void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}


// Generates a simple trajectory with two waypoints to move TIAGo's arm 
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(2);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.2;
  goal.trajectory.points[index].positions[1] = 0.0;
  goal.trajectory.points[index].positions[2] = -1.5;
  goal.trajectory.points[index].positions[3] = 1.94;
  goal.trajectory.points[index].positions[4] = -1.57;
  goal.trajectory.points[index].positions[5] = -0.5;
  goal.trajectory.points[index].positions[6] = 0.0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

  // Second trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = -1;
  goal.trajectory.points[index].positions[1] = -1;
  goal.trajectory.points[index].positions[2] = -1.26;
  goal.trajectory.points[index].positions[3] = -0.01;
  goal.trajectory.points[index].positions[4] = 1.57;
  goal.trajectory.points[index].positions[5] = 0.04;
  goal.trajectory.points[index].positions[6] = 0.0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "run_traj_control");
  ros::NodeHandle nh;

  ROS_INFO("Starting run_traj_control node...");

  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

  //  arm control action client
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
  boost::shared_ptr<arm_control_client> ArmClient;
  ArmClient.reset(new arm_control_client("/arm_controller/follow_joint_trajectory"));

  int retries = 0;
  while (!ArmClient->waitForServer(ros::Duration(2.0)) && ros::ok() && retries < 5) {
    ROS_WARN("Waiting for arm controller action server...");
    retries++;
  }

  if (retries == 5) {
    ROS_ERROR("Failed to connect to arm controller action server.");
    return EXIT_FAILURE;
  }

  // callback
  ros::Subscriber sub = nh.subscribe<std_msgs::Empty>("/arm_trigger", 1,
    [&](const std_msgs::Empty::ConstPtr&) {
      control_msgs::FollowJointTrajectoryGoal goal;

      goal.trajectory.joint_names = {
        "arm_1_joint", "arm_2_joint", "arm_3_joint",
        "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"
      };

      goal.trajectory.points.resize(2);
      // first frame
      goal.trajectory.points[0].positions = {0.2, 0.0, -1.5, 1.94, -1.57, -0.5, 0.0};
      goal.trajectory.points[0].velocities = std::vector<double>(7, 1.0);
      goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

      // second frame
      goal.trajectory.points[1].positions = {2.5, 0.2, -2.1, 1.9, 1.0, -0.5, 0.0};
      goal.trajectory.points[1].velocities = std::vector<double>(7, 0.0);
      goal.trajectory.points[1].time_from_start = ros::Duration(4.0);

      goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
      ArmClient->sendGoal(goal);

      ROS_INFO("Sent arm trajectory goal, waiting for result...");
      ArmClient->waitForResult();

      if (ArmClient->getState().isDone()) {
        ROS_INFO("Arm motion finished.");
      } else {
        ROS_WARN("Arm motion did not complete normally.");
      }
    });

  ros::spin();
  return EXIT_SUCCESS;
}


