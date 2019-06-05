/*******************************************************************************
# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
*******************************************************************************/
#ifndef LOCOBOT_GAZEBO_H
#define LOCOBOT_GAZEBO_H

#define GRIPPER_OPEN_POS 0.03
#define GRIPPER_CLOSE_POS 0.0

#include <ros/ros.h>
//#include "message_header.h"

#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <trajectory_msgs/JointTrajectory.h>


// This is for SimpleActionClient
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <boost/thread/recursive_mutex.hpp>
#include <locobot_control/JointCommand.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#define NUMBER_OF_ARM_JOINTS 5


class GazeboInteface {

private:

	ros::NodeHandle node_handle_;

	ros::Publisher joint_1_pub;
	ros::Publisher joint_2_pub;
	ros::Publisher joint_3_pub;
	ros::Publisher joint_4_pub;
	ros::Publisher joint_5_pub;
	ros::Publisher joint_6_pub;
	ros::Publisher joint_7_pub;
	ros::Publisher head_pan_pub;
	ros::Publisher head_tilt_pub;
	ros::Publisher pub_arr[9];
	ros::Publisher gripper_state_pub;

	ros::Subscriber joint_command_sub_;
	ros::Subscriber gripper_open_sub_;
	ros::Subscriber gripper_close_sub_;
	ros::Subscriber stop_joints_sub_;
	ros::Subscriber joint_state_sub_;

	ros::ServiceServer joint_command_server_;
	ros::ServiceServer joint_torque_server_;
	ros::ServiceClient torque_command_client_;

	ros::Timer gripper_timer;

	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *smooth_joint_trajectory_server_; 

	bool torque_control_;
	double arm_state[7];
	int gripper_state;


public:

	GazeboInteface();

	bool jointCommandMsgCallback(locobot_control::JointCommand::Request &req,
                                              locobot_control::JointCommand::Response &res);
	bool jointTorqueCommandMsgCallback(locobot_control::JointCommand::Request &req,
                                              locobot_control::JointCommand::Response &res);
	void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
	void recordArmJoints(const sensor_msgs::JointState::ConstPtr &msg);
	void stopExecution(const std_msgs::Empty &msg);
	void gripperOpen(const std_msgs::Empty &mg);
	void gripperClose(const std_msgs::Empty &mg);
	void executeJointTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
	void SetJointTorque(const int id, const double value);
	void gripperStateCallback(const ros::TimerEvent &);
};

#endif
