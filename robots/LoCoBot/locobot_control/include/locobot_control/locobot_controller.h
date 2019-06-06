/*******************************************************************************
* Copyright (c) Facebook, Inc. and its affiliates.
* This source code is licensed under the MIT license found in the
* LICENSE file in the root directory of this source tree.

* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * Modified from dynamixel_workbench_controllers/dynamixel_workbench_controllers.cpp
 * Authors: Adithya Murali, Tao Chen, Dhiraj Gandhi
 */
#ifndef LOCOBOT_CONTROLLERS_H
#define LOCOBOT_CONTROLLERS_H

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

// This is for SimpleActionClient
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <time.h>
#include <mutex>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <locobot_control/JointCommand.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0
#define SYNC_READ_HANDLER_FOR_TORQUE_ENABLE 1
#define SYNC_READ_HANDLER_FOR_SHUTDOWN 2
// #define DEBUG

#define GRIPPER_OPEN_MOTOR_POS -1.147412
#define GRIPPER_CLOSE_MOTOR_POS 0.84368409
#define GRIPPER_OPEN_MOVEIT -0.025
#define GRIPPER_CLOSE_MOVEIT -0.002
#define GRIPPER_OPEN_VALUE 2800
#define GRIPPER_CLOSE_VALUE 1800
#define GRIPPER_MAX_LOAD 500
#define GRIPPER_PWM 500

#define HARDWARE_ERROR_STATUS_OVERLOAD 5
#define HARDWARE_ERROR_STATUS_ELECTRICAL_SHOCK 4
#define HARDWARE_ERROR_STATUS_ENCODER 3
#define HARDWARE_ERROR_STATUS_OVERHEATING 2
#define HARDWARE_ERROR_STATUS_INPUT_VOLTAGE 0

typedef struct {
  std::string item_name;
  int32_t value;
} ItemValue;

class LoCoBotController {
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher joint_states_pub_;
  ros::Publisher tilt_state_pub_;
  ros::Publisher pan_state_pub_;
  ros::Publisher status_pub_;
  ros::Publisher gripper_state_pub_;

  // ROS Topic Subscriber
  ros::Subscriber joint_command_sub_;
  ros::Subscriber traj_command_sub_;
  ros::Subscriber stop_execution_sub_;
  ros::Subscriber stop_execution_j4_sub_;
  ros::Subscriber stop_execution_j5_sub_;
  ros::Subscriber gripper_open_sub_;
  ros::Subscriber gripper_close_sub_;
  ros::Subscriber set_tilt_sub_;
  ros::Subscriber set_pan_sub_;

  // ROS Service Server
  ros::ServiceServer dynamixel_command_server_;
  ros::ServiceServer joint_command_server_;

  // ROS Service Client

  // ROS Actionlib
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
      *smooth_joint_trajectory_server_; //!< smooth point-to-point trajectory follower

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;

  // For MoveIt
  std::mutex dynamixel_mutex_;
  std::mutex hardware_mutex_;

  // Misc Dynamixel parameters
  bool torque_control_;

  std::map<std::string, std::vector<std::string>> group_motors_;
  std::map<std::string, bool> use_group_;
  std::map<std::string, uint32_t> dynamixel_name_2ids_;
  std::vector<std::pair < std::string, ItemValue>> dynamixel_info_;
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list_;
  std::map<int, int> motor_id_2state_list_id_;
  std::map<int, int> state_list_id_2motor_id_;
  std::map<std::string, const ControlItem *> control_items_;

  sensor_msgs::JointState joint_state_msg_;
  std_msgs::Int8 gripper_state_msg_;

  int num_motors_;

  bool is_joint_state_topic_;
  bool is_cmd_vel_topic_;
  bool use_moveit_;
  bool is_hardware_ok_;
  bool is_moving_;
  bool is_initialized_;

  double wheel_separation_;
  double wheel_radius_;

  trajectory_msgs::JointTrajectory *jnt_tra_msg_;

  double read_period_;
  double write_period_;
  double pub_period_;

  int gripper_state_;
  int gripper_cmd_;
  int prev_gripper_state_;
  int prev_gripper_load_;

  void stopExecutionCallback(const std_msgs::Empty::ConstPtr &msg);
  void stopExecutionJ4Callback(const std_msgs::Empty::ConstPtr &msg);
  void stopExecutionJ5Callback(const std_msgs::Empty::ConstPtr &msg);
  void gripperCloseCallback(const std_msgs::Empty::ConstPtr &msg);
  void gripperOpenCallback(const std_msgs::Empty::ConstPtr &msg);
  void setTiltCallback(const std_msgs::Float64::ConstPtr &msg);
  void setPanCallback(const std_msgs::Float64::ConstPtr &msg);
  void cameraStatePublish(void);
  bool jointCommandMsgCallback(locobot_control::JointCommand::Request &req,
                               locobot_control::JointCommand::Response &res);
  int getBit(int n, int k);

 public:
  LoCoBotController();
  ~LoCoBotController();

  /**
   * \brief Callback for the joint_velocity_controller
   *
   * @param goal action goal
   */
  void execute_joint_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
  bool hardwareOK();
  bool hardwareStatusPrint();
  bool controlLoop(void);
  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool stopDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool getPresentPosition(std::vector<std::string> dxl_name);

  double getReadPeriod() { return read_period_; }
  double getWritePeriod() { return write_period_; }
  double getPublishPeriod() { return pub_period_; }

  void initActionlib(void);
  void initPublisher(void);
  void initSubscriber(void);

  void initServer();

  void readCallback(const ros::TimerEvent &);
  void publishCallback(const ros::TimerEvent &);
  void gripperController(const ros::TimerEvent &);
  void hardwareStatusPublish(const ros::TimerEvent &);

  void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
  bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                   dynamixel_workbench_msgs::DynamixelCommand::Response &res);
};

#endif //LOCOBOT_CONTROLLERS_H
