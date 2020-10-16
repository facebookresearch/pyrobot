/***************************************************************************

*
* @package: franka_ros_controllers
* @metapackage: franka_ros_interface
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019-2020, Saif Sidhik.
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
**************************************************************************/
#include <franka_ros_controllers/position_joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace franka_ros_controllers {

bool PositionJointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  desired_joints_subscriber_ = node_handle.subscribe(
      "/franka_ros_interface/motion_controller/arm/joint_commands", 20, &PositionJointPositionController::jointPosCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "PositionJointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  if (!node_handle.getParam("/robot_config/joint_names", joint_limits_.joint_names)) {
    ROS_ERROR("PositionJointPositionController: Could not parse joint names");
  }
  if (joint_limits_.joint_names.size() != 7) {
    ROS_ERROR_STREAM("PositionJointPositionController: Wrong number of joint names, got "
                     << joint_limits_.joint_names.size() << " instead of 7 names!");
    return false;
  }
  std::map<std::string, double> pos_limit_lower_map;
  std::map<std::string, double> pos_limit_upper_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_position_limit/lower", pos_limit_lower_map) ) {
  ROS_ERROR(
      "PositionJointPositionController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }
  if (!node_handle.getParam("/robot_config/joint_config/joint_position_limit/upper", pos_limit_upper_map) ) {
  ROS_ERROR(
      "PositionJointPositionController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }

  for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
    if (pos_limit_lower_map.find(joint_limits_.joint_names[i]) != pos_limit_lower_map.end())
      {
        joint_limits_.position_lower.push_back(pos_limit_lower_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("PositionJointPositionController: Unable to find lower position limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
    if (pos_limit_upper_map.find(joint_limits_.joint_names[i]) != pos_limit_upper_map.end())
      {
        joint_limits_.position_upper.push_back(pos_limit_upper_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("PositionJointPositionController: Unable to find upper position limit  values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
  }  

  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_limits_.joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "PositionJointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  double controller_state_publish_rate(30.0);
  if (!node_handle.getParam("controller_state_publish_rate", controller_state_publish_rate)) {
    ROS_INFO_STREAM("PositionJointPositionController: Did not find controller_state_publish_rate. Using default "
                    << controller_state_publish_rate << " [Hz].");
  }
  trigger_publish_ = franka_hw::TriggerRate(controller_state_publish_rate);

  dynamic_reconfigure_joint_controller_params_node_ =
      ros::NodeHandle("/franka_ros_interface/position_joint_position_controller/arm/controller_parameters_config");

  dynamic_server_joint_controller_params_ = std::make_unique<
      dynamic_reconfigure::Server<franka_ros_controllers::joint_controller_paramsConfig>>(
      dynamic_reconfigure_joint_controller_params_node_);

  dynamic_server_joint_controller_params_->setCallback(
      boost::bind(&PositionJointPositionController::jointControllerParamCallback, this, _1, _2));

  publisher_controller_states_.init(node_handle, "/franka_ros_interface/motion_controller/arm/joint_controller_states", 1);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> > lock(
        publisher_controller_states_);
    publisher_controller_states_.msg_.controller_name = "position_joint_position_controller";
    publisher_controller_states_.msg_.names.resize(joint_limits_.joint_names.size());
    publisher_controller_states_.msg_.joint_controller_states.resize(joint_limits_.joint_names.size());

  }

  return true;
}

void PositionJointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pos_[i] = position_joint_handles_[i].getPosition();
  }
  pos_d_ = initial_pos_;
  prev_pos_ = initial_pos_;
  pos_d_target_ = initial_pos_;
}

void PositionJointPositionController::update(const ros::Time& time,
                                            const ros::Duration& period) {
  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i].setCommand(pos_d_[i]);
  }
  double filter_val = filter_joint_pos_ * filter_factor_;
  for (size_t i = 0; i < 7; ++i) {
    prev_pos_[i] = position_joint_handles_[i].getPosition();
    pos_d_[i] = filter_val * pos_d_target_[i] + (1.0 - filter_val) * pos_d_[i];
  }

  if (trigger_publish_() && publisher_controller_states_.trylock()) {
    for (size_t i = 0; i < 7; ++i){

      publisher_controller_states_.msg_.joint_controller_states[i].set_point = pos_d_target_[i];
      publisher_controller_states_.msg_.joint_controller_states[i].process_value = pos_d_[i];
      publisher_controller_states_.msg_.joint_controller_states[i].time_step = period.toSec();

      publisher_controller_states_.msg_.joint_controller_states[i].header.stamp = time;

    }

    publisher_controller_states_.unlockAndPublish();        
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  filter_joint_pos_ = param_change_filter_ * target_filter_joint_pos_ + (1.0 - param_change_filter_) * filter_joint_pos_;

}

bool PositionJointPositionController::checkPositionLimits(std::vector<double> positions)
{
  // bool retval = true;
  for (size_t i = 0;  i < 7; ++i){
    if (!((positions[i] <= joint_limits_.position_upper[i]) && (positions[i] >= joint_limits_.position_lower[i]))){
      return true;
    }
  }

  return false;
}

void PositionJointPositionController::jointPosCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg) {

    if (msg->mode == franka_core_msgs::JointCommand::POSITION_MODE){
      if (msg->position.size() != 7) {
        ROS_ERROR_STREAM(
            "PositionJointPositionController: Published Commands are not of size 7");
        pos_d_ = prev_pos_;
        pos_d_target_ = prev_pos_;
      }
      else if (checkPositionLimits(msg->position)) {
         ROS_ERROR_STREAM(
            "PositionJointPositionController: Commanded positions are beyond allowed position limits.");
        pos_d_ = prev_pos_;
        pos_d_target_ = prev_pos_;

      }
      else
      {
        std::copy_n(msg->position.begin(), 7, pos_d_target_.begin());
      }
      
    }
    // else ROS_ERROR_STREAM("PositionJointPositionController: Published Command msg are not of JointCommand::POSITION_MODE! Dropping message");
}

void PositionJointPositionController::jointControllerParamCallback(franka_ros_controllers::joint_controller_paramsConfig& config,
                               uint32_t level){
  target_filter_joint_pos_ = config.position_joint_delta_filter;
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::PositionJointPositionController,
                       controller_interface::ControllerBase)
