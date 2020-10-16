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
#include <franka_ros_controllers/velocity_joint_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>


namespace franka_ros_controllers {

bool VelocityJointVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  desired_joints_subscriber_ = node_handle.subscribe(
      "/franka_ros_interface/motion_controller/arm/joint_commands", 20, &VelocityJointVelocityController::jointVelCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "VelocityJointVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }
  if (!node_handle.getParam("/robot_config/joint_names", joint_limits_.joint_names)) {
    ROS_ERROR("VelocityJointVelocityController: Could not parse joint names");
  }
  if (joint_limits_.joint_names.size() != 7) {
    ROS_ERROR_STREAM("VelocityJointVelocityController: Wrong number of joint names, got "
                     << joint_limits_.joint_names.size() << " instead of 7 names!");
    return false;
  }
  std::map<std::string, double> vel_limit_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_velocity_limit", vel_limit_map) ) {
  ROS_ERROR(
      "VelocityJointVelocityController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }
  

  for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
    if (vel_limit_map.find(joint_limits_.joint_names[i]) != vel_limit_map.end())
      {
        joint_limits_.velocity.push_back(vel_limit_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("VelocityJointVelocityController: Unable to find lower velocity limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
  }  

  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_limits_.joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "VelocityJointVelocityController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  double controller_state_publish_rate(30.0);
  if (!node_handle.getParam("controller_state_publish_rate", controller_state_publish_rate)) {
    ROS_INFO_STREAM("VelocityJointVelocityController: Did not find controller_state_publish_rate. Using default "
                    << controller_state_publish_rate << " [Hz].");
  }
  trigger_publish_ = franka_hw::TriggerRate(controller_state_publish_rate);

  dynamic_reconfigure_joint_controller_params_node_ =
      ros::NodeHandle("/franka_ros_interface/velocity_joint_velocity_controller/arm/controller_parameters_config");

  dynamic_server_joint_controller_params_ = std::make_unique<
      dynamic_reconfigure::Server<franka_ros_controllers::joint_controller_paramsConfig>>(
      dynamic_reconfigure_joint_controller_params_node_);

  dynamic_server_joint_controller_params_->setCallback(
      boost::bind(&VelocityJointVelocityController::jointControllerParamCallback, this, _1, _2));

  publisher_controller_states_.init(node_handle, "/franka_ros_interface/motion_controller/arm/joint_controller_states", 1);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> > lock(
        publisher_controller_states_);
    publisher_controller_states_.msg_.controller_name = "velocity_joint_velocity_controller";
    publisher_controller_states_.msg_.names.resize(joint_limits_.joint_names.size());
    publisher_controller_states_.msg_.joint_controller_states.resize(joint_limits_.joint_names.size());

  }

  return true;
}

void VelocityJointVelocityController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_vel_[i] = velocity_joint_handles_[i].getVelocity();
  }
  vel_d_ = initial_vel_;
  prev_d_ = vel_d_;
}

void VelocityJointVelocityController::update(const ros::Time& time,
                                            const ros::Duration& period) {
  for (size_t i = 0; i < 7; ++i) {
    velocity_joint_handles_[i].setCommand(vel_d_[i]);
  }
  double filter_val = filter_joint_vel_ * filter_factor_;
  for (size_t i = 0; i < 7; ++i) {
    prev_d_[i] = velocity_joint_handles_[i].getVelocity();
    vel_d_[i] = filter_val * vel_d_target_[i] + (1.0 - filter_val) * vel_d_[i];
  }

  if (trigger_publish_() && publisher_controller_states_.trylock()) {
    for (size_t i = 0; i < 7; ++i){

      publisher_controller_states_.msg_.joint_controller_states[i].set_point = vel_d_target_[i];
      publisher_controller_states_.msg_.joint_controller_states[i].process_value = vel_d_[i];
      publisher_controller_states_.msg_.joint_controller_states[i].time_step = period.toSec();

      publisher_controller_states_.msg_.joint_controller_states[i].header.stamp = time;

    }

    publisher_controller_states_.unlockAndPublish();        
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  filter_joint_vel_ = param_change_filter_ * target_filter_joint_vel_ + (1.0 - param_change_filter_) * filter_joint_vel_;

}

bool VelocityJointVelocityController::checkVelocityLimits(std::vector<double> velocities)
{
  // bool retval = true;
  for (size_t i = 0;  i < 7; ++i){
    if (!(abs(velocities[i]) <= joint_limits_.velocity[i])){
      return true;
    }
  }

  return false;
}

void VelocityJointVelocityController::jointVelCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg) {

    if (msg->mode == franka_core_msgs::JointCommand::VELOCITY_MODE){
      if (msg->velocity.size() != 7) {
        ROS_ERROR_STREAM(
            "VelocityJointVelocityController: Published Commands are not of size 7");
        vel_d_ = prev_d_;
        vel_d_target_ = prev_d_;
      }
      else if (checkVelocityLimits(msg->velocity)) {
         ROS_ERROR_STREAM(
            "VelocityJointVelocityController: Commanded velocities are beyond allowed velocity limits.");
        vel_d_ = prev_d_;
        vel_d_target_ = prev_d_;

      }
      else
      {
        std::copy_n(msg->velocity.begin(), 7, vel_d_target_.begin());
      }
      
    }
    // else ROS_ERROR_STREAM("VelocityJointVelocityController: Published Command msg are not of JointCommand::Velocity! Dropping message");
}

void VelocityJointVelocityController::jointControllerParamCallback(franka_ros_controllers::joint_controller_paramsConfig& config,
                               uint32_t level){
  target_filter_joint_vel_ = config.velocity_joint_delta_filter;
}

void VelocityJointVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::VelocityJointVelocityController,
                       controller_interface::ControllerBase)
