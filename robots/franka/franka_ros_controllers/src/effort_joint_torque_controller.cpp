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
#include <franka_ros_controllers/effort_joint_torque_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_ros_controllers {

bool EffortJointTorqueController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("/robot_config/arm_id", arm_id)) {
    ROS_ERROR("EffortJointTorqueController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("/robot_config/joint_names", joint_limits_.joint_names) || joint_limits_.joint_names.size() != 7) {
    ROS_ERROR(
        "EffortJointTorqueController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  bool enable_coriolis;
  if (!node_handle.getParam("/franka_ros_interface/effort_joint_torque_controller/compensate_coriolis", enable_coriolis)) {
    ROS_ERROR("EffortJointTorqueController: Could not read parameter compensate_coriolis");
    return false;
  }

  if (enable_coriolis==false){
    coriolis_factor_ = 0.0;
    ROS_INFO_STREAM("EffortJointTorqueController: Coriolis compensation disabled!");
  }
  else{
    ROS_INFO_STREAM("EffortJointTorqueController: Coriolis compensation enabled!");
  }

  std::map<std::string, double> torque_limit_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_effort_limit", torque_limit_map))
  {
    ROS_ERROR("EffortJointTorqueController: Failed to find joint effort limits on the param server. Aborting controller init");
    return false;
  }

  for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
    if (torque_limit_map.find(joint_limits_.joint_names[i]) != torque_limit_map.end())
      {
        joint_limits_.effort.push_back(torque_limit_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("EffortJointTorqueController: Unable to find torque limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
  }  

  double controller_state_publish_rate(30.0);
  if (!node_handle.getParam("controller_state_publish_rate", controller_state_publish_rate)) {
    ROS_INFO_STREAM("EffortJointTorqueController: Did not find controller_state_publish_rate. Using default "
                    << controller_state_publish_rate << " [Hz].");
  }
  trigger_publish_ = franka_hw::TriggerRate(controller_state_publish_rate);

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "EffortJointTorqueController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "EffortJointTorqueController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "EffortJointTorqueController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_limits_.joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "EffortJointTorqueController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  desired_joints_subscriber_ = node_handle.subscribe(
      "/franka_ros_interface/motion_controller/arm/joint_commands", 20, &EffortJointTorqueController::jointCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  publisher_controller_states_.init(node_handle, "/franka_ros_interface/motion_controller/arm/joint_controller_states", 1);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> > lock(
        publisher_controller_states_);
    publisher_controller_states_.msg_.controller_name = "effort_joint_torque_controller";
    publisher_controller_states_.msg_.names.resize(joint_limits_.joint_names.size());
    publisher_controller_states_.msg_.joint_controller_states.resize(joint_limits_.joint_names.size());

  }

  return true;
}

void EffortJointTorqueController::starting(const ros::Time& /*time*/) {

  std::fill(jnt_cmd_.begin(), jnt_cmd_.end(), 0);
  prev_jnt_cmd_ = jnt_cmd_;
  ROS_WARN_STREAM("EffortJointTorqueController: Using raw torque controller! Be extremely careful and send smooth commands.");
}

void EffortJointTorqueController::update(const ros::Time& time,
                                             const ros::Duration& period) {
  
  std::array<double, 7> coriolis = model_handle_->getCoriolis();

  std::array<double, 7> compensated_cmd{};
  for (size_t i = 0; i < 7; ++i) {
    compensated_cmd[i] = coriolis_factor_ * coriolis[i] + jnt_cmd_[i];
  }
  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(compensated_cmd, prev_jnt_cmd_);

  if (trigger_publish_() && publisher_controller_states_.trylock()) {
      for (size_t i = 0; i < 7; ++i){

        publisher_controller_states_.msg_.joint_controller_states[i].set_point = jnt_cmd_[i];
        publisher_controller_states_.msg_.joint_controller_states[i].process_value = compensated_cmd[i];
        publisher_controller_states_.msg_.joint_controller_states[i].time_step = period.toSec();
        publisher_controller_states_.msg_.joint_controller_states[i].command = tau_d_saturated[i];

        publisher_controller_states_.msg_.joint_controller_states[i].header.stamp = time;

      }

      publisher_controller_states_.unlockAndPublish();

    }

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);

    prev_jnt_cmd_[i] = tau_d_saturated[i];

  }

}

bool EffortJointTorqueController::checkTorqueLimits(std::vector<double> torques)
{
  for (size_t i = 0;  i < 7; ++i){
    if (!(abs(torques[i]) < joint_limits_.effort[i])){
      return true;
    }
  }

  return false;
}

std::array<double, 7> EffortJointTorqueController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated, const std::array<double, 7>& prev_tau) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - prev_tau[i];
    tau_d_saturated[i] = prev_tau[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

void EffortJointTorqueController::jointCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg) {

  if (msg->mode == franka_core_msgs::JointCommand::TORQUE_MODE){
    if (msg->effort.size() != 7) {
      ROS_ERROR_STREAM(
          "EffortJointTorqueController: Published Commands are not of size 7");
      std::fill(jnt_cmd_.begin(), jnt_cmd_.end(), 0);
      jnt_cmd_= prev_jnt_cmd_;
    }
    else if (checkTorqueLimits(msg->effort)) {
         ROS_ERROR_STREAM(
            "EffortJointTorqueController: Commanded torques are beyond allowed torque limits.");
        std::fill(jnt_cmd_.begin(), jnt_cmd_.end(), 0);
        jnt_cmd_= prev_jnt_cmd_;
    }
    else {
      std::copy_n(msg->effort.begin(), 7, jnt_cmd_.begin());

    }
  }
  // else ROS_ERROR_STREAM("EffortJointTorqueController: Published Command msg are not of JointCommand::TORQUE_MODE! Dropping message");
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::EffortJointTorqueController,
                       controller_interface::ControllerBase)
