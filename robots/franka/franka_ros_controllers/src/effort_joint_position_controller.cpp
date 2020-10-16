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
#include <franka_ros_controllers/effort_joint_position_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_ros_controllers {

bool EffortJointPositionController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("/robot_config/arm_id", arm_id)) {
    ROS_ERROR("EffortJointPositionController: Could not read parameter arm_id");
    return false;
  }

  franka_state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface_ == nullptr) {
    ROS_ERROR("EffortJointPositionController: Could not get Franka state interface from hardware");
    return false;
  }
  if (!node_handle.getParam("/robot_config/joint_names", joint_limits_.joint_names) || joint_limits_.joint_names.size() != 7) {
    ROS_ERROR(
        "EffortJointPositionController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "EffortJointPositionController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "EffortJointPositionController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  k_gains_target_.clear();
  d_gains_target_.clear();


  std::map<std::string, double> pos_limit_lower_map;
  std::map<std::string, double> pos_limit_upper_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_position_limit/lower", pos_limit_lower_map) ) {
  ROS_ERROR(
      "EffortJointPositionController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }
  if (!node_handle.getParam("/robot_config/joint_config/joint_position_limit/upper", pos_limit_upper_map) ) {
  ROS_ERROR(
      "EffortJointPositionController: Joint limits parameters not provided, aborting "
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
        ROS_ERROR("EffortJointPositionController: Unable to find lower position limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
    if (pos_limit_upper_map.find(joint_limits_.joint_names[i]) != pos_limit_upper_map.end())
      {
        joint_limits_.position_upper.push_back(pos_limit_upper_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("EffortJointPositionController: Unable to find upper position limit  values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
  }  

  double controller_state_publish_rate(30.0);
  if (!node_handle.getParam("controller_state_publish_rate", controller_state_publish_rate)) {
    ROS_INFO_STREAM("EffortJointPositionController: Did not find controller_state_publish_rate. Using default "
                    << controller_state_publish_rate << " [Hz].");
  }
  trigger_publish_ = franka_hw::TriggerRate(controller_state_publish_rate);


  try {
    franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("EffortJointPositionController: Exception getting franka state handle: " << ex.what());
    return false;
  }


  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "EffortJointPositionController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_limits_.joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "EffortJointPositionController: Exception getting joint handles: " << ex.what());
      return false;
    }
    k_gains_target_.push_back(k_gains_[i]);
    d_gains_target_.push_back(d_gains_[i]);
  }

  dynamic_reconfigure_controller_gains_node_ =
      ros::NodeHandle("/franka_ros_interface/effort_joint_position_controller/arm/controller_parameters_config");

  dynamic_server_controller_config_ = std::make_unique<
      dynamic_reconfigure::Server<franka_ros_controllers::joint_controller_paramsConfig>>(

      dynamic_reconfigure_controller_gains_node_);
  dynamic_server_controller_config_->setCallback(
      boost::bind(&EffortJointPositionController::controllerConfigCallback, this, _1, _2));

  desired_joints_subscriber_ = node_handle.subscribe(
      "/franka_ros_interface/motion_controller/arm/joint_commands", 20, &EffortJointPositionController::jointCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  publisher_controller_states_.init(node_handle, "/franka_ros_interface/motion_controller/arm/joint_controller_states", 1);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> > lock(
        publisher_controller_states_);
    publisher_controller_states_.msg_.controller_name = "effort_joint_position_controller";
    publisher_controller_states_.msg_.names.resize(joint_limits_.joint_names.size());
    publisher_controller_states_.msg_.joint_controller_states.resize(joint_limits_.joint_names.size());

  }

  for (size_t i = 0; i < 7; ++i) { // this has to be done again; apparently when the dyn callback is initialised everything is set to zeros again!?
    k_gains_target_[i] = k_gains_[i];
    d_gains_target_[i] = d_gains_[i];
  }

  return true;
}

void EffortJointPositionController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = franka_state_handle_->getRobotState();
  for (size_t i = 0; i < 7; ++i) {
    initial_pos_[i] = robot_state.q[i];
  }
  prev_pos_ = initial_pos_;
  pos_d_target_ = initial_pos_;

  std::fill(p_error_last_.begin(), p_error_last_.end(), 0);
  d_error_ = p_error_last_;
}

void EffortJointPositionController::update(const ros::Time& time,
                                             const ros::Duration& period) {
  franka::RobotState robot_state = franka_state_handle_->getRobotState();

  std::array<double, 7> error = p_error_last_;
  std::array<double, 7> error_dot = d_error_;

  for (size_t i = 0; i < 7; i++) {

    error[i] = (pos_d_target_[i] - robot_state.q[i]);

    if (period.toSec() > 0.0)
    {
      error_dot[i] = (error[i] - p_error_last_[i]) / period.toSec();
    }

  }

  d_error_ = error_dot;

  // Compute torque command using PD control law
  std::array<double, 7> tau_d_calculated{};
  for (size_t i = 0; i < 7; ++i) {

    tau_d_calculated[i] = k_gains_[i] * error[i]  + 
                          d_gains_[i] * d_error_[i];

  }

  p_error_last_ = error;

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  if (trigger_publish_() && publisher_controller_states_.trylock()) {
      for (size_t i = 0; i < 7; ++i){

        publisher_controller_states_.msg_.joint_controller_states[i].set_point = pos_d_target_[i];
        publisher_controller_states_.msg_.joint_controller_states[i].process_value = robot_state.q[i];
        publisher_controller_states_.msg_.joint_controller_states[i].process_value_dot = d_error_[i];
        publisher_controller_states_.msg_.joint_controller_states[i].error = error[i];
        publisher_controller_states_.msg_.joint_controller_states[i].time_step = period.toSec();
        publisher_controller_states_.msg_.joint_controller_states[i].command = tau_d_calculated[i];

        publisher_controller_states_.msg_.joint_controller_states[i].p = k_gains_[i];
        publisher_controller_states_.msg_.joint_controller_states[i].d = d_gains_[i];
        publisher_controller_states_.msg_.joint_controller_states[i].header.stamp = time;

      }

      publisher_controller_states_.unlockAndPublish();

    }

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);

    prev_pos_[i] = robot_state.q[i];

    k_gains_[i] = filter_params_ * k_gains_target_[i] + (1.0 - filter_params_) * k_gains_[i];
    d_gains_[i] = filter_params_ * d_gains_target_[i] + (1.0 - filter_params_) * d_gains_[i];
  }

}

bool EffortJointPositionController::checkPositionLimits(std::vector<double> positions)
{
  for (size_t i = 0;  i < 7; ++i){
    if (!((positions[i] <= joint_limits_.position_upper[i]) && (positions[i] >= joint_limits_.position_lower[i]))){
      return true;
    }
  }

  return false;
}

std::array<double, 7> EffortJointPositionController::saturateTorqueRate(
  const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

void EffortJointPositionController::jointCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg) {

  if (msg->mode == franka_core_msgs::JointCommand::POSITION_MODE){
    if (msg->position.size() != 7) {
      ROS_ERROR_STREAM(
          "EffortJointPositionController: Published Commands are not of size 7");
      pos_d_target_ = prev_pos_;
      std::fill(p_error_last_.begin(), p_error_last_.end(), 0);
      d_error_ = p_error_last_;
    }
    else if (checkPositionLimits(msg->position)) {
         ROS_ERROR_STREAM(
            "PositionJointPositionController: Commanded positions are beyond allowed position limits.");
        pos_d_target_ = prev_pos_;
        std::fill(p_error_last_.begin(), p_error_last_.end(), 0);
        d_error_ = p_error_last_;
    }
    else {
      std::copy_n(msg->position.begin(), 7, pos_d_target_.begin());

    }
  }
  // else ROS_ERROR_STREAM("EffortJointPositionController: Published Command msg are not of JointCommand::POSITION_MODE! Dropping message");
}

void EffortJointPositionController::controllerConfigCallback(
    franka_ros_controllers::joint_controller_paramsConfig& config,
    uint32_t /*level*/) {

    k_gains_target_[0] = config.j1_k;
    k_gains_target_[1] = config.j2_k;
    k_gains_target_[2] = config.j3_k;
    k_gains_target_[3] = config.j4_k;
    k_gains_target_[4] = config.j5_k;
    k_gains_target_[5] = config.j6_k;
    k_gains_target_[6] = config.j7_k;

    d_gains_target_[0] = config.j1_d;
    d_gains_target_[1] = config.j2_d;
    d_gains_target_[2] = config.j3_d;
    d_gains_target_[3] = config.j4_d;
    d_gains_target_[4] = config.j5_d;
    d_gains_target_[5] = config.j6_d;
    d_gains_target_[6] = config.j7_d;

}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_ros_controllers::EffortJointPositionController,
                       controller_interface::ControllerBase)
