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
#pragma once

#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <franka_ros_controllers/joint_controller_paramsConfig.h>

#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointControllerStates.h>
#include <franka_core_msgs/JointLimits.h>

#include <mutex>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace franka_ros_controllers {

class VelocityJointVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::array<double, 7> initial_vel_{};
  std::array<double, 7> prev_d_{};
  std::array<double, 7> vel_d_target_{};
  std::array<double, 7> vel_d_;

    // joint_cmd subscriber
  ros::Subscriber desired_joints_subscriber_;

  double filter_joint_vel_{0.3};
  double target_filter_joint_vel_{0.3};
  double filter_factor_{0.01};

  double param_change_filter_{0.005};

  franka_core_msgs::JointLimits joint_limits_;

  // Dynamic reconfigure
  std::unique_ptr< dynamic_reconfigure::Server<franka_ros_controllers::joint_controller_paramsConfig> > dynamic_server_joint_controller_params_;
  ros::NodeHandle dynamic_reconfigure_joint_controller_params_node_;

  franka_hw::TriggerRate trigger_publish_;
  realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> publisher_controller_states_;

  bool checkVelocityLimits(std::vector<double> velocities);


  void jointControllerParamCallback(franka_ros_controllers::joint_controller_paramsConfig& config,
                               uint32_t level);
  void jointVelCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg);
};

}  // namespace franka_ros_controllers
