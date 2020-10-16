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

#include <memory>
#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <franka_ros_controllers/joint_controller_paramsConfig.h>
#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointControllerStates.h>
#include <franka_core_msgs/JointLimits.h>

#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <mutex>

namespace franka_ros_controllers {

class EffortJointPositionController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaStateInterface,
                                            hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};

  double filter_params_{0.005};
  std::vector<double> k_gains_;
  std::vector<double> k_gains_target_;
  std::vector<double> d_gains_;
  std::vector<double> d_gains_target_;
  double coriolis_factor_{1.0};
  std::array<double, 7> pos_d_;
  std::array<double, 7> initial_pos_;
  std::array<double, 7> prev_pos_;
  std::array<double, 7> pos_d_target_;
  std::array<double, 7> d_error_;
  std::array<double, 7> p_error_last_;
  
  franka_hw::FrankaStateInterface* franka_state_interface_{};
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_{};

  franka_hw::TriggerRate rate_trigger_{1.0};
  
  ros::Subscriber desired_joints_subscriber_;
  std::unique_ptr< dynamic_reconfigure::Server<franka_ros_controllers::joint_controller_paramsConfig> > dynamic_server_controller_config_;
  ros::NodeHandle dynamic_reconfigure_controller_gains_node_;

  franka_core_msgs::JointLimits joint_limits_;

  franka_hw::TriggerRate trigger_publish_;
  realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> publisher_controller_states_;

  bool checkPositionLimits(std::vector<double> positions);

  void controllerConfigCallback(franka_ros_controllers::joint_controller_paramsConfig& config,
                               uint32_t level);
  void jointCmdCallback(const franka_core_msgs::JointCommandConstPtr& msg);
};

}  // namespace franka_ros_controllers